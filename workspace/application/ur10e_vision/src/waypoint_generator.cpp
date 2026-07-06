#include <memory>
#include <string>
#include <algorithm>
#include <functional>
#include <cmath>
#include <vector>
#include <optional>
#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;
using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using Vec3 = Eigen::Vector3d;
using OptVec3 = std::optional<Vec3>;

namespace
{
rclcpp::QoS latchedQos()
{
    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();
    return qos;
}
}  // namespace

class WaypointGenerator : public rclcpp::Node
{
public:
    WaypointGenerator()
    : Node("waypoint_generator")
    {
        // ---- parameters (waypoint_generator section of vision_params.yaml) ----
        surface_topic_ = declare_parameter<std::string>("surface_cloud_topic", "/ur10e_vision/surface_cloud");
        output_frame_  = declare_parameter<std::string>("output_frame", "base_link");
        camera_frame_  = declare_parameter<std::string>("camera_optical_frame", "camera_color_optical_frame");
        raster_dir_    = declare_parameter<std::string>("raster_direction", "horizontal");

        tool_width_     = declare_parameter<double>("tool_width", 0.05);
        waypoint_space_ = declare_parameter<double>("waypoint_spacing", 0.02);
        standoff_       = declare_parameter<double>("standoff", 0.05);
        approach_sign_  = declare_parameter<double>("approach_axis_sign", -1.0);
        raster_length_  = declare_parameter<double>("raster_length", 0.30);
        raster_height_  = declare_parameter<double>("raster_height", 0.20);

        raster_center_x_ = declare_parameter<double>("raster_center_x", 0.0);
        raster_center_y_ = declare_parameter<double>("raster_center_y", 0.0);

        publish_wp_tf_  = declare_parameter<bool>("publish_waypoint_tf", true);
        wp_tf_prefix_   = declare_parameter<std::string>("waypoint_tf_prefix", "clean_wp_");

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        static_bcast_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        pcl_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            surface_topic_, latchedQos(),
            std::bind(&WaypointGenerator::pclCb, this, _1));

        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ur10e_vision/path_markers", latchedQos());

        RCLCPP_INFO(get_logger(),
            "waypoint_generator up. tool_width=%.3f spacing=%.3f standoff=%.3f raster=%s",
            tool_width_, waypoint_space_, standoff_, raster_dir_.c_str());
    }

private:

    void pclCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Skip re-streamed snapshots; only regenerate on a new capture (stamp).
        if (have_last_stamp_ &&
            msg->header.stamp.sec == last_stamp_.sec &&
            msg->header.stamp.nanosec == last_stamp_.nanosec)
        {
            return;
        }
        last_stamp_ = msg->header.stamp;
        have_last_stamp_ = true;

        RCLCPP_INFO(get_logger(), "Got surface cloud %u x %u (%s)",
            msg->width, msg->height, msg->header.frame_id.c_str());
        generate(msg);
    }


    static bool isValid(const PointT & p)
    {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    }

    // Median 3D distance between neighbouring valid pixels: metres-per-pixel,
    // used to convert the metric raster pitch/spacing into pixel steps.
    double medianStep(const CloudT & cloud, bool horizontal) const
    {
        std::vector<double> d;
        d.reserve(cloud.size());
        const int w = static_cast<int>(cloud.width);
        const int h = static_cast<int>(cloud.height);
        for (int v = 0; v < h; ++v)
        {
            for (int u = 0; u < w; ++u)
            {
                const int nu = horizontal ? u + 1 : u;
                const int nv = horizontal ? v : v + 1;
                if (nu >= w || nv >= h) continue;
                const PointT & a = cloud.at(u, v);
                const PointT & b = cloud.at(nu, nv);
                if (!isValid(a) || !isValid(b)) continue;
                const double dd = (a.getVector3fMap() - b.getVector3fMap()).norm();
                if (std::isfinite(dd) && dd > 0.0) d.push_back(dd);
            }
        }
        if (d.empty()) return 0.0;
        const size_t mid = d.size() / 2;
        std::nth_element(d.begin(), d.begin() + mid, d.end());
        return d[mid];
    }

    // Single grid pixel -> point (nullopt if that pixel is NaN).
    static OptVec3 pixel(const CloudT & cloud, int u, int v)
    {
        if (u < 0 || v < 0 || u >= static_cast<int>(cloud.width) || v >= static_cast<int>(cloud.height))
            return std::nullopt;
        const PointT & p = cloud.at(u, v);
        if (!isValid(p)) return std::nullopt;
        return p.getVector3fMap().cast<double>();
    }

    // 1-D Catmull-Rom: passes through p1/p2, tangents from p0/p3 (C1-smooth, so
    // no orientation jump at the measured/filled seam). t in [0,1] between p1,p2.
    static Vec3 catmullRom(const Vec3 & p0, const Vec3 & p1, const Vec3 & p2, const Vec3 & p3, double t)
    {
        const double t2 = t * t, t3 = t2 * t;
        return 0.5 * ((2.0 * p1) + (-p0 + p2) * t +
                      (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
                      (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
    }

    static bool fillLine(std::vector<OptVec3> & line)
    {
        std::vector<int> valid;
        for (int i = 0; i < static_cast<int>(line.size()); ++i)
            if (line[i]) valid.push_back(i);
        if (valid.empty()) return false;

        // interior: linear interpolation between consecutive valid cells
        for (size_t k = 0; k + 1 < valid.size(); ++k)
        {
            const int a = valid[k], b = valid[k + 1];
            for (int i = a + 1; i < b; ++i)
            {
                const double t = static_cast<double>(i - a) / static_cast<double>(b - a);
                line[i] = (1.0 - t) * (*line[a]) + t * (*line[b]);
            }
        }

        // edges: continue the trend of the two outermost valid cells (or hold
        // constant if there is only one valid cell to anchor on)
        const int f = valid.front(), l = valid.back();
        const Vec3 head_step = (valid.size() >= 2) ? Vec3(*line[f] - *line[valid[1]]) : Vec3(Vec3::Zero());
        for (int i = f - 1; i >= 0; --i) line[i] = *line[f] + head_step * (f - i);
        const Vec3 tail_step = (valid.size() >= 2) ? Vec3(*line[l] - *line[valid[valid.size() - 2]]) : Vec3(Vec3::Zero());
        for (int i = l + 1; i < static_cast<int>(line.size()); ++i) line[i] = *line[l] + tail_step * (i - l);
        return true;
    }

    static void fillGrid(std::vector<std::vector<OptVec3>> & g)
    {
        for (auto & row : g) fillLine(row);

        const int R = static_cast<int>(g.size());
        const int C = static_cast<int>(g[0].size());
        for (int c = 0; c < C; ++c)
        {
            bool any_empty = false;
            std::vector<OptVec3> col(R);
            for (int r = 0; r < R; ++r) { col[r] = g[r][c]; if (!col[r]) any_empty = true; }
            if (!any_empty) continue;
            if (!fillLine(col)) continue;                 // column had no data either
            for (int r = 0; r < R; ++r) if (!g[r][c]) g[r][c] = col[r];
        }
    }

    static void smoothFilled(std::vector<std::vector<OptVec3>> & g,
                             const std::vector<std::vector<bool>> & measured)
    {
        const int R = static_cast<int>(g.size());
        const int C = static_cast<int>(g[0].size());
        for (int iter = 0; iter < 2; ++iter)
        {
            for (int r = 0; r < R; ++r)             // along rows
                for (int c = 2; c < C - 2; ++c)
                    if (!measured[r][c])
                        g[r][c] = catmullRom(*g[r][c - 2], *g[r][c - 1], *g[r][c + 1], *g[r][c + 2], 0.5);
            for (int c = 0; c < C; ++c)             // down columns
                for (int r = 2; r < R - 2; ++r)
                    if (!measured[r][c])
                        g[r][c] = catmullRom(*g[r - 2][c], *g[r - 1][c], *g[r + 1][c], *g[r + 2][c], 0.5);
        }
    }

    OptVec3 normalAt(const std::vector<std::vector<OptVec3>> & g, int r, int c,
                     const Vec3 & pos, const OptVec3 & cam) const
    {
        const int R = static_cast<int>(g.size());
        const int C = static_cast<int>(g[0].size());
        auto at = [&](int rr, int cc) -> OptVec3 {
            if (rr < 0 || cc < 0 || rr >= R || cc >= C) return std::nullopt;
            return g[rr][cc];
        };
        auto tangent = [&](OptVec3 lo, OptVec3 hi) -> OptVec3 {
            if (lo && hi) return *hi - *lo;         // central difference
            if (hi) return *hi - pos;               // forward
            if (lo) return pos - *lo;               // backward
            return std::nullopt;
        };
        const OptVec3 t_along = tangent(at(r, c - 1), at(r, c + 1));
        const OptVec3 t_pitch = tangent(at(r - 1, c), at(r + 1, c));
        if (!t_along || !t_pitch) return std::nullopt;

        Vec3 n = t_along->cross(*t_pitch);
        if (n.norm() < 1e-9) return std::nullopt;
        n.normalize();
        if (cam && n.dot(*cam - pos) < 0.0) n = -n;
        return n;
    }

    OptVec3 cameraOrigin() const
    {
        try
        {
            const auto tf = tf_buffer_->lookupTransform(
                output_frame_, camera_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
            const auto & t = tf.transform.translation;
            return Vec3(t.x, t.y, t.z);
        }
        catch (const std::exception & e)
        {
            RCLCPP_WARN(get_logger(), "No TF %s<-%s: %s (normals unoriented)",
                output_frame_.c_str(), camera_frame_.c_str(), e.what());
            return std::nullopt;
        }
    }

    void generate(const sensor_msgs::msg::PointCloud2::SharedPtr & msg)
    {
        CloudT cloud;
        pcl::fromROSMsg(*msg, cloud);
        if (cloud.height <= 1)
        {
            RCLCPP_WARN(get_logger(), "Cloud is not organised; cannot raster.");
            return;
        }
        const int w = static_cast<int>(cloud.width);
        const int h = static_cast<int>(cloud.height);

        const double m_h = medianStep(cloud, true);
        const double m_v = medianStep(cloud, false);
        if (m_h <= 0.0 || m_v <= 0.0)
        {
            RCLCPP_WARN(get_logger(), "Invalid metres-per-pixel (m_h=%.4f m_v=%.4f).", m_h, m_v);
            return;
        }
        const int along_px = std::max(1, static_cast<int>(std::round(waypoint_space_ / m_h)));
        const int pitch_px = std::max(1, static_cast<int>(std::round(tool_width_ / m_v)));

        const bool vertical = (raster_dir_ == "vertical");

        const double m_along = vertical ? m_v : m_h;
        const double m_pass  = vertical ? m_h : m_v;
        const int along_half = static_cast<int>(std::round(0.5 * raster_length_ / m_along));
        const int pass_half  = static_cast<int>(std::round(0.5 * raster_height_ / m_pass));

        // Window centre = image centre shifted by the raster_center offsets
        // (metres in the camera image plane, converted to pixels).
        const int cu = w / 2 + static_cast<int>(std::round(raster_center_x_ / m_h));
        const int cv = h / 2 + static_cast<int>(std::round(raster_center_y_ / m_v));
        const int along_ctr  = vertical ? cv : cu;
        const int pass_ctr   = vertical ? cu : cv;
        const int along_max  = vertical ? h : w;
        const int pass_max   = vertical ? w : h;

        const int along_lo = std::max(0, along_ctr - along_half);
        const int along_hi = std::min(along_max, along_ctr + along_half);
        const int pass_lo  = std::max(0, pass_ctr - pass_half);
        const int pass_hi  = std::min(pass_max, pass_ctr + pass_half);

        // Pixel coordinates of every pass line and every along-step.
        std::vector<int> pass_px, along_px_list;
        for (int p = pass_lo; p < pass_hi; p += pitch_px) pass_px.push_back(p);
        for (int q = along_lo; q < along_hi; q += along_px) along_px_list.push_back(q);
        if (pass_px.size() < 1 || along_px_list.size() < 2)
        {
            RCLCPP_WARN(get_logger(),
                "raster_length/raster_height too small for a grid (need >=2 along-steps).");
            return;
        }

        std::vector<std::vector<OptVec3>> grid(pass_px.size(),
                                               std::vector<OptVec3>(along_px_list.size()));
        std::vector<std::vector<bool>> measured(pass_px.size(),
                                                std::vector<bool>(along_px_list.size(), false));
        for (size_t r = 0; r < pass_px.size(); ++r)
        {
            for (size_t c = 0; c < along_px_list.size(); ++c)
            {
                const int u = vertical ? pass_px[r] : along_px_list[c];
                const int v = vertical ? along_px_list[c] : pass_px[r];
                grid[r][c] = pixel(cloud, u, v);
                measured[r][c] = grid[r][c].has_value();
            }
        }
        fillGrid(grid);
        smoothFilled(grid, measured);

        const OptVec3 cam = cameraOrigin();

        // Emit serpentine passes; skip only the trimmed (off-surface) ends.
        std::vector<Vec3> positions, normals;
        bool serpentine = false;
        for (size_t r = 0; r < grid.size(); ++r)
        {
            std::vector<int> cols(along_px_list.size());
            for (int c = 0; c < static_cast<int>(cols.size()); ++c) cols[c] = c;
            if (serpentine) std::reverse(cols.begin(), cols.end());
            serpentine = !serpentine;

            for (int c : cols)
            {
                if (!grid[r][c]) continue;
                const Vec3 pos = *grid[r][c];
                OptVec3 n = normalAt(grid, static_cast<int>(r), c, pos, cam);
                if (!n)  // last resort: face the camera so a pose always exists
                    n = cam ? (*cam - pos).normalized() : Vec3::UnitZ();
                positions.push_back(pos);
                normals.push_back(*n);
            }
        }

        if (positions.size() < 2)
        {
            RCLCPP_WARN(get_logger(), "Only %zu waypoints; check the surface cloud.",
                positions.size());
            return;
        }

        publish(positions, normals, msg->header.stamp);
        RCLCPP_INFO(get_logger(),
            "Generated %zu waypoints (pitch=%dpx/%.3fm along=%dpx/%.3fm, gaps bridged).",
            positions.size(), pitch_px, tool_width_, along_px, waypoint_space_);
    }

    void publish(const std::vector<Vec3> & positions,
                 const std::vector<Vec3> & normals,
                 const builtin_interfaces::msg::Time & stamp)
    {
        geometry_msgs::msg::PoseArray pa;
        pa.header.frame_id = output_frame_;
        pa.header.stamp = stamp;

        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        const size_t n = positions.size();

        for (size_t i = 0; i < n; ++i)
        {
            Vec3 z = approach_sign_ * normals[i];                  // tool +Z into/out of surface
            z.normalize();

            Vec3 t = (i + 1 < n) ? (positions[i + 1] - positions[i])
                                 : (positions[i] - positions[i - 1]);
            t = t - t.dot(z) * z;                                  // project onto tangent plane
            if (t.norm() < 1e-6)
            {
                t = z.cross(Vec3::UnitX());
                if (t.norm() < 1e-6) t = z.cross(Vec3::UnitY());
            }
            const Vec3 x = t.normalized();
            const Vec3 y = z.cross(x);

            Eigen::Matrix3d R;
            R.col(0) = x; R.col(1) = y; R.col(2) = z;
            Eigen::Quaterniond quat(R);
            quat.normalize();

            const Vec3 wp = positions[i] + standoff_ * normals[i];

            geometry_msgs::msg::Pose pose;
            pose.position.x = wp.x(); pose.position.y = wp.y(); pose.position.z = wp.z();
            pose.orientation.x = quat.x(); pose.orientation.y = quat.y();
            pose.orientation.z = quat.z(); pose.orientation.w = quat.w();
            pa.poses.push_back(pose);

            if (publish_wp_tf_)
            {
                geometry_msgs::msg::TransformStamped ts;
                ts.header.frame_id = output_frame_;
                ts.header.stamp = stamp;
                char child[64];
                std::snprintf(child, sizeof(child), "%s%03zu", wp_tf_prefix_.c_str(), i);
                ts.child_frame_id = child;
                ts.transform.translation.x = wp.x();
                ts.transform.translation.y = wp.y();
                ts.transform.translation.z = wp.z();
                ts.transform.rotation = pose.orientation;
                transforms.push_back(ts);
            }
        }

        if (!transforms.empty()) static_bcast_->sendTransform(transforms);
        marker_pub_->publish(buildMarkers(pa, normals, stamp));
    }

    visualization_msgs::msg::MarkerArray buildMarkers(
        const geometry_msgs::msg::PoseArray & pa,
        const std::vector<Vec3> & normals,
        const builtin_interfaces::msg::Time & stamp) const
    {
        visualization_msgs::msg::MarkerArray ma;

        visualization_msgs::msg::Marker line;
        line.header.frame_id = output_frame_;
        line.header.stamp = stamp;
        line.ns = "cleaning_path";
        line.id = 0;
        line.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line.action = visualization_msgs::msg::Marker::ADD;
        line.scale.x = 0.004;
        line.color.r = 0.1f; line.color.g = 0.8f; line.color.b = 1.0f; line.color.a = 1.0f;
        for (const auto & pose : pa.poses) line.points.push_back(pose.position);
        ma.markers.push_back(line);

        for (size_t i = 0; i < pa.poses.size(); ++i)
        {
            visualization_msgs::msg::Marker arr;
            arr.header.frame_id = output_frame_;
            arr.header.stamp = stamp;
            arr.ns = "normals";
            arr.id = static_cast<int>(i + 1);
            arr.type = visualization_msgs::msg::Marker::ARROW;
            arr.action = visualization_msgs::msg::Marker::ADD;
            arr.scale.x = 0.004; arr.scale.y = 0.008; arr.scale.z = 0.0;
            arr.color.r = 1.0f; arr.color.g = 0.2f; arr.color.b = 0.2f; arr.color.a = 0.9f;
            geometry_msgs::msg::Point s = pa.poses[i].position;
            geometry_msgs::msg::Point e;
            e.x = s.x + 0.03 * normals[i].x();
            e.y = s.y + 0.03 * normals[i].y();
            e.z = s.z + 0.03 * normals[i].z();
            arr.points.push_back(s);
            arr.points.push_back(e);
            ma.markers.push_back(arr);
        }
        return ma;
    }

    std::string surface_topic_, output_frame_, camera_frame_, raster_dir_, wp_tf_prefix_;
    double tool_width_, waypoint_space_, standoff_, approach_sign_, raster_length_, raster_height_;
    double raster_center_x_, raster_center_y_;
    bool publish_wp_tf_;

    builtin_interfaces::msg::Time last_stamp_;   // stamp of last processed cloud
    bool have_last_stamp_{false};

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_bcast_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointGenerator>());
    rclcpp::shutdown();
    return 0;
}
