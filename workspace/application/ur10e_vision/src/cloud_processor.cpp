#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

namespace
{
inline bool isValid(const PointT & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
}  // namespace

class CloudProcessor : public rclcpp::Node
{
public:
  CloudProcessor()
  : rclcpp::Node("cloud_processor")
  {
    input_topic_ = declare_parameter<std::string>("input_cloud_topic", "/camera/camera/points");
    output_frame_ = declare_parameter<std::string>("output_frame", "base_link");
    capture_on_start_ = declare_parameter<bool>("capture_on_start", false);
    depth_min_ = declare_parameter<double>("depth_min", 0.15);
    depth_max_ = declare_parameter<double>("depth_max", 4.0);
    max_fill_gap_px_ = declare_parameter<int>("max_fill_gap_px", 120);
    republish_rate_hz_ = declare_parameter<double>("republish_rate_hz", 2.0);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 1.0);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto sensor_qos = rclcpp::SensorDataQoS();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sensor_qos,
      std::bind(&CloudProcessor::cloudCb, this, std::placeholders::_1));

    rclcpp::QoS latched(1);
    latched.transient_local().reliable();
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/ur10e_vision/surface_cloud", latched);
    roi_pub_ = create_publisher<sensor_msgs::msg::RegionOfInterest>(
      "/ur10e_vision/roi", latched);

    trigger_srv_ = create_service<std_srvs::srv::Trigger>(
      "/ur10e_vision/trigger",
      std::bind(&CloudProcessor::triggerCb, this,
        std::placeholders::_1, std::placeholders::_2));

    // Re-stream the last computed cloud; filling only ever runs in process().
    if (republish_rate_hz_ > 0.0) {
      republish_timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / republish_rate_hz_),
        std::bind(&CloudProcessor::republishCb, this));
    }

    RCLCPP_INFO(get_logger(),
      "cloud_processor up. input=%s output_frame=%s depth=[%.2f, %.2f] %s",
      input_topic_.c_str(), output_frame_.c_str(),
      depth_min_, depth_max_,
      capture_on_start_ ? "(auto-capture first frame)" : "(waiting for /ur10e_vision/trigger)");
  }

private:
  void cloudCb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    last_msg_ = msg;
    if (capture_on_start_ && !captured_) {
      if (process(msg)) {
        captured_ = true;
      }
    }
  }

  void triggerCb(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!last_msg_) {
      res->success = false;
      res->message = "No point cloud received yet on " + input_topic_;
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }
    res->success = process(last_msg_);
    res->message = res->success
      ? "Surface cloud captured and published."
      : "Processing failed (no valid surface in ROI / TF unavailable).";
  }

  // Full pipeline. Returns true if a non-empty surface cloud was published.
  bool process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    auto cloud = std::make_shared<CloudT>();
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->height <= 1 || cloud->width <= 1) {
      RCLCPP_ERROR(get_logger(),
        "Input cloud is not organized (h=%u w=%u). The projected-grid raster "
        "needs an organized cloud from a depth camera.",
        cloud->height, cloud->width);
      return false;
    }
    cloud->is_dense = false;

    const uint32_t W = cloud->width;
    const uint32_t H = cloud->height;

    // --- 1. No cropping: ROI is the whole frame (raster window is set downstream).
    const uint32_t x0 = 0;
    const uint32_t y0 = 0;
    const uint32_t roi_w = W;
    const uint32_t roi_h = H;

    // --- 2. Depth-band passthrough (camera optical Z), keep organized.
    {
      pcl::PassThrough<PointT> pt;
      pt.setKeepOrganized(true);
      pt.setInputCloud(cloud);
      pt.setFilterFieldName("z");
      pt.setFilterLimits(depth_min_, depth_max_);
      pt.filter(*cloud);
    }

    // --- 3. Interior hole filling (bilinear, gap-capped).
    const size_t filled = fillHoles(*cloud);

    size_t valid_count = 0;
    for (const auto & p : cloud->points) {
      if (isValid(p)) ++valid_count;
    }
    if (valid_count == 0) {
      RCLCPP_WARN(get_logger(), "No valid points after depth filtering.");
      return false;
    }
    RCLCPP_INFO(get_logger(),
      "Hole fill: patched %zu interior cells (%zu valid points total).",
      filled, valid_count);

    // --- 4. Transform into output_frame (NaNs preserved). The Gazebo cloud lags
    // the TF stream, so fall back to the latest transform if the exact stamp is
    // unavailable -- correct for a static snapshot.
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        output_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(),
        "TF at cloud stamp unavailable (%s); using latest transform instead.",
        ex.what());
      try {
        tf = tf_buffer_->lookupTransform(
          output_frame_, msg->header.frame_id, rclcpp::Time(0, 0, RCL_ROS_TIME),
          rclcpp::Duration::from_seconds(tf_timeout_));
      } catch (const tf2::TransformException & ex2) {
        RCLCPP_ERROR(get_logger(), "TF %s <- %s failed: %s",
          output_frame_.c_str(), msg->header.frame_id.c_str(), ex2.what());
        return false;
      }
    }
    const Eigen::Affine3d a = tf2::transformToEigen(tf);
    auto out = std::make_shared<CloudT>();
    pcl::transformPointCloud(*cloud, *out, a.cast<float>());
    out->header = cloud->header;
    out->width = cloud->width;
    out->height = cloud->height;
    out->is_dense = false;

    // --- 5. Store the snapshot and publish once; the timer re-streams it.
    pcl::toROSMsg(*out, last_cloud_msg_);
    last_cloud_msg_.header.frame_id = output_frame_;
    last_cloud_msg_.header.stamp = msg->header.stamp;

    last_roi_.x_offset = x0;
    last_roi_.y_offset = y0;
    last_roi_.width = roi_w;
    last_roi_.height = roi_h;
    last_roi_.do_rectify = false;

    have_output_ = true;
    publishOutputs();

    RCLCPP_INFO(get_logger(),
      "Published surface_cloud (%ux%u organized) in %s; ROI [%u,%u %ux%u].",
      out->width, out->height, output_frame_.c_str(), x0, y0, roi_w, roi_h);
    return true;
  }

  void publishOutputs()
  {
    if (!have_output_) return;
    cloud_pub_->publish(last_cloud_msg_);
    roi_pub_->publish(last_roi_);
  }

  void republishCb()
  {
    publishOutputs();
  }

  // Blend the row (h) and column (v) estimates for one hole cell; nullopt = leave NaN.
  static std::optional<Eigen::Vector3f> reconcile(
    bool h_valid, const Eigen::Vector3f & h_est,
    bool v_valid, const Eigen::Vector3f & v_est)
  {
    if (h_valid && v_valid) return Eigen::Vector3f(0.5f * (h_est + v_est));
    if (h_valid) return h_est;
    if (v_valid) return v_est;
    return std::nullopt;
  }

  // Interior, gap-capped, bilinear hole fill. Row and column passes each read only
  // originally-valid cells (so fills never cascade); reconcile() blends the two.
  size_t fillHoles(CloudT & cloud) const
  {
    const int W = static_cast<int>(cloud.width);
    const int H = static_cast<int>(cloud.height);
    const int cap = max_fill_gap_px_;

    std::vector<bool> h_has(cloud.size(), false), v_has(cloud.size(), false);
    std::vector<Eigen::Vector3f> h_est(cloud.size()), v_est(cloud.size());

    // Linearly interpolate the open interval between valid endpoints a and b.
    auto interpRun = [&](int a_idx, int b_idx, int span,
                         const std::function<int(int)> & idx_at,
                         std::vector<Eigen::Vector3f> & est,
                         std::vector<bool> & has) {
      const Eigen::Vector3f pa = cloud.points[a_idx].getVector3fMap();
      const Eigen::Vector3f pb = cloud.points[b_idx].getVector3fMap();
      for (int k = 1; k < span; ++k) {
        const float t = static_cast<float>(k) / static_cast<float>(span);
        const int cur = idx_at(k);
        est[cur] = (1.0f - t) * pa + t * pb;
        has[cur] = true;
      }
    };

    // Row pass: scan each row, bridge bracketed gaps <= cap.
    for (int v = 0; v < H; ++v) {
      int last = -1;
      for (int u = 0; u < W; ++u) {
        const int idx = v * W + u;
        if (!isValid(cloud.points[idx])) continue;
        const int span = u - last;
        if (last >= 0 && span - 1 > 0 && span - 1 <= cap) {
          interpRun(v * W + last, idx, span,
                    [&](int k) { return v * W + (last + k); }, h_est, h_has);
        }
        last = u;
      }
    }

    // Column pass: scan each column, bridge bracketed gaps <= cap.
    for (int u = 0; u < W; ++u) {
      int last = -1;
      for (int v = 0; v < H; ++v) {
        const int idx = v * W + u;
        if (!isValid(cloud.points[idx])) continue;
        const int span = v - last;
        if (last >= 0 && span - 1 > 0 && span - 1 <= cap) {
          interpRun(last * W + u, idx, span,
                    [&](int k) { return (last + k) * W + u; }, v_est, v_has);
        }
        last = v;
      }
    }

    // Reconcile the two estimates and write patched points back into the cloud.
    size_t filled = 0;
    for (size_t i = 0; i < cloud.size(); ++i) {
      if (isValid(cloud.points[i])) continue;
      const auto blended = reconcile(h_has[i], h_est[i], v_has[i], v_est[i]);
      if (blended) {
        cloud.points[i].x = blended->x();
        cloud.points[i].y = blended->y();
        cloud.points[i].z = blended->z();
        ++filled;
      }
    }
    return filled;
  }

  // params
  std::string input_topic_;
  std::string output_frame_;
  bool capture_on_start_{false};
  double depth_min_{0.15};
  double depth_max_{4.0};
  int max_fill_gap_px_{120};
  double republish_rate_hz_{2.0};
  double tf_timeout_{1.0};

  // state
  bool captured_{false};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_msg_;
  sensor_msgs::msg::PointCloud2 last_cloud_msg_;   // last computed surface cloud
  sensor_msgs::msg::RegionOfInterest last_roi_;    // ROI paired with it
  bool have_output_{false};                        // a snapshot exists to republish

  // ros
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudProcessor>());
  rclcpp::shutdown();
  return 0;
}
