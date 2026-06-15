// cloud_processor.cpp
//
// Node A of the ur10e_vision cleaning-path pipeline.
//
// Takes a single organized RGB-D point cloud snapshot from the simulated D435i,
// crops it to a central ROI (75% of image area by default), filters it, keeps
// only the NEAREST connected surface cluster, transforms it into base_link and
// republishes it (latched) for the Python path_generator.
//
// The cloud is kept ORGANIZED throughout: rejected points are set to NaN rather
// than deleted, so the downstream node can address the surface by pixel and do
// its "projected 2D grid + raycast" raster as a simple organized lookup.

#include <limits>
#include <memory>
#include <string>
#include <vector>

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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

inline bool isValid(const PointT & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

inline void invalidate(PointT & p)
{
  p.x = p.y = p.z = kNaN;
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
    crop_area_fraction_ = declare_parameter<double>("crop_area_fraction", 0.75);
    depth_min_ = declare_parameter<double>("depth_min", 0.15);
    depth_max_ = declare_parameter<double>("depth_max", 3.0);
    sor_mean_k_ = declare_parameter<int>("sor_mean_k", 30);
    sor_stddev_mul_ = declare_parameter<double>("sor_stddev_mul", 1.0);
    cluster_tolerance_ = declare_parameter<double>("cluster_tolerance", 0.02);
    min_cluster_size_ = declare_parameter<int>("min_cluster_size", 200);
    max_cluster_size_ = declare_parameter<int>("max_cluster_size", 1000000);
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

    RCLCPP_INFO(get_logger(),
      "cloud_processor up. input=%s output_frame=%s crop_area=%.2f "
      "depth=[%.2f, %.2f] %s",
      input_topic_.c_str(), output_frame_.c_str(), crop_area_fraction_,
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

    // --- 1. Central crop (75% of area => linear fraction sqrt(area_fraction)).
    const double lin = std::sqrt(std::clamp(crop_area_fraction_, 0.0, 1.0));
    const uint32_t roi_w = static_cast<uint32_t>(std::round(W * lin));
    const uint32_t roi_h = static_cast<uint32_t>(std::round(H * lin));
    const uint32_t x0 = (W - roi_w) / 2;
    const uint32_t y0 = (H - roi_h) / 2;
    const uint32_t x1 = x0 + roi_w;
    const uint32_t y1 = y0 + roi_h;

    for (uint32_t v = 0; v < H; ++v) {
      for (uint32_t u = 0; u < W; ++u) {
        if (u < x0 || u >= x1 || v < y0 || v >= y1) {
          invalidate(cloud->at(u, v));
        }
      }
    }

    // --- 2. Depth-band passthrough (camera optical Z), keep organized.
    {
      pcl::PassThrough<PointT> pt;
      pt.setKeepOrganized(true);
      pt.setInputCloud(cloud);
      pt.setFilterFieldName("z");
      pt.setFilterLimits(depth_min_, depth_max_);
      pt.filter(*cloud);
    }

    // --- 3. Statistical outlier removal, keep organized.
    {
      pcl::StatisticalOutlierRemoval<PointT> sor;
      sor.setKeepOrganized(true);
      sor.setInputCloud(cloud);
      sor.setMeanK(sor_mean_k_);
      sor.setStddevMulThresh(sor_stddev_mul_);
      sor.filter(*cloud);
    }

    // --- 4. Euclidean clustering -> keep NEAREST large cluster.
    auto valid = std::make_shared<std::vector<int>>();
    valid->reserve(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
      if (isValid(cloud->points[i])) {
        valid->push_back(static_cast<int>(i));
      }
    }
    if (valid->size() < static_cast<size_t>(min_cluster_size_)) {
      RCLCPP_WARN(get_logger(),
        "Only %zu valid points after filtering (< min_cluster_size=%d).",
        valid->size(), min_cluster_size_);
      return false;
    }

    auto tree = std::make_shared<pcl::search::KdTree<PointT>>();
    tree->setInputCloud(cloud, valid);

    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.setIndices(valid);
    ec.extract(clusters);

    if (clusters.empty()) {
      RCLCPP_WARN(get_logger(), "No clusters found in ROI.");
      return false;
    }

    // Nearest cluster = smallest mean optical-Z (closest to camera).
    int best = -1;
    double best_z = std::numeric_limits<double>::max();
    for (size_t c = 0; c < clusters.size(); ++c) {
      double sz = 0.0;
      for (int idx : clusters[c].indices) {
        sz += cloud->points[idx].z;
      }
      const double mean_z = sz / clusters[c].indices.size();
      if (mean_z < best_z) {
        best_z = mean_z;
        best = static_cast<int>(c);
      }
    }

    std::vector<bool> keep(cloud->size(), false);
    for (int idx : clusters[best].indices) {
      keep[idx] = true;
    }
    for (size_t i = 0; i < cloud->size(); ++i) {
      if (!keep[i]) {
        invalidate(cloud->points[i]);
      }
    }
    RCLCPP_INFO(get_logger(),
      "Kept nearest cluster: %zu pts, mean depth %.3f m (of %zu clusters).",
      clusters[best].indices.size(), best_z, clusters.size());

    // --- 5. Transform organized cloud into output_frame (NaNs preserved).
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(
        output_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(get_logger(), "TF %s <- %s failed: %s",
        output_frame_.c_str(), msg->header.frame_id.c_str(), ex.what());
      return false;
    }
    const Eigen::Affine3d a = tf2::transformToEigen(tf);
    auto out = std::make_shared<CloudT>();
    pcl::transformPointCloud(*cloud, *out, a.cast<float>());
    out->header = cloud->header;
    out->width = cloud->width;
    out->height = cloud->height;
    out->is_dense = false;

    // --- 6. Publish (latched).
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*out, out_msg);
    out_msg.header.frame_id = output_frame_;
    out_msg.header.stamp = msg->header.stamp;
    cloud_pub_->publish(out_msg);

    sensor_msgs::msg::RegionOfInterest roi;
    roi.x_offset = x0;
    roi.y_offset = y0;
    roi.width = roi_w;
    roi.height = roi_h;
    roi.do_rectify = false;
    roi_pub_->publish(roi);

    RCLCPP_INFO(get_logger(),
      "Published surface_cloud (%ux%u organized) in %s; ROI [%u,%u %ux%u].",
      out->width, out->height, output_frame_.c_str(), x0, y0, roi_w, roi_h);
    return true;
  }

  // params
  std::string input_topic_;
  std::string output_frame_;
  bool capture_on_start_{false};
  double crop_area_fraction_{0.75};
  double depth_min_{0.15};
  double depth_max_{3.0};
  int sor_mean_k_{30};
  double sor_stddev_mul_{1.0};
  double cluster_tolerance_{0.02};
  int min_cluster_size_{200};
  int max_cluster_size_{1000000};
  double tf_timeout_{1.0};

  // state
  bool captured_{false};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_msg_;

  // ros
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
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
