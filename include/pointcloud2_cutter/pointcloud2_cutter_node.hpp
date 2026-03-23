#pragma once

#include "pointcloud2_cutter/pointcloud_filter.hpp"
#include "pointcloud2_cutter/region_selector.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace pointcloud2_cutter
{

class PointCloud2CutterNode : public rclcpp::Node
{
public:
  PointCloud2CutterNode();

private:
  void handlePose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishLaserScan(
    const sensor_msgs::msg::PointCloud2 & msg,
    const std::shared_ptr<sensor_msgs::msg::PointCloud2> & prefiltered);
  void publishRegionMarkers();

  std::pair<double, double> currentLaserZ() const;

  mutable std::mutex pose_mutex_;
  double current_x_{};
  double current_y_{};
  bool has_pose_{false};

  std::string output_frame_id_;

  RegionSelector region_selector_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr region_marker_pub_;

  bool publish_scan_{false};
  double scan_height_min_{};
  double scan_height_max_{};
  double scan_angle_min_{};
  double scan_angle_max_{};
  double scan_angle_increment_{};
  double scan_range_min_{};
  double scan_range_max_{};
  std::size_t scan_bin_count_{0};
  std::string scan_frame_id_;
  std::string region_marker_frame_id_;
  bool region_markers_published_{false};
  std::size_t max_filtered_points_{0};
};

}  // namespace pointcloud2_cutter
