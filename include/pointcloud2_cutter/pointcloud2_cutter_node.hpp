#pragma once

#include "pointcloud2_cutter/pointcloud_filter.hpp"
#include "pointcloud2_cutter/region_selector.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mutex>
#include <optional>
#include <string>

namespace pointcloud2_cutter
{

class PointCloud2CutterNode : public rclcpp::Node
{
public:
  PointCloud2CutterNode();

private:
  void handlePose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

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
};

}  // namespace pointcloud2_cutter

