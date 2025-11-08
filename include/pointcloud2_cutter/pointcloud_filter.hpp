#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <utility>

namespace pointcloud2_cutter
{

class PointCloudFilter
{
public:
  static std::shared_ptr<sensor_msgs::msg::PointCloud2> filterByHeight(
    const sensor_msgs::msg::PointCloud2 & input,
    double z_min,
    double z_max,
    const std::string & frame_id);

  static std::shared_ptr<sensor_msgs::msg::PointCloud2> limitPointCount(
    const std::shared_ptr<sensor_msgs::msg::PointCloud2> & input,
    std::size_t max_points);
};

}  // namespace pointcloud2_cutter
