#include "pointcloud2_cutter/pointcloud_filter.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <array>
#include <vector>

namespace pointcloud2_cutter
{

std::shared_ptr<sensor_msgs::msg::PointCloud2> PointCloudFilter::filterByHeight(
  const sensor_msgs::msg::PointCloud2 & input,
  double z_min,
  double z_max,
  const std::string & frame_id)
{
  std::vector<std::array<float, 3>> kept_points;
  kept_points.reserve(input.width * input.height);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(input, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    const float z = *iter_z;
    if (z < z_min || z > z_max)
    {
      continue;
    }

    kept_points.push_back({*iter_x, *iter_y, z});
  }

  auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
  output->header = input.header;
  output->header.frame_id = frame_id;
  output->height = 1;
  output->width = static_cast<uint32_t>(kept_points.size());
  output->is_bigendian = input.is_bigendian;
  output->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*output);
  modifier.setPointCloud2Fields(3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(kept_points.size());

  sensor_msgs::PointCloud2Iterator<float> out_x(*output, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(*output, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(*output, "z");

  for (size_t i = 0; i < kept_points.size(); ++i, ++out_x, ++out_y, ++out_z)
  {
    const auto & point = kept_points[i];
    *out_x = point[0];
    *out_y = point[1];
    *out_z = point[2];
  }

  return output;
}

}  // namespace pointcloud2_cutter
