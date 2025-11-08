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

std::shared_ptr<sensor_msgs::msg::PointCloud2> PointCloudFilter::limitPointCount(
  const std::shared_ptr<sensor_msgs::msg::PointCloud2> & input,
  std::size_t max_points)
{
  if (!input || max_points == 0)
  {
    return input;
  }

  const std::size_t total_points =
    static_cast<std::size_t>(input->width) * static_cast<std::size_t>(input->height ? input->height : 1);
  if (total_points <= max_points)
  {
    return input;
  }

  const std::size_t stride = (total_points + max_points - 1) / max_points;

  auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
  output->header = input->header;
  output->height = 1;
  output->width = static_cast<uint32_t>(max_points);
  output->is_bigendian = input->is_bigendian;
  output->is_dense = input->is_dense;

  sensor_msgs::PointCloud2Modifier modifier(*output);
  modifier.setPointCloud2Fields(3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(max_points);

  sensor_msgs::PointCloud2ConstIterator<float> in_x(*input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> in_y(*input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> in_z(*input, "z");
  sensor_msgs::PointCloud2Iterator<float> out_x(*output, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(*output, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(*output, "z");

  std::size_t copied = 0;
  for (std::size_t idx = 0; idx < total_points && copied < max_points; ++idx, ++in_x, ++in_y, ++in_z)
  {
    if (idx % stride != 0)
    {
      continue;
    }

    *out_x = *in_x;
    *out_y = *in_y;
    *out_z = *in_z;

    ++out_x;
    ++out_y;
    ++out_z;
    ++copied;
  }

  if (copied < max_points)
  {
    const auto bytes_per_point = output->point_step;
    output->width = static_cast<uint32_t>(copied);
    output->data.resize(bytes_per_point * copied);
    output->row_step = output->point_step * output->width;
  }
  else
  {
    output->row_step = output->point_step * output->width;
  }

  return output;
}

}  // namespace pointcloud2_cutter
