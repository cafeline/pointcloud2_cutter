#include <gtest/gtest.h>

#include "pointcloud2_cutter/pointcloud_filter.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{

sensor_msgs::msg::PointCloud2 makePointCloud()
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "input_frame";
  cloud.height = 1;
  cloud.width = 3;
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(3);

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

  // Point 0: inside range
  *iter_x = 0.0f;
  *iter_y = 0.0f;
  *iter_z = 1.0f;

  ++iter_x;
  ++iter_y;
  ++iter_z;

  // Point 1: below range
  *iter_x = 0.0f;
  *iter_y = 0.0f;
  *iter_z = 0.1f;

  ++iter_x;
  ++iter_y;
  ++iter_z;

  // Point 2: above range
  *iter_x = 0.0f;
  *iter_y = 0.0f;
  *iter_z = 5.0f;

  return cloud;
}

}  // namespace

TEST(PointCloudFilterTest, FiltersByHeight)
{
  auto cloud = makePointCloud();
  auto filtered = pointcloud2_cutter::PointCloudFilter::filterByHeight(cloud, 0.5, 2.0, "output_frame");

  ASSERT_NE(filtered, nullptr);
  EXPECT_EQ(filtered->header.frame_id, "output_frame");
  EXPECT_EQ(filtered->width, 1u);

  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*filtered, "z");
  EXPECT_FLOAT_EQ(iter_z[0], 1.0f);
}
