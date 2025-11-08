#include "pointcloud2_cutter/region_marker_helper.hpp"

#include <gtest/gtest.h>

#include <utility>

namespace pointcloud2_cutter
{

Region make_region(double xmin, double xmax, double ymin, double ymax,
                   double zmin, double zmax, std::string name = "")
{
  Region region;
  region.x_min = xmin;
  region.x_max = xmax;
  region.y_min = ymin;
  region.y_max = ymax;
  region.laser_z_min = zmin;
  region.laser_z_max = zmax;
  region.priority = 0;
  region.name = std::move(name);
  return region;
}

TEST(RegionMarkerHelperTest, CreatesCubeAndLabelPerRegion)
{
  std::vector<Region> regions{
    make_region(0.0, 10.0, -5.0, 5.0, 1.0, 2.0, "MainHall"),
    make_region(5.0, 15.0, -2.0, 8.0, 0.5, 1.5, "Lobby")
  };

  RegionMarkerConfig config;
  config.frame_id = "map";
  config.alpha = 0.3;

  auto markers = create_region_markers(regions, config);

  ASSERT_EQ(markers.markers.size(), regions.size() * 3);

  const auto & first_cube = markers.markers[0];
  EXPECT_EQ(first_cube.type, visualization_msgs::msg::Marker::CUBE);
  EXPECT_NEAR(first_cube.scale.x, 10.0, 1e-6);
  EXPECT_NEAR(first_cube.scale.y, 10.0, 1e-6);
  EXPECT_DOUBLE_EQ(first_cube.scale.z, regions.front().laser_z_max - regions.front().laser_z_min);
  EXPECT_DOUBLE_EQ(first_cube.pose.position.z,
    (regions.front().laser_z_min + regions.front().laser_z_max) * 0.5);
  EXPECT_FLOAT_EQ(first_cube.color.a, 0.3f);
  EXPECT_EQ(first_cube.header.frame_id, "map");

  const auto & first_label = markers.markers[1];
  EXPECT_EQ(first_label.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  EXPECT_EQ(first_label.header.frame_id, "map");
  EXPECT_GT(first_label.scale.z, config.text_height);
  EXPECT_NE(first_label.text.find("MainHall"), std::string::npos);
  EXPECT_NE(first_label.text.find("x:["), std::string::npos);
  EXPECT_NE(first_label.text.find("y:["), std::string::npos);
  EXPECT_NE(first_label.text.find("z:["), std::string::npos);

  const auto & height_marker = markers.markers[2];
  EXPECT_EQ(height_marker.type, visualization_msgs::msg::Marker::LINE_LIST);
  ASSERT_EQ(height_marker.points.size(), 8u);
  EXPECT_DOUBLE_EQ(height_marker.points.front().z, regions.front().laser_z_min);
  EXPECT_DOUBLE_EQ(height_marker.points[1].z, regions.front().laser_z_max);
}

}  // namespace pointcloud2_cutter
