#include <gtest/gtest.h>

#include "pointcloud2_cutter/region_selector.hpp"

#include <string>

namespace
{

std::string testResourcePath()
{
  return std::string(TEST_RESOURCES_DIR) + "/test_regions.yaml";
}

}  // namespace

TEST(RegionSelectorTest, SelectsHighestPriorityRegion)
{
  pointcloud2_cutter::RegionSelector selector(1.0, 4.0);
  selector.loadFromFile(testResourcePath());

  auto [z_min, z_max] = selector.determineLaserZ(0.0, 0.0);
  EXPECT_DOUBLE_EQ(z_min, 2.0);
  EXPECT_DOUBLE_EQ(z_max, 3.0);
}

TEST(RegionSelectorTest, FallsBackToDefaultOutsideRegions)
{
  pointcloud2_cutter::RegionSelector selector(1.0, 4.0);
  selector.loadFromFile(testResourcePath());

  auto [z_min, z_max] = selector.determineLaserZ(10.0, 10.0);
  EXPECT_DOUBLE_EQ(z_min, 1.0);
  EXPECT_DOUBLE_EQ(z_max, 4.0);
}

