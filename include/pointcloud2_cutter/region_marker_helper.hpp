#pragma once

#include "pointcloud2_cutter/region_selector.hpp"

#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <vector>

namespace pointcloud2_cutter
{

struct RegionMarkerConfig
{
  std::string frame_id{"map"};
  double alpha{0.3};
  double min_height{0.1};
  double text_height{0.5};
};

visualization_msgs::msg::MarkerArray create_region_markers(
  const std::vector<Region> & regions,
  const RegionMarkerConfig & config);

}  // namespace pointcloud2_cutter
