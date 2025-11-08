#include "pointcloud2_cutter/region_marker_helper.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/point.hpp>

namespace pointcloud2_cutter
{
namespace
{
constexpr std::array<std::array<float, 3>, 8> kColorPalette{{
  {0.894f, 0.102f, 0.110f},
  {0.216f, 0.494f, 0.722f},
  {0.302f, 0.686f, 0.290f},
  {0.596f, 0.306f, 0.639f},
  {1.000f, 0.498f, 0.000f},
  {1.000f, 1.000f, 0.200f},
  {0.651f, 0.337f, 0.157f},
  {0.090f, 0.745f, 0.811f}
}};

double clamp_positive(double value, double min_value)
{
  return value > min_value ? value : min_value;
}

visualization_msgs::msg::Marker create_volume_marker(
  const Region & region, const RegionMarkerConfig & config,
  const std::array<float, 3> & color, int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config.frame_id;
  marker.ns = "regions";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  const double center_x = (region.x_min + region.x_max) * 0.5;
  const double center_y = (region.y_min + region.y_max) * 0.5;
  const double span_x = clamp_positive(region.x_max - region.x_min, config.min_height);
  const double span_y = clamp_positive(region.y_max - region.y_min, config.min_height);
  const double span_z_raw = region.laser_z_max - region.laser_z_min;
  const double span_z = span_z_raw > 0.0 ? span_z_raw : 0.0;

  marker.pose.position.x = center_x;
  marker.pose.position.y = center_y;
  marker.pose.position.z = region.laser_z_min + span_z_raw * 0.5;

  marker.scale.x = span_x;
  marker.scale.y = span_y;
  marker.scale.z = span_z;

  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = static_cast<float>(config.alpha);

  return marker;
}

std::string build_label_text(const Region & region)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(1);
  ss << "x:[" << region.x_min << " ~ " << region.x_max << "]\n";
  ss << "y:[" << region.y_min << " ~ " << region.y_max << "]\n";
  ss << "laser z:[" << region.laser_z_min << " ~ " << region.laser_z_max << "]";
  return ss.str();
}

visualization_msgs::msg::Marker create_label_marker(
  const Region & region, const RegionMarkerConfig & config,
  int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config.frame_id;
  marker.ns = "region_labels";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  const double label_scale_factor = 10.6;
  const double label_offset_factor = 0.5;

  marker.pose.position.x = (region.x_min + region.x_max) * 0.5;
  marker.pose.position.y = (region.y_min + region.y_max) * 0.5;
  marker.pose.position.z =
    region.laser_z_max + config.text_height * label_scale_factor * label_offset_factor;
  marker.scale.z = config.text_height * label_scale_factor;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  if (!region.name.empty())
  {
    marker.text = region.name + "\n" + build_label_text(region);
  }
  else
  {
    marker.text = build_label_text(region);
  }
  return marker;
}

visualization_msgs::msg::Marker create_height_marker(
  const Region & region, const RegionMarkerConfig & config,
  const std::array<float, 3> & color, int id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config.frame_id;
  marker.ns = "region_height";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.05;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = static_cast<float>(config.alpha);

  const std::array<std::array<double, 2>, 4> corners{{
    {region.x_min, region.y_min},
    {region.x_max, region.y_min},
    {region.x_max, region.y_max},
    {region.x_min, region.y_max}
  }};

  marker.points.reserve(corners.size() * 2);
  for (const auto & corner : corners)
  {
    geometry_msgs::msg::Point bottom;
    bottom.x = corner[0];
    bottom.y = corner[1];
    bottom.z = region.laser_z_min;

    geometry_msgs::msg::Point top = bottom;
    top.z = region.laser_z_max;

    marker.points.emplace_back(bottom);
    marker.points.emplace_back(top);
  }

  return marker;
}
}  // namespace

visualization_msgs::msg::MarkerArray create_region_markers(
  const std::vector<Region> & regions,
  const RegionMarkerConfig & config)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.reserve(regions.size() * 3);

  int marker_id = 0;
  for (size_t idx = 0; idx < regions.size(); ++idx)
  {
    const auto & region = regions[idx];
    const auto & color = kColorPalette[idx % kColorPalette.size()];

    marker_array.markers.emplace_back(
      create_volume_marker(region, config, color, marker_id++));
    marker_array.markers.emplace_back(
      create_label_marker(region, config, marker_id++));
    marker_array.markers.emplace_back(
      create_height_marker(region, config, color, marker_id++));
  }

  return marker_array;
}

}  // namespace pointcloud2_cutter
