#include "pointcloud2_cutter/region_selector.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>

namespace pointcloud2_cutter
{

namespace
{
constexpr const char * kRootKey = "/**";
constexpr const char * kParamsKey = "ros__parameters";
constexpr const char * kRegionsKey = "regions";

Region parseRegion(const YAML::Node & node, double default_min, double default_max, size_t index)
{
  Region region;
  region.x_min = node["x_min"].as<double>();
  region.x_max = node["x_max"].as<double>();
  region.y_min = node["y_min"].as<double>();
  region.y_max = node["y_max"].as<double>();
  const int fallback_priority = -static_cast<int>(index);
  region.priority = node["priority"] ? node["priority"].as<int>() : fallback_priority;

  if (node["laser_height_min"] && node["laser_height_max"])
  {
    region.laser_z_min = node["laser_height_min"].as<double>();
    region.laser_z_max = node["laser_height_max"].as<double>();
  }
  else if (node["localization_laser_height_min"] && node["localization_laser_height_max"])
  {
    region.laser_z_min = node["localization_laser_height_min"].as<double>();
    region.laser_z_max = node["localization_laser_height_max"].as<double>();
  }
  else
  {
    region.laser_z_min = default_min;
    region.laser_z_max = default_max;
  }

  return region;
}
}  // namespace

RegionSelector::RegionSelector(double default_laser_z_min, double default_laser_z_max)
: default_laser_z_min_(default_laser_z_min), default_laser_z_max_(default_laser_z_max)
{
}

void RegionSelector::loadFromFile(const std::string & yaml_path)
{
  YAML::Node config = YAML::LoadFile(yaml_path);

  const auto root = config[kRootKey];
  if (!root)
  {
    throw std::runtime_error("YAML root '/**' が見つかりません");
  }

  const auto params = root[kParamsKey];
  if (!params)
  {
    throw std::runtime_error("YAML 'ros__parameters' が見つかりません");
  }

  const auto regions_node = params[kRegionsKey];
  if (!regions_node)
  {
    regions_.clear();
    return;
  }

  regions_.clear();
  for (const auto & region_node : regions_node)
  {
    regions_.push_back(parseRegion(region_node, default_laser_z_min_, default_laser_z_max_, regions_.size()));
  }
}

std::pair<double, double> RegionSelector::determineLaserZ(double x, double y) const
{
  const Region * selected = nullptr;
  int highest_priority = std::numeric_limits<int>::min();

  for (const auto & region : regions_)
  {
    if (region.contains(x, y) && region.priority >= highest_priority)
    {
      if (!selected || region.priority > highest_priority)
      {
        selected = &region;
        highest_priority = region.priority;
      }
    }
  }

  if (selected)
  {
    return {selected->laser_z_min, selected->laser_z_max};
  }

  return defaultLaserZ();
}

}  // namespace pointcloud2_cutter
