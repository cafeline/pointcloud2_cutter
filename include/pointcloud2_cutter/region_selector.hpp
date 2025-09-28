#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace pointcloud2_cutter
{

struct Region
{
  double x_min{};
  double x_max{};
  double y_min{};
  double y_max{};
  double laser_z_min{};
  double laser_z_max{};
  int priority{};

  bool contains(double x, double y) const noexcept
  {
    return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
  }
};

class RegionSelector
{
public:
  RegionSelector(double default_laser_z_min, double default_laser_z_max);

  void loadFromFile(const std::string & yaml_path);

  std::pair<double, double> determineLaserZ(double x, double y) const;

  std::pair<double, double> defaultLaserZ() const noexcept
  {
    return {default_laser_z_min_, default_laser_z_max_};
  }

  const std::vector<Region> & regions() const noexcept { return regions_; }

private:
  double default_laser_z_min_{};
  double default_laser_z_max_{};
  std::vector<Region> regions_;
};

}  // namespace pointcloud2_cutter
