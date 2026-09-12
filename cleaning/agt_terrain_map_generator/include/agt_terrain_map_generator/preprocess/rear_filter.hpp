#pragma once

#include <array>

#include "agt_terrain_map_generator/terrain_types.hpp"

namespace agt_terrain_map_generator
{

struct RearFilterBox
{
  double x_min{-2.0};
  double x_max{0.0};
  double y_min{-0.8};
  double y_max{0.8};
  double z_min{-0.5};
  double z_max{2.0};

  bool valid() const
  {
    return x_min < x_max && y_min < y_max && z_min < z_max;
  }

  bool contains(const Point3f & body_point) const
  {
    return body_point.x >= x_min && body_point.x <= x_max &&
           body_point.y >= y_min && body_point.y <= y_max &&
           body_point.z >= z_min && body_point.z <= z_max;
  }
};

struct RearFilterOptions
{
  bool enabled{false};
  RearFilterBox box{};
};

class RearFilter final
{
public:
  explicit RearFilter(const RearFilterOptions & options)
  : options_(options) {}

  bool removes(const Point3f & body_point) const
  {
    return options_.enabled && options_.box.contains(body_point);
  }

private:
  RearFilterOptions options_;
};

}  // namespace agt_terrain_map_generator
