#pragma once

#include "agt_terrain_map_generator/geometry/robot_geometry_provider.hpp"

namespace agt_terrain_map_generator
{

struct SelfFilterOptions
{
  bool enabled{true};
};

class SelfFilter final
{
public:
  SelfFilter(const SelfFilterOptions & options, const RobotGeometryProvider & geometry)
  : options_(options), geometry_(geometry) {}

  bool removes(const Point3f & body_point) const
  {
    return options_.enabled && geometry_.body_box().contains(body_point);
  }

private:
  SelfFilterOptions options_;
  const RobotGeometryProvider & geometry_;
};

}  // namespace agt_terrain_map_generator
