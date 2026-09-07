#pragma once

#include "agt_terrain_map_generator/grid/traversability_grid.hpp"

namespace agt_terrain_map_generator
{

struct TraversabilityOptions
{
  double slope_weight{0.35};
  double obstacle_weight{0.35};
  double roughness_weight{0.15};
  double confidence_weight{0.15};
  double trajectory_bonus{0.10};
  double slope_safe_degree{10.0};
  double slope_warning_degree{20.0};
  double slope_blocked_degree{30.0};
  double obstacle_ignore_height_m{0.15};
  double obstacle_blocked_height_m{0.50};
  double roughness_variance_m2{0.04};
  double min_confidence{0.50};
};

class TraversabilityBuilder final
{
public:
  explicit TraversabilityBuilder(const TraversabilityOptions & options);

  bool build(
    const GridGeometry & geometry,
    const ElevationGrid & elevation,
    const SlopeGrid & slope_deg,
    const ObstacleGrid & obstacle_height_m,
    const ConfidenceGrid & confidence,
    const FreeCorridorGrid & free_corridor,
    TraversabilityGrid & traversability) const;

private:
  TraversabilityOptions options_;
};

}  // namespace agt_terrain_map_generator
