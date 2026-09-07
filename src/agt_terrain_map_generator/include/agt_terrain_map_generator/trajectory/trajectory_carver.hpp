#pragma once

#include <vector>

#include "agt_terrain_map_generator/grid/traversability_grid.hpp"
#include "agt_terrain_map_generator/trajectory/pose_loader.hpp"

namespace agt_terrain_map_generator
{

struct TrajectoryCarverOptions
{
  bool enabled{true};
  double robot_radius_m{0.45};
  double inflation_m{0.20};
};

class TrajectoryFreeCarver final
{
public:
  explicit TrajectoryFreeCarver(const TrajectoryCarverOptions & options);

  bool carve(
    const GridGeometry & geometry,
    const std::vector<TrajectoryPose> & poses,
    FreeCorridorGrid & free_corridor) const;

private:
  TrajectoryCarverOptions options_;
};

}  // namespace agt_terrain_map_generator
