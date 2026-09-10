#pragma once

#include <cstddef>
#include <vector>

#include "agt_terrain_map_generator/cleaning/collision_model.hpp"
#include "agt_terrain_map_generator/mapping_assets.hpp"
#include "agt_terrain_map_generator/terrain_types.hpp"

namespace agt_terrain_map_generator
{

using Cloud = PointCloud;

struct CleaningStatistics
{
  std::size_t input_points{0U};
  std::size_t robot_removed{0U};
  std::size_t output_points{0U};
};

class RobotMeshFilter
{
public:
  // Input is in model.lidar_frame and is transformed to base_link through the
  // fixed URDF relation before collision testing.
  bool filter(
    const Cloud & input,
    Cloud & output,
    const CollisionModel & model,
    CleaningStatistics & statistics) const;

  // Input is a completed map-frame cloud.  Every optimized map_from_base pose
  // contributes one historical robot collision volume, which is necessary to
  // remove robot remnants accumulated into a global map.
  bool filter_map(
    const Cloud & input,
    Cloud & output,
    const CollisionModel & model,
    const std::vector<RigidTransform3d> & map_from_base,
    CleaningStatistics & statistics) const;
};

}  // namespace agt_terrain_map_generator
