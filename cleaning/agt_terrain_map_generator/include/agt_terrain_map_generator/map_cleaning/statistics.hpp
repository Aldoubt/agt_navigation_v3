#pragma once

#include <cstddef>

namespace agt_terrain_map_generator
{

struct VoxelPersistenceStatistics
{
  std::size_t input_points{0U};
  std::size_t total_voxels{0U};
  std::size_t stable_voxels{0U};
  std::size_t unstable_voxels{0U};
  double average_observation{0.0};
  int max_observation{0};
};

}  // namespace agt_terrain_map_generator
