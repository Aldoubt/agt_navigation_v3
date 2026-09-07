#pragma once

#include "agt_terrain_map_generator/map_cleaning/voxel_observation.hpp"

namespace agt_terrain_map_generator
{

class NeighborhoodAnalyzer
{
public:
  float computeSupport(const VoxelIndex & index, const VoxelMap & map) const;
};

}  // namespace agt_terrain_map_generator
