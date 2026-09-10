#pragma once

#include "agt_terrain_map_generator/map_cleaning/voxel_observation.hpp"

namespace agt_terrain_map_generator
{

struct StaticConfidenceCell
{
  float score{0.0F};
  float temporal_score{0.0F};
  float fine_scale_score{0.0F};
  float structure_scale_score{0.0F};
  float scene_scale_score{0.0F};
  float ground_score{0.5F};
  // v0.3.2-b compatibility fields for the original 0.10 m calculation.
  float observation_score{0.0F};
  float neighborhood_score{0.0F};
  // Reserved for later validated estimators. They intentionally remain 0.5.
  float surface_score{0.5F};
  float viewpoint_score{0.5F};
};

struct TerrainPrior
{
  bool is_ground{false};
  float ground_confidence{0.0F};
};

using TerrainPriorMap = std::unordered_map<VoxelIndex, TerrainPrior, VoxelIndexHash>;

struct StaticConfidenceWeights
{
  float temporal{0.30F};
  float fine{0.25F};
  float structure{0.25F};
  float scene{0.10F};
  float ground{0.10F};
};

struct VoxelConfidence
{
  VoxelIndex index;
  StaticConfidenceCell confidence;
  int observation_count{0};
  int patch_count{0};
};

}  // namespace agt_terrain_map_generator
