#pragma once

#include <unordered_map>

#include "agt_terrain_map_generator/map_cleaning/voxel_observation.hpp"

namespace agt_terrain_map_generator
{

struct MultiScaleVoxel
{
  VoxelIndex fine_index;
  VoxelIndex structure_index;
  VoxelIndex scene_index;
};

struct MultiScaleVoxelOptions
{
  double fine_resolution_m{0.10};
  double structure_resolution_m{0.25};
  double scene_resolution_m{0.50};
};

struct MultiScaleVoxelMaps
{
  // These are index/occupancy maps only. No raw point cloud is copied.
  VoxelMap fine;
  VoxelMap structure;
  VoxelMap scene;
  std::unordered_map<VoxelIndex, MultiScaleVoxel, VoxelIndexHash> relations;
};

class MultiScaleVoxelBuilder
{
public:
  explicit MultiScaleVoxelBuilder(MultiScaleVoxelOptions options = {});

  bool build(const VoxelMap & fine_voxels, MultiScaleVoxelMaps & output) const;

private:
  MultiScaleVoxelOptions options_;
};

}  // namespace agt_terrain_map_generator
