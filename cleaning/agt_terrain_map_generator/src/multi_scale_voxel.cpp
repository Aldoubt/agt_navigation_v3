#include "agt_terrain_map_generator/map_cleaning/static_confidence/multi_scale_voxel.hpp"

#include <algorithm>
#include <cmath>

namespace agt_terrain_map_generator
{
namespace
{

VoxelIndex remap(
  const VoxelIndex & index, const double source_resolution, const double target_resolution)
{
  return {
    static_cast<int>(std::floor(index.x * source_resolution / target_resolution)),
    static_cast<int>(std::floor(index.y * source_resolution / target_resolution)),
    static_cast<int>(std::floor(index.z * source_resolution / target_resolution))};
}

void merge_observation(VoxelObservation & target, const VoxelObservation & source)
{
  // Coarse cells provide spatial support only. Retaining the strongest source
  // observation avoids fabricating temporal evidence by summing neighbours.
  target.observation_count = std::max(target.observation_count, source.observation_count);
  target.patch_count = std::max(target.patch_count, source.patch_count);
}

}  // namespace

MultiScaleVoxelBuilder::MultiScaleVoxelBuilder(MultiScaleVoxelOptions options)
: options_(options)
{
}

bool MultiScaleVoxelBuilder::build(
  const VoxelMap & fine_voxels, MultiScaleVoxelMaps & output) const
{
  output = {};
  if (!(options_.fine_resolution_m > 0.0) || !(options_.structure_resolution_m > 0.0) ||
    !(options_.scene_resolution_m > 0.0))
  {
    return false;
  }
  output.fine = fine_voxels;
  output.structure.reserve(fine_voxels.size() / 4U + 1U);
  output.scene.reserve(fine_voxels.size() / 16U + 1U);
  output.relations.reserve(fine_voxels.size());
  for (const auto & entry : fine_voxels) {
    const MultiScaleVoxel relation{
      entry.first,
      remap(entry.first, options_.fine_resolution_m, options_.structure_resolution_m),
      remap(entry.first, options_.fine_resolution_m, options_.scene_resolution_m)};
    merge_observation(output.structure[relation.structure_index], entry.second);
    merge_observation(output.scene[relation.scene_index], entry.second);
    output.relations.emplace(entry.first, relation);
  }
  return true;
}

}  // namespace agt_terrain_map_generator
