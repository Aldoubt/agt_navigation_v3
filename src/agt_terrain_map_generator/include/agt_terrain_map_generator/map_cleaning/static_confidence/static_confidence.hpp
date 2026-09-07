#pragma once

#include <vector>

#include "agt_terrain_map_generator/map_cleaning/static_confidence/confidence_types.hpp"
#include "agt_terrain_map_generator/map_cleaning/static_confidence/multi_scale_voxel.hpp"

namespace agt_terrain_map_generator
{

class StaticConfidenceEvaluator
{
public:
  explicit StaticConfidenceEvaluator(StaticConfidenceWeights weights = {});

  // v0.3.2-b single-scale regression interface.
  bool evaluate(const VoxelMap & voxel_map, std::vector<VoxelConfidence> & output) const;

  bool evaluateMultiScale(
    const VoxelMap & fine_voxels,
    const TerrainPriorMap & terrain_prior,
    const MultiScaleVoxelOptions & scales,
    std::vector<VoxelConfidence> & output) const;

  float computeObservationScore(const VoxelObservation & voxel) const;

private:
  StaticConfidenceWeights weights_;
};

}  // namespace agt_terrain_map_generator
