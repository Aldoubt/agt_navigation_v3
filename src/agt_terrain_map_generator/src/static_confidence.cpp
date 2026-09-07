#include "agt_terrain_map_generator/map_cleaning/static_confidence/static_confidence.hpp"

#include "agt_terrain_map_generator/map_cleaning/static_confidence/neighborhood_analyzer.hpp"

namespace agt_terrain_map_generator
{

StaticConfidenceEvaluator::StaticConfidenceEvaluator(StaticConfidenceWeights weights)
: weights_(weights)
{
}

float StaticConfidenceEvaluator::computeObservationScore(const VoxelObservation & voxel) const
{
  switch (voxel.observation_count) {
    case 1:
      return 0.2F;
    case 2:
      return 0.5F;
    case 3:
      return 0.7F;
    case 4:
      return 0.85F;
    default:
      return voxel.observation_count >= 5 ? 1.0F : 0.0F;
  }
}

bool StaticConfidenceEvaluator::evaluate(
  const VoxelMap & voxel_map, std::vector<VoxelConfidence> & output) const
{
  output.clear();
  output.reserve(voxel_map.size());
  NeighborhoodAnalyzer neighborhood;
  for (const auto & entry : voxel_map) {
    StaticConfidenceCell confidence;
    confidence.observation_score = computeObservationScore(entry.second);
    confidence.neighborhood_score = neighborhood.computeSupport(entry.first, voxel_map);
    confidence.score = 0.5F * confidence.observation_score +
      0.5F * confidence.neighborhood_score;
    output.push_back({
      entry.first, confidence, entry.second.observation_count, entry.second.patch_count});
  }
  return true;
}

bool StaticConfidenceEvaluator::evaluateMultiScale(
  const VoxelMap & fine_voxels,
  const TerrainPriorMap & terrain_prior,
  const MultiScaleVoxelOptions & scales,
  std::vector<VoxelConfidence> & output) const
{
  output.clear();
  MultiScaleVoxelMaps maps;
  MultiScaleVoxelBuilder builder(scales);
  if (!builder.build(fine_voxels, maps)) {
    return false;
  }
  output.reserve(maps.fine.size());
  NeighborhoodAnalyzer neighborhood;
  for (const auto & entry : maps.fine) {
    const auto relation = maps.relations.find(entry.first);
    if (relation == maps.relations.end()) {
      return false;
    }
    StaticConfidenceCell confidence;
    confidence.temporal_score = computeObservationScore(entry.second);
    confidence.fine_scale_score = neighborhood.computeSupport(entry.first, maps.fine);
    confidence.structure_scale_score = neighborhood.computeSupport(
      relation->second.structure_index, maps.structure);
    confidence.scene_scale_score = neighborhood.computeSupport(
      relation->second.scene_index, maps.scene);
    const auto prior = terrain_prior.find(entry.first);
    confidence.ground_score = prior != terrain_prior.end() && prior->second.is_ground ?
      prior->second.ground_confidence : 0.5F;
    confidence.observation_score = confidence.temporal_score;
    confidence.neighborhood_score = confidence.fine_scale_score;
    confidence.score = weights_.temporal * confidence.temporal_score +
      weights_.fine * confidence.fine_scale_score +
      weights_.structure * confidence.structure_scale_score +
      weights_.scene * confidence.scene_scale_score +
      weights_.ground * confidence.ground_score;
    output.push_back({
      entry.first, confidence, entry.second.observation_count, entry.second.patch_count});
  }
  return true;
}

}  // namespace agt_terrain_map_generator
