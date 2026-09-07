#include "agt_terrain_map_generator/map_cleaning/voxel_persistence_filter.hpp"

#include <cmath>
#include <limits>

namespace agt_terrain_map_generator
{
namespace
{

VoxelIndex key_for(const Eigen::Vector3f & point, const double resolution)
{
  return {
    static_cast<int>(std::floor(point.x() / resolution)),
    static_cast<int>(std::floor(point.y() / resolution)),
    static_cast<int>(std::floor(point.z() / resolution))};
}

Eigen::Vector3f center_for(const VoxelIndex & key, const double resolution)
{
  const float half = static_cast<float>(resolution * 0.5);
  return Eigen::Vector3f{
    static_cast<float>(key.x * resolution) + half,
    static_cast<float>(key.y * resolution) + half,
    static_cast<float>(key.z * resolution) + half};
}

}  // namespace

VoxelPersistenceFilter::VoxelPersistenceFilter(VoxelPersistenceOptions options)
: options_(options)
{
}

bool VoxelPersistenceFilter::analyze(
  const std::vector<ProvenancePoint> & input,
  VoxelPersistenceResult & output) const
{
  output = {};
  if (!(options_.resolution_m > 0.0) || options_.min_observation < 1) {
    return false;
  }

  auto & observations = output.voxel_map;
  observations.reserve(input.size() / 2U + 1U);
  for (const auto & point : input) {
    if (!point.point.allFinite()) {
      continue;
    }
    ++output.statistics.input_points;
    auto & observation = observations[key_for(point.point, options_.resolution_m)];
    if (observation.patch_ids.insert(point.patch_id).second) {
      ++observation.patch_count;
    }
    if (observation.pose_ids.insert(point.pose_id).second) {
      ++observation.observation_count;
    }
    if (observation.observation_count == 1) {
      observation.first_seen = point.timestamp;
      observation.last_seen = point.timestamp;
    } else {
      observation.first_seen = std::min(observation.first_seen, point.timestamp);
      observation.last_seen = std::max(observation.last_seen, point.timestamp);
    }
  }

  output.statistics.total_voxels = observations.size();
  double observation_sum = 0.0;
  for (const auto & entry : observations) {
    const bool stable = entry.second.observation_count >= options_.min_observation ||
      entry.second.patch_count >= options_.min_observation;
    PersistenceVoxel voxel{entry.first, center_for(entry.first, options_.resolution_m), entry.second};
    if (stable) {
      output.stable_voxels.push_back(std::move(voxel));
      ++output.statistics.stable_voxels;
    } else {
      output.unstable_voxels.push_back(std::move(voxel));
      ++output.statistics.unstable_voxels;
    }
    observation_sum += entry.second.observation_count;
    output.statistics.max_observation = std::max(
      output.statistics.max_observation, entry.second.observation_count);
  }
  if (!observations.empty()) {
    output.statistics.average_observation = observation_sum / observations.size();
  }
  return true;
}

}  // namespace agt_terrain_map_generator
