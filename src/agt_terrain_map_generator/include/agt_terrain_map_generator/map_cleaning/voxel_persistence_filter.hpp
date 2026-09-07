#pragma once

#include <vector>

#include "agt_terrain_map_generator/map_cleaning/point_provenance.hpp"
#include "agt_terrain_map_generator/map_cleaning/statistics.hpp"
#include "agt_terrain_map_generator/map_cleaning/voxel_observation.hpp"

namespace agt_terrain_map_generator
{

struct VoxelPersistenceOptions
{
  bool enabled{true};
  double resolution_m{0.10};
  int min_observation{3};
};

struct PersistenceVoxel
{
  VoxelIndex index;
  Eigen::Vector3f center{Eigen::Vector3f::Zero()};
  VoxelObservation observation;
};

struct VoxelPersistenceResult
{
  VoxelPersistenceStatistics statistics;
  VoxelMap voxel_map;
  std::vector<PersistenceVoxel> stable_voxels;
  std::vector<PersistenceVoxel> unstable_voxels;
};

// Analyses repeat observations only.  It deliberately does not discard raw
// map points; a later dynamic classifier may consume this evidence.
class VoxelPersistenceFilter
{
public:
  explicit VoxelPersistenceFilter(VoxelPersistenceOptions options = {});

  bool analyze(
    const std::vector<ProvenancePoint> & input,
    VoxelPersistenceResult & output) const;

private:
  VoxelPersistenceOptions options_;
};

}  // namespace agt_terrain_map_generator
