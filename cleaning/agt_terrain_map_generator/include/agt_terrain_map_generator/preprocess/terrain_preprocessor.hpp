#pragma once

#include <cstddef>

#include "agt_terrain_map_generator/geometry/robot_geometry_provider.hpp"
#include "agt_terrain_map_generator/mapping_assets.hpp"
#include "agt_terrain_map_generator/preprocess/rear_filter.hpp"
#include "agt_terrain_map_generator/preprocess/self_filter.hpp"

namespace agt_terrain_map_generator
{

struct VoxelFilterOptions
{
  bool enabled{true};
  double leaf_size_m{0.20};
};

struct TerrainPreprocessorOptions
{
  SelfFilterOptions self_filter{};
  RearFilterOptions rear_filter{};
  VoxelFilterOptions voxel{};
};

struct FilterStatistics
{
  std::size_t total_input_points{0U};
  std::size_t invalid_removed_points{0U};
  std::size_t self_removed_points{0U};
  std::size_t rear_removed_points{0U};
  std::size_t voxel_removed_points{0U};
  std::size_t final_points{0U};
};

// Patch-local preprocessing for the offline terrain path only.  `body_from_local`
// allows robot-relative filters to be applied in base/body coordinates even
// though the cloud itself remains gravity-levelled for Patchwork++.
class TerrainPreprocessor final
{
public:
  TerrainPreprocessor(
    const TerrainPreprocessorOptions & options,
    const RobotGeometryProvider & geometry);

  bool process(
    const PointCloud & gravity_level_cloud,
    const RigidTransform3d & body_from_local,
    PointCloud & output,
    FilterStatistics & statistics) const;

private:
  TerrainPreprocessorOptions options_;
  SelfFilter self_filter_;
  RearFilter rear_filter_;
};

}  // namespace agt_terrain_map_generator
