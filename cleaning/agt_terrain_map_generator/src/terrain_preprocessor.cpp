#include "agt_terrain_map_generator/preprocess/terrain_preprocessor.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_set>

namespace agt_terrain_map_generator
{
namespace
{

struct VoxelKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    const auto hx = std::hash<std::int64_t>{}(key.x);
    const auto hy = std::hash<std::int64_t>{}(key.y);
    const auto hz = std::hash<std::int64_t>{}(key.z);
    return hx ^ (hy << 1U) ^ (hz << 2U);
  }
};

bool finite(const Point3f & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

}  // namespace

TerrainPreprocessor::TerrainPreprocessor(
  const TerrainPreprocessorOptions & options,
  const RobotGeometryProvider & geometry)
: options_(options), self_filter_(options.self_filter, geometry), rear_filter_(options.rear_filter)
{
  if (!geometry.body_box().valid()) {
    throw std::invalid_argument("robot geometry body box is invalid");
  }
  if (!options_.rear_filter.box.valid()) {
    throw std::invalid_argument("rear filter box is invalid");
  }
  if (options_.voxel.enabled && options_.voxel.leaf_size_m <= 0.0) {
    throw std::invalid_argument("voxel.leaf_size_m must be > 0 when voxel filter is enabled");
  }
}

bool TerrainPreprocessor::process(
  const PointCloud & gravity_level_cloud,
  const RigidTransform3d & body_from_local,
  PointCloud & output,
  FilterStatistics & statistics) const
{
  output.clear();
  statistics = FilterStatistics{};
  statistics.total_input_points = gravity_level_cloud.size();
  if (!body_from_local.has_valid_rotation()) {
    return false;
  }

  output.reserve(gravity_level_cloud.size());
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
  if (options_.voxel.enabled) {
    occupied.reserve(gravity_level_cloud.size());
  }

  for (const auto & local_point : gravity_level_cloud) {
    if (!finite(local_point)) {
      ++statistics.invalid_removed_points;
      continue;
    }

    const Point3f body_point = body_from_local.transform(local_point);
    if (self_filter_.removes(body_point)) {
      ++statistics.self_removed_points;
      continue;
    }
    if (rear_filter_.removes(body_point)) {
      ++statistics.rear_removed_points;
      continue;
    }

    if (options_.voxel.enabled) {
      const double leaf = options_.voxel.leaf_size_m;
      const VoxelKey key{
        static_cast<std::int64_t>(std::floor(static_cast<double>(local_point.x) / leaf)),
        static_cast<std::int64_t>(std::floor(static_cast<double>(local_point.y) / leaf)),
        static_cast<std::int64_t>(std::floor(static_cast<double>(local_point.z) / leaf))};
      if (!occupied.insert(key).second) {
        ++statistics.voxel_removed_points;
        continue;
      }
    }
    output.push_back(local_point);
  }

  statistics.final_points = output.size();
  return true;
}

}  // namespace agt_terrain_map_generator
