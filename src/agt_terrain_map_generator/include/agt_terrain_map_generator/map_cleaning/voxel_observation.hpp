#pragma once

#include <cstddef>
#include <unordered_set>
#include <unordered_map>

namespace agt_terrain_map_generator
{

struct VoxelIndex
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelIndexHash
{
  std::size_t operator()(const VoxelIndex & index) const
  {
    const auto h1 = std::hash<int>{}(index.x);
    const auto h2 = std::hash<int>{}(index.y);
    const auto h3 = std::hash<int>{}(index.z);
    return h1 ^ (h2 << 1U) ^ (h3 << 7U);
  }
};

struct VoxelObservation
{
  // Number of distinct pose observations, rather than raw point count.  This
  // prevents a dense single scan of a person from becoming "persistent".
  int observation_count{0};
  int patch_count{0};
  double first_seen{0.0};
  double last_seen{0.0};
  std::unordered_set<int> patch_ids;
  std::unordered_set<int> pose_ids;
};

using VoxelMap = std::unordered_map<VoxelIndex, VoxelObservation, VoxelIndexHash>;

}  // namespace agt_terrain_map_generator
