#pragma once

#include <cstdint>
#include <vector>

#include "agt_terrain_map_generator/terrain_types.hpp"

namespace agt_terrain_map_generator
{

enum class TraversabilityState : std::uint8_t
{
  UNKNOWN = 0U,
  FREE = 1U,
  BLOCKED = 2U,
};

struct TraversabilityCell
{
  // 0.0 is the best traversable surface; 1.0 is blocked.
  float cost{1.0F};
  float confidence{0.0F};
  TraversabilityState state{TraversabilityState::UNKNOWN};
};

struct TraversabilityGrid
{
  GridGeometry geometry;
  std::vector<TraversabilityCell> cells;
};

// Confidence is rasterized separately from the elevation cells so the
// traversability API can explicitly state all of its inputs.
using ElevationGrid = std::vector<ElevationCell>;
using SlopeGrid = std::vector<float>;
using ObstacleGrid = std::vector<float>;  // maximum height above local ground, metres
using ConfidenceGrid = std::vector<float>;

struct FreeCorridorGrid
{
  GridGeometry geometry;
  // [0, 1]: trajectory-derived evidence that a cell has been traversed.
  std::vector<float> confidence;
};

inline bool same_grid_geometry(const GridGeometry & lhs, const GridGeometry & rhs)
{
  return lhs.resolution == rhs.resolution && lhs.origin_x == rhs.origin_x &&
         lhs.origin_y == rhs.origin_y && lhs.width == rhs.width &&
         lhs.height == rhs.height && lhs.frame_id == rhs.frame_id;
}

}  // namespace agt_terrain_map_generator
