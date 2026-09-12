#pragma once

#include "agt_terrain_map_generator/terrain_types.hpp"

namespace agt_terrain_map_generator
{

class ObstacleBuilder
{
public:
  virtual ~ObstacleBuilder() = default;

  virtual bool build(
    const PointCloud & non_ground,
    const GridGeometry & geometry,
    const std::vector<ElevationCell> & elevation,
    std::vector<std::uint8_t> & obstacle) = 0;
};

}  // namespace agt_terrain_map_generator
