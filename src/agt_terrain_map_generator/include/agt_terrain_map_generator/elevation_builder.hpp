#pragma once

#include "agt_terrain_map_generator/terrain_types.hpp"

namespace agt_terrain_map_generator
{

class ElevationBuilder
{
public:
  virtual ~ElevationBuilder() = default;

  virtual bool build(
    const PointCloud & ground,
    const GridGeometry & geometry,
    std::vector<ElevationCell> & elevation) = 0;
};

}  // namespace agt_terrain_map_generator
