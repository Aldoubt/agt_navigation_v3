#pragma once

#include "agt_terrain_map_generator/terrain_types.hpp"

namespace agt_terrain_map_generator
{

class SlopeBuilder
{
public:
  virtual ~SlopeBuilder() = default;

  virtual bool build(
    const GridGeometry & geometry,
    const std::vector<ElevationCell> & elevation,
    std::vector<float> & slope_deg) = 0;
};

}  // namespace agt_terrain_map_generator
