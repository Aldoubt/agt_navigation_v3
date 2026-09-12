#pragma once

#include "agt_terrain_map_generator/terrain_types.hpp"

namespace agt_terrain_map_generator
{

class MapExporter
{
public:
  virtual ~MapExporter() = default;

  virtual bool export_maps(
    const TerrainGrid & grid,
    const GenerationContext & context) = 0;
};

}  // namespace agt_terrain_map_generator
