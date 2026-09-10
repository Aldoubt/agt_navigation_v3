#pragma once

#include <string>

#include "agt_terrain_map_generator/grid/traversability_grid.hpp"
#include "agt_terrain_map_generator/terrain_types.hpp"

namespace agt_terrain_map_generator
{

class TerrainPackageExporter final
{
public:
  bool export_package(
    const ElevationGrid & elevation,
    const SlopeGrid & slope_deg,
    const ObstacleGrid & obstacle_height_m,
    const ConfidenceGrid & confidence,
    const FreeCorridorGrid & free_corridor,
    const TraversabilityGrid & traversability,
    const GenerationContext & context,
    const std::string & parameter_summary,
    std::string & error) const;
};

}  // namespace agt_terrain_map_generator
