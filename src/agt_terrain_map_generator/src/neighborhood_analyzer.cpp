#include "agt_terrain_map_generator/map_cleaning/static_confidence/neighborhood_analyzer.hpp"

namespace agt_terrain_map_generator
{

float NeighborhoodAnalyzer::computeSupport(const VoxelIndex & index, const VoxelMap & map) const
{
  int occupied = 0;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (map.find({index.x + dx, index.y + dy, index.z + dz}) != map.end()) {
          ++occupied;
        }
      }
    }
  }
  return static_cast<float>(occupied) / 27.0F;
}

}  // namespace agt_terrain_map_generator
