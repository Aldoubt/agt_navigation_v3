#pragma once

#include "agt_terrain_map_generator/obstacle_builder.hpp"

namespace agt_terrain_map_generator
{

struct HeightObstacleOptions
{
  double min_height_m{0.25};
  double max_height_m{3.0};
  double min_ground_confidence{0.5};
};

class HeightObstacleBuilder final : public ObstacleBuilder
{
public:
  explicit HeightObstacleBuilder(const HeightObstacleOptions & options);

  bool build(
    const PointCloud & non_ground,
    const GridGeometry & geometry,
    const std::vector<ElevationCell> & elevation,
    std::vector<std::uint8_t> & obstacle) override;

private:
  HeightObstacleOptions options_;
};

}  // namespace agt_terrain_map_generator
