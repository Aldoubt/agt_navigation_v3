#pragma once

#include <cstdint>

#include "agt_terrain_map_generator/elevation_builder.hpp"

namespace agt_terrain_map_generator
{

struct MedianElevationOptions
{
  std::uint32_t min_points_per_cell{3U};
  double max_variance_m2{0.04};
};

class MedianElevationBuilder final : public ElevationBuilder
{
public:
  explicit MedianElevationBuilder(const MedianElevationOptions & options);

  bool build(
    const PointCloud & ground,
    const GridGeometry & geometry,
    std::vector<ElevationCell> & elevation) override;

private:
  MedianElevationOptions options_;
};

}  // namespace agt_terrain_map_generator
