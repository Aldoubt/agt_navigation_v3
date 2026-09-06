#include "agt_terrain_map_generator/height_obstacle_builder.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace agt_terrain_map_generator
{

HeightObstacleBuilder::HeightObstacleBuilder(const HeightObstacleOptions & options)
: options_(options)
{
  if (!std::isfinite(options_.min_height_m) || !std::isfinite(options_.max_height_m) ||
    options_.min_height_m < 0.0 || options_.max_height_m <= options_.min_height_m)
  {
    throw std::invalid_argument("invalid obstacle height limits");
  }
  if (!std::isfinite(options_.min_ground_confidence) ||
    options_.min_ground_confidence < 0.0 || options_.min_ground_confidence > 1.0)
  {
    throw std::invalid_argument("min_ground_confidence must be in [0, 1]");
  }
}

bool HeightObstacleBuilder::build(
  const PointCloud & non_ground,
  const GridGeometry & geometry,
  const std::vector<ElevationCell> & elevation,
  std::vector<std::uint8_t> & obstacle)
{
  const std::size_t expected =
    static_cast<std::size_t>(geometry.width) * static_cast<std::size_t>(geometry.height);
  if (geometry.width == 0U || geometry.height == 0U || geometry.resolution <= 0.0 ||
    elevation.size() != expected)
  {
    return false;
  }

  obstacle.assign(expected, 0U);
  for (const auto & point : non_ground) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    const auto ix = static_cast<long>(
      std::floor((static_cast<double>(point.x) - geometry.origin_x) / geometry.resolution));
    const auto iy = static_cast<long>(
      std::floor((static_cast<double>(point.y) - geometry.origin_y) / geometry.resolution));
    if (ix < 0 || iy < 0 || ix >= static_cast<long>(geometry.width) ||
      iy >= static_cast<long>(geometry.height))
    {
      continue;
    }

    const std::size_t index =
      static_cast<std::size_t>(iy) * geometry.width + static_cast<std::size_t>(ix);
    const auto & ground = elevation[index];
    if (!std::isfinite(ground.median_height) ||
      static_cast<double>(ground.confidence) < options_.min_ground_confidence)
    {
      continue;
    }

    const double height_above_ground =
      static_cast<double>(point.z) - static_cast<double>(ground.median_height);
    if (height_above_ground >= options_.min_height_m &&
      height_above_ground <= options_.max_height_m)
    {
      obstacle[index] = 1U;
    }
  }
  return true;
}

}  // namespace agt_terrain_map_generator
