#include "agt_terrain_map_generator/trajectory/trajectory_carver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace agt_terrain_map_generator
{

TrajectoryFreeCarver::TrajectoryFreeCarver(const TrajectoryCarverOptions & options)
: options_(options)
{
  if (!std::isfinite(options_.robot_radius_m) || !std::isfinite(options_.inflation_m) ||
    options_.robot_radius_m <= 0.0 || options_.inflation_m < 0.0)
  {
    throw std::invalid_argument("invalid trajectory carver radius or inflation");
  }
}

bool TrajectoryFreeCarver::carve(
  const GridGeometry & geometry,
  const std::vector<TrajectoryPose> & poses,
  FreeCorridorGrid & free_corridor) const
{
  if (geometry.width == 0U || geometry.height == 0U || geometry.resolution <= 0.0) {
    return false;
  }
  const std::size_t count = static_cast<std::size_t>(geometry.width) * geometry.height;
  free_corridor.geometry = geometry;
  free_corridor.confidence.assign(count, 0.0F);
  if (!options_.enabled || poses.empty()) {
    return true;
  }

  const double radius = options_.robot_radius_m + options_.inflation_m;
  const int cell_radius = static_cast<int>(std::ceil(radius / geometry.resolution));
  const auto mark_footprint = [&](const double px, const double py) {
      const int cx = static_cast<int>(std::floor((px - geometry.origin_x) / geometry.resolution));
      const int cy = static_cast<int>(std::floor((py - geometry.origin_y) / geometry.resolution));
      for (int dy = -cell_radius; dy <= cell_radius; ++dy) {
        for (int dx = -cell_radius; dx <= cell_radius; ++dx) {
          const int ix = cx + dx;
          const int iy = cy + dy;
          if (ix < 0 || iy < 0 || ix >= static_cast<int>(geometry.width) ||
            iy >= static_cast<int>(geometry.height))
          {
            continue;
          }
          const double cell_x = geometry.origin_x + (static_cast<double>(ix) + 0.5) * geometry.resolution;
          const double cell_y = geometry.origin_y + (static_cast<double>(iy) + 0.5) * geometry.resolution;
          if (std::hypot(cell_x - px, cell_y - py) <= radius) {
            free_corridor.confidence[static_cast<std::size_t>(iy) * geometry.width + ix] = 1.0F;
          }
        }
      }
    };

  mark_footprint(poses.front().x, poses.front().y);
  for (std::size_t i = 1U; i < poses.size(); ++i) {
    const auto & from = poses[i - 1U];
    const auto & to = poses[i];
    const double distance = std::hypot(to.x - from.x, to.y - from.y);
    const std::size_t steps = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(distance / (0.5 * geometry.resolution))));
    for (std::size_t step = 1U; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      mark_footprint(from.x + ratio * (to.x - from.x), from.y + ratio * (to.y - from.y));
    }
  }
  return true;
}

}  // namespace agt_terrain_map_generator
