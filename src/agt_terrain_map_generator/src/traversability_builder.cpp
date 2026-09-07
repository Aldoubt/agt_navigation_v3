#include "agt_terrain_map_generator/terrain/traversability_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace agt_terrain_map_generator
{
namespace
{

bool finite(const float value)
{
  return std::isfinite(value);
}

float clamp01(const double value)
{
  return static_cast<float>(std::clamp(value, 0.0, 1.0));
}

double local_roughness_variance(
  const GridGeometry & geometry, const ElevationGrid & elevation, const std::uint32_t x,
  const std::uint32_t y)
{
  double sum = 0.0;
  double sum_squared = 0.0;
  std::size_t count = 0U;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      const int nx = static_cast<int>(x) + dx;
      const int ny = static_cast<int>(y) + dy;
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(geometry.width) ||
        ny >= static_cast<int>(geometry.height))
      {
        continue;
      }
      const auto & cell = elevation[static_cast<std::size_t>(ny) * geometry.width + nx];
      if (finite(cell.median_height)) {
        const double height = cell.median_height;
        sum += height;
        sum_squared += height * height;
        ++count;
      }
    }
  }
  if (count < 2U) {
    return 0.0;
  }
  const double mean = sum / static_cast<double>(count);
  return std::max(0.0, sum_squared / static_cast<double>(count) - mean * mean);
}

}  // namespace

TraversabilityBuilder::TraversabilityBuilder(const TraversabilityOptions & options)
: options_(options)
{
  if (!std::isfinite(options_.slope_weight) || !std::isfinite(options_.obstacle_weight) ||
    !std::isfinite(options_.roughness_weight) || !std::isfinite(options_.confidence_weight) ||
    !std::isfinite(options_.trajectory_bonus) || options_.slope_weight < 0.0 ||
    options_.obstacle_weight < 0.0 || options_.roughness_weight < 0.0 ||
    options_.confidence_weight < 0.0 || options_.trajectory_bonus < 0.0)
  {
    throw std::invalid_argument("traversability weights must be finite and non-negative");
  }
  if (!std::isfinite(options_.slope_safe_degree) || !std::isfinite(options_.slope_warning_degree) ||
    !std::isfinite(options_.slope_blocked_degree) || options_.slope_safe_degree < 0.0 ||
    options_.slope_safe_degree >= options_.slope_warning_degree ||
    options_.slope_warning_degree >= options_.slope_blocked_degree)
  {
    throw std::invalid_argument("invalid traversability slope thresholds");
  }
  if (!std::isfinite(options_.obstacle_ignore_height_m) ||
    !std::isfinite(options_.obstacle_blocked_height_m) || options_.obstacle_ignore_height_m < 0.0 ||
    options_.obstacle_ignore_height_m >= options_.obstacle_blocked_height_m ||
    !std::isfinite(options_.roughness_variance_m2) || options_.roughness_variance_m2 <= 0.0 ||
    !std::isfinite(options_.min_confidence) || options_.min_confidence < 0.0 ||
    options_.min_confidence > 1.0)
  {
    throw std::invalid_argument("invalid traversability terrain thresholds");
  }
}

bool TraversabilityBuilder::build(
  const GridGeometry & geometry,
  const ElevationGrid & elevation,
  const SlopeGrid & slope_deg,
  const ObstacleGrid & obstacle_height_m,
  const ConfidenceGrid & confidence,
  const FreeCorridorGrid & free_corridor,
  TraversabilityGrid & traversability) const
{
  const std::size_t expected = static_cast<std::size_t>(geometry.width) * geometry.height;
  if (geometry.width == 0U || geometry.height == 0U || geometry.resolution <= 0.0 ||
    elevation.size() != expected || slope_deg.size() != expected ||
    obstacle_height_m.size() != expected || confidence.size() != expected ||
    !same_grid_geometry(geometry, free_corridor.geometry) || free_corridor.confidence.size() != expected)
  {
    return false;
  }

  traversability.geometry = geometry;
  traversability.cells.assign(expected, TraversabilityCell{});
  for (std::uint32_t y = 0U; y < geometry.height; ++y) {
    for (std::uint32_t x = 0U; x < geometry.width; ++x) {
      const std::size_t index = static_cast<std::size_t>(y) * geometry.width + x;
      auto & output = traversability.cells[index];
      const double trajectory_confidence = finite(free_corridor.confidence[index]) ?
        std::clamp(static_cast<double>(free_corridor.confidence[index]), 0.0, 1.0) : 0.0;
      const double terrain_confidence = finite(confidence[index]) ?
        std::clamp(static_cast<double>(confidence[index]), 0.0, 1.0) : 0.0;
      const double effective_confidence = std::max(terrain_confidence, trajectory_confidence);
      output.confidence = static_cast<float>(effective_confidence);

      const double obstacle_height = finite(obstacle_height_m[index]) ?
        std::max(0.0, static_cast<double>(obstacle_height_m[index])) : 0.0;
      const bool slope_is_known = finite(slope_deg[index]);
      const double slope = slope_is_known ? std::max(0.0, static_cast<double>(slope_deg[index])) : 0.0;

      // A physically high obstacle or a slope above the tracked vehicle's
      // blocked threshold wins over trajectory evidence.
      if (obstacle_height > options_.obstacle_blocked_height_m ||
        (slope_is_known && slope > options_.slope_blocked_degree))
      {
        output.cost = 1.0F;
        output.state = TraversabilityState::BLOCKED;
        continue;
      }

      // Traversed cells may fill sparse map confidence, but unobserved cells
      // without trajectory evidence remain unknown rather than free.
      if (!finite(elevation[index].median_height) ||
        effective_confidence < options_.min_confidence || (!slope_is_known && trajectory_confidence <= 0.0))
      {
        output.cost = 1.0F;
        output.state = TraversabilityState::UNKNOWN;
        continue;
      }

      double slope_normalized = 0.0;
      if (slope > options_.slope_safe_degree) {
        if (slope <= options_.slope_warning_degree) {
          slope_normalized = 0.5 * (slope - options_.slope_safe_degree) /
            (options_.slope_warning_degree - options_.slope_safe_degree);
        } else {
          slope_normalized = 0.5 + 0.5 * (slope - options_.slope_warning_degree) /
            (options_.slope_blocked_degree - options_.slope_warning_degree);
        }
      }
      const double obstacle_normalized = obstacle_height <= options_.obstacle_ignore_height_m ? 0.0 :
        (obstacle_height - options_.obstacle_ignore_height_m) /
        (options_.obstacle_blocked_height_m - options_.obstacle_ignore_height_m);
      const double roughness_normalized = local_roughness_variance(geometry, elevation, x, y) /
        options_.roughness_variance_m2;
      const double cost = options_.slope_weight * std::clamp(slope_normalized, 0.0, 1.0) +
        options_.obstacle_weight * std::clamp(obstacle_normalized, 0.0, 1.0) +
        options_.roughness_weight * std::clamp(roughness_normalized, 0.0, 1.0) -
        options_.confidence_weight * effective_confidence -
        options_.trajectory_bonus * trajectory_confidence;
      output.cost = clamp01(cost);
      output.state = TraversabilityState::FREE;
    }
  }
  return true;
}

}  // namespace agt_terrain_map_generator
