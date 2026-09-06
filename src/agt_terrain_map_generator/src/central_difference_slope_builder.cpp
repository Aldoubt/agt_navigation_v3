#include "agt_terrain_map_generator/central_difference_slope_builder.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>

namespace agt_terrain_map_generator
{
namespace
{

bool valid_cell(const ElevationCell & cell, const double min_confidence)
{
  return std::isfinite(cell.median_height) &&
         std::isfinite(cell.confidence) &&
         static_cast<double>(cell.confidence) >= min_confidence;
}

}  // namespace

CentralDifferenceSlopeBuilder::CentralDifferenceSlopeBuilder(
  const CentralDifferenceSlopeOptions & options)
: options_(options)
{
  if (!std::isfinite(options_.min_confidence) ||
    options_.min_confidence < 0.0 || options_.min_confidence > 1.0)
  {
    throw std::invalid_argument("min_confidence must be in [0, 1]");
  }
}

bool CentralDifferenceSlopeBuilder::build(
  const GridGeometry & geometry,
  const std::vector<ElevationCell> & elevation,
  std::vector<float> & slope_deg)
{
  const std::size_t expected =
    static_cast<std::size_t>(geometry.width) * static_cast<std::size_t>(geometry.height);
  if (geometry.width == 0U || geometry.height == 0U || geometry.resolution <= 0.0 ||
    elevation.size() != expected)
  {
    return false;
  }

  slope_deg.assign(expected, std::numeric_limits<float>::quiet_NaN());
  const auto index = [&geometry](const std::uint32_t x, const std::uint32_t y) {
      return static_cast<std::size_t>(y) * geometry.width + x;
    };

  for (std::uint32_t y = 0U; y < geometry.height; ++y) {
    for (std::uint32_t x = 0U; x < geometry.width; ++x) {
      const std::size_t center_idx = index(x, y);
      const auto & center = elevation[center_idx];
      if (!valid_cell(center, options_.min_confidence)) {
        continue;
      }

      bool have_dx = false;
      bool have_dy = false;
      double dz_dx = 0.0;
      double dz_dy = 0.0;

      if (x > 0U && x + 1U < geometry.width) {
        const auto & left = elevation[index(x - 1U, y)];
        const auto & right = elevation[index(x + 1U, y)];
        if (valid_cell(left, options_.min_confidence) &&
          valid_cell(right, options_.min_confidence))
        {
          dz_dx = (static_cast<double>(right.median_height) - left.median_height) /
            (2.0 * geometry.resolution);
          have_dx = true;
        }
      }
      if (!have_dx && x + 1U < geometry.width) {
        const auto & right = elevation[index(x + 1U, y)];
        if (valid_cell(right, options_.min_confidence)) {
          dz_dx = (static_cast<double>(right.median_height) - center.median_height) /
            geometry.resolution;
          have_dx = true;
        }
      }
      if (!have_dx && x > 0U) {
        const auto & left = elevation[index(x - 1U, y)];
        if (valid_cell(left, options_.min_confidence)) {
          dz_dx = (static_cast<double>(center.median_height) - left.median_height) /
            geometry.resolution;
          have_dx = true;
        }
      }

      if (y > 0U && y + 1U < geometry.height) {
        const auto & down = elevation[index(x, y - 1U)];
        const auto & up = elevation[index(x, y + 1U)];
        if (valid_cell(down, options_.min_confidence) &&
          valid_cell(up, options_.min_confidence))
        {
          dz_dy = (static_cast<double>(up.median_height) - down.median_height) /
            (2.0 * geometry.resolution);
          have_dy = true;
        }
      }
      if (!have_dy && y + 1U < geometry.height) {
        const auto & up = elevation[index(x, y + 1U)];
        if (valid_cell(up, options_.min_confidence)) {
          dz_dy = (static_cast<double>(up.median_height) - center.median_height) /
            geometry.resolution;
          have_dy = true;
        }
      }
      if (!have_dy && y > 0U) {
        const auto & down = elevation[index(x, y - 1U)];
        if (valid_cell(down, options_.min_confidence)) {
          dz_dy = (static_cast<double>(center.median_height) - down.median_height) /
            geometry.resolution;
          have_dy = true;
        }
      }

      if (!have_dx && !have_dy) {
        continue;
      }

      const double gradient = std::hypot(have_dx ? dz_dx : 0.0, have_dy ? dz_dy : 0.0);
      slope_deg[center_idx] = static_cast<float>(std::atan(gradient) * 180.0 / M_PI);
    }
  }

  return true;
}

}  // namespace agt_terrain_map_generator
