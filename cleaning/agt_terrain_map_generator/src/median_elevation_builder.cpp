#include "agt_terrain_map_generator/median_elevation_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <vector>

namespace agt_terrain_map_generator
{
namespace
{

bool cell_index(
  const Point3f & point,
  const GridGeometry & geometry,
  std::size_t & index)
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
    geometry.resolution <= 0.0)
  {
    return false;
  }

  const auto ix = static_cast<long>(
    std::floor((static_cast<double>(point.x) - geometry.origin_x) / geometry.resolution));
  const auto iy = static_cast<long>(
    std::floor((static_cast<double>(point.y) - geometry.origin_y) / geometry.resolution));
  if (ix < 0 || iy < 0 || ix >= static_cast<long>(geometry.width) ||
    iy >= static_cast<long>(geometry.height))
  {
    return false;
  }

  index = static_cast<std::size_t>(iy) * geometry.width + static_cast<std::size_t>(ix);
  return true;
}

float median_sorted(const std::vector<float> & values)
{
  const std::size_t n = values.size();
  if (n % 2U == 1U) {
    return values[n / 2U];
  }
  return 0.5F * (values[n / 2U - 1U] + values[n / 2U]);
}

}  // namespace

MedianElevationBuilder::MedianElevationBuilder(const MedianElevationOptions & options)
: options_(options)
{
  if (options_.min_points_per_cell == 0U) {
    throw std::invalid_argument("min_points_per_cell must be >= 1");
  }
  if (!std::isfinite(options_.max_variance_m2) || options_.max_variance_m2 <= 0.0) {
    throw std::invalid_argument("max_variance_m2 must be > 0");
  }
}

bool MedianElevationBuilder::build(
  const PointCloud & ground,
  const GridGeometry & geometry,
  std::vector<ElevationCell> & elevation)
{
  if (geometry.width == 0U || geometry.height == 0U || geometry.resolution <= 0.0) {
    return false;
  }

  const std::size_t cell_count =
    static_cast<std::size_t>(geometry.width) * static_cast<std::size_t>(geometry.height);
  elevation.assign(cell_count, ElevationCell{});
  std::vector<std::vector<float>> samples(cell_count);

  for (const auto & point : ground) {
    std::size_t index = 0U;
    if (cell_index(point, geometry, index)) {
      samples[index].push_back(point.z);
    }
  }

  for (std::size_t i = 0U; i < cell_count; ++i) {
    auto & values = samples[i];
    if (values.empty()) {
      continue;
    }

    std::sort(values.begin(), values.end());
    const float median = median_sorted(values);

    double mean = 0.0;
    for (const float value : values) {
      mean += value;
    }
    mean /= static_cast<double>(values.size());

    double variance = 0.0;
    for (const float value : values) {
      const double delta = static_cast<double>(value) - mean;
      variance += delta * delta;
    }
    variance /= static_cast<double>(values.size());

    const double count_confidence = std::min(
      1.0,
      static_cast<double>(values.size()) /
      static_cast<double>(options_.min_points_per_cell));
    const double variance_confidence = variance <= options_.max_variance_m2 ?
      1.0 : options_.max_variance_m2 / variance;

    auto & cell = elevation[i];
    cell.median_height = median;
    cell.variance = static_cast<float>(variance);
    cell.point_count = static_cast<std::uint32_t>(values.size());
    cell.confidence = static_cast<float>(count_confidence * variance_confidence);
  }

  return true;
}

}  // namespace agt_terrain_map_generator
