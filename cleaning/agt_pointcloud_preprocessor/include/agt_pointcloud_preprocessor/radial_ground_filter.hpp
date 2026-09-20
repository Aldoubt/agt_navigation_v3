#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace agt_pointcloud_preprocessor
{

struct TerrainPoint
{
  double x{};
  double y{};
  double z{};
};

struct GroundResult
{
  bool is_ground{false};
  double ground_z{};
  bool ground_reference_valid{false};
};

struct RadialGroundOptions
{
  double sensor_ground_z_m{-0.20};
  double radial_bin_deg{1.5};
  double local_max_slope_deg{35.0};
  double global_max_slope_deg{35.0};
  double min_height_threshold_m{0.08};
  double max_ground_gap_m{0.75};
  double max_ground_range_m{15.0};
};

class RadialGroundFilter
{
public:
  explicit RadialGroundFilter(RadialGroundOptions options)
  : options_(options)
  {
    if (options_.radial_bin_deg <= 0.0 || options_.radial_bin_deg > 30.0) {
      throw std::invalid_argument("radial_bin_deg must be in (0, 30]");
    }
    if (options_.local_max_slope_deg <= 0.0 || options_.local_max_slope_deg >= 89.0 ||
      options_.global_max_slope_deg <= 0.0 || options_.global_max_slope_deg >= 89.0)
    {
      throw std::invalid_argument("ground slope limits must be in (0, 89) degrees");
    }
    if (options_.min_height_threshold_m <= 0.0 || options_.max_ground_gap_m <= 0.0 ||
      options_.max_ground_range_m <= 0.0)
    {
      throw std::invalid_argument("ground thresholds and ranges must be positive");
    }
  }

  std::vector<GroundResult> classify(const std::vector<TerrainPoint> & points) const
  {
    constexpr double kPi = 3.14159265358979323846;
    const double bin_rad = options_.radial_bin_deg * kPi / 180.0;
    const std::size_t ray_count = static_cast<std::size_t>(std::ceil(2.0 * kPi / bin_rad));
    std::vector<std::vector<std::size_t>> rays(ray_count);
    std::vector<double> ranges(points.size(), 0.0);
    std::vector<GroundResult> result(
      points.size(), GroundResult{false, options_.sensor_ground_z_m, true});

    for (std::size_t index = 0; index < points.size(); ++index) {
      const auto & point = points[index];
      const double range = std::hypot(point.x, point.y);
      ranges[index] = range;
      double angle = std::atan2(point.y, point.x) + kPi;
      angle = std::clamp(angle, 0.0, std::nextafter(2.0 * kPi, 0.0));
      const auto ray = std::min(
        ray_count - 1U, static_cast<std::size_t>(std::floor(angle / bin_rad)));
      rays[ray].push_back(index);
    }

    const double local_tangent = std::tan(options_.local_max_slope_deg * kPi / 180.0);
    const double global_tangent = std::tan(options_.global_max_slope_deg * kPi / 180.0);
    for (auto & ray : rays) {
      std::sort(ray.begin(), ray.end(), [&](const std::size_t lhs, const std::size_t rhs) {
        if (std::abs(ranges[lhs] - ranges[rhs]) > 1e-6) {
          return ranges[lhs] < ranges[rhs];
        }
        return points[lhs].z < points[rhs].z;
      });

      double previous_ground_range = 0.0;
      double previous_ground_z = options_.sensor_ground_z_m;
      for (const auto index : ray) {
        const auto & point = points[index];
        const double range = ranges[index];
        result[index].ground_z = previous_ground_z;
        if (range > options_.max_ground_range_m) {
          continue;
        }

        const double global_limit = options_.min_height_threshold_m + global_tangent * range;
        const bool global_consistent =
          std::abs(point.z - options_.sensor_ground_z_m) <= global_limit;
        const double gap = std::max(0.0, range - previous_ground_range);
        const double local_limit = options_.min_height_threshold_m + local_tangent * gap;
        const bool local_consistent = gap > options_.max_ground_gap_m ?
          global_consistent : std::abs(point.z - previous_ground_z) <= local_limit;

        if (global_consistent && local_consistent) {
          result[index].is_ground = true;
          result[index].ground_z = point.z;
          previous_ground_range = range;
          previous_ground_z = point.z;
        }
      }
    }
    return result;
  }

private:
  RadialGroundOptions options_;
};

}  // namespace agt_pointcloud_preprocessor
