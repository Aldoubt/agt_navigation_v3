#include "agt_terrain_map_generator/export/terrain_package_exporter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>

namespace agt_terrain_map_generator
{
namespace fs = std::filesystem;
namespace
{

bool write_pgm(const fs::path & path, const GridGeometry & geometry, const std::vector<std::uint8_t> & pixels)
{
  const std::size_t expected = static_cast<std::size_t>(geometry.width) * geometry.height;
  if (pixels.size() != expected) {
    return false;
  }
  std::ofstream output(path, std::ios::binary);
  if (!output) {
    return false;
  }
  output << "P5\n" << geometry.width << ' ' << geometry.height << "\n255\n";
  for (std::uint32_t row = 0U; row < geometry.height; ++row) {
    const std::uint32_t source_y = geometry.height - 1U - row;
    output.write(
      reinterpret_cast<const char *>(pixels.data() + static_cast<std::size_t>(source_y) * geometry.width),
      geometry.width);
  }
  return static_cast<bool>(output);
}

std::vector<std::uint8_t> elevation_pixels(const ElevationGrid & elevation)
{
  float low = std::numeric_limits<float>::infinity();
  float high = -std::numeric_limits<float>::infinity();
  for (const auto & cell : elevation) {
    if (std::isfinite(cell.median_height)) {
      low = std::min(low, cell.median_height);
      high = std::max(high, cell.median_height);
    }
  }
  std::vector<std::uint8_t> result(elevation.size(), 205U);
  const float scale = high > low ? 254.0F / (high - low) : 1.0F;
  for (std::size_t index = 0U; index < elevation.size(); ++index) {
    if (std::isfinite(elevation[index].median_height)) {
      result[index] = static_cast<std::uint8_t>(std::clamp(
        (elevation[index].median_height - low) * scale, 0.0F, 254.0F));
    }
  }
  return result;
}

}  // namespace

bool TerrainPackageExporter::export_package(
  const ElevationGrid & elevation,
  const SlopeGrid & slope_deg,
  const ObstacleGrid & obstacle_height_m,
  const ConfidenceGrid & confidence,
  const FreeCorridorGrid & free_corridor,
  const TraversabilityGrid & traversability,
  const GenerationContext & context,
  const std::string & parameter_summary,
  std::string & error) const
{
  error.clear();
  const auto & geometry = traversability.geometry;
  const std::size_t expected = static_cast<std::size_t>(geometry.width) * geometry.height;
  if (geometry.width == 0U || geometry.height == 0U || elevation.size() != expected ||
    slope_deg.size() != expected || obstacle_height_m.size() != expected ||
    confidence.size() != expected || traversability.cells.size() != expected ||
    !same_grid_geometry(geometry, free_corridor.geometry) || free_corridor.confidence.size() != expected)
  {
    error = "terrain package layers do not share one valid grid geometry";
    return false;
  }

  const fs::path package_root = fs::path(context.output_root) / "terrain_package";
  std::error_code create_error;
  fs::create_directories(package_root, create_error);
  if (create_error) {
    error = "cannot create terrain package directory: " + create_error.message();
    return false;
  }

  std::vector<std::uint8_t> slope(expected, 205U);
  std::vector<std::uint8_t> obstacle(expected, 254U);
  std::vector<std::uint8_t> confidence_image(expected, 205U);
  std::vector<std::uint8_t> corridor(expected, 0U);
  std::vector<std::uint8_t> traversability_image(expected, 205U);
  for (std::size_t index = 0U; index < expected; ++index) {
    if (std::isfinite(slope_deg[index])) {
      slope[index] = static_cast<std::uint8_t>(std::clamp(
        254.0 - static_cast<double>(slope_deg[index]) / 30.0 * 254.0, 0.0, 254.0));
    }
    if (std::isfinite(obstacle_height_m[index])) {
      obstacle[index] = static_cast<std::uint8_t>(std::clamp(
        254.0 - static_cast<double>(obstacle_height_m[index]) / 0.5 * 254.0, 0.0, 254.0));
    }
    if (std::isfinite(confidence[index])) {
      confidence_image[index] = static_cast<std::uint8_t>(std::clamp(
        static_cast<double>(confidence[index]) * 254.0, 0.0, 254.0));
    }
    if (std::isfinite(free_corridor.confidence[index])) {
      corridor[index] = static_cast<std::uint8_t>(std::clamp(
        static_cast<double>(free_corridor.confidence[index]) * 254.0, 0.0, 254.0));
    }
    const auto & cell = traversability.cells[index];
    if (cell.state == TraversabilityState::BLOCKED) {
      traversability_image[index] = 0U;
    } else if (cell.state == TraversabilityState::FREE) {
      traversability_image[index] = static_cast<std::uint8_t>(std::clamp(
        (1.0 - static_cast<double>(cell.cost)) * 254.0, 0.0, 254.0));
    }
  }

  if (!write_pgm(package_root / "elevation.pgm", geometry, elevation_pixels(elevation)) ||
    !write_pgm(package_root / "slope.pgm", geometry, slope) ||
    !write_pgm(package_root / "obstacle.pgm", geometry, obstacle) ||
    !write_pgm(package_root / "confidence.pgm", geometry, confidence_image) ||
    !write_pgm(package_root / "trajectory_free.pgm", geometry, corridor) ||
    !write_pgm(package_root / "traversability.pgm", geometry, traversability_image))
  {
    error = "failed to write one or more terrain package PGM layers";
    return false;
  }

  std::ofstream metadata(package_root / "metadata.yaml");
  if (!metadata) {
    error = "failed to write terrain package metadata";
    return false;
  }
  metadata << "terrain_version: v0.3\n"
           << "resolution: " << geometry.resolution << "\n"
           << "origin: [" << geometry.origin_x << ", " << geometry.origin_y << ", 0.0]\n"
           << "frame: " << geometry.frame_id << "\n"
           << "map_id: " << context.map_id << "\n"
           << "map_version: " << context.map_version << "\n"
           << "parameters: " << parameter_summary << "\n";
  if (!metadata) {
    error = "failed while writing terrain package metadata";
    return false;
  }
  return true;
}

}  // namespace agt_terrain_map_generator
