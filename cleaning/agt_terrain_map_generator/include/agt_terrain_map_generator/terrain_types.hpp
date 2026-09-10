#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace agt_terrain_map_generator
{

struct Point3f
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
};

using PointCloud = std::vector<Point3f>;

struct GridGeometry
{
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  std::string frame_id{"map"};
};

struct ElevationCell
{
  float median_height{std::numeric_limits<float>::quiet_NaN()};
  float variance{std::numeric_limits<float>::quiet_NaN()};
  float confidence{0.0F};
  std::uint32_t point_count{0U};
};

struct TerrainGrid
{
  GridGeometry geometry;
  std::vector<ElevationCell> elevation;
  std::vector<float> slope_deg;
  std::vector<std::uint8_t> obstacle;
  std::vector<std::int8_t> occupancy;
};

struct GenerationContext
{
  std::string source_pcd;
  std::string output_root;
  std::string map_id;
  std::string map_version;
};

}  // namespace agt_terrain_map_generator
