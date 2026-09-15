#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "agt_terrain_map_generator/central_difference_slope_builder.hpp"
#include "agt_terrain_map_generator/grid/traversability_grid.hpp"
#include "agt_terrain_map_generator/median_elevation_builder.hpp"
#include "agt_terrain_map_generator/patchwork_adapter/patchwork_adapter.hpp"

namespace fs = std::filesystem;
namespace agt = agt_terrain_map_generator;
namespace
{
struct Options {std::string input_pcd, output_dir, config; std::string mq2_dir;};
struct Counts {std::size_t occupied{0}, free{0}, unknown{0}, trajectory_conflicts{0};};

[[noreturn]] void usage(const std::string & error = "")
{
  if (!error.empty()) {std::cerr << "error: " << error << "\n";}
  std::cerr << "Usage: terrain_patchwork_offline --input-pcd FILE --output-dir DIR --config patchwork.yaml [--mq2-dir DIR]\n";
  std::exit(2);
}
Options args(int argc, char ** argv)
{
  Options result;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    if (key == "--help" || key == "-h") {usage();}
    if (i + 1 == argc) {usage("missing value for " + key);}
    const std::string value(argv[++i]);
    if (key == "--input-pcd") {result.input_pcd = value;}
    else if (key == "--output-dir") {result.output_dir = value;}
    else if (key == "--config") {result.config = value;}
    else if (key == "--mq2-dir") {result.mq2_dir = value;}
    else {usage("unknown option " + key);}
  }
  if (result.input_pcd.empty() || result.output_dir.empty() || result.config.empty()) {
    usage("--input-pcd, --output-dir and --config are required");
  }
  return result;
}
double yaml_number(const fs::path & path, const std::string & key, const double fallback)
{
  std::ifstream input(path); if (!input) {throw std::runtime_error("cannot open config: " + path.string());}
  std::string line;
  while (std::getline(input, line)) {
    const auto hash = line.find('#'); if (hash != std::string::npos) {line.resize(hash);}
    const auto colon = line.find(':'); if (colon == std::string::npos) {continue;}
    const std::string name = line.substr(0, colon);
    if (name.find(key) == std::string::npos) {continue;}
    const std::string value = line.substr(colon + 1);
    try {return std::stod(value);} catch (const std::exception &) {throw std::runtime_error("invalid " + key + " in " + path.string());}
  }
  return fallback;
}
agt::GridGeometry geometry(const agt::PointCloud & cloud, const double resolution)
{
  double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
  double max_x = -min_x, max_y = -min_x;
  for (const auto & p : cloud) {min_x = std::min(min_x, static_cast<double>(p.x)); min_y = std::min(min_y, static_cast<double>(p.y)); max_x = std::max(max_x, static_cast<double>(p.x)); max_y = std::max(max_y, static_cast<double>(p.y));}
  if (!std::isfinite(min_x)) {throw std::runtime_error("no finite ground points");}
  agt::GridGeometry grid; grid.resolution = resolution; grid.origin_x = std::floor(min_x / resolution) * resolution; grid.origin_y = std::floor(min_y / resolution) * resolution;
  grid.width = static_cast<std::uint32_t>(std::max(1.0, std::ceil((max_x - grid.origin_x) / resolution) + 1.0));
  grid.height = static_cast<std::uint32_t>(std::max(1.0, std::ceil((max_y - grid.origin_y) / resolution) + 1.0)); return grid;
}
void pgm(const fs::path & path, const agt::GridGeometry & grid, const std::vector<std::uint8_t> & data)
{
  std::ofstream out(path, std::ios::binary); if (!out) {throw std::runtime_error("cannot write " + path.string());}
  out << "P5\n" << grid.width << " " << grid.height << "\n255\n";
  for (std::uint32_t y = 0; y < grid.height; ++y) {const auto source = grid.height - 1U - y; out.write(reinterpret_cast<const char *>(data.data() + static_cast<std::size_t>(source) * grid.width), grid.width);}
}
std::vector<std::uint8_t> elevation_image(const agt::ElevationGrid & elevation)
{
  float lo = std::numeric_limits<float>::infinity(), hi = -lo;
  for (const auto & c : elevation) {if (std::isfinite(c.median_height)) {lo = std::min(lo, c.median_height); hi = std::max(hi, c.median_height);}}
  std::vector<std::uint8_t> image(elevation.size(), 205U); const float scale = hi > lo ? 254.F / (hi - lo) : 1.F;
  for (std::size_t i = 0; i < elevation.size(); ++i) {if (std::isfinite(elevation[i].median_height)) {image[i] = static_cast<std::uint8_t>(std::clamp((elevation[i].median_height - lo) * scale, 0.F, 254.F));}}
  return image;
}
Counts pgm_counts(const fs::path & map)
{
  std::ifstream in(map, std::ios::binary); Counts result; std::string magic; std::uint32_t w, h, max; in >> magic >> w >> h >> max; in.get();
  if (!in || magic != "P5" || max != 255U) {throw std::runtime_error("cannot read MQ2 PGM " + map.string());}
  std::vector<unsigned char> pixels(static_cast<std::size_t>(w) * h); in.read(reinterpret_cast<char *>(pixels.data()), pixels.size());
  for (const auto value : pixels) {if (value == 0U) {++result.occupied;} else if (value == 254U) {++result.free;} else {++result.unknown;}} return result;
}
std::string yaml_scalar_or_null(const fs::path & path, const std::string & key)
{
  std::ifstream input(path); std::string line;
  while (std::getline(input, line)) {
    const auto colon = line.find(':');
    if (colon != std::string::npos && line.substr(0, colon) == key) {
      const auto value = line.substr(colon + 1U);
      const auto first = value.find_first_not_of(" \t");
      return first == std::string::npos ? "null" : value.substr(first);
    }
  }
  return "null";
}
Counts grid_counts(const agt::PointCloud & ground, const agt::PointCloud & nonground, const agt::GridGeometry & grid)
{
  const std::size_t size = static_cast<std::size_t>(grid.width) * grid.height; std::vector<bool> seen_ground(size), seen_nonground(size);
  const auto mark = [&grid, size](const agt::PointCloud & cloud, std::vector<bool> & target) {for (const auto & p : cloud) {const auto x = static_cast<long>(std::floor((p.x-grid.origin_x)/grid.resolution)); const auto y = static_cast<long>(std::floor((p.y-grid.origin_y)/grid.resolution)); if (x >= 0 && y >= 0 && x < grid.width && y < grid.height) {target[static_cast<std::size_t>(y)*grid.width+x] = true;}}};
  mark(ground, seen_ground); mark(nonground, seen_nonground); Counts r; for (std::size_t i=0; i<size; ++i) {if (seen_nonground[i]) {++r.occupied;} else if (seen_ground[i]) {++r.free;} else {++r.unknown;}} return r;
}
std::string stamp()
{
  const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()); std::tm tm{}; gmtime_r(&now, &tm); char text[32]; std::strftime(text, sizeof(text), "%Y-%m-%dT%H:%M:%SZ", &tm); return text;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options opt = args(argc, argv); const double sensor_height = yaml_number(opt.config, "sensor_height_m", -1.0); const double resolution = yaml_number(opt.config, "resolution_m", 0.10); const double tolerance = yaml_number(opt.config, "ground_tolerance_m", 0.20);
    if (!(sensor_height > 0.0) || !(resolution > 0.0) || tolerance < 0.0) {throw std::runtime_error("patchwork config values must be positive (tolerance >= 0)");}
    pcl::PointCloud<pcl::PointXYZ> input; if (pcl::io::loadPCDFile(opt.input_pcd, input) < 0) {throw std::runtime_error("cannot read input PCD: " + opt.input_pcd);}
    agt::PatchworkAdapter adapter({sensor_height, resolution, tolerance}); agt::GroundCloud ground; agt::NonGroundCloud nonground;
    if (!adapter.segment(input, ground, nonground)) {throw std::runtime_error("segmentation produced no finite points");}
    fs::create_directories(opt.output_dir); const fs::path output(opt.output_dir);
    pcl::io::savePCDFileBinary((output / "ground.pcd").string(), ground); pcl::io::savePCDFileBinary((output / "nonground.pcd").string(), nonground);
    agt::PointCloud terrain_ground; terrain_ground.reserve(ground.size()); for (const auto & p : ground) {terrain_ground.push_back({p.x,p.y,p.z});}
    const auto grid = geometry(terrain_ground, resolution); agt::MedianElevationBuilder elevation_builder({1U, 0.04}); agt::ElevationGrid elevation; elevation_builder.build(terrain_ground, grid, elevation); agt::CentralDifferenceSlopeBuilder slope_builder({0.0}); agt::SlopeGrid slope; slope_builder.build(grid, elevation, slope);
    pgm(output / "elevation.pgm", grid, elevation_image(elevation)); std::vector<std::uint8_t> slope_image(slope.size(), 205U); for (std::size_t i=0;i<slope.size();++i) {if (std::isfinite(slope[i])) {slope_image[i] = static_cast<std::uint8_t>(std::clamp(254.0 - slope[i]/30.0*254.0, 0.0, 254.0));}} pgm(output / "slope.pgm", grid, slope_image);
    const std::string status = ground.empty() ? "FAIL" : (nonground.empty() ? "REVIEW" : "PASS"); const double total = static_cast<double>(ground.size()+nonground.size());
    std::ofstream metadata(output / "terrain_metadata.yaml"); metadata << std::fixed << std::setprecision(4) << "format_version: mq3-patchwork-v1\ninput_pcd: " << fs::absolute(opt.input_pcd).string() << "\nsensor_height_m: " << sensor_height << "\nground_points: " << ground.size() << "\nnonground_points: " << nonground.size() << "\nground_ratio: " << ground.size()/total << "\nnonground_ratio: " << nonground.size()/total << "\nresolution: " << resolution << "\ntimestamp: " << stamp() << "\npatchwork_status: " << status << "\n";
    const Counts mq3 = grid_counts(terrain_ground, [&] {agt::PointCloud c; for (const auto & p:nonground) c.push_back({p.x,p.y,p.z}); return c;}(), grid); Counts mq2; bool has_mq2 = false; fs::path mq2_dir = opt.mq2_dir.empty() ? fs::path(opt.input_pcd).parent_path().parent_path() / "navigation" : fs::path(opt.mq2_dir);
    if (fs::exists(mq2_dir / "map.pgm")) {mq2 = pgm_counts(mq2_dir / "map.pgm"); has_mq2 = true;}
    const auto mq2_conflicts = has_mq2 ? yaml_scalar_or_null(mq2_dir / "metadata.yaml", "trajectory_conflict_cells_before_carve") : "null";
    std::ofstream report(output / "mq3_vs_mq2_report.yaml"); report << "format_version: mq3-vs-mq2-v1\n" << "mq2_source: " << (has_mq2 ? fs::absolute(mq2_dir).string() : "unavailable") << "\nmq2_b:\n  occupied_cells: " << (has_mq2 ? std::to_string(mq2.occupied) : "null") << "\n  free_cells: " << (has_mq2 ? std::to_string(mq2.free) : "null") << "\n  unknown_cells: " << (has_mq2 ? std::to_string(mq2.unknown) : "null") << "\n  ground_points: null\n  nonground_points: null\n  trajectory_conflict_cells: " << mq2_conflicts << "\nmq3:\n  occupied_cells: " << mq3.occupied << "\n  free_cells: " << mq3.free << "\n  unknown_cells: " << mq3.unknown << "\n  ground_points: " << ground.size() << "\n  nonground_points: " << nonground.size() << "\n  trajectory_conflict_cells: null\n";
    std::cout << "MQ3 Patchwork offline " << status << ": " << output << "\n"; return 0;
  } catch (const std::exception & e) {std::cerr << "MQ3 Patchwork offline ERROR: " << e.what() << "\n"; return 1;}
}
