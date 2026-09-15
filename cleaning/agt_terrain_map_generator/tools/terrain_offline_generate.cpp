#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "agt_terrain_map_generator/central_difference_slope_builder.hpp"
#include "agt_terrain_map_generator/export/terrain_package_exporter.hpp"
#include "agt_terrain_map_generator/geometry/robot_geometry_provider.hpp"
#include "agt_terrain_map_generator/gravity_level_patch_preprocessor.hpp"
#include "agt_terrain_map_generator/height_obstacle_builder.hpp"
#include "agt_terrain_map_generator/mapping_assets.hpp"
#include "agt_terrain_map_generator/median_elevation_builder.hpp"
#include "agt_terrain_map_generator/pcl_patch_set_source.hpp"
#include "agt_terrain_map_generator/preprocess/terrain_preprocessor.hpp"
#include "agt_terrain_map_generator/terrain/traversability_builder.hpp"
#include "agt_terrain_map_generator/trajectory/pose_loader.hpp"
#include "agt_terrain_map_generator/trajectory/trajectory_carver.hpp"

#ifdef AGT_HAS_PATCHWORKPP
#include "agt_terrain_map_generator/patchworkpp_ground_segmenter.hpp"
#endif

namespace fs = std::filesystem;
namespace agt = agt_terrain_map_generator;

namespace
{

struct Options
{
  std::string map_directory;
  std::string poses_txt;
  std::string patches_directory;
  std::string output_root;
  double sensor_height_m{0.0};
  double resolution_m{0.10};
  double margin_m{1.0};
  double voxel_leaf_m{0.20};
  std::uint32_t elevation_min_points{3U};
  double elevation_max_variance_m2{0.04};
  double min_confidence{0.50};
  double obstacle_min_height_m{0.25};
  double obstacle_max_height_m{3.0};
  double trajectory_radius_m{0.45};
  double trajectory_inflation_m{0.20};
};

[[noreturn]] void usage(const std::string & message = "")
{
  if (!message.empty()) {
    std::cerr << "error: " << message << "\n\n";
  }
  std::cerr
    << "Usage: terrain_offline_generate --map-directory DIR --output-root DIR "
       "--sensor-height-m H [options]\n"
    << "Options:\n"
    << "  --poses-txt FILE\n"
    << "  --patches-directory DIR\n"
    << "  --resolution-m R              default 0.10\n"
    << "  --margin-m M                  default 1.0\n"
    << "  --voxel-leaf-m V              default 0.20\n"
    << "  --elevation-min-points N      default 3\n"
    << "  --elevation-max-variance-m2 V default 0.04\n"
    << "  --min-confidence C            default 0.50\n"
    << "  --obstacle-min-height-m H     default 0.25\n"
    << "  --obstacle-max-height-m H     default 3.0\n"
    << "  --trajectory-radius-m R       default 0.45\n"
    << "  --trajectory-inflation-m I    default 0.20\n";
  std::exit(2);
}

double parse_double(const std::string & value, const std::string & name)
{
  std::size_t used = 0U;
  const double parsed = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(parsed)) {
    usage("invalid " + name + ": " + value);
  }
  return parsed;
}

std::uint32_t parse_u32(const std::string & value, const std::string & name)
{
  std::size_t used = 0U;
  const auto parsed = std::stoul(value, &used);
  if (used != value.size() || parsed == 0UL ||
    parsed > std::numeric_limits<std::uint32_t>::max())
  {
    usage("invalid " + name + ": " + value);
  }
  return static_cast<std::uint32_t>(parsed);
}

Options parse_args(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (key == "--help" || key == "-h") {
      usage();
    }
    if (i + 1 >= argc) {
      usage("missing value for " + key);
    }
    const std::string value = argv[++i];
    if (key == "--map-directory") {
      options.map_directory = value;
    } else if (key == "--poses-txt") {
      options.poses_txt = value;
    } else if (key == "--patches-directory") {
      options.patches_directory = value;
    } else if (key == "--output-root") {
      options.output_root = value;
    } else if (key == "--sensor-height-m") {
      options.sensor_height_m = parse_double(value, key);
    } else if (key == "--resolution-m") {
      options.resolution_m = parse_double(value, key);
    } else if (key == "--margin-m") {
      options.margin_m = parse_double(value, key);
    } else if (key == "--voxel-leaf-m") {
      options.voxel_leaf_m = parse_double(value, key);
    } else if (key == "--elevation-min-points") {
      options.elevation_min_points = parse_u32(value, key);
    } else if (key == "--elevation-max-variance-m2") {
      options.elevation_max_variance_m2 = parse_double(value, key);
    } else if (key == "--min-confidence") {
      options.min_confidence = parse_double(value, key);
    } else if (key == "--obstacle-min-height-m") {
      options.obstacle_min_height_m = parse_double(value, key);
    } else if (key == "--obstacle-max-height-m") {
      options.obstacle_max_height_m = parse_double(value, key);
    } else if (key == "--trajectory-radius-m") {
      options.trajectory_radius_m = parse_double(value, key);
    } else if (key == "--trajectory-inflation-m") {
      options.trajectory_inflation_m = parse_double(value, key);
    } else {
      usage("unknown option: " + key);
    }
  }

  if (options.map_directory.empty() || options.output_root.empty()) {
    usage("--map-directory and --output-root are required");
  }
  if (!(options.sensor_height_m > 0.0)) {
    usage("--sensor-height-m must be a measured value > 0");
  }
  if (!(options.resolution_m > 0.0) || options.margin_m < 0.0 ||
    !(options.voxel_leaf_m > 0.0) || !(options.elevation_max_variance_m2 > 0.0) ||
    options.min_confidence < 0.0 || options.min_confidence > 1.0 ||
    options.obstacle_min_height_m < 0.0 ||
    options.obstacle_max_height_m <= options.obstacle_min_height_m ||
    !(options.trajectory_radius_m > 0.0) || options.trajectory_inflation_m < 0.0)
  {
    usage("invalid numeric option");
  }
  return options;
}

agt::GridGeometry make_geometry(
  const agt::PointCloud & ground, const agt::PointCloud & non_ground,
  const double resolution, const double margin)
{
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();

  const auto visit = [&](const agt::PointCloud & cloud) {
      for (const auto & point : cloud) {
        min_x = std::min(min_x, static_cast<double>(point.x));
        min_y = std::min(min_y, static_cast<double>(point.y));
        max_x = std::max(max_x, static_cast<double>(point.x));
        max_y = std::max(max_y, static_cast<double>(point.y));
      }
    };
  visit(ground);
  visit(non_ground);
  if (!std::isfinite(min_x) || !std::isfinite(min_y) ||
    !std::isfinite(max_x) || !std::isfinite(max_y))
  {
    throw std::runtime_error("segmented map has no finite points");
  }

  agt::GridGeometry geometry;
  geometry.resolution = resolution;
  geometry.origin_x = min_x - margin;
  geometry.origin_y = min_y - margin;
  geometry.width = static_cast<std::uint32_t>(std::max(
    1.0, std::ceil((max_x + margin - geometry.origin_x) / resolution)));
  geometry.height = static_cast<std::uint32_t>(std::max(
    1.0, std::ceil((max_y + margin - geometry.origin_y) / resolution)));
  geometry.frame_id = "map";
  return geometry;
}

void append_transformed(
  const agt::PointCloud & local, const agt::RigidTransform3d & map_from_local,
  agt::PointCloud & map_cloud)
{
  map_cloud.reserve(map_cloud.size() + local.size());
  for (const auto & point : local) {
    map_cloud.push_back(map_from_local.transform(point));
  }
}

std::string parameter_summary(const Options & o, const std::size_t patches)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "\"patches=" << patches
         << ",sensor_height_m=" << o.sensor_height_m
         << ",resolution_m=" << o.resolution_m
         << ",margin_m=" << o.margin_m
         << ",voxel_leaf_m=" << o.voxel_leaf_m
         << ",elevation_min_points=" << o.elevation_min_points
         << ",elevation_max_variance_m2=" << o.elevation_max_variance_m2
         << ",min_confidence=" << o.min_confidence
         << ",obstacle_height_m=[" << o.obstacle_min_height_m
         << "," << o.obstacle_max_height_m << "]"
         << ",trajectory_radius_m=" << o.trajectory_radius_m
         << ",trajectory_inflation_m=" << o.trajectory_inflation_m
         << "\"";
  return stream.str();
}

}  // namespace

int main(int argc, char ** argv)
{
#ifndef AGT_HAS_PATCHWORKPP
  (void)argc;
  (void)argv;
  std::cerr
    << "terrain_offline_generate was built without Patchwork++; install the "
       "native patchworkpp package and rebuild agt_terrain_map_generator\n";
  return 3;
#else
  try {
    const Options options = parse_args(argc, argv);

    agt::MappingAssetSet assets;
    assets.map_directory = options.map_directory;
    assets.poses_txt = options.poses_txt;
    assets.patches_directory = options.patches_directory;

    agt::PclPatchSetSource source;
    std::string error;
    if (!source.open(assets, error)) {
      throw std::runtime_error(error);
    }

    agt::RobotBodyBox body_box;
    agt::StaticRobotGeometryProvider geometry_provider(body_box);
    agt::TerrainPreprocessorOptions preprocess_options;
    preprocess_options.self_filter.enabled = true;
    preprocess_options.rear_filter.enabled = false;
    preprocess_options.voxel.enabled = true;
    preprocess_options.voxel.leaf_size_m = options.voxel_leaf_m;
    agt::TerrainPreprocessor terrain_preprocessor(
      preprocess_options, geometry_provider);
    agt::GravityLevelPatchPreprocessor gravity_leveler;

    agt::PatchworkppOptions patchwork_options;
    patchwork_options.sensor_height_m = options.sensor_height_m;
    patchwork_options.min_range_m = 1.0;
    patchwork_options.max_range_m = 80.0;
    patchwork_options.verbose = false;
    patchwork_options.enable_rnr = false;
    patchwork_options.enable_rvpf = true;
    patchwork_options.enable_tgr = true;
    agt::PatchworkppGroundSegmenter segmenter(patchwork_options);

    agt::PointCloud map_ground;
    agt::PointCloud map_non_ground;
    std::size_t patch_count = 0U;
    std::size_t raw_points = 0U;
    std::size_t filtered_points = 0U;

    for (;;) {
      agt::PatchAsset asset;
      agt::PointCloud body_cloud;
      error.clear();
      if (!source.next(asset, body_cloud, error)) {
        if (!error.empty()) {
          throw std::runtime_error(error);
        }
        break;
      }
      ++patch_count;
      raw_points += body_cloud.size();

      agt::PreparedPatch prepared;
      if (!gravity_leveler.process(asset, body_cloud, prepared, error)) {
        throw std::runtime_error(error);
      }

      agt::PointCloud filtered;
      agt::FilterStatistics statistics;
      if (!terrain_preprocessor.process(
          prepared.local_cloud, prepared.body_from_local, filtered, statistics))
      {
        throw std::runtime_error(
          "terrain preprocessing failed for " + asset.patch_name);
      }
      filtered_points += filtered.size();
      if (filtered.empty()) {
        continue;
      }

      agt::PointCloud ground_local;
      agt::PointCloud non_ground_local;
      if (!segmenter.process(filtered, ground_local, non_ground_local)) {
        throw std::runtime_error(
          "Patchwork++ failed for " + asset.patch_name);
      }
      append_transformed(ground_local, prepared.map_from_local, map_ground);
      append_transformed(non_ground_local, prepared.map_from_local, map_non_ground);
    }

    if (patch_count == 0U || map_ground.empty()) {
      throw std::runtime_error("no usable segmented ground from patch set");
    }

    const auto grid = make_geometry(
      map_ground, map_non_ground, options.resolution_m, options.margin_m);

    agt::MedianElevationBuilder elevation_builder({
      options.elevation_min_points, options.elevation_max_variance_m2});
    agt::ElevationGrid elevation;
    if (!elevation_builder.build(map_ground, grid, elevation)) {
      throw std::runtime_error("median elevation build failed");
    }

    agt::CentralDifferenceSlopeBuilder slope_builder({options.min_confidence});
    agt::SlopeGrid slope;
    if (!slope_builder.build(grid, elevation, slope)) {
      throw std::runtime_error("slope build failed");
    }

    agt::HeightObstacleBuilder obstacle_builder({
      options.obstacle_min_height_m, options.obstacle_max_height_m,
      options.min_confidence});
    agt::ObstacleGrid obstacle_height;
    if (!obstacle_builder.build_height_above_ground(
        map_non_ground, grid, elevation, obstacle_height))
    {
      throw std::runtime_error("height-above-ground obstacle build failed");
    }

    agt::ConfidenceGrid confidence(elevation.size(), 0.0F);
    for (std::size_t i = 0U; i < elevation.size(); ++i) {
      confidence[i] = elevation[i].confidence;
    }

    const fs::path poses_path = !options.poses_txt.empty() ?
      fs::path(options.poses_txt) : fs::path(options.map_directory) / "poses.txt";
    agt::PoseLoader pose_loader;
    std::vector<agt::TrajectoryPose> poses;
    if (!pose_loader.load(poses_path.string(), poses, error)) {
      throw std::runtime_error(error);
    }

    agt::TrajectoryFreeCarver carver({
      true, options.trajectory_radius_m, options.trajectory_inflation_m});
    agt::FreeCorridorGrid corridor;
    if (!carver.carve(grid, poses, corridor)) {
      throw std::runtime_error("trajectory corridor build failed");
    }

    agt::TraversabilityOptions traversability_options;
    traversability_options.min_confidence = options.min_confidence;
    agt::TraversabilityBuilder traversability_builder(traversability_options);
    agt::TraversabilityGrid traversability;
    if (!traversability_builder.build(
        grid, elevation, slope, obstacle_height, confidence, corridor,
        traversability))
    {
      throw std::runtime_error("traversability build failed");
    }

    agt::GenerationContext context;
    context.source_pcd = options.map_directory;
    context.output_root = fs::absolute(options.output_root).string();
    context.map_id = "mq3_frozen_patch_set";
    context.map_version = "mq3-p0";

    agt::TerrainPackageExporter exporter;
    if (!exporter.export_package(
        elevation, slope, obstacle_height, confidence, corridor,
        traversability, context, parameter_summary(options, patch_count), error))
    {
      throw std::runtime_error(error);
    }

    std::size_t free_count = 0U;
    std::size_t blocked_count = 0U;
    std::size_t unknown_count = 0U;
    for (const auto & cell : traversability.cells) {
      if (cell.state == agt::TraversabilityState::FREE) {
        ++free_count;
      } else if (cell.state == agt::TraversabilityState::BLOCKED) {
        ++blocked_count;
      } else {
        ++unknown_count;
      }
    }

    std::cout
      << "MQ3 OFFLINE TERRAIN PASS\n"
      << "patches=" << patch_count
      << " raw_points=" << raw_points
      << " filtered_points=" << filtered_points
      << " ground_points=" << map_ground.size()
      << " non_ground_points=" << map_non_ground.size() << "\n"
      << "grid=" << grid.width << "x" << grid.height
      << " resolution=" << grid.resolution
      << " origin=[" << grid.origin_x << "," << grid.origin_y << "]\n"
      << "free=" << free_count
      << " occupied=" << blocked_count
      << " unknown=" << unknown_count << "\n"
      << "output=" << (fs::absolute(options.output_root) / "terrain_package")
      << "\n";
    return 0;
  } catch (const std::exception & ex) {
    std::cerr << "MQ3 OFFLINE TERRAIN ERROR: " << ex.what() << "\n";
    return 1;
  }
#endif
}
