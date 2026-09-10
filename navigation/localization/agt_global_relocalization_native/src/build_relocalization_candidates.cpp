
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <agt_global_relocalization_native/polar_context.hpp>

namespace fs = std::filesystem;
using agt_relocalization::PolarContextEntry;
using agt_relocalization::PolarContextParams;

namespace {

struct Options {
  std::string map_dir;
  std::string output;
  PolarContextParams params;
  std::size_t min_patch_points{300};
};

void usage() {
  std::cerr
    << "Usage: build_relocalization_candidates --map-dir MAP_DIR --output DIR"
    << " [--rings 20] [--sectors 60] [--max-radius 35]"
    << " [--min-radius 0.5] [--z-offset 3] [--max-height 30]"
    << " [--min-patch-points 300]" << std::endl;
}

bool parse(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value after " + a);
      return argv[++i];
    };
    if (a == "--map-dir") o.map_dir = next();
    else if (a == "--output") o.output = next();
    else if (a == "--rings") o.params.rings = std::stoi(next());
    else if (a == "--sectors") o.params.sectors = std::stoi(next());
    else if (a == "--max-radius") o.params.max_radius = std::stod(next());
    else if (a == "--min-radius") o.params.min_radius = std::stod(next());
    else if (a == "--z-offset") o.params.z_offset = std::stod(next());
    else if (a == "--max-height") o.params.max_height = std::stod(next());
    else if (a == "--min-patch-points") o.min_patch_points = std::stoul(next());
    else throw std::runtime_error("unknown argument: " + a);
  }
  return !o.map_dir.empty() && !o.output.empty();
}

std::vector<Eigen::Vector3d> load_xyz(const fs::path& path) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(path.string(), cloud) != 0) {
    throw std::runtime_error("failed to load patch: " + path.string());
  }
  std::vector<Eigen::Vector3d> points;
  points.reserve(cloud.size());
  for (const auto& p : cloud) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      points.emplace_back(p.x, p.y, p.z);
    }
  }
  return points;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    Options o;
    if (!parse(argc, argv, o)) {
      usage();
      return 2;
    }

    const fs::path map_dir = fs::absolute(fs::path(o.map_dir));
    const fs::path poses_path = map_dir / "poses.txt";
    const fs::path patches_dir = map_dir / "patches";
    if (!fs::is_regular_file(poses_path)) {
      throw std::runtime_error("poses.txt not found: " + poses_path.string());
    }
    if (!fs::is_directory(patches_dir)) {
      throw std::runtime_error("patches directory not found: " + patches_dir.string());
    }
    fs::create_directories(o.output);

    std::ifstream poses(poses_path);
    if (!poses) throw std::runtime_error("failed to open poses.txt");

    std::vector<PolarContextEntry> entries;
    std::size_t skipped = 0;
    std::string patch_name;
    double tx, ty, tz, qw, qx, qy, qz;
    while (poses >> patch_name >> tx >> ty >> tz >> qw >> qx >> qy >> qz) {
      const fs::path patch_path = patches_dir / patch_name;
      if (!fs::is_regular_file(patch_path)) {
        ++skipped;
        continue;
      }

      const auto body_points = load_xyz(patch_path);
      if (body_points.size() < o.min_patch_points) {
        ++skipped;
        continue;
      }

      PolarContextEntry entry;
      entry.patch_name = patch_name;
      entry.translation = Eigen::Vector3d(tx, ty, tz);
      entry.orientation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();

      // PGO stores each keyframe patch in FAST-LIO body coordinates. Convert it
      // into a gravity-level, yaw-neutral keyframe frame before building the
      // descriptor. A live base_link query is therefore comparable without
      // baking the MID360 installation pitch into the place descriptor.
      const auto level_points =
        agt_relocalization::level_patch_points(body_points, entry.orientation);
      entry.descriptor =
        agt_relocalization::make_polar_context(level_points, o.params);
      entries.push_back(std::move(entry));
    }

    if (entries.empty()) {
      throw std::runtime_error("no usable keyframe patches were found");
    }

    const fs::path db_path = fs::path(o.output) / "polar_context.db";
    agt_relocalization::save_polar_context_db(db_path.string(), entries, o.params);

    const fs::path metadata = fs::path(o.output) / "polar_context.yaml";
    std::ofstream meta(metadata);
    if (!meta) throw std::runtime_error("failed to create polar_context.yaml");
    meta << "schema_version: 1\n";
    meta << "backend: agt_polar_context\n";
    meta << "source_map_dir: \"" << map_dir.string() << "\"\n";
    meta << "source_poses: poses.txt\n";
    meta << "source_patches: patches\n";
    meta << "database: polar_context.db\n";
    meta << "entries: " << entries.size() << "\n";
    meta << "skipped: " << skipped << "\n";
    meta << "rings: " << o.params.rings << "\n";
    meta << "sectors: " << o.params.sectors << "\n";
    meta << "max_radius_m: " << o.params.max_radius << "\n";
    meta << "min_radius_m: " << o.params.min_radius << "\n";
    meta << "z_offset_m: " << o.params.z_offset << "\n";
    meta << "max_height_m: " << o.params.max_height << "\n";

    std::cout << "RELOCALIZATION CANDIDATES BUILT\n"
              << "map_dir=" << map_dir.string() << "\n"
              << "output=" << fs::absolute(o.output).string() << "\n"
              << "entries=" << entries.size() << "\n"
              << "skipped=" << skipped << "\n"
              << "database=" << db_path.string() << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "CANDIDATE ASSET BUILD FAILED: " << e.what() << std::endl;
    return 1;
  }
}
