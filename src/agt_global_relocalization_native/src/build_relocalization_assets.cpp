#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cpu_bbs3d/bbs3d.hpp>

namespace fs = std::filesystem;

namespace {

struct Options {
  std::string map;
  std::string output;
  double map_leaf{0.35};
  double bbs_min_level_res{0.25};
  int bbs_max_level{6};
};

void usage() {
  std::cerr
    << "Usage: build_relocalization_assets --map GLOBAL_MAP.pcd --output DIR"
    << " [--map-leaf 0.35] [--bbs-min-level-res 0.25] [--bbs-max-level 6]"
    << std::endl;
}

bool parse(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value after " + a);
      return argv[++i];
    };
    if (a == "--map") o.map = next();
    else if (a == "--output") o.output = next();
    else if (a == "--map-leaf") o.map_leaf = std::stod(next());
    else if (a == "--bbs-min-level-res") o.bbs_min_level_res = std::stod(next());
    else if (a == "--bbs-max-level") o.bbs_max_level = std::stoi(next());
    else throw std::runtime_error("unknown argument: " + a);
  }
  return !o.map.empty() && !o.output.empty();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_and_downsample(const std::string& path, double leaf) {
  auto raw = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile(path, *raw) != 0) {
    throw std::runtime_error("failed to load PCD: " + path);
  }
  auto out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  vg.setInputCloud(raw);
  vg.filter(*out);
  if (out->empty()) throw std::runtime_error("empty map after downsample");
  return out;
}

std::vector<Eigen::Vector3d> to_eigen(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(cloud.size());
  for (const auto& p : cloud) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      out.emplace_back(p.x, p.y, p.z);
    }
  }
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    Options o;
    if (!parse(argc, argv, o)) {
      usage();
      return 2;
    }
    if (o.map_leaf <= 0.0 || o.bbs_min_level_res <= 0.0 || o.bbs_max_level < 1) {
      throw std::runtime_error("invalid map/BBS resolution settings");
    }

    fs::create_directories(o.output);
    const auto map_cloud = load_and_downsample(o.map, o.map_leaf);
    const auto map_points = to_eigen(*map_cloud);

    cpu::BBS3D bbs;
    bbs.set_tar_points(map_points, o.bbs_min_level_res, o.bbs_max_level);
    bbs.set_trans_search_range(map_points);

    if (!bbs.save_voxel_params(o.output)) {
      throw std::runtime_error("failed to save 3D-BBS voxel parameters");
    }
    if (!bbs.save_voxelmaps_pcd(o.output)) {
      throw std::runtime_error("failed to save 3D-BBS voxel coordinates");
    }

    const fs::path downsampled_path = fs::path(o.output) / "global_map_downsampled.pcd";
    if (pcl::io::savePCDFileBinary(downsampled_path.string(), *map_cloud) != 0) {
      throw std::runtime_error("failed to save downsampled global map");
    }

    const fs::path metadata_path = fs::path(o.output) / "relocalization_assets.yaml";
    std::ofstream meta(metadata_path);
    if (!meta) throw std::runtime_error("failed to create relocalization_assets.yaml");
    meta << "schema_version: 1\n";
    meta << "source_map: \"" << fs::absolute(o.map).string() << "\"\n";
    meta << "downsampled_map: global_map_downsampled.pcd\n";
    meta << "bbs_voxelmap_dir: voxelmaps_coords\n";
    meta << "map_leaf_m: " << o.map_leaf << "\n";
    meta << "bbs_min_level_res_m: " << o.bbs_min_level_res << "\n";
    meta << "bbs_max_level: " << o.bbs_max_level << "\n";
    meta << "points: " << map_points.size() << "\n";
    meta.close();

    std::cout << "RELOCALIZATION ASSETS BUILT\n"
              << "output=" << fs::absolute(o.output).string() << "\n"
              << "points=" << map_points.size() << "\n"
              << "map=" << downsampled_path.string() << "\n"
              << "bbs=" << (fs::path(o.output) / "voxelmaps_coords").string()
              << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "RELOCALIZATION ASSET BUILD FAILED: " << e.what() << std::endl;
    return 1;
  }
}
