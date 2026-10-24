#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <kiss_matcher/KISSMatcher.hpp>
#include <small_gicp/registration/registration_helper.hpp>

namespace fs = std::filesystem;
namespace {

struct Options {
  std::string map;
  std::string scan;
  std::string assets_dir;
  double map_leaf{0.35};
  double scan_leaf{0.25};
  double kiss_voxel{0.50};
  int kiss_min_inliers{10};
  int kiss_max_correspondences{5000};
  bool kiss_use_quatro{true};
  double gicp_max_corr{1.5};
  double local_map_radius_xy{35.0};
  double local_map_half_height{8.0};
  std::size_t min_local_map_points{800};
  int threads{4};
};

std::string next(int& i, int argc, char** argv) {
  if (i + 1 >= argc) throw std::runtime_error("missing argument value");
  return argv[++i];
}

bool parse(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--map") o.map = next(i, argc, argv);
    else if (a == "--scan") o.scan = next(i, argc, argv);
    else if (a == "--assets-dir") o.assets_dir = next(i, argc, argv);
    else if (a == "--map-leaf") o.map_leaf = std::stod(next(i, argc, argv));
    else if (a == "--scan-leaf") o.scan_leaf = std::stod(next(i, argc, argv));
    else if (a == "--kiss-voxel") o.kiss_voxel = std::stod(next(i, argc, argv));
    else if (a == "--kiss-min-inliers") o.kiss_min_inliers = std::stoi(next(i, argc, argv));
    else if (a == "--kiss-max-correspondences") o.kiss_max_correspondences = std::stoi(next(i, argc, argv));
    else if (a == "--kiss-use-quatro") o.kiss_use_quatro = std::stoi(next(i, argc, argv)) != 0;
    else if (a == "--gicp-max-corr") o.gicp_max_corr = std::stod(next(i, argc, argv));
    else if (a == "--local-map-radius-xy") o.local_map_radius_xy = std::stod(next(i, argc, argv));
    else if (a == "--local-map-half-height") o.local_map_half_height = std::stod(next(i, argc, argv));
    else if (a == "--min-local-map-points") o.min_local_map_points = static_cast<std::size_t>(std::stoul(next(i, argc, argv)));
    else if (a == "--threads") o.threads = std::stoi(next(i, argc, argv));
    else if (a == "--timeout") {
      // The Python orchestrator owns the hard subprocess timeout. KISS has no
      // cancellable timeout API at the locked revision, so do not pretend this
      // value interrupts matcher.estimate() internally.
      (void)std::stod(next(i, argc, argv));
    } else throw std::runtime_error("unknown argument: " + a);
  }
  return !o.map.empty() && !o.scan.empty();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(const std::string& path) {
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile(path, *cloud) != 0 || cloud->empty()) {
    throw std::runtime_error("failed to load non-empty PCD: " + path);
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& raw, double leaf) {
  if (leaf <= 0.0) throw std::runtime_error("voxel leaf must be positive");
  auto out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  filter.setInputCloud(raw);
  filter.filter(*out);
  if (out->empty()) throw std::runtime_error("empty PCD after downsample");
  return out;
}

std::vector<Eigen::Vector3d> to_eigen(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(cloud.size());
  for (const auto& p : cloud) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) out.emplace_back(p.x, p.y, p.z);
  }
  return out;
}

std::vector<Eigen::Vector3f> to_float(const std::vector<Eigen::Vector3d>& points) {
  std::vector<Eigen::Vector3f> out;
  out.reserve(points.size());
  for (const auto& p : points) out.push_back(p.cast<float>());
  return out;
}

std::vector<Eigen::Vector3d> crop_local_map(
    const std::vector<Eigen::Vector3d>& map_points, const Eigen::Vector3d& center,
    double radius_xy, double half_height) {
  const double radius_sq = radius_xy * radius_xy;
  std::vector<Eigen::Vector3d> local;
  for (const auto& p : map_points) {
    const double dx = p.x() - center.x();
    const double dy = p.y() - center.y();
    if (dx * dx + dy * dy <= radius_sq && std::abs(p.z() - center.z()) <= half_height) {
      local.push_back(p);
    }
  }
  return local;
}

void emit_failure(const std::string& message) {
  std::cout << "{\"success\":false,\"backend\":\"kiss_gicp\",\"message\":\"" << message << "\"}" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    Options o;
    if (!parse(argc, argv, o)) {
      std::cerr << "Usage: kiss_gicp_localizer --map MAP.pcd --scan SCAN.pcd "
                   "[--kiss-voxel M] [--kiss-min-inliers N] [--timeout SEC]" << std::endl;
      return 2;
    }
    if (o.kiss_voxel < 0.005 || o.kiss_min_inliers < 1 || o.kiss_max_correspondences < 1) {
      throw std::runtime_error("invalid KISS matcher limits");
    }

    std::string map_for_runtime = o.map;
    if (!o.assets_dir.empty()) {
      const fs::path candidate = fs::path(o.assets_dir) / "global_map_downsampled.pcd";
      if (fs::is_regular_file(candidate)) map_for_runtime = candidate.string();
    }
    const auto map_raw = load_cloud(map_for_runtime);
    const auto map_cloud = map_for_runtime == o.map ? downsample(map_raw, o.map_leaf) : map_raw;
    const auto scan_cloud = downsample(load_cloud(o.scan), o.scan_leaf);
    const auto map_points = to_eigen(*map_cloud);
    const auto scan_points = to_eigen(*scan_cloud);
    if (scan_points.size() < 300) throw std::runtime_error("query scan too sparse after downsample");

    // Stage 1: no-initial-pose global KISS registration. Query and map already
    // share the orchestrator's mapping_body contract; the result is T_map_body.
    kiss_matcher::KISSMatcherConfig config(static_cast<float>(o.kiss_voxel));
    config.use_quatro_ = o.kiss_use_quatro;
    config.use_ratio_test_ = true;
    config.num_max_corr_ = o.kiss_max_correspondences;
    kiss_matcher::KISSMatcher matcher(config);
    const auto started = std::chrono::steady_clock::now();
    const auto solution = matcher.estimate(to_float(scan_points), to_float(map_points));
    const auto elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    const std::size_t final_inliers = matcher.getNumFinalInliers();
    if (!solution.valid || static_cast<int>(final_inliers) < o.kiss_min_inliers) {
      emit_failure("KISS-Matcher found no sufficiently supported global pose");
      return 3;
    }
    Eigen::Isometry3d coarse = Eigen::Isometry3d::Identity();
    coarse.linear() = solution.rotation.cast<double>();
    coarse.translation() = solution.translation.cast<double>();
    if (!coarse.matrix().allFinite() || std::abs(coarse.linear().determinant() - 1.0) > 1.0e-3) {
      emit_failure("KISS-Matcher returned an invalid transform");
      return 3;
    }

    // Stage 2 deliberately matches the BBS backend: refine against the coarse
    // hit neighborhood with small_gicp and publish the same JSON pose contract.
    auto local_map_points = crop_local_map(
        map_points, coarse.translation(), o.local_map_radius_xy, o.local_map_half_height);
    const bool used_full_map = local_map_points.size() < o.min_local_map_points;
    const auto& gicp_target = used_full_map ? map_points : local_map_points;
    small_gicp::RegistrationSetting setting;
    setting.num_threads = std::max(1, o.threads);
    setting.downsampling_resolution = std::min(o.map_leaf, o.scan_leaf);
    setting.max_correspondence_distance = o.gicp_max_corr;
    const auto result = small_gicp::align(gicp_target, scan_points, coarse, setting);
    if (!result.converged || result.num_inliers == 0) {
      emit_failure("small_gicp did not converge after KISS coarse registration");
      return 4;
    }

    const auto& pose = result.T_target_source;
    const Eigen::Quaterniond q(pose.rotation());
    const Eigen::Quaterniond coarse_q(coarse.rotation());
    const double overlap = std::clamp(
        static_cast<double>(result.num_inliers) / static_cast<double>(scan_points.size()), 0.0, 1.0);
    const double fitness = result.error / static_cast<double>(std::max<std::size_t>(1, result.num_inliers));
    // KISS inlier counts and BBS scores are not interchangeable. For KISS the
    // common score is the independently measured post-GICP overlap only.
    const double score = overlap;
    std::cout << "{\"success\":true,\"backend\":\"kiss_gicp\""
              << ",\"x\":" << pose.translation().x()
              << ",\"y\":" << pose.translation().y()
              << ",\"z\":" << pose.translation().z()
              << ",\"qx\":" << q.x() << ",\"qy\":" << q.y()
              << ",\"qz\":" << q.z() << ",\"qw\":" << q.w()
              << ",\"coarse_x\":" << coarse.translation().x()
              << ",\"coarse_y\":" << coarse.translation().y()
              << ",\"coarse_z\":" << coarse.translation().z()
              << ",\"coarse_qx\":" << coarse_q.x() << ",\"coarse_qy\":" << coarse_q.y()
              << ",\"coarse_qz\":" << coarse_q.z() << ",\"coarse_qw\":" << coarse_q.w()
              << ",\"score\":" << score << ",\"fitness\":" << fitness << ",\"overlap\":" << overlap
              << ",\"kiss_elapsed_ms\":" << elapsed_ms
              << ",\"kiss_final_inliers\":" << final_inliers
              << ",\"kiss_rotation_inliers\":" << matcher.getNumRotationInliers()
              << ",\"gicp_target_points\":" << gicp_target.size()
              << ",\"gicp_full_map_fallback\":" << (used_full_map ? "true" : "false")
              << "}" << std::endl;
    return 0;
  } catch (const std::exception& e) {
    emit_failure(e.what());
    return 1;
  }
}
