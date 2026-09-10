#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cpu_bbs3d/bbs3d.hpp>
#include <small_gicp/registration/registration_helper.hpp>

namespace fs = std::filesystem;

namespace {

struct Options {
  std::string map;
  std::string scan;
  std::string assets_dir;
  double timeout_sec{8.0};
  double map_leaf{0.35};
  double scan_leaf{0.25};
  double bbs_min_level_res{0.25};
  int bbs_max_level{6};
  double bbs_score_threshold{0.30};
  double roll_pitch_range_deg{12.0};
  double gicp_max_corr{1.5};
  double local_map_radius_xy{35.0};
  double local_map_half_height{8.0};
  std::size_t min_local_map_points{800};
  int threads{4};
  bool diagnostic{false};
  double xy_range{-1.0};
  double yaw_range_deg{-1.0};
};

void usage() {
  std::cerr << "Usage: bbs_gicp_localizer --map MAP.pcd --scan SCAN.pcd [--assets-dir DIR] [--timeout SEC]"
            << " [--roll-pitch-range-deg DEG] [--bbs-score-threshold 0..1]"
            << " [--local-map-radius-xy M] [--local-map-half-height M]"
            << " [--diagnostic --xy-range M --yaw-range-deg DEG]" << std::endl;
}

bool parse(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value after " + a);
      return argv[++i];
    };
    if (a == "--map") o.map = next();
    else if (a == "--scan") o.scan = next();
    else if (a == "--assets-dir") o.assets_dir = next();
    else if (a == "--timeout") o.timeout_sec = std::stod(next());
    else if (a == "--map-leaf") o.map_leaf = std::stod(next());
    else if (a == "--scan-leaf") o.scan_leaf = std::stod(next());
    else if (a == "--bbs-min-level-res") o.bbs_min_level_res = std::stod(next());
    else if (a == "--bbs-max-level") o.bbs_max_level = std::stoi(next());
    else if (a == "--bbs-score-threshold") o.bbs_score_threshold = std::stod(next());
    else if (a == "--roll-pitch-range-deg") o.roll_pitch_range_deg = std::stod(next());
    else if (a == "--gicp-max-corr") o.gicp_max_corr = std::stod(next());
    else if (a == "--local-map-radius-xy") o.local_map_radius_xy = std::stod(next());
    else if (a == "--local-map-half-height") o.local_map_half_height = std::stod(next());
    else if (a == "--min-local-map-points") o.min_local_map_points = static_cast<std::size_t>(std::stoul(next()));
    else if (a == "--threads") o.threads = std::stoi(next());
    else if (a == "--diagnostic") o.diagnostic = true;
    else if (a == "--xy-range") o.xy_range = std::stod(next());
    else if (a == "--yaw-range-deg") o.yaw_range_deg = std::stod(next());
    else throw std::runtime_error("unknown argument: " + a);
  }
  return !o.map.empty() && !o.scan.empty();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(const std::string& path) {
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile(path, *cloud) != 0) throw std::runtime_error("failed to load PCD: " + path);
  if (cloud->empty()) throw std::runtime_error("empty PCD: " + path);
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& raw, double leaf) {
  if (leaf <= 0.0) throw std::runtime_error("voxel leaf must be positive");
  auto out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  vg.setInputCloud(raw);
  vg.filter(*out);
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

std::vector<Eigen::Vector3d> crop_local_map(
  const std::vector<Eigen::Vector3d>& map_points,
  const Eigen::Vector3d& center,
  double radius_xy,
  double half_height) {
  const double radius_sq = radius_xy * radius_xy;
  std::vector<Eigen::Vector3d> local;
  local.reserve(std::min<std::size_t>(map_points.size(), 100000));
  for (const auto& p : map_points) {
    const double dx = p.x() - center.x();
    const double dy = p.y() - center.y();
    if ((dx * dx + dy * dy) > radius_sq) continue;
    if (std::abs(p.z() - center.z()) > half_height) continue;
    local.push_back(p);
  }
  return local;
}

void emit_failure(const std::string& message) {
  std::cout << "{\"success\":false,\"message\":\"" << message << "\"}" << std::endl;
}

std::size_t count_voxels(const std::vector<Eigen::Vector3d>& points, double resolution) {
  std::set<std::tuple<long long, long long, long long>> voxels;
  for (const auto& p : points) {
    voxels.emplace(
      static_cast<long long>(std::floor(p.x() / resolution)),
      static_cast<long long>(std::floor(p.y() / resolution)),
      static_cast<long long>(std::floor(p.z() / resolution)));
  }
  return voxels.size();
}

std::size_t estimate_candidates(
  const Eigen::Vector3d& min_xyz, const Eigen::Vector3d& max_xyz,
  double top_resolution, double yaw_range_deg) {
  const auto cells = [top_resolution](double lo, double hi) -> std::size_t {
    return static_cast<std::size_t>(std::ceil(std::max(0.0, hi - lo) / top_resolution)) + 1U;
  };
  // This is the initial translation-grid estimate. BBS keeps its search tree
  // private, so this deliberately reports a reproducible lower-level diagnostic.
  const std::size_t translations = cells(min_xyz.x(), max_xyz.x()) *
    cells(min_xyz.y(), max_xyz.y()) * cells(min_xyz.z(), max_xyz.z());
  const std::size_t yaw_divisions = std::max<std::size_t>(1U,
    static_cast<std::size_t>(std::ceil((2.0 * yaw_range_deg) / 10.0)));
  return translations * yaw_divisions;
}

void emit_diagnostic(
  const Options& o, const std::vector<Eigen::Vector3d>& map_points,
  const std::vector<Eigen::Vector3d>& scan_points, cpu::BBS3D& bbs,
  bool localized, const std::string& timeout_reason) {
  const auto range = bbs.get_trans_search_range();
  const auto angular = bbs.get_angular_search_range();
  const double top_res = o.bbs_min_level_res * std::pow(2.0, o.bbs_max_level);
  std::cout << "{\"diagnostic\":true"
            << ",\"voxel_resolution\":" << o.bbs_min_level_res
            << ",\"top_voxel_resolution\":" << top_res
            << ",\"search_range_min\":[" << range.first.x() << "," << range.first.y() << "," << range.first.z() << "]"
            << ",\"search_range_max\":[" << range.second.x() << "," << range.second.y() << "," << range.second.z() << "]"
            << ",\"yaw_range_deg\":[" << angular.front().z() * 180.0 / M_PI << "," << angular.back().z() * 180.0 / M_PI << "]"
            << ",\"query_points\":" << scan_points.size()
            << ",\"map_voxels\":" << count_voxels(map_points, o.bbs_min_level_res)
            << ",\"candidate_count\":" << estimate_candidates(range.first, range.second, top_res, o.yaw_range_deg > 0.0 ? o.yaw_range_deg : 180.0)
            << ",\"candidate_count_kind\":\"estimated_initial_translation_grid_times_yaw\""
            << ",\"search_time\":" << bbs.get_elapsed_time()
            << ",\"best_score_valid\":" << (localized && !bbs.has_timed_out() ? "true" : "false")
            << ",\"best_score\":";
  if (localized && !bbs.has_timed_out()) std::cout << bbs.get_best_score();
  else std::cout << "null";
  std::cout << ",\"best_score_percentage\":";
  if (localized && !bbs.has_timed_out()) std::cout << bbs.get_best_score_percentage();
  else std::cout << "null";
  std::cout
            << ",\"localized\":" << (localized ? "true" : "false")
            << ",\"timeout_reason\":\"" << timeout_reason << "\"}" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    Options o;
    if (!parse(argc, argv, o)) {
      usage();
      return 2;
    }

    // Prefer the offline downsampled map when available. The original PCD remains a
    // fallback so early field bringup is not blocked on asset generation.
    std::string map_for_runtime = o.map;
    if (!o.assets_dir.empty()) {
      const fs::path candidate = fs::path(o.assets_dir) / "global_map_downsampled.pcd";
      if (fs::is_regular_file(candidate)) map_for_runtime = candidate.string();
    }
    const auto map_raw = load_cloud(map_for_runtime);
    const auto map_cloud = (map_for_runtime == o.map) ? downsample(map_raw, o.map_leaf) : map_raw;
    const auto scan_cloud = downsample(load_cloud(o.scan), o.scan_leaf);
    const auto map_points = to_eigen(*map_cloud);
    const auto scan_points = to_eigen(*scan_cloud);
    if (scan_points.size() < 300) throw std::runtime_error("query scan too sparse after downsample");

    // Stage 1: true no-initial-pose global coarse search. The orchestrator first removes
    // MID360 installation tilt using the URDF/static TF. Prefer a prebuilt hierarchical
    // voxel map, otherwise build it from the PCD for compatibility with first bringup.
    cpu::BBS3D bbs;
    bbs.set_num_threads(std::max(1, o.threads));
    bool loaded_bbs_assets = false;
    if (!o.assets_dir.empty() && fs::is_directory(o.assets_dir)) {
      loaded_bbs_assets = bbs.set_voxelmaps_coords(o.assets_dir);
    }
    if (!loaded_bbs_assets) {
      bbs.set_tar_points(map_points, o.bbs_min_level_res, o.bbs_max_level);
      bbs.set_trans_search_range(map_points);
    }
    if (o.diagnostic && o.xy_range > 0.0) {
      Eigen::Vector3d map_min = map_points.front();
      Eigen::Vector3d map_max = map_points.front();
      for (const auto& p : map_points) {
        map_min = map_min.cwiseMin(p);
        map_max = map_max.cwiseMax(p);
      }
      const Eigen::Vector3d center = (map_min + map_max) * 0.5;
      bbs.set_trans_search_range(
        Eigen::Vector3d(center.x() - o.xy_range, center.y() - o.xy_range, map_min.z()),
        Eigen::Vector3d(center.x() + o.xy_range, center.y() + o.xy_range, map_max.z()));
    }
    bbs.set_src_points(scan_points);
    const double rp = o.roll_pitch_range_deg * M_PI / 180.0;
    const double yaw = (o.diagnostic && o.yaw_range_deg > 0.0) ?
      o.yaw_range_deg * M_PI / 180.0 : M_PI;
    bbs.set_angular_search_range(Eigen::Vector3d(-rp, -rp, -yaw), Eigen::Vector3d(rp, rp, yaw));
    bbs.set_score_threshold_percentage(o.bbs_score_threshold);
    if (o.timeout_sec > 0.0) {
      bbs.enable_timeout();
      bbs.set_timeout_duration_in_msec(static_cast<int>(o.timeout_sec * 1000.0));
    }
    bbs.localize();
    if (o.diagnostic) {
      std::string reason = "none";
      if (bbs.has_timed_out()) reason = "bbs_timeout";
      else if (!bbs.has_localized()) reason = "no_valid_pose_before_timeout";
      emit_diagnostic(o, map_points, scan_points, bbs, bbs.has_localized(), reason);
      // Diagnostic mode is intentionally BBS-only. It must not invoke GICP or
      // change the production recovery path.
      return bbs.has_localized() ? 0 : 3;
    }
    if (!bbs.has_localized()) {
      emit_failure(bbs.has_timed_out() ? "3D-BBS timed out" : "3D-BBS found no valid pose");
      return 3;
    }
    const Eigen::Matrix4d coarse_m = bbs.get_global_pose();
    Eigen::Isometry3d coarse(coarse_m);
    const double coarse_score = std::clamp(bbs.get_best_score_percentage(), 0.0, 1.0);

    // Stage 2: precise GICP refinement against only the BBS-hit neighborhood. This bounds
    // preprocessing/KdTree cost on large maps. If the crop is unexpectedly sparse, use
    // the complete downsampled map rather than converting a data problem into a hard fail.
    auto local_map_points = crop_local_map(
      map_points, coarse.translation(), o.local_map_radius_xy, o.local_map_half_height);
    const bool used_full_map_for_gicp = local_map_points.size() < o.min_local_map_points;
    const auto& gicp_target = used_full_map_for_gicp ? map_points : local_map_points;

    small_gicp::RegistrationSetting setting;
    setting.num_threads = std::max(1, o.threads);
    setting.downsampling_resolution = std::min(o.map_leaf, o.scan_leaf);
    setting.max_correspondence_distance = o.gicp_max_corr;
    auto result = small_gicp::align(gicp_target, scan_points, coarse, setting);
    if (!result.converged || result.num_inliers == 0) {
      emit_failure("small_gicp did not converge");
      return 4;
    }

    const auto& T = result.T_target_source;
    const Eigen::Quaterniond q(T.rotation());
    const Eigen::Quaterniond coarse_q(coarse.rotation());
    const double overlap = std::clamp(
      static_cast<double>(result.num_inliers) / static_cast<double>(scan_points.size()), 0.0, 1.0);
    const double fitness = result.error / static_cast<double>(std::max<std::size_t>(1, result.num_inliers));
    const double score = std::clamp(0.60 * coarse_score + 0.40 * overlap, 0.0, 1.0);

    std::cout << "{\"success\":true"
              << ",\"x\":" << T.translation().x()
              << ",\"y\":" << T.translation().y()
              << ",\"z\":" << T.translation().z()
              << ",\"qx\":" << q.x() << ",\"qy\":" << q.y()
              << ",\"qz\":" << q.z() << ",\"qw\":" << q.w()
              << ",\"coarse_x\":" << coarse.translation().x()
              << ",\"coarse_y\":" << coarse.translation().y()
              << ",\"coarse_z\":" << coarse.translation().z()
              << ",\"coarse_qx\":" << coarse_q.x()
              << ",\"coarse_qy\":" << coarse_q.y()
              << ",\"coarse_qz\":" << coarse_q.z()
              << ",\"coarse_qw\":" << coarse_q.w()
              << ",\"score\":" << score
              << ",\"fitness\":" << fitness
              << ",\"overlap\":" << overlap
              << ",\"bbs_score\":" << coarse_score
              << ",\"bbs_elapsed_ms\":" << bbs.get_elapsed_time()
              << ",\"bbs_assets_loaded\":" << (loaded_bbs_assets ? "true" : "false")
              << ",\"gicp_target_points\":" << gicp_target.size()
              << ",\"gicp_full_map_fallback\":" << (used_full_map_for_gicp ? "true" : "false")
              << "}" << std::endl;
    return 0;
  } catch (const std::exception& e) {
    emit_failure(e.what());
    return 1;
  }
}
