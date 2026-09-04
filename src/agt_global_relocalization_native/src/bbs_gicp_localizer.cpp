#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cpu_bbs3d/bbs3d.hpp>
#include <small_gicp/registration/registration_helper.hpp>

namespace {

struct Options {
  std::string map;
  std::string scan;
  double timeout_sec{8.0};
  double map_leaf{0.35};
  double scan_leaf{0.25};
  double bbs_min_level_res{0.25};
  int bbs_max_level{6};
  double bbs_score_threshold{0.30};
  double roll_pitch_range_deg{12.0};
  double gicp_max_corr{1.5};
  int threads{4};
};

void usage() {
  std::cerr << "Usage: bbs_gicp_localizer --map MAP.pcd --scan SCAN.pcd [--timeout SEC]"
            << " [--roll-pitch-range-deg DEG] [--bbs-score-threshold 0..1]" << std::endl;
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
    else if (a == "--timeout") o.timeout_sec = std::stod(next());
    else if (a == "--map-leaf") o.map_leaf = std::stod(next());
    else if (a == "--scan-leaf") o.scan_leaf = std::stod(next());
    else if (a == "--bbs-min-level-res") o.bbs_min_level_res = std::stod(next());
    else if (a == "--bbs-max-level") o.bbs_max_level = std::stoi(next());
    else if (a == "--bbs-score-threshold") o.bbs_score_threshold = std::stod(next());
    else if (a == "--roll-pitch-range-deg") o.roll_pitch_range_deg = std::stod(next());
    else if (a == "--gicp-max-corr") o.gicp_max_corr = std::stod(next());
    else if (a == "--threads") o.threads = std::stoi(next());
    else throw std::runtime_error("unknown argument: " + a);
  }
  return !o.map.empty() && !o.scan.empty();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_and_downsample(const std::string& path, double leaf) {
  auto raw = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile(path, *raw) != 0) throw std::runtime_error("failed to load PCD: " + path);
  auto out = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  vg.setInputCloud(raw);
  vg.filter(*out);
  if (out->empty()) throw std::runtime_error("empty PCD after downsample: " + path);
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

void emit_failure(const std::string& message) {
  std::cout << "{\"success\":false,\"message\":\"" << message << "\"}" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    Options o;
    if (!parse(argc, argv, o)) { usage(); return 2; }

    const auto map_cloud = load_and_downsample(o.map, o.map_leaf);
    const auto scan_cloud = load_and_downsample(o.scan, o.scan_leaf);
    const auto map_points = to_eigen(*map_cloud);
    const auto scan_points = to_eigen(*scan_cloud);
    if (scan_points.size() < 300) throw std::runtime_error("query scan too sparse after downsample");

    // Stage 1: true no-initial-pose global coarse search. The orchestrator first
    // removes the MID360 mounting tilt using the URDF/static TF; roll/pitch here
    // therefore cover residual chassis/terrain attitude while yaw searches 360 deg.
    cpu::BBS3D bbs;
    bbs.set_num_threads(std::max(1, o.threads));
    bbs.set_tar_points(map_points, o.bbs_min_level_res, o.bbs_max_level);
    bbs.set_src_points(scan_points);
    bbs.set_trans_search_range(map_points);
    const double rp = o.roll_pitch_range_deg * M_PI / 180.0;
    bbs.set_angular_search_range(Eigen::Vector3d(-rp, -rp, -M_PI), Eigen::Vector3d(rp, rp, M_PI));
    bbs.set_score_threshold_percentage(o.bbs_score_threshold);
    if (o.timeout_sec > 0.0) {
      bbs.enable_timeout();
      bbs.set_timeout_duration_in_msec(static_cast<int>(o.timeout_sec * 1000.0));
    }
    bbs.localize();
    if (!bbs.has_localized()) {
      emit_failure(bbs.has_timed_out() ? "3D-BBS timed out" : "3D-BBS found no valid pose");
      return 3;
    }
    const Eigen::Matrix4d coarse_m = bbs.get_global_pose();
    Eigen::Isometry3d coarse(coarse_m);
    const double coarse_score = std::clamp(bbs.get_best_score_percentage(), 0.0, 1.0);

    // Stage 2: precise GICP refinement. We intentionally use upstream small_gicp rather
    // than the Ikunio node's TF publisher so map->odom ownership stays in AGT Localization Manager.
    small_gicp::RegistrationSetting setting;
    setting.num_threads = std::max(1, o.threads);
    setting.downsampling_resolution = std::min(o.map_leaf, o.scan_leaf);
    setting.max_correspondence_distance = o.gicp_max_corr;
    auto result = small_gicp::align(map_points, scan_points, coarse, setting);
    if (!result.converged || result.num_inliers == 0) {
      emit_failure("small_gicp did not converge");
      return 4;
    }

    const auto& T = result.T_target_source;
    const Eigen::Quaterniond q(T.rotation());
    const double overlap = std::clamp(static_cast<double>(result.num_inliers) / static_cast<double>(scan_points.size()), 0.0, 1.0);
    const double fitness = result.error / static_cast<double>(std::max<std::size_t>(1, result.num_inliers));
    const double score = std::clamp(0.60 * coarse_score + 0.40 * overlap, 0.0, 1.0);

    std::cout << "{\"success\":true"
              << ",\"x\":" << T.translation().x()
              << ",\"y\":" << T.translation().y()
              << ",\"z\":" << T.translation().z()
              << ",\"qx\":" << q.x() << ",\"qy\":" << q.y()
              << ",\"qz\":" << q.z() << ",\"qw\":" << q.w()
              << ",\"score\":" << score
              << ",\"fitness\":" << fitness
              << ",\"overlap\":" << overlap
              << ",\"bbs_score\":" << coarse_score
              << ",\"bbs_elapsed_ms\":" << bbs.get_elapsed_time()
              << "}" << std::endl;
    return 0;
  } catch (const std::exception& e) {
    emit_failure(e.what());
    return 1;
  }
}
