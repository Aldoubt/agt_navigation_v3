
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
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

#include <agt_global_relocalization_native/polar_context.hpp>

namespace fs = std::filesystem;

namespace {

struct Options {
  std::string map;
  std::string scan;
  std::string assets_dir;
  double timeout_sec{18.0};
  double map_leaf{0.35};
  double scan_leaf{0.35};
  double bbs_score_threshold{0.05};
  double roll_pitch_range_deg{0.0};
  double candidate_xy_radius{4.0};
  double candidate_z_radius{2.0};
  double candidate_yaw_range_deg{0.0};
  int candidate_top_k{2};
  int descriptor_prefilter{40};
  double per_candidate_timeout_sec{8.0};
  double gicp_max_corr{2.0};
  double local_map_radius_xy{30.0};
  double local_map_half_height{8.0};
  std::size_t min_local_map_points{800};
  int threads{8};
  // T_base_body maps FAST-LIO / Batch-LIO IMU-body coordinates into robot base_link.
  // It is composed from the measured base_link->lidar_link mount and the pinned
  // MID360 LiDAR/IMU extrinsic (Batch-LIO: p_body = R * p_lidar + t).
  Eigen::Vector3d base_from_body_t{0.25960014, -0.02326770, 0.45244230};
  Eigen::Quaterniond base_from_body_q{0.994959177, -0.000477000, 0.100267018, 0.001592000};
};

struct RankedCandidate {
  agt_relocalization::PolarContextEntry entry;
  double ring_distance{std::numeric_limits<double>::infinity()};
  double sector_similarity{-1.0};
  int sector_shift{0};
  double yaw_estimate{0.0};
};

struct CoarseResult {
  bool valid{false};
  RankedCandidate candidate;
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  double bbs_score{0.0};
  double elapsed_ms{0.0};
};

void usage() {
  std::cerr
    << "Usage: candidate_bbs_gicp_localizer --map MAP.pcd --scan SCAN.pcd"
    << " --assets-dir DIR [--timeout 10] [--candidate-top-k 3]"
    << " [--candidate-xy-radius 4] [--candidate-yaw-range-deg 0]"
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
    else if (a == "--scan") o.scan = next();
    else if (a == "--assets-dir") o.assets_dir = next();
    else if (a == "--timeout") o.timeout_sec = std::stod(next());
    else if (a == "--map-leaf") o.map_leaf = std::stod(next());
    else if (a == "--scan-leaf") o.scan_leaf = std::stod(next());
    else if (a == "--bbs-score-threshold") o.bbs_score_threshold = std::stod(next());
    else if (a == "--roll-pitch-range-deg") o.roll_pitch_range_deg = std::stod(next());
    else if (a == "--candidate-xy-radius") o.candidate_xy_radius = std::stod(next());
    else if (a == "--candidate-z-radius") o.candidate_z_radius = std::stod(next());
    else if (a == "--candidate-yaw-range-deg") o.candidate_yaw_range_deg = std::stod(next());
    else if (a == "--candidate-top-k") o.candidate_top_k = std::stoi(next());
    else if (a == "--descriptor-prefilter") o.descriptor_prefilter = std::stoi(next());
    else if (a == "--per-candidate-timeout") o.per_candidate_timeout_sec = std::stod(next());
    else if (a == "--gicp-max-corr") o.gicp_max_corr = std::stod(next());
    else if (a == "--local-map-radius-xy") o.local_map_radius_xy = std::stod(next());
    else if (a == "--local-map-half-height") o.local_map_half_height = std::stod(next());
    else if (a == "--min-local-map-points") o.min_local_map_points = std::stoul(next());
    else if (a == "--base-from-body-tx") o.base_from_body_t.x() = std::stod(next());
    else if (a == "--base-from-body-ty") o.base_from_body_t.y() = std::stod(next());
    else if (a == "--base-from-body-tz") o.base_from_body_t.z() = std::stod(next());
    else if (a == "--base-from-body-qx") o.base_from_body_q.x() = std::stod(next());
    else if (a == "--base-from-body-qy") o.base_from_body_q.y() = std::stod(next());
    else if (a == "--base-from-body-qz") o.base_from_body_q.z() = std::stod(next());
    else if (a == "--base-from-body-qw") o.base_from_body_q.w() = std::stod(next());
    else if (a == "--threads") o.threads = std::stoi(next());
    else throw std::runtime_error("unknown argument: " + a);
  }
  return !o.map.empty() && !o.scan.empty() && !o.assets_dir.empty();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud(const std::string& path) {
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if (pcl::io::loadPCDFile(path, *cloud) != 0) {
    throw std::runtime_error("failed to load PCD: " + path);
  }
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
  if (out->empty()) throw std::runtime_error("empty cloud after downsample");
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

std::vector<Eigen::Vector3d> rotate_points_yaw(
    const std::vector<Eigen::Vector3d>& points, double yaw) {
  const Eigen::Matrix3d rotation =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::vector<Eigen::Vector3d> out;
  out.reserve(points.size());
  for (const auto& p : points) {
    out.push_back(rotation * p);
  }
  return out;
}

std::vector<Eigen::Vector3d> crop_local_map(
    const std::vector<Eigen::Vector3d>& map_points,
    const Eigen::Vector3d& center,
    double radius_xy,
    double half_height) {
  const double r2 = radius_xy * radius_xy;
  std::vector<Eigen::Vector3d> out;
  out.reserve(std::min<std::size_t>(map_points.size(), 100000));
  for (const auto& p : map_points) {
    const double dx = p.x() - center.x();
    const double dy = p.y() - center.y();
    if (dx * dx + dy * dy > r2) continue;
    if (std::abs(p.z() - center.z()) > half_height) continue;
    out.push_back(p);
  }
  return out;
}

void emit_failure(const std::string& message) {
  std::cout << "{\"success\":false,\"message\":\"" << message << "\"}" << std::endl;
}

std::vector<RankedCandidate> rank_candidates(
    const agt_relocalization::PolarContext& query,
    const std::vector<agt_relocalization::PolarContextEntry>& entries,
    const agt_relocalization::PolarContextParams& params,
    int prefilter) {
  std::vector<RankedCandidate> ranked;
  ranked.reserve(entries.size());
  for (const auto& entry : entries) {
    RankedCandidate c;
    c.entry = entry;
    c.ring_distance = agt_relocalization::ring_key_distance(query, entry.descriptor);
    ranked.push_back(std::move(c));
  }

  std::sort(ranked.begin(), ranked.end(),
    [](const auto& a, const auto& b) { return a.ring_distance < b.ring_distance; });
  if (prefilter > 0 && ranked.size() > static_cast<std::size_t>(prefilter)) {
    ranked.resize(static_cast<std::size_t>(prefilter));
  }

  const double sector_rad = 2.0 * M_PI / static_cast<double>(params.sectors);
  for (auto& c : ranked) {
    const auto shift =
      agt_relocalization::best_sector_shift(query, c.entry.descriptor);
    c.sector_similarity = shift.first;
    c.sector_shift = shift.second;
    c.yaw_estimate = agt_relocalization::wrap_angle(
      agt_relocalization::yaw_from_quaternion(c.entry.orientation) +
      static_cast<double>(c.sector_shift) * sector_rad);
  }

  // The rotational similarity removes most repeated ring-key candidates, while
  // ring distance is retained as a deterministic tie-breaker.
  std::sort(ranked.begin(), ranked.end(), [](const auto& a, const auto& b) {
    if (std::abs(a.sector_similarity - b.sector_similarity) > 1e-9) {
      return a.sector_similarity > b.sector_similarity;
    }
    return a.ring_distance < b.ring_distance;
  });
  return ranked;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    Options o;
    if (!parse(argc, argv, o)) {
      usage();
      return 2;
    }
    if (o.candidate_top_k <= 0 || o.descriptor_prefilter <= 0 ||
        o.candidate_xy_radius <= 0.0 || o.candidate_z_radius <= 0.0) {
      throw std::runtime_error("invalid candidate-search settings");
    }

    const fs::path assets(o.assets_dir);
    const fs::path db_path = assets / "polar_context.db";
    if (!fs::is_regular_file(db_path)) {
      throw std::runtime_error(
        "polar_context.db missing; run build_relocalization_candidates for this map package");
    }

    agt_relocalization::PolarContextParams descriptor_params;
    auto db_entries =
      agt_relocalization::load_polar_context_db(db_path.string(), descriptor_params);

    // poses.txt stores T_map_body, while live relocalization queries are in base_link.
    // Convert each candidate seed to T_map_base using the calibrated T_base_body.
    Eigen::Isometry3d T_base_body = Eigen::Isometry3d::Identity();
    const Eigen::Quaterniond q_base_body = o.base_from_body_q.normalized();
    T_base_body.linear() = q_base_body.toRotationMatrix();
    T_base_body.translation() = o.base_from_body_t;
    const Eigen::Isometry3d T_body_base = T_base_body.inverse();
    for (auto& entry : db_entries) {
      Eigen::Isometry3d T_map_body = Eigen::Isometry3d::Identity();
      T_map_body.linear() = entry.orientation.normalized().toRotationMatrix();
      T_map_body.translation() = entry.translation;
      const Eigen::Isometry3d T_map_base = T_map_body * T_body_base;
      entry.translation = T_map_base.translation();
      entry.orientation = Eigen::Quaterniond(T_map_base.rotation()).normalized();
    }

    std::string map_for_runtime = o.map;
    const fs::path ds_map = assets / "global_map_downsampled.pcd";
    if (fs::is_regular_file(ds_map)) map_for_runtime = ds_map.string();

    const auto map_raw = load_cloud(map_for_runtime);
    const auto map_cloud =
      (map_for_runtime == o.map) ? downsample(map_raw, o.map_leaf) : map_raw;
    const auto scan_cloud = downsample(load_cloud(o.scan), o.scan_leaf);
    const auto map_points = to_eigen(*map_cloud);
    const auto scan_points = to_eigen(*scan_cloud);
    if (scan_points.size() < 300) {
      throw std::runtime_error("query scan too sparse after downsample");
    }

    const auto query_descriptor =
      agt_relocalization::make_polar_context(scan_points, descriptor_params);
    auto candidates = rank_candidates(
      query_descriptor, db_entries, descriptor_params, o.descriptor_prefilter);
    if (candidates.empty()) throw std::runtime_error("descriptor retrieval returned no candidates");

    const auto started = std::chrono::steady_clock::now();
    CoarseResult best;
    const int try_count =
      std::min<int>(o.candidate_top_k, static_cast<int>(candidates.size()));

    for (int i = 0; i < try_count; ++i) {
      const auto now = std::chrono::steady_clock::now();
      const double used_sec =
        std::chrono::duration<double>(now - started).count();
      const double remaining = o.timeout_sec - used_sec;
      if (remaining <= 0.15) break;

      const auto& c = candidates[static_cast<std::size_t>(i)];
      cpu::BBS3D bbs;
      bbs.set_num_threads(std::max(1, o.threads));
      if (!bbs.set_voxelmaps_coords(o.assets_dir)) {
        // Candidate mode is deliberately asset-backed. Building a whole-map
        // hierarchy online reintroduces the startup latency this path removes.
        throw std::runtime_error("failed to load BBS voxel assets");
      }

      // Polar-context retrieval already provides the dominant yaw hypothesis.
      // Pre-rotate the query by that yaw and let 3D-BBS search only the residual
      // rotation around identity. This is both faster and avoids an upstream
      // 3D-BBS hierarchy edge case where a narrow, non-zero absolute angular
      // interval can be represented as zero rotation at the coarsest levels and
      // then be pruned before the true residual is introduced.
      const double candidate_yaw =
        agt_relocalization::yaw_from_quaternion(c.entry.orientation);
      const Eigen::Matrix3d yaw_neutral_tilt =
        Eigen::AngleAxisd(-candidate_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        c.entry.orientation.normalized().toRotationMatrix();
      const Eigen::Matrix3d seed_rotation =
        Eigen::AngleAxisd(c.yaw_estimate, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        yaw_neutral_tilt;
      std::vector<Eigen::Vector3d> seeded_scan_points;
      seeded_scan_points.reserve(scan_points.size());
      for (const auto& p : scan_points) seeded_scan_points.push_back(seed_rotation * p);
      bbs.set_src_points(seeded_scan_points);
      bbs.set_trans_search_range(
        Eigen::Vector3d(
          c.entry.translation.x() - o.candidate_xy_radius,
          c.entry.translation.y() - o.candidate_xy_radius,
          c.entry.translation.z() - o.candidate_z_radius),
        Eigen::Vector3d(
          c.entry.translation.x() + o.candidate_xy_radius,
          c.entry.translation.y() + o.candidate_xy_radius,
          c.entry.translation.z() + o.candidate_z_radius));

      const double rp = o.roll_pitch_range_deg * M_PI / 180.0;
      const double yr = o.candidate_yaw_range_deg * M_PI / 180.0;
      bbs.set_angular_search_range(
        Eigen::Vector3d(-rp, -rp, -yr),
        Eigen::Vector3d( rp,  rp,  yr));
      bbs.set_score_threshold_percentage(o.bbs_score_threshold);
      bbs.enable_timeout();
      const double candidate_timeout =
        std::max(0.10, std::min(o.per_candidate_timeout_sec, remaining));
      bbs.set_timeout_duration_in_msec(
        static_cast<int>(candidate_timeout * 1000.0));
      bbs.localize();

      if (!bbs.has_localized() || bbs.has_timed_out()) continue;
      const double score =
        std::clamp(bbs.get_best_score_percentage(), 0.0, 1.0);
      if (!best.valid || score > best.bbs_score) {
        best.valid = true;
        best.candidate = c;
        const Eigen::Isometry3d residual_pose(bbs.get_global_pose());
        Eigen::Isometry3d orientation_seed = Eigen::Isometry3d::Identity();
        orientation_seed.linear() = seed_rotation;
        best.pose = residual_pose * orientation_seed;
        best.bbs_score = score;
        best.elapsed_ms = bbs.get_elapsed_time();
      }

      // A strong geometric score plus a strong descriptor match is enough to
      // stop trying weaker descriptor candidates.
      // Polar Context is the place/yaw selector; BBS only needs to provide a
      // geometrically useful coarse seed. Final acceptance is intentionally
      // deferred to GICP fitness/overlap and the ROS-side quality gates.
      if (score >= 0.65 && c.sector_similarity >= 0.85) break;
    }

    if (!best.valid) {
      const auto& first = candidates.front();
      emit_failure(
        "candidate-guided 3D-BBS found no valid pose; top_patch=" +
        first.entry.patch_name);
      return 3;
    }

    auto local_map_points = crop_local_map(
      map_points, best.pose.translation(),
      o.local_map_radius_xy, o.local_map_half_height);
    const bool full_map_fallback =
      local_map_points.size() < o.min_local_map_points;
    const auto& gicp_target =
      full_map_fallback ? map_points : local_map_points;

    small_gicp::RegistrationSetting setting;
    setting.num_threads = std::max(1, o.threads);
    setting.downsampling_resolution = std::min(o.map_leaf, o.scan_leaf);
    setting.max_correspondence_distance = o.gicp_max_corr;
    auto result =
      small_gicp::align(gicp_target, scan_points, best.pose, setting);
    if (!result.converged || result.num_inliers == 0) {
      emit_failure("small_gicp did not converge after candidate-guided BBS");
      return 4;
    }

    const auto& T = result.T_target_source;
    const Eigen::Quaterniond q(T.rotation());
    const Eigen::Quaterniond coarse_q(best.pose.rotation());
    const double overlap = std::clamp(
      static_cast<double>(result.num_inliers) /
      static_cast<double>(scan_points.size()), 0.0, 1.0);
    const double fitness =
      result.error / static_cast<double>(
        std::max<std::size_t>(1, result.num_inliers));
    const double score = std::clamp(
      0.50 * best.bbs_score +
      0.25 * overlap +
      0.25 * std::max(0.0, best.candidate.sector_similarity),
      0.0, 1.0);

    std::cout << "{\"success\":true"
              << ",\"x\":" << T.translation().x()
              << ",\"y\":" << T.translation().y()
              << ",\"z\":" << T.translation().z()
              << ",\"qx\":" << q.x()
              << ",\"qy\":" << q.y()
              << ",\"qz\":" << q.z()
              << ",\"qw\":" << q.w()
              << ",\"coarse_x\":" << best.pose.translation().x()
              << ",\"coarse_y\":" << best.pose.translation().y()
              << ",\"coarse_z\":" << best.pose.translation().z()
              << ",\"coarse_qx\":" << coarse_q.x()
              << ",\"coarse_qy\":" << coarse_q.y()
              << ",\"coarse_qz\":" << coarse_q.z()
              << ",\"coarse_qw\":" << coarse_q.w()
              << ",\"score\":" << score
              << ",\"fitness\":" << fitness
              << ",\"overlap\":" << overlap
              << ",\"bbs_score\":" << best.bbs_score
              << ",\"bbs_elapsed_ms\":" << best.elapsed_ms
              << ",\"candidate_patch\":\"" << best.candidate.entry.patch_name << "\""
              << ",\"descriptor_ring_distance\":" << best.candidate.ring_distance
              << ",\"descriptor_similarity\":" << best.candidate.sector_similarity
              << ",\"descriptor_shift\":" << best.candidate.sector_shift
              << ",\"descriptor_yaw_seed_deg\":"
              << best.candidate.yaw_estimate * 180.0 / M_PI
              << ",\"gicp_target_points\":" << gicp_target.size()
              << ",\"gicp_full_map_fallback\":"
              << (full_map_fallback ? "true" : "false")
              << "}" << std::endl;
    return 0;
  } catch (const std::exception& e) {
    emit_failure(e.what());
    return 1;
  }
}
