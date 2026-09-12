#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/reduction_omp.hpp>

namespace {
struct Options {
  std::string map, scan;
  double x{0.0}, y{0.0}, z{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
  double radius{20.0}, half_height{5.0}, map_leaf{0.25}, scan_leaf{0.25};
  double max_corr{1.5};
  int threads{4};
  std::string debug_crop;
  std::string constraint_mode{"full_se3"};
  double max_roll_delta_deg{10.0};
  double max_pitch_delta_deg{10.0};
};

std::string next(int& i, int argc, char** argv) {
  if (i + 1 >= argc) throw std::runtime_error("missing argument value");
  return argv[++i];
}

bool parse(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    if (a == "--map") o.map = next(i, argc, argv);
    else if (a == "--scan") o.scan = next(i, argc, argv);
    else if (a == "--x") o.x = std::stod(next(i, argc, argv));
    else if (a == "--y") o.y = std::stod(next(i, argc, argv));
    else if (a == "--z") o.z = std::stod(next(i, argc, argv));
    else if (a == "--qx") o.qx = std::stod(next(i, argc, argv));
    else if (a == "--qy") o.qy = std::stod(next(i, argc, argv));
    else if (a == "--qz") o.qz = std::stod(next(i, argc, argv));
    else if (a == "--qw") o.qw = std::stod(next(i, argc, argv));
    else if (a == "--radius") o.radius = std::stod(next(i, argc, argv));
    else if (a == "--half-height") o.half_height = std::stod(next(i, argc, argv));
    else if (a == "--map-leaf") o.map_leaf = std::stod(next(i, argc, argv));
    else if (a == "--scan-leaf") o.scan_leaf = std::stod(next(i, argc, argv));
    else if (a == "--max-corr") o.max_corr = std::stod(next(i, argc, argv));
    else if (a == "--threads") o.threads = std::stoi(next(i, argc, argv));
    else if (a == "--debug-crop") o.debug_crop = next(i, argc, argv);
    else if (a == "--constraint-mode") o.constraint_mode = next(i, argc, argv);
    else if (a == "--max-roll-delta-deg") o.max_roll_delta_deg = std::stod(next(i, argc, argv));
    else if (a == "--max-pitch-delta-deg") o.max_pitch_delta_deg = std::stod(next(i, argc, argv));
    else throw std::runtime_error("unknown argument: " + a);
  }
  return !o.map.empty() && !o.scan.empty();
}

Eigen::Matrix3d rpy_matrix(double roll, double pitch, double yaw) {
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Vector3d rpy_from_matrix(const Eigen::Matrix3d& r) {
  const double pitch = std::asin(std::clamp(-r(2, 0), -1.0, 1.0));
  const double roll = std::atan2(r(2, 1), r(2, 2));
  const double yaw = std::atan2(r(1, 0), r(0, 0));
  return {roll, pitch, yaw};
}

// Experimental 4-DoF Gauss-Newton optimizer.  It retains the initial map-frame
// Euler roll/pitch exactly and solves for all translations plus map-frame yaw.
// This type is intentionally local to the opt-in experiment; full_se3 continues
// to use small_gicp::align unchanged.
struct GravityConstrainedOptimizer {
  GravityConstrainedOptimizer() = default;
  explicit GravityConstrainedOptimizer(double roll, double pitch)
  : fixed_roll(roll), fixed_pitch(pitch) {}

  template <typename TargetPointCloud, typename SourcePointCloud, typename TargetTree,
            typename CorrespondenceRejector, typename TerminationCriteria,
            typename Reduction, typename Factor, typename GeneralFactor>
  small_gicp::RegistrationResult optimize(
      const TargetPointCloud& target, const SourcePointCloud& source,
      const TargetTree& target_tree, const CorrespondenceRejector& rejector,
      const TerminationCriteria& criteria, Reduction& reduction,
      const Eigen::Isometry3d& init_t, std::vector<Factor>& factors,
      GeneralFactor& general_factor) const {
    small_gicp::RegistrationResult result(init_t);
    result.T_target_source.linear() = rpy_matrix(fixed_roll, fixed_pitch,
                                                  rpy_from_matrix(init_t.rotation()).z());
    for (int i = 0; i < max_iterations && !result.converged; ++i) {
      auto [h, b, e] = reduction.linearize(
          target, source, target_tree, rejector, result.T_target_source, factors);
      general_factor.update_linearized_system(
          target, source, target_tree, result.T_target_source, &h, &b, &e);

      // small_gicp's increment is [rotation_body, translation_body].  Map-frame
      // yaw maps to R^T * z in the body rotation coordinates; all translations
      // remain free.  This is the local tangent basis of the constrained manifold.
      Eigen::Matrix<double, 6, 4> basis = Eigen::Matrix<double, 6, 4>::Zero();
      basis.block<3, 1>(0, 0) = result.T_target_source.rotation().transpose() *
                                Eigen::Vector3d::UnitZ();
      basis.block<3, 3>(3, 1) = Eigen::Matrix3d::Identity();
      const Eigen::Matrix4d reduced_h = basis.transpose() * h * basis;
      const Eigen::Vector4d reduced_b = basis.transpose() * b;
      const Eigen::Vector4d reduced_delta =
          (reduced_h + lambda * Eigen::Matrix4d::Identity()).ldlt().solve(-reduced_b);
      const Eigen::Matrix<double, 6, 1> delta = basis * reduced_delta;

      result.converged = criteria.converged(delta);
      result.T_target_source = result.T_target_source * small_gicp::se3_exp(delta);
      const double yaw = rpy_from_matrix(result.T_target_source.rotation()).z();
      result.T_target_source.linear() = rpy_matrix(fixed_roll, fixed_pitch, yaw);
      result.iterations = i;
      result.H = h;
      result.b = b;
      result.error = e;
    }
    result.num_inliers = std::count_if(
        factors.begin(), factors.end(), [](const auto& factor) { return factor.inlier(); });
    return result;
  }

  double fixed_roll{0.0};
  double fixed_pitch{0.0};
  int max_iterations{20};
  double lambda{1e-6};
};

small_gicp::RegistrationResult gravity_constrained_align(
    const std::vector<Eigen::Vector3d>& target,
    const std::vector<Eigen::Vector3d>& source,
    const Eigen::Isometry3d& initial,
    const small_gicp::RegistrationSetting& setting) {
  auto [target_points, target_tree] = small_gicp::preprocess_points(
      target, setting.downsampling_resolution, 10, setting.num_threads);
  auto [source_points, _source_tree] = small_gicp::preprocess_points(
      source, setting.downsampling_resolution, 10, setting.num_threads);
  const Eigen::Vector3d initial_rpy = rpy_from_matrix(initial.rotation());
  using Registration = small_gicp::Registration<
      small_gicp::GICPFactor, small_gicp::ParallelReductionOMP,
      small_gicp::NullFactor, small_gicp::DistanceRejector, GravityConstrainedOptimizer>;
  Registration registration;
  registration.reduction.num_threads = setting.num_threads;
  registration.rejector.max_dist_sq =
      setting.max_correspondence_distance * setting.max_correspondence_distance;
  registration.criteria.rotation_eps = setting.rotation_eps;
  registration.criteria.translation_eps = setting.translation_eps;
  registration.optimizer.max_iterations = setting.max_iterations;
  registration.optimizer.lambda = 1e-6;
  registration.optimizer.fixed_roll = initial_rpy.x();
  registration.optimizer.fixed_pitch = initial_rpy.y();
  return registration.align(*target_points, *source_points, *target_tree, initial);
}

// Experimental bounded SE(3) optimizer.  Every iteration solves the native
// 6-DoF GICP linear system, then projects only map-frame Euler roll/pitch into
// the configured interval around the initial pose.  Translation and yaw remain
// unconstrained.  This is an opt-in clipping prior, not a change to full_se3.
struct GravityPriorOptimizer {
  template <typename TargetPointCloud, typename SourcePointCloud, typename TargetTree,
            typename CorrespondenceRejector, typename TerminationCriteria,
            typename Reduction, typename Factor, typename GeneralFactor>
  small_gicp::RegistrationResult optimize(
      const TargetPointCloud& target, const SourcePointCloud& source,
      const TargetTree& target_tree, const CorrespondenceRejector& rejector,
      const TerminationCriteria& criteria, Reduction& reduction,
      const Eigen::Isometry3d& init_t, std::vector<Factor>& factors,
      GeneralFactor& general_factor) const {
    small_gicp::RegistrationResult result(init_t);
    for (int i = 0; i < max_iterations && !result.converged; ++i) {
      auto [h, b, e] = reduction.linearize(
          target, source, target_tree, rejector, result.T_target_source, factors);
      general_factor.update_linearized_system(
          target, source, target_tree, result.T_target_source, &h, &b, &e);
      const Eigen::Matrix<double, 6, 1> delta =
          (h + lambda * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(-b);
      result.converged = criteria.converged(delta);
      result.T_target_source = result.T_target_source * small_gicp::se3_exp(delta);
      Eigen::Vector3d rpy = rpy_from_matrix(result.T_target_source.rotation());
      rpy.x() = std::clamp(rpy.x(), initial_roll - max_roll_delta,
                           initial_roll + max_roll_delta);
      rpy.y() = std::clamp(rpy.y(), initial_pitch - max_pitch_delta,
                           initial_pitch + max_pitch_delta);
      result.T_target_source.linear() = rpy_matrix(rpy.x(), rpy.y(), rpy.z());
      result.iterations = i;
      result.H = h;
      result.b = b;
      result.error = e;
    }
    result.num_inliers = std::count_if(
        factors.begin(), factors.end(), [](const auto& factor) { return factor.inlier(); });
    return result;
  }

  double initial_roll{0.0};
  double initial_pitch{0.0};
  double max_roll_delta{0.0};
  double max_pitch_delta{0.0};
  int max_iterations{20};
  double lambda{1e-6};
};

small_gicp::RegistrationResult gravity_prior_align(
    const std::vector<Eigen::Vector3d>& target,
    const std::vector<Eigen::Vector3d>& source,
    const Eigen::Isometry3d& initial,
    const small_gicp::RegistrationSetting& setting,
    double max_roll_delta, double max_pitch_delta) {
  auto [target_points, target_tree] = small_gicp::preprocess_points(
      target, setting.downsampling_resolution, 10, setting.num_threads);
  auto [source_points, _source_tree] = small_gicp::preprocess_points(
      source, setting.downsampling_resolution, 10, setting.num_threads);
  const Eigen::Vector3d initial_rpy = rpy_from_matrix(initial.rotation());
  using Registration = small_gicp::Registration<
      small_gicp::GICPFactor, small_gicp::ParallelReductionOMP,
      small_gicp::NullFactor, small_gicp::DistanceRejector, GravityPriorOptimizer>;
  Registration registration;
  registration.reduction.num_threads = setting.num_threads;
  registration.rejector.max_dist_sq =
      setting.max_correspondence_distance * setting.max_correspondence_distance;
  registration.criteria.rotation_eps = setting.rotation_eps;
  registration.criteria.translation_eps = setting.translation_eps;
  registration.optimizer.max_iterations = setting.max_iterations;
  registration.optimizer.initial_roll = initial_rpy.x();
  registration.optimizer.initial_pitch = initial_rpy.y();
  registration.optimizer.max_roll_delta = max_roll_delta;
  registration.optimizer.max_pitch_delta = max_pitch_delta;
  return registration.align(*target_points, *source_points, *target_tree, initial);
}

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

Cloud::Ptr load(const std::string& path) {
  auto cloud = pcl::make_shared<Cloud>();
  if (pcl::io::loadPCDFile(path, *cloud) != 0 || cloud->empty()) {
    throw std::runtime_error("failed to load non-empty PCD: " + path);
  }
  return cloud;
}

Cloud::Ptr downsample(const Cloud::ConstPtr& input, double leaf) {
  auto out = pcl::make_shared<Cloud>();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
  filter.setInputCloud(input);
  filter.filter(*out);
  if (out->empty()) throw std::runtime_error("empty cloud after downsample");
  return out;
}

std::vector<Eigen::Vector3d> eigen(const Cloud& cloud) {
  std::vector<Eigen::Vector3d> out;
  out.reserve(cloud.size());
  for (const auto& p : cloud) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      out.emplace_back(p.x, p.y, p.z);
    }
  }
  return out;
}

std::vector<Eigen::Vector3d> crop(const std::vector<Eigen::Vector3d>& points,
                                  const Eigen::Vector3d& center,
                                  double radius, double half_height) {
  const double r2 = radius * radius;
  std::vector<Eigen::Vector3d> out;
  for (const auto& p : points) {
    const double dx = p.x() - center.x();
    const double dy = p.y() - center.y();
    if (dx * dx + dy * dy <= r2 && std::abs(p.z() - center.z()) <= half_height) out.push_back(p);
  }
  return out;
}

void fail(const std::string& message) {
  std::cout << "{\"success\":false,\"message\":\"" << message << "\"}" << std::endl;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    Options o;
    if (!parse(argc, argv, o)) throw std::runtime_error("--map and --scan are required");
    if (o.constraint_mode != "full_se3" && o.constraint_mode != "gravity_constrained" &&
        o.constraint_mode != "gravity_prior") {
      throw std::runtime_error(
          "--constraint-mode must be full_se3, gravity_constrained, or gravity_prior");
    }
    if (o.max_roll_delta_deg < 0.0 || o.max_pitch_delta_deg < 0.0) {
      throw std::runtime_error("gravity-prior roll/pitch limits must be non-negative");
    }
    const auto map = downsample(load(o.map), o.map_leaf);
    const auto scan = downsample(load(o.scan), o.scan_leaf);
    const auto map_points = eigen(*map);
    const Eigen::Vector3d center(o.x, o.y, o.z);
    const auto local = crop(map_points, center, o.radius, o.half_height);
    if (local.size() < 1000) throw std::runtime_error("local map has fewer than 1000 points");
    if (!o.debug_crop.empty()) {
      Cloud debug_crop;
      debug_crop.reserve(local.size());
      for (const auto& point : local) {
        debug_crop.emplace_back(static_cast<float>(point.x()), static_cast<float>(point.y()),
                                static_cast<float>(point.z()));
      }
      if (pcl::io::savePCDFileBinary(o.debug_crop, debug_crop) != 0) {
        throw std::runtime_error("failed to write debug local-map crop");
      }
    }
    const auto scan_points = eigen(*scan);
    Eigen::Isometry3d initial = Eigen::Isometry3d::Identity();
    initial.translation() = center;
    initial.linear() = Eigen::Quaterniond(o.qw, o.qx, o.qy, o.qz).normalized().toRotationMatrix();
    small_gicp::RegistrationSetting setting;
    setting.num_threads = std::max(1, o.threads);
    setting.downsampling_resolution = std::min(o.map_leaf, o.scan_leaf);
    setting.max_correspondence_distance = o.max_corr;
    const auto result = o.constraint_mode == "gravity_constrained"
      ? gravity_constrained_align(local, scan_points, initial, setting)
      : o.constraint_mode == "gravity_prior"
        ? gravity_prior_align(local, scan_points, initial, setting,
            o.max_roll_delta_deg * M_PI / 180.0, o.max_pitch_delta_deg * M_PI / 180.0)
        : small_gicp::align(local, scan_points, initial, setting);
    const auto& pose = result.T_target_source;
    const Eigen::Vector3d initial_rpy = rpy_from_matrix(initial.rotation());
    const Eigen::Vector3d refined_rpy = rpy_from_matrix(pose.rotation());
    const Eigen::Quaterniond q(pose.rotation());
    const double overlap = std::clamp(static_cast<double>(result.num_inliers) /
                                      static_cast<double>(scan_points.size()), 0.0, 1.0);
    const double fitness = result.error / static_cast<double>(std::max<std::size_t>(1, result.num_inliers));
    const bool prior_clipped = (o.constraint_mode == "gravity_prior") &&
      (std::abs(refined_rpy.x() - initial_rpy.x()) >=
          o.max_roll_delta_deg * M_PI / 180.0 - 1e-9 ||
       std::abs(refined_rpy.y() - initial_rpy.y()) >=
          o.max_pitch_delta_deg * M_PI / 180.0 - 1e-9);
    if (!result.converged || result.num_inliers == 0) {
      std::cout << "{\"success\":false,\"message\":\"GICP did not converge\""
                << ",\"constraint_mode\":\"" << o.constraint_mode << "\""
                << ",\"x\":" << pose.translation().x() << ",\"y\":" << pose.translation().y()
                << ",\"z\":" << pose.translation().z()
                << ",\"qx\":" << q.x() << ",\"qy\":" << q.y()
                << ",\"qz\":" << q.z() << ",\"qw\":" << q.w()
                << ",\"fitness\":" << fitness << ",\"overlap\":" << overlap
                << ",\"query_points\":" << scan_points.size()
                << ",\"map_points\":" << local.size()
                << ",\"roll_delta_deg\":" << (refined_rpy.x() - initial_rpy.x()) * 180.0 / M_PI
                << ",\"pitch_delta_deg\":" << (refined_rpy.y() - initial_rpy.y()) * 180.0 / M_PI
                << ",\"gravity_prior_clipped\":" << (prior_clipped ? "true" : "false")
                << "}" << std::endl;
      return 1;
    }
    // Record local observability rather than treating convergence as proof that
    // the pose is correct. The raw 6-DoF Hessian mixes rotational/translational
    // units, so this is a diagnostic signal first; field rejection stays
    // configurable in agt_map_tracker after replay distributions are measured.
    const Eigen::Matrix<double, 6, 6> hessian =
      0.5 * (result.H.template cast<double>() + result.H.transpose().template cast<double>());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(hessian);
    Eigen::Matrix<double, 6, 1> hessian_eigenvalues =
      Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN());
    double hessian_condition_number = 1.0e30;
    bool hessian_degenerate = true;
    if (eig.info() == Eigen::Success) {
      hessian_eigenvalues = eig.eigenvalues();
      const double min_eig = hessian_eigenvalues.minCoeff();
      const double max_eig = hessian_eigenvalues.maxCoeff();
      if (std::isfinite(min_eig) && std::isfinite(max_eig) && min_eig > 1.0e-12) {
        hessian_condition_number = max_eig / min_eig;
        if (!std::isfinite(hessian_condition_number)) hessian_condition_number = 1.0e30;
        hessian_degenerate = hessian_condition_number > 1.0e10;
      }
    }
    std::cout << "{\"success\":true"
              << ",\"constraint_mode\":\"" << o.constraint_mode << "\""
              << ",\"max_roll_delta_deg\":" << o.max_roll_delta_deg
              << ",\"max_pitch_delta_deg\":" << o.max_pitch_delta_deg
              << ",\"roll_delta_deg\":" << (refined_rpy.x() - initial_rpy.x()) * 180.0 / M_PI
              << ",\"pitch_delta_deg\":" << (refined_rpy.y() - initial_rpy.y()) * 180.0 / M_PI
              << ",\"gravity_prior_clipped\":" << (prior_clipped ? "true" : "false")
              << ",\"x\":" << pose.translation().x() << ",\"y\":" << pose.translation().y()
              << ",\"z\":" << pose.translation().z()
              << ",\"qx\":" << q.x() << ",\"qy\":" << q.y()
              << ",\"qz\":" << q.z() << ",\"qw\":" << q.w()
              << ",\"fitness\":" << fitness << ",\"overlap\":" << overlap
              << ",\"hessian_eigenvalues\":["
              << hessian_eigenvalues(0) << "," << hessian_eigenvalues(1) << ","
              << hessian_eigenvalues(2) << "," << hessian_eigenvalues(3) << ","
              << hessian_eigenvalues(4) << "," << hessian_eigenvalues(5) << "]"
              << ",\"hessian_condition_number\":" << hessian_condition_number
              << ",\"hessian_degenerate\":" << (hessian_degenerate ? "true" : "false")
              << ",\"map_points\":" << local.size() << ",\"query_points\":" << scan_points.size()
              << "}" << std::endl;
    return 0;
  } catch (const std::exception& e) {
    fail(e.what());
    return 1;
  }
}
