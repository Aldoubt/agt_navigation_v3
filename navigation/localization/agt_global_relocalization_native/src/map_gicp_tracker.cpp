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

namespace {
struct Options {
  std::string map, scan;
  double x{0.0}, y{0.0}, z{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
  double radius{20.0}, half_height{5.0}, map_leaf{0.25}, scan_leaf{0.25};
  double max_corr{1.5};
  int threads{4};
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
    else throw std::runtime_error("unknown argument: " + a);
  }
  return !o.map.empty() && !o.scan.empty();
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
    const auto map = downsample(load(o.map), o.map_leaf);
    const auto scan = downsample(load(o.scan), o.scan_leaf);
    const auto map_points = eigen(*map);
    const Eigen::Vector3d center(o.x, o.y, o.z);
    const auto local = crop(map_points, center, o.radius, o.half_height);
    if (local.size() < 1000) throw std::runtime_error("local map has fewer than 1000 points");
    const auto scan_points = eigen(*scan);
    Eigen::Isometry3d initial = Eigen::Isometry3d::Identity();
    initial.translation() = center;
    initial.linear() = Eigen::Quaterniond(o.qw, o.qx, o.qy, o.qz).normalized().toRotationMatrix();
    small_gicp::RegistrationSetting setting;
    setting.num_threads = std::max(1, o.threads);
    setting.downsampling_resolution = std::min(o.map_leaf, o.scan_leaf);
    setting.max_correspondence_distance = o.max_corr;
    const auto result = small_gicp::align(local, scan_points, initial, setting);
    if (!result.converged || result.num_inliers == 0) throw std::runtime_error("GICP did not converge");
    const auto& pose = result.T_target_source;
    const Eigen::Quaterniond q(pose.rotation());
    const double overlap = std::clamp(static_cast<double>(result.num_inliers) /
                                      static_cast<double>(scan_points.size()), 0.0, 1.0);
    const double fitness = result.error / static_cast<double>(std::max<std::size_t>(1, result.num_inliers));
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
