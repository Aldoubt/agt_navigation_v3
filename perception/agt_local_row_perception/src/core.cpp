#include "agt_local_row_perception/core.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <map>
#include <random>
#include <set>
#include <stdexcept>

namespace agt_local_row_perception
{
namespace
{
constexpr double pi = 3.14159265358979323846;
double clamp01(double v) {return std::clamp(v, 0.0, 1.0);}
double angle_difference(double a, double b) {return std::atan2(std::sin(a - b), std::cos(a - b));}
double quantile(std::vector<double> values, double fraction)
{
  if (values.empty()) {return 0.0;}
  const auto index = static_cast<std::size_t>(fraction * (values.size() - 1));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}
bool inside_polygon(const Eigen::Vector2d & p, const std::vector<Eigen::Vector2d> & polygon)
{
  bool inside = false;
  if (polygon.size() < 3) {return false;}
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & a = polygon[i];
    const auto & b = polygon[j];
    if ((a.y() > p.y()) != (b.y() > p.y()) &&
      p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x())
    {inside = !inside;}
  }
  return inside;
}
Eigen::Isometry3d horizontal_pose(const Eigen::Isometry3d & input)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = input.translation();
  const double yaw = std::atan2(input.linear()(1, 0), input.linear()(0, 0));
  result.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return result;
}
struct Ground
{
  bool valid{false};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  double offset{0.0};
};
Ground fit_ground(const std::vector<Eigen::Vector3d> & points, const Config & cfg)
{
  // One low representative per XY cell prevents dense foliage from winning by point count.
  std::map<std::pair<int, int>, std::vector<double>> heights;
  for (const auto & p : points) {
    if (p.z() < cfg.ground_z_min_m || p.z() > cfg.ground_z_max_m) {continue;}
    heights[{static_cast<int>(std::floor(p.x() / cfg.grid_resolution_m)),
      static_cast<int>(std::floor(p.y() / cfg.grid_resolution_m))}].push_back(p.z());
  }
  std::vector<Eigen::Vector3d> seeds;
  for (auto & item : heights) {
    seeds.emplace_back((item.first.first + 0.5) * cfg.grid_resolution_m,
      (item.first.second + 0.5) * cfg.grid_resolution_m,
      quantile(item.second, cfg.ground_seed_percentile));
  }
  Ground ground;
  if (seeds.size() < cfg.ground_min_cells) {return ground;}
  std::mt19937 generator(1403);
  std::uniform_int_distribution<std::size_t> pick(0, seeds.size() - 1);
  const double min_nz = std::cos(cfg.ground_max_tilt_deg * pi / 180.0);
  std::vector<std::size_t> best;
  for (unsigned iteration = 0; iteration < cfg.ground_ransac_iterations; ++iteration) {
    const auto i = pick(generator), j = pick(generator), k = pick(generator);
    Eigen::Vector3d n = (seeds[j] - seeds[i]).cross(seeds[k] - seeds[i]);
    if (n.norm() < 1e-6) {continue;}
    n.normalize();
    if (n.z() < 0) {n = -n;}
    if (n.z() < min_nz) {continue;}
    const double d = -n.dot(seeds[i]);
    std::vector<std::size_t> inliers;
    for (std::size_t s = 0; s < seeds.size(); ++s) {
      if (std::abs(n.dot(seeds[s]) + d) <= cfg.ground_tolerance_m) {inliers.push_back(s);}
    }
    if (inliers.size() > best.size()) {best = std::move(inliers);}
  }
  if (best.size() < cfg.ground_min_cells) {return ground;}
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Vector2d lo = seeds[best.front()].head<2>();
  Eigen::Vector2d hi = lo;
  for (const auto i : best) {
    mean += seeds[i];
    lo = lo.cwiseMin(seeds[i].head<2>()); hi = hi.cwiseMax(seeds[i].head<2>());
  }
  mean /= static_cast<double>(best.size());
  if ((hi - lo).minCoeff() < cfg.ground_min_xy_span_m) {return ground;}
  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  for (const auto i : best) {const auto delta = (seeds[i] - mean).eval(); scatter += delta * delta.transpose();}
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
  if (solver.info() != Eigen::Success || solver.eigenvalues()(1) <= 1e-6) {return ground;}
  ground.normal = solver.eigenvectors().col(0);
  if (ground.normal.z() < 0) {ground.normal = -ground.normal;}
  ground.offset = -ground.normal.dot(mean);
  double residual = 0.0;
  for (const auto i : best) {residual += std::pow(ground.normal.dot(seeds[i]) + ground.offset, 2);}
  ground.valid = ground.normal.z() >= min_nz &&
    std::sqrt(residual / best.size()) <= cfg.ground_max_residual_m &&
    mean.z() >= cfg.ground_z_min_m && mean.z() <= cfg.ground_z_max_m;
  return ground;
}
struct Sample {double x, y, weight; bool left; unsigned frames;};
struct Fit
{
  bool valid{false};
  Eigen::Vector3d theta{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Zero()};
  unsigned left{0}, right{0}, frames{0};
  double span{0.0}, x_low{0.0}, x_high{0.0};
};
Fit fit_row(const std::vector<Sample> & samples, const Config & cfg)
{
  Fit fit;
  std::vector<std::size_t> left, right;
  for (std::size_t i = 0; i < samples.size(); ++i) {
    (samples[i].left ? left : right).push_back(i);
  }
  if (left.size() < cfg.fit_min_side_cells || right.size() < cfg.fit_min_side_cells) {return fit;}
  std::mt19937 generator(3719);
  Eigen::Vector3d best_theta = Eigen::Vector3d::Zero();
  double best_score = -1.0;
  for (unsigned iteration = 0; iteration < cfg.fit_ransac_iterations; ++iteration) {
    const bool use_left = iteration % 2 == 0;
    const auto & first = use_left ? left : right;
    const auto & other = use_left ? right : left;
    std::uniform_int_distribution<std::size_t> pick_first(0, first.size() - 1);
    std::uniform_int_distribution<std::size_t> pick_other(0, other.size() - 1);
    const auto & p = samples[first[pick_first(generator)]];
    const auto & q = samples[first[pick_first(generator)]];
    const auto & r = samples[other[pick_other(generator)]];
    if (std::abs(p.x - q.x) < cfg.fit_min_span_m / 3.0) {continue;}
    const double a = (q.y - p.y) / (q.x - p.x);
    if (std::abs(std::atan(a)) > cfg.max_heading_rad) {continue;}
    const double b1 = p.y - a * p.x, b2 = r.y - a * r.x;
    Eigen::Vector3d theta(a, use_left ? b1 : b2, use_left ? b2 : b1);
    const double width = (theta.y() - theta.z()) / std::sqrt(1 + a * a);
    if (width < cfg.min_width_m || width > cfg.max_width_m) {continue;}
    double lscore = 0.0, rscore = 0.0;
    for (const auto & sample : samples) {
      const double distance = std::abs(sample.y - a * sample.x -
        (sample.left ? theta.y() : theta.z())) / std::sqrt(1 + a * a);
      if (distance <= cfg.fit_inlier_distance_m) {
        (sample.left ? lscore : rscore) += sample.weight;
      }
    }
    const double score = std::min(lscore, rscore) + 0.1 * (lscore + rscore);
    if (score > best_score) {best_score = score; best_theta = theta;}
  }
  if (best_score < 0) {return fit;}
  // IRLS on cell representatives; repeated returns never become independent samples.
  Eigen::Vector3d theta = best_theta;
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
  const double scale_x = std::max(1.0, cfg.fit_x_max_m - cfg.fit_x_min_m);
  Eigen::Matrix3d column_scale = Eigen::Matrix3d::Identity();
  column_scale(0, 0) = 1.0 / scale_x;
  for (unsigned iteration = 0; iteration < 10; ++iteration) {
    hessian.setZero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (const auto & sample : samples) {
      const Eigen::Vector3d j(sample.x, sample.left ? 1.0 : 0.0, sample.left ? 0.0 : 1.0);
      const double residual = sample.y - j.dot(theta);
      if (std::abs(residual) / std::sqrt(1 + theta.x() * theta.x()) >
        2.0 * cfg.fit_inlier_distance_m) {continue;}
      const double robust = std::min(1.0, cfg.fit_huber_delta_m / std::max(1e-9, std::abs(residual)));
      const double w = sample.weight * robust;
      hessian += w * j * j.transpose(); rhs += w * j * sample.y;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(column_scale * hessian * column_scale);
    if (eigen.info() != Eigen::Success || eigen.eigenvalues().minCoeff() <= 1e-8 ||
      eigen.eigenvalues().maxCoeff() / eigen.eigenvalues().minCoeff() > cfg.covariance_condition_max)
    {return fit;}
    const Eigen::Vector3d update = hessian.ldlt().solve(rhs);
    if (!update.allFinite()) {return fit;}
    const double change = (update - theta).norm();
    theta = update;
    if (change < 1e-6) {break;}
  }
  double sum_squared = 0.0, sum_weight = 0.0;
  std::vector<double> left_frames, right_frames;
  double lx_min = cfg.fit_x_max_m, rx_min = lx_min;
  double lx_max = cfg.fit_x_min_m, rx_max = lx_max;
  hessian.setZero();
  for (const auto & sample : samples) {
    const Eigen::Vector3d j(sample.x, sample.left ? 1.0 : 0.0, sample.left ? 0.0 : 1.0);
    const double residual = sample.y - j.dot(theta);
    if (std::abs(residual) / std::sqrt(1 + theta.x() * theta.x()) > cfg.fit_inlier_distance_m) {continue;}
    if (sample.left) {++fit.left; lx_min = std::min(lx_min, sample.x); lx_max = std::max(lx_max, sample.x);}
    else {++fit.right; rx_min = std::min(rx_min, sample.x); rx_max = std::max(rx_max, sample.x);}
    (sample.left ? left_frames : right_frames).push_back(sample.frames);
    sum_squared += sample.weight * residual * residual;
    sum_weight += sample.weight;
    hessian += sample.weight * j * j.transpose();
  }
  fit.x_low = std::max(lx_min, rx_min);
  fit.x_high = std::min(lx_max, rx_max);
  fit.span = fit.x_high - fit.x_low;
  fit.frames = static_cast<unsigned>(std::min(quantile(left_frames, 0.5), quantile(right_frames, 0.5)));
  const double width = (theta.y() - theta.z()) / std::sqrt(1 + theta.x() * theta.x());
  if (fit.left < cfg.fit_min_side_cells || fit.right < cfg.fit_min_side_cells ||
    fit.span < cfg.fit_min_span_m || width < cfg.min_width_m || width > cfg.max_width_m ||
    std::abs(std::atan(theta.x())) > cfg.max_heading_rad ||
    hessian.fullPivLu().rank() < 3) {return fit;}
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> final_eigen(column_scale * hessian * column_scale);
  if (final_eigen.info() != Eigen::Success || final_eigen.eigenvalues().minCoeff() <= 1e-8 ||
    final_eigen.eigenvalues().maxCoeff() / final_eigen.eigenvalues().minCoeff() > cfg.covariance_condition_max)
  {return fit;}
  const double variance = std::max(cfg.covariance_variance_floor_m2,
    sum_squared / std::max(1.0, sum_weight - 3.0));
  fit.theta = theta;
  fit.covariance = variance * hessian.inverse();
  fit.valid = fit.covariance.allFinite();
  return fit;
}
}  // namespace

PoseHistory::PoseHistory(double window_sec, double max_gap_sec)
: window_sec_(window_sec), max_gap_sec_(max_gap_sec) {}
void PoseHistory::reset(std::uint64_t epoch) {poses_.clear(); epoch_ = epoch;}
bool PoseHistory::insert(const Pose & pose)
{
  if (!(pose.stamp > 0) || !std::isfinite(pose.stamp) ||
    !pose.odom_from_body.matrix().allFinite()) {return false;}
  if (poses_.empty()) {epoch_ = pose.epoch;}
  if (pose.epoch != epoch_) {reset(pose.epoch);}
  if (!poses_.empty() && pose.stamp <= poses_.back().stamp) {return false;}
  poses_.push_back(pose);
  while (poses_.size() > 2 && poses_.front().stamp < pose.stamp - window_sec_) {poses_.pop_front();}
  return true;
}
std::optional<Pose> PoseHistory::at(double stamp, std::uint64_t epoch) const
{
  if (poses_.empty() || epoch != epoch_ || !std::isfinite(stamp) ||
    stamp < poses_.front().stamp - 1e-8 || stamp > poses_.back().stamp + 1e-8) {return std::nullopt;}
  const auto upper = std::lower_bound(poses_.begin(), poses_.end(), stamp,
    [](const Pose & pose, double time) {return pose.stamp < time;});
  if (upper == poses_.end()) {return poses_.back();}
  if (std::abs(upper->stamp - stamp) < 1e-8) {return *upper;}
  if (upper == poses_.begin()) {return std::nullopt;}
  const auto lower = std::prev(upper);
  const double dt = upper->stamp - lower->stamp;
  if (dt <= 0 || dt > max_gap_sec_) {return std::nullopt;}
  const double fraction = (stamp - lower->stamp) / dt;
  Pose result;
  result.stamp = stamp; result.epoch = epoch;
  result.odom_from_body.translation() = (1 - fraction) * lower->odom_from_body.translation() +
    fraction * upper->odom_from_body.translation();
  const Eigen::Quaterniond q0(lower->odom_from_body.linear()), q1(upper->odom_from_body.linear());
  result.odom_from_body.linear() = q0.slerp(fraction, q1).normalized().toRotationMatrix();
  return result;
}
double PoseHistory::latest_stamp() const {return poses_.empty() ? 0.0 : poses_.back().stamp;}

bool Shape::contains(const Eigen::Vector3d & body_point) const
{
  const Eigen::Vector3d p = body_from_shape.inverse() * body_point;
  if (type == Type::Box) {return (p.cwiseAbs().array() <= (size.array() / 2.0 + padding)).all();}
  return p.head<2>().squaredNorm() <= std::pow(radius + padding, 2) &&
    std::abs(p.z()) <= height / 2.0 + padding;
}
bool Shape::blocks_ray(const Eigen::Vector3d & origin, const Eigen::Vector3d & end) const
{
  const auto inverse = body_from_shape.inverse();
  const Eigen::Vector3d o = inverse * origin, delta = inverse.linear() * (end - origin);
  double low = 0.0, high = 1.0;
  const Eigen::Vector3d extent = type == Type::Box ?
    (size / 2.0 + Eigen::Vector3d::Constant(padding)).eval() :
    Eigen::Vector3d(radius + padding, radius + padding, height / 2.0 + padding);
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(delta(axis)) < 1e-10) {
      if (std::abs(o(axis)) > extent(axis)) {return false;}
    } else {
      double a = (-extent(axis) - o(axis)) / delta(axis);
      double b = (extent(axis) - o(axis)) / delta(axis);
      if (a > b) {std::swap(a, b);}
      low = std::max(low, a); high = std::min(high, b);
      if (low > high) {return false;}
    }
  }
  if (type == Type::Cylinder) {
    const double a = delta.head<2>().squaredNorm();
    const double b = 2 * o.head<2>().dot(delta.head<2>());
    const double c = o.head<2>().squaredNorm() - std::pow(radius + padding, 2);
    if (a < 1e-12) {if (c > 0) {return false;}}
    else {
      const double discriminant = b * b - 4 * a * c;
      if (discriminant < 0) {return false;}
      low = std::max(low, (-b - std::sqrt(discriminant)) / (2 * a));
      high = std::min(high, (-b + std::sqrt(discriminant)) / (2 * a));
    }
  }
  return high >= low && high > 1e-5 && low < 1 - 1e-5;
}

void Config::validate() const
{
  const std::vector<double> finite{x_min_m, x_max_m, fit_x_min_m, fit_x_max_m,
    min_range_m, min_width_m, max_width_m, side_min_abs_y_m, ground_seed_percentile,
    ground_z_min_m, ground_z_max_m, ground_max_tilt_deg, boundary_min_height_m,
    boundary_max_height_m, obstacle_min_height_m, obstacle_max_height_m,
    low_coverage_height_m, quality_min, clearance_min_coverage, covariance_condition_max,
    max_heading_rad};
  for (const auto value : finite) {
    if (!std::isfinite(value)) {throw std::invalid_argument("finite configuration required");}
  }
  const std::vector<double> positive{pose_history_sec, pose_max_gap_sec, temporal_window_sec,
    evidence_tau_sec, max_range_m, grid_resolution_m, y_abs_max_m, ground_tolerance_m,
    ground_max_residual_m, ground_min_xy_span_m, fit_inlier_distance_m, fit_huber_delta_m,
    fit_min_span_m, uncertainty_lateral_scale_m, uncertainty_heading_scale_rad,
    temporal_lateral_scale_m, temporal_heading_scale_rad, temporal_width_scale_m,
    visibility_max_age_sec, clearance_lookahead_m, path_spacing_m, covariance_variance_floor_m2};
  for (const auto value : positive) {
    if (!(value > 0) || !std::isfinite(value)) {throw std::invalid_argument("positive finite configuration required");}
  }
  if (max_frames == 0 || min_frame_support == 0 || ground_min_cells < 3 ||
    fit_min_side_cells < 3 || ground_ransac_iterations == 0 || fit_ransac_iterations == 0 ||
    x_max_m <= x_min_m || fit_x_max_m <= fit_x_min_m || fit_x_min_m < x_min_m ||
    fit_x_max_m > x_max_m || side_min_abs_y_m < 0 || side_min_abs_y_m >= y_abs_max_m || min_width_m <= 0 ||
    max_width_m <= min_width_m || boundary_max_height_m <= boundary_min_height_m ||
    obstacle_max_height_m <= obstacle_min_height_m || obstacle_min_height_m <= 0 ||
    low_coverage_height_m < obstacle_min_height_m || low_coverage_height_m >= obstacle_max_height_m || min_range_m < 0 ||
    max_range_m <= min_range_m || ground_seed_percentile < 0 || ground_seed_percentile > 0.5 ||
    ground_z_min_m >= ground_z_max_m || ground_max_tilt_deg <= 0 || ground_max_tilt_deg >= 45 ||
    quality_min < 0 || quality_min > 1 || clearance_min_coverage <= 0 || clearance_min_coverage > 1 ||
    covariance_condition_max <= 1 || max_heading_rad <= 0 || max_heading_rad >= pi / 2)
  {throw std::invalid_argument("invalid row configuration bounds");}
  const double cells = std::ceil((x_max_m - x_min_m) / grid_resolution_m) *
    std::ceil(2 * y_abs_max_m / grid_resolution_m);
  if (!std::isfinite(cells) || cells > 2000000) {throw std::invalid_argument("BEV grid exceeds cell budget");}
}

RowPerception::RowPerception(Config config, Geometry geometry)
: config_(std::move(config)), geometry_(std::move(geometry)) {config_.validate();}
void RowPerception::reset(std::uint64_t epoch)
{
  epoch_ = epoch; frames_.clear(); previous_.reset();
  last_stamp_ = -std::numeric_limits<double>::infinity();
}
RowState RowPerception::process(const Scan & scan, const PoseHistory & poses, double speed_mps)
{
  RowState result;
  result.stamp = scan.stamp; result.odom_epoch = scan.epoch;
  result.points_input = scan.points.size();
  if (scan.epoch != epoch_) {reset(scan.epoch);}
  if (!(scan.stamp > 0) || !std::isfinite(scan.stamp) || scan.stamp <= last_stamp_) {
    result.reasons.push_back("nonmonotonic_scan_stamp"); return result;
  }
  if (!scan.body_from_sensor.matrix().allFinite()) {
    result.reasons.push_back("sensor_extrinsic_nonfinite"); return result;
  }
  const auto reference = poses.at(scan.stamp, scan.epoch);
  if (!reference) {result.reasons.push_back("reference_pose_unavailable"); return result;}
  result.odom_from_carrier = horizontal_pose(reference->odom_from_body);
  const Eigen::Isometry3d carrier_from_odom = result.odom_from_carrier.inverse();
  Frame frame{next_frame_id_++, scan.stamp, {}};
  frame.returns.reserve(scan.points.size());
  const Eigen::Vector3d origin_body = scan.body_from_sensor.translation();
  for (const auto & point : scan.points) {
    if (!point.xyz.allFinite()) {continue;}
    if (!std::isfinite(point.stamp) || point.stamp <= 0 || point.stamp > scan.stamp + 1e-8) {
      ++result.points_missing_pose; continue;
    }
    const double range = point.xyz.norm();
    if (range < config_.min_range_m || range > config_.max_range_m) {continue;}
    const auto pose = poses.at(point.stamp, scan.epoch);
    if (!pose) {++result.points_missing_pose; continue;}
    const Eigen::Vector3d point_body = scan.body_from_sensor * point.xyz;
    bool self = false, blocked = false;
    for (const auto & shape : geometry_.self_shapes) {
      self = self || shape.contains(point_body);
      blocked = blocked || shape.blocks_ray(origin_body, point_body);
    }
    if (self) {++result.points_self;} else if (blocked) {++result.points_occluded;}
    frame.returns.push_back({pose->odom_from_body * point_body,
      pose->odom_from_body * origin_body, point.stamp, self, blocked});
    if (!self && !blocked) {++result.points_used;}
  }
  if (result.points_missing_pose > 0) {result.reasons.push_back("some_point_poses_unavailable");}
  // Partial undeskewed scans cannot authorize row navigation.
  const bool pose_complete = result.points_missing_pose == 0;
  frames_.push_back(std::move(frame));
  last_stamp_ = scan.stamp;
  while (!frames_.empty() && (frames_.front().stamp < scan.stamp - config_.temporal_window_sec ||
    frames_.size() > config_.max_frames)) {frames_.pop_front();}
  std::vector<Eigen::Vector3d> points;
  for (const auto & retained : frames_) {
    for (const auto & item : retained.returns) {
      if (item.self || item.occluded) {continue;}
      if (scan.stamp - item.stamp > config_.visibility_max_age_sec) {continue;}
      const Eigen::Vector3d p = carrier_from_odom * item.odom_point;
      if (p.x() >= config_.x_min_m && p.x() < config_.x_max_m &&
        std::abs(p.y()) < config_.y_abs_max_m) {points.push_back(p);}
    }
  }
  const Ground ground = fit_ground(points, config_);
  result.ground_valid = ground.valid;
  result.ground_normal = ground.normal; result.ground_offset = ground.offset;
  if (!ground.valid) {result.reasons.push_back("ground_unobservable");}
  result.grid_width = static_cast<unsigned>(std::ceil((config_.x_max_m - config_.x_min_m) / config_.grid_resolution_m));
  result.grid_height = static_cast<unsigned>(std::ceil(2 * config_.y_abs_max_m / config_.grid_resolution_m));
  result.cells.resize(static_cast<std::size_t>(result.grid_width) * result.grid_height);
  const auto index_of = [&](const Eigen::Vector2d & p) -> int {
      const int x = static_cast<int>(std::floor((p.x() - config_.x_min_m) / config_.grid_resolution_m));
      const int y = static_cast<int>(std::floor((p.y() + config_.y_abs_max_m) / config_.grid_resolution_m));
      if (x < 0 || y < 0 || x >= static_cast<int>(result.grid_width) || y >= static_cast<int>(result.grid_height)) {return -1;}
      return y * static_cast<int>(result.grid_width) + x;
    };
  std::vector<std::set<std::uint64_t>> support(result.cells.size());
  std::vector<bool> obstacles(result.cells.size(), false), blocked(result.cells.size(), false);
  for (unsigned y = 0; y < result.grid_height; ++y) {
    for (unsigned x = 0; x < result.grid_width; ++x) {
      auto & cell = result.cells[y * result.grid_width + x];
      cell.center = {config_.x_min_m + (x + 0.5) * config_.grid_resolution_m,
        -config_.y_abs_max_m + (y + 0.5) * config_.grid_resolution_m};
      const Eigen::Vector3d base_point = reference->odom_from_body.inverse() *
        (result.odom_from_carrier * Eigen::Vector3d(cell.center.x(), cell.center.y(), 0));
      if (inside_polygon(base_point.head<2>(), geometry_.footprint)) {cell.classification = CellClass::Robot;}
      else if (ground.valid) {
        const double ground_z = -(ground.normal.x() * cell.center.x() +
          ground.normal.y() * cell.center.y() + ground.offset) / ground.normal.z();
        const Eigen::Vector3d target_body = reference->odom_from_body.inverse() *
          (result.odom_from_carrier * Eigen::Vector3d(cell.center.x(), cell.center.y(),
          ground_z + config_.low_coverage_height_m));
        for (const auto & shape : geometry_.self_shapes) {
          if (shape.blocks_ray(origin_body, target_body)) {blocked[y * result.grid_width + x] = true; break;}
        }
      }
    }
  }
  for (const auto & retained : frames_) {
    std::set<int> frame_boundary_cells;
    for (const auto & item : retained.returns) {
      const Eigen::Vector3d p = carrier_from_odom * item.odom_point;
      const Eigen::Vector3d origin = carrier_from_odom * item.odom_origin;
      const int index = index_of(p.head<2>());
      if (item.self || item.occluded) {
        // Body-shadow diagnostic follows the actual ray; it never becomes free evidence.
        if (index >= 0 && !item.self) {blocked[index] = true;}
        continue;
      }
      const double h = ground.normal.dot(p) + ground.offset;
      if (index >= 0 && ground.valid) {
        auto & cell = result.cells[index];
        cell.min_height_m = std::min(cell.min_height_m, h);
        cell.max_height_m = std::max(cell.max_height_m, h);
        cell.last_stamp = std::max(cell.last_stamp, item.stamp);
        if (std::abs(h) <= config_.ground_tolerance_m) {
          cell.last_ground_stamp = std::max(cell.last_ground_stamp, item.stamp);
        }
        if (h >= config_.obstacle_min_height_m && h <= config_.obstacle_max_height_m) {obstacles[index] = true;}
        if (h >= config_.boundary_min_height_m && h <= config_.boundary_max_height_m) {
          cell.last_boundary_stamp = std::max(cell.last_boundary_stamp, item.stamp);
          const unsigned layer = std::min(7U, static_cast<unsigned>(8 *
            (h - config_.boundary_min_height_m) /
            (config_.boundary_max_height_m - config_.boundary_min_height_m)));
          cell.height_layers |= static_cast<std::uint8_t>(1U << layer);
          frame_boundary_cells.insert(index);
        }
      }
      if (!ground.valid) {continue;}
      const double horizontal_range = (p.head<2>() - origin.head<2>()).norm();
      const unsigned steps = static_cast<unsigned>(std::ceil(horizontal_range / (config_.grid_resolution_m * 0.5)));
      if (steps == 0 || steps > 2000) {continue;}
      for (unsigned step = 0; step < steps; ++step) {
        const double fraction = static_cast<double>(step) / steps;
        const Eigen::Vector3d ray = origin + fraction * (p - origin);
        const int ray_index = index_of(ray.head<2>());
        if (ray_index < 0) {continue;}
        auto & cell = result.cells[ray_index];
        if (cell.classification == CellClass::Robot) {continue;}
        const double ray_height = ground.normal.dot(ray) + ground.offset;
        // A high ray does not certify the low near-field obstacle band.
        if (ray_height >= config_.obstacle_min_height_m &&
          ray_height <= config_.low_coverage_height_m && item.stamp >= scan.stamp - config_.visibility_max_age_sec)
        {
          cell.last_low_band_stamp = std::max(cell.last_low_band_stamp, item.stamp);
          cell.last_stamp = std::max(cell.last_stamp, item.stamp);
        }
      }
    }
    const double age = std::max(0.0, scan.stamp - retained.stamp);
    for (const auto index : frame_boundary_cells) {
      support[index].insert(retained.id);
      result.cells[index].evidence_weight += std::exp(-age / config_.evidence_tau_sec);
    }
  }
  std::vector<Sample> samples;
  for (std::size_t index = 0; index < result.cells.size(); ++index) {
    auto & cell = result.cells[index];
    cell.ground_observed = scan.stamp - cell.last_ground_stamp <= config_.visibility_max_age_sec;
    cell.low_band_observed = scan.stamp - cell.last_low_band_stamp <= config_.visibility_max_age_sec;
    cell.frame_support = support[index].size();
    if (cell.classification != CellClass::Robot) {
      if (obstacles[index]) {cell.classification = CellClass::Occupied;}
      else if (std::isfinite(cell.last_stamp) && scan.stamp - cell.last_stamp > config_.visibility_max_age_sec) {cell.classification = CellClass::Stale;}
      else if (blocked[index]) {cell.classification = CellClass::Occluded;}
      else if (cell.ground_observed && cell.low_band_observed) {cell.classification = CellClass::Free;}
    }
    if (cell.evidence_weight > 0 && scan.stamp - cell.last_boundary_stamp <= config_.visibility_max_age_sec &&
      cell.center.x() >= config_.fit_x_min_m &&
      cell.center.x() <= config_.fit_x_max_m && std::abs(cell.center.y()) >= config_.side_min_abs_y_m)
    {samples.push_back({cell.center.x(), cell.center.y(), std::min(1.0, cell.evidence_weight),
        cell.center.y() > 0, cell.frame_support});}
  }
  const Fit fit = ground.valid ? fit_row(samples, config_) : Fit{};
  if (!fit.valid) {
    result.reasons.push_back("bilateral_row_fit_unobservable");
    if (!geometry_.field_verified) {result.reasons.push_back("vehicle_geometry_not_field_verified");}
    previous_.reset(); return result;
  }
  result.line_parameters = fit.theta; result.parameter_covariance = fit.covariance;
  result.left_support = fit.left; result.right_support = fit.right; result.frame_support = fit.frames;
  const double a = fit.theta.x(), c = (fit.theta.y() + fit.theta.z()) / 2, s = std::sqrt(1 + a * a);
  result.lateral_error_m = -c / s; result.heading_error_rad = -std::atan(a);
  result.width_m = (fit.theta.y() - fit.theta.z()) / s;
  result.boundary_left_m = fit.theta.y() / s; result.boundary_right_m = -fit.theta.z() / s;
  Eigen::Matrix<double, 2, 3> jacobian;
  jacobian << c * a / (s * s * s), -1 / (2 * s), -1 / (2 * s),
    -1 / (1 + a * a), 0, 0;
  result.error_covariance = jacobian * fit.covariance * jacobian.transpose();
  result.quality_uncertainty = std::exp(-result.error_covariance(0, 0) /
    std::pow(config_.uncertainty_lateral_scale_m, 2) - result.error_covariance(1, 1) /
    std::pow(config_.uncertainty_heading_scale_rad, 2));
  result.quality_support = clamp01(static_cast<double>(std::min(fit.left, fit.right)) /
    (2 * config_.fit_min_side_cells)) * clamp01(fit.span / (2 * config_.fit_min_span_m));
  result.quality_temporal = clamp01(static_cast<double>(fit.frames) / config_.min_frame_support);
  result.quality_width = 1.0;
  if (previous_ && previous_->row_valid) {
    // Transport the previous fitted line to the current carrier before measuring stability.
    const auto current_from_previous = carrier_from_odom * previous_->odom_from_carrier;
    const auto & old = previous_->line_parameters;
    const Eigen::Vector3d p0 = current_from_previous * Eigen::Vector3d(0, (old.y() + old.z()) / 2, 0);
    const Eigen::Vector3d p1 = current_from_previous * Eigen::Vector3d(1, old.x() + (old.y() + old.z()) / 2, 0);
    const Eigen::Vector3d direction = p1 - p0;
    if (std::abs(direction.x()) > 1e-4) {
      const double old_a = direction.y() / direction.x();
      const double old_c = p0.y() - old_a * p0.x();
      const double old_error = -old_c / std::sqrt(1 + old_a * old_a);
      const double de = result.lateral_error_m - old_error;
      const double dh = angle_difference(result.heading_error_rad, -std::atan(old_a));
      result.quality_temporal *= std::exp(-de * de / std::pow(config_.temporal_lateral_scale_m, 2) -
        dh * dh / std::pow(config_.temporal_heading_scale_rad, 2));
    } else {result.quality_temporal = 0;}
    result.quality_width = std::exp(-std::pow(result.width_m - previous_->width_m, 2) /
      std::pow(config_.temporal_width_scale_m, 2));
  }
  result.quality = std::pow(clamp01(result.quality_uncertainty) * clamp01(result.quality_support) *
    clamp01(result.quality_temporal) * clamp01(result.quality_width), 0.25);
  result.row_valid = pose_complete && fit.frames >= config_.min_frame_support && result.quality >= config_.quality_min;
  if (fit.frames < config_.min_frame_support) {result.reasons.push_back("insufficient_independent_frames");}
  if (result.quality < config_.quality_min) {result.reasons.push_back("row_quality_low");}
  if (result.row_valid) {
    for (double x = fit.x_low; x <= fit.x_high; x += config_.path_spacing_m) {
      result.odom_centerline.push_back(result.odom_from_carrier * Eigen::Vector3d(x, a * x + c, 0));
    }
  }
  if (geometry_.footprint.size() >= 3) {
    double vehicle_left = 0.0, vehicle_right = 0.0, vehicle_front = 0.0;
    for (const auto & vertex : geometry_.footprint) {
      vehicle_left = std::max(vehicle_left, vertex.y());
      vehicle_right = std::max(vehicle_right, -vertex.y()); vehicle_front = std::max(vehicle_front, vertex.x());
    }
    result.clearance_left_m = std::max(0.0, result.boundary_left_m - vehicle_left);
    result.clearance_right_m = std::max(0.0, result.boundary_right_m - vehicle_right);
    const double stop_distance = geometry_.braking_deceleration_mps2 > 0 ?
      std::abs(speed_mps) * geometry_.command_latency_sec + speed_mps * speed_mps /
      (2 * geometry_.braking_deceleration_mps2) + geometry_.safety_margin_m :
      std::numeric_limits<double>::infinity();
    const double needed = std::max(config_.clearance_lookahead_m, stop_distance);
    // Bound the footprint at both the current and row-following headings. A long
    // tracked vehicle needs more side room when its body aligns with a slanted row.
    const double row_heading = std::atan(a);
    double swept_left = vehicle_left, swept_right = vehicle_right;
    for (const auto & vertex : geometry_.footprint) {
      const double aligned_y = std::sin(row_heading) * vertex.x() + std::cos(row_heading) * vertex.y();
      swept_left = std::max(swept_left, aligned_y);
      swept_right = std::max(swept_right, -aligned_y);
    }
    const double y0 = -swept_right - geometry_.safety_margin_m;
    const double y1 = swept_left + geometry_.safety_margin_m;
    double continuous = 0.0;
    bool collision = false;
    for (double x = vehicle_front; x < config_.x_max_m; x += config_.grid_resolution_m) {
      unsigned observed = 0, expected = 0;
      bool obstacle = false;
      const double center = a * (x + config_.grid_resolution_m / 2) + c;
      // Union of the carrier-forward and centerline envelopes also covers lateral entry.
      const double low_y = std::min(0.0, center) + y0;
      const double high_y = std::max(0.0, center) + y1;
      for (double y = low_y; y <= high_y; y += config_.grid_resolution_m) {
        ++expected;
        const int index = index_of(Eigen::Vector2d(x + config_.grid_resolution_m / 2, y));
        if (index < 0) {continue;}
        const auto & cell = result.cells[index];
        observed += cell.classification == CellClass::Free ? 1U : 0U;
        obstacle = obstacle || cell.classification == CellClass::Occupied;
      }
      if (obstacle) {collision = true; break;}
      if (expected == 0 || static_cast<double>(observed) / expected < config_.clearance_min_coverage) {break;}
      continuous += config_.grid_resolution_m;
    }
    result.observed_forward_m = continuous;
    result.clearance_front_m = continuous;
    result.clearance_valid = geometry_.field_verified && result.row_valid && ground.valid &&
      std::isfinite(needed) && continuous >= needed && result.clearance_left_m > geometry_.safety_margin_m &&
      result.clearance_right_m > geometry_.safety_margin_m && !collision && speed_mps >= -1e-3 &&
      geometry_.vehicle_height_m > config_.obstacle_min_height_m &&
      geometry_.vehicle_height_m <= config_.obstacle_max_height_m;
    if (continuous < needed) {result.reasons.push_back("swept_stopping_corridor_unknown");}
    if (collision) {result.reasons.push_back("swept_corridor_obstacle");}
    if (speed_mps < -1e-3) {result.reasons.push_back("reverse_sweep_not_supported");}
    if (geometry_.field_verified && geometry_.vehicle_height_m > config_.obstacle_max_height_m) {
      result.reasons.push_back("obstacle_height_band_below_vehicle");
    }
  } else {result.reasons.push_back("vehicle_footprint_missing");}
  if (!geometry_.field_verified) {result.reasons.push_back("vehicle_geometry_not_field_verified");}
  if (!std::isfinite(speed_mps)) {result.reasons.push_back("motion_speed_unavailable");}
  previous_ = result;
  return result;
}
}  // namespace agt_local_row_perception
