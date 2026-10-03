#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace agt_local_row_perception
{
struct Point
{
  Eigen::Vector3d xyz{Eigen::Vector3d::Zero()};
  double stamp{0.0};  // Absolute sensor timestamp, seconds. Never receipt time.
  float intensity{0.0F};
};

struct Pose
{
  double stamp{0.0};
  std::uint64_t epoch{0};
  Eigen::Isometry3d odom_from_body{Eigen::Isometry3d::Identity()};
};

class PoseHistory
{
public:
  explicit PoseHistory(double window_sec = 10.0, double max_gap_sec = 0.15);
  bool insert(const Pose & pose);
  void reset(std::uint64_t epoch);
  std::optional<Pose> at(double stamp, std::uint64_t epoch) const;
  std::uint64_t epoch() const {return epoch_;}
  double latest_stamp() const;

private:
  double window_sec_;
  double max_gap_sec_;
  std::uint64_t epoch_{0};
  std::deque<Pose> poses_;
};

struct Shape
{
  enum class Type {Box, Cylinder};
  std::string id;
  Type type{Type::Box};
  Eigen::Isometry3d body_from_shape{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  double radius{0.0};
  double height{0.0};
  double padding{0.0};
  bool contains(const Eigen::Vector3d & body_point) const;
  bool blocks_ray(const Eigen::Vector3d & origin, const Eigen::Vector3d & end) const;
};

struct Geometry
{
  std::string frame_id{"base_link"};
  bool field_verified{false};
  std::vector<Eigen::Vector2d> footprint;
  std::vector<Shape> self_shapes;
  double vehicle_height_m{0.0};
  double braking_deceleration_mps2{0.0};
  double command_latency_sec{0.0};
  double safety_margin_m{0.0};
  Eigen::Isometry3d body_from_sensor{Eigen::Isometry3d::Identity()};
  bool sensor_transform_supplied{false};
};

// Development settings. They do not certify geometry, braking, or detection performance.
struct Config
{
  double pose_history_sec{10.0};
  double pose_max_gap_sec{0.15};
  double temporal_window_sec{0.8};
  double evidence_tau_sec{0.45};
  unsigned max_frames{8};
  double min_range_m{0.1};
  double max_range_m{15.0};
  double grid_resolution_m{0.12};
  double x_min_m{-1.0};
  double x_max_m{8.0};
  double y_abs_max_m{4.0};
  double ground_seed_percentile{0.2};
  double ground_tolerance_m{0.08};
  double ground_max_tilt_deg{18.0};
  double ground_max_residual_m{0.08};
  double ground_z_min_m{-2.0};
  double ground_z_max_m{0.1};
  double ground_min_xy_span_m{0.8};
  unsigned ground_min_cells{12};
  unsigned ground_ransac_iterations{100};
  double boundary_min_height_m{0.15};
  double boundary_max_height_m{2.0};
  double obstacle_min_height_m{0.10};
  double obstacle_max_height_m{1.5};
  double low_coverage_height_m{0.15};
  double fit_x_min_m{0.3};
  double fit_x_max_m{7.0};
  double side_min_abs_y_m{0.25};
  double min_width_m{0.6};
  double max_width_m{6.0};
  double max_heading_rad{0.7};
  double fit_inlier_distance_m{0.15};
  double fit_huber_delta_m{0.10};
  double fit_min_span_m{1.8};
  unsigned fit_min_side_cells{8};
  unsigned fit_ransac_iterations{160};
  unsigned min_frame_support{2};
  double uncertainty_lateral_scale_m{0.15};
  double uncertainty_heading_scale_rad{0.10};
  double temporal_lateral_scale_m{0.2};
  double temporal_heading_scale_rad{0.15};
  double temporal_width_scale_m{0.3};
  double quality_min{0.45};
  double covariance_variance_floor_m2{0.0025};
  double covariance_condition_max{10000.0};
  double visibility_max_age_sec{0.25};
  double clearance_lookahead_m{2.0};
  double clearance_min_coverage{1.0};
  double path_spacing_m{0.15};
  void validate() const;
};

enum class CellClass : std::uint8_t
{
  Unobserved = 0, Free = 1, Occupied = 2, Robot = 3,
  Occluded = 4, Stale = 5
};

struct Cell
{
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  CellClass classification{CellClass::Unobserved};
  double evidence_weight{0.0};
  double last_stamp{-std::numeric_limits<double>::infinity()};
  double last_boundary_stamp{-std::numeric_limits<double>::infinity()};
  double last_ground_stamp{-std::numeric_limits<double>::infinity()};
  double last_low_band_stamp{-std::numeric_limits<double>::infinity()};
  double min_height_m{std::numeric_limits<double>::infinity()};
  double max_height_m{-std::numeric_limits<double>::infinity()};
  std::uint32_t frame_support{0};
  std::uint8_t height_layers{0};
  bool ground_observed{false};
  bool low_band_observed{false};
};

struct RowState
{
  double stamp{0.0};
  std::uint64_t odom_epoch{0};
  bool ground_valid{false};
  bool row_valid{false};
  bool clearance_valid{false};
  double lateral_error_m{0.0};
  double heading_error_rad{0.0};
  double width_m{0.0};
  double boundary_left_m{0.0};
  double boundary_right_m{0.0};
  double clearance_left_m{0.0};
  double clearance_right_m{0.0};
  double clearance_front_m{0.0};
  double observed_forward_m{0.0};
  Eigen::Matrix2d error_covariance{Eigen::Matrix2d::Zero()};
  double quality{0.0};
  double quality_uncertainty{0.0};
  double quality_support{0.0};
  double quality_temporal{0.0};
  double quality_width{0.0};
  std::uint32_t left_support{0};
  std::uint32_t right_support{0};
  std::uint32_t frame_support{0};
  Eigen::Vector3d ground_normal{Eigen::Vector3d::UnitZ()};
  double ground_offset{0.0};
  Eigen::Vector3d line_parameters{Eigen::Vector3d::Zero()}; // a, b_left, b_right
  Eigen::Matrix3d parameter_covariance{Eigen::Matrix3d::Zero()};
  Eigen::Isometry3d odom_from_carrier{Eigen::Isometry3d::Identity()};
  std::vector<Eigen::Vector3d> odom_centerline;
  std::vector<Cell> cells;
  std::vector<std::string> reasons;
  unsigned grid_width{0};
  unsigned grid_height{0};
  std::uint64_t points_input{0};
  std::uint64_t points_self{0};
  std::uint64_t points_occluded{0};
  std::uint64_t points_missing_pose{0};
  std::uint64_t points_used{0};
};

struct Scan
{
  double stamp{0.0}; // Reference time, usually end of scan.
  std::uint64_t epoch{0};
  Eigen::Isometry3d body_from_sensor{Eigen::Isometry3d::Identity()};
  std::vector<Point> points;
};

class RowPerception
{
public:
  explicit RowPerception(Config config, Geometry geometry = {});
  RowState process(const Scan & scan, const PoseHistory & poses, double speed_mps = 0.0);
  void reset(std::uint64_t epoch);
  const Config & config() const {return config_;}
  const Geometry & geometry() const {return geometry_;}

private:
  struct Return
  {
    Eigen::Vector3d odom_point;
    Eigen::Vector3d odom_origin;
    double stamp;
    bool self{false};
    bool occluded{false};
  };
  struct Frame
  {
    std::uint64_t id;
    double stamp;
    std::vector<Return> returns;
  };
  Config config_;
  Geometry geometry_;
  std::uint64_t epoch_{0};
  std::uint64_t next_frame_id_{0};
  double last_stamp_{-std::numeric_limits<double>::infinity()};
  std::deque<Frame> frames_;
  std::optional<RowState> previous_;
};

Config load_config(const std::string & path);
Geometry load_geometry(const std::string & path);
}  // namespace agt_local_row_perception
