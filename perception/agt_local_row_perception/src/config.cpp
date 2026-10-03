#include "agt_local_row_perception/core.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <stdexcept>
#include <set>

namespace agt_local_row_perception
{
namespace
{
Eigen::Vector3d vector3(const YAML::Node & node, const std::string & key,
  const Eigen::Vector3d & fallback = Eigen::Vector3d::Zero())
{
  if (!node[key]) {return fallback;}
  const auto values = node[key].as<std::vector<double>>();
  if (values.size() != 3) {throw std::invalid_argument(key + " requires three values");}
  const Eigen::Vector3d result(values[0], values[1], values[2]);
  if (!result.allFinite()) {throw std::invalid_argument(key + " must be finite");}
  return result;
}
Eigen::Isometry3d transform(const YAML::Node & node, const std::string & translation_key)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = vector3(node, translation_key);
  const auto rpy = vector3(node, "rpy");
  result.linear() = (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
  return result;
}
}  // namespace

Config load_config(const std::string & path)
{
  Config result;
  if (path.empty()) {result.validate(); return result;}
  const auto document = YAML::LoadFile(path);
  const auto node = document["row_perception"] ? document["row_perception"] : document;
#define READ_PARAMETER(name) if (node[#name]) {result.name = node[#name].as<decltype(result.name)>();}
  READ_PARAMETER(pose_history_sec)
  READ_PARAMETER(pose_max_gap_sec)
  READ_PARAMETER(temporal_window_sec)
  READ_PARAMETER(evidence_tau_sec)
  READ_PARAMETER(max_frames)
  READ_PARAMETER(min_range_m)
  READ_PARAMETER(max_range_m)
  READ_PARAMETER(grid_resolution_m)
  READ_PARAMETER(x_min_m)
  READ_PARAMETER(x_max_m)
  READ_PARAMETER(y_abs_max_m)
  READ_PARAMETER(ground_seed_percentile)
  READ_PARAMETER(ground_tolerance_m)
  READ_PARAMETER(ground_max_tilt_deg)
  READ_PARAMETER(ground_max_residual_m)
  READ_PARAMETER(ground_z_min_m)
  READ_PARAMETER(ground_z_max_m)
  READ_PARAMETER(ground_min_xy_span_m)
  READ_PARAMETER(ground_min_cells)
  READ_PARAMETER(ground_ransac_iterations)
  READ_PARAMETER(boundary_min_height_m)
  READ_PARAMETER(boundary_max_height_m)
  READ_PARAMETER(obstacle_min_height_m)
  READ_PARAMETER(obstacle_max_height_m)
  READ_PARAMETER(low_coverage_height_m)
  READ_PARAMETER(fit_x_min_m)
  READ_PARAMETER(fit_x_max_m)
  READ_PARAMETER(side_min_abs_y_m)
  READ_PARAMETER(min_width_m)
  READ_PARAMETER(max_width_m)
  READ_PARAMETER(max_heading_rad)
  READ_PARAMETER(fit_inlier_distance_m)
  READ_PARAMETER(fit_huber_delta_m)
  READ_PARAMETER(fit_min_span_m)
  READ_PARAMETER(fit_min_side_cells)
  READ_PARAMETER(fit_ransac_iterations)
  READ_PARAMETER(min_frame_support)
  READ_PARAMETER(uncertainty_lateral_scale_m)
  READ_PARAMETER(uncertainty_heading_scale_rad)
  READ_PARAMETER(temporal_lateral_scale_m)
  READ_PARAMETER(temporal_heading_scale_rad)
  READ_PARAMETER(temporal_width_scale_m)
  READ_PARAMETER(quality_min)
  READ_PARAMETER(covariance_variance_floor_m2)
  READ_PARAMETER(covariance_condition_max)
  READ_PARAMETER(visibility_max_age_sec)
  READ_PARAMETER(clearance_lookahead_m)
  READ_PARAMETER(clearance_min_coverage)
  READ_PARAMETER(path_spacing_m)
#undef READ_PARAMETER
  result.validate();
  return result;
}

Geometry load_geometry(const std::string & path)
{
  Geometry result;
  if (path.empty()) {return result;}
  const auto document = YAML::LoadFile(path);
  const auto node = document["geometry"] ? document["geometry"] : document;
  if (node["frame_id"]) {result.frame_id = node["frame_id"].as<std::string>();}
  if (result.frame_id.empty()) {throw std::invalid_argument("geometry frame_id must not be empty");}
  if (node["field_verified"]) {result.field_verified = node["field_verified"].as<bool>();}
  if (node["vehicle_height_m"]) {result.vehicle_height_m = node["vehicle_height_m"].as<double>();}
  if (node["braking_deceleration_mps2"]) {result.braking_deceleration_mps2 = node["braking_deceleration_mps2"].as<double>();}
  if (node["command_latency_sec"]) {result.command_latency_sec = node["command_latency_sec"].as<double>();}
  if (node["safety_margin_m"]) {result.safety_margin_m = node["safety_margin_m"].as<double>();}
  if (node["sensor_transform"]) {
    if (!node["sensor_transform"]["translation_xyz"] || !node["sensor_transform"]["rpy"]) {
      throw std::invalid_argument("sensor_transform requires explicit translation_xyz and rpy");
    }
    result.body_from_sensor = transform(node["sensor_transform"], "translation_xyz");
    result.sensor_transform_supplied = true;
  }
  if (node["footprint"]) {
    for (const auto & vertex : node["footprint"]) {
      const auto xy = vertex.as<std::vector<double>>();
      if (xy.size() != 2 || !std::isfinite(xy[0]) || !std::isfinite(xy[1])) {
        throw std::invalid_argument("footprint vertices require finite x,y");
      }
      result.footprint.emplace_back(xy[0], xy[1]);
    }
    if (result.footprint.size() < 3) {throw std::invalid_argument("footprint requires at least three vertices");}
    double orientation = 0.0;
    for (std::size_t i = 0; i < result.footprint.size(); ++i) {
      const Eigen::Vector2d a = result.footprint[(i + 1) % result.footprint.size()] - result.footprint[i];
      const Eigen::Vector2d b = result.footprint[(i + 2) % result.footprint.size()] -
        result.footprint[(i + 1) % result.footprint.size()];
      const double cross = a.x() * b.y() - a.y() * b.x();
      if (a.squaredNorm() <= 1e-12 || std::abs(cross) <= 1e-12 ||
        (orientation != 0.0 && cross * orientation <= 0.0))
      {throw std::invalid_argument("footprint must be a nondegenerate convex polygon in boundary order");}
      orientation = cross;
      for (std::size_t j = 0; j < result.footprint.size(); ++j) {
        if (j == i || j == (i + 1) % result.footprint.size()) {continue;}
        const Eigen::Vector2d delta = result.footprint[j] - result.footprint[i];
        const double edge_cross = a.x() * delta.y() - a.y() * delta.x();
        if (edge_cross * orientation <= 0.0) {
          throw std::invalid_argument("footprint vertices must lie on one side of every boundary edge");
        }
      }
    }
  }
  if (node["self_shapes"]) {
    std::set<std::string> ids;
    for (const auto & item : node["self_shapes"]) {
      Shape shape;
      shape.id = item["id"] ? item["id"].as<std::string>() : "unnamed";
      if (shape.id.empty() || !ids.insert(shape.id).second) {
        throw std::invalid_argument("self shape IDs must be unique and nonempty");
      }
      if (!item["center_xyz"]) {throw std::invalid_argument("self shape requires explicit center_xyz");}
      const auto type = item["type"].as<std::string>();
      shape.body_from_shape = transform(item, "center_xyz");
      shape.padding = item["padding_m"] ? item["padding_m"].as<double>() : 0.0;
      if (!std::isfinite(shape.padding) || shape.padding < 0) {throw std::invalid_argument("shape padding must be finite and nonnegative");}
      if (type == "box") {
        shape.type = Shape::Type::Box; shape.size = vector3(item, "size_xyz");
        if (shape.size.minCoeff() <= 0) {throw std::invalid_argument("box dimensions must be positive");}
      } else if (type == "cylinder") {
        shape.type = Shape::Type::Cylinder;
        shape.radius = item["radius_m"].as<double>(); shape.height = item["height_m"].as<double>();
        if (!(shape.radius > 0) || !(shape.height > 0) ||
          !std::isfinite(shape.radius) || !std::isfinite(shape.height))
        {throw std::invalid_argument("cylinder dimensions must be finite and positive");}
      } else {throw std::invalid_argument("self shape type must be box or cylinder");}
      result.self_shapes.push_back(shape);
    }
  }
  for (const auto value : {result.vehicle_height_m, result.braking_deceleration_mps2,
    result.command_latency_sec, result.safety_margin_m})
  {if (!std::isfinite(value) || value < 0) {throw std::invalid_argument("vehicle parameters must be finite and nonnegative");}}
  if (result.field_verified && (result.footprint.size() < 3 || result.self_shapes.empty() ||
    result.vehicle_height_m <= 0 || result.braking_deceleration_mps2 <= 0 ||
    result.command_latency_sec <= 0 || result.safety_margin_m <= 0))
  {throw std::invalid_argument("field_verified geometry needs footprint, self shapes, height, braking, latency, margin");}
  return result;
}
}  // namespace agt_local_row_perception
