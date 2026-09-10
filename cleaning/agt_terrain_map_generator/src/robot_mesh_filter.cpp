#include "agt_terrain_map_generator/cleaning/robot_mesh_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>

namespace agt_terrain_map_generator
{
namespace
{

Eigen::Isometry3d to_eigen(const RigidTransform3d & transform)
{
  Eigen::Quaterniond rotation(transform.qw, transform.qx, transform.qy, transform.qz);
  rotation.normalize();
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = rotation.toRotationMatrix();
  result.translation() = Eigen::Vector3d(transform.tx, transform.ty, transform.tz);
  return result;
}

bool valid(const Point3f & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool inside(const Eigen::Vector3d & point, const CollisionShape & shape)
{
  const Eigen::Vector3d local = shape.pose.inverse() * point;
  if (shape.type == CollisionShape::Type::BOX) {
    return std::abs(local.x()) <= 0.5 * shape.dimensions.x() &&
           std::abs(local.y()) <= 0.5 * shape.dimensions.y() &&
           std::abs(local.z()) <= 0.5 * shape.dimensions.z();
  }
  const double radial_distance = std::hypot(local.x(), local.y());
  return radial_distance <= shape.dimensions.x() &&
         std::abs(local.z()) <= 0.5 * shape.dimensions.y();
}

bool inside_any(const Eigen::Vector3d & point, const std::vector<CollisionShape> & shapes)
{
  for (const auto & shape : shapes) {
    if (inside(point, shape)) {
      return true;
    }
  }
  return false;
}

struct CellKey
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const CellKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct CellKeyHash
{
  std::size_t operator()(const CellKey & key) const
  {
    const auto hx = std::hash<std::int64_t>{}(key.x);
    const auto hy = std::hash<std::int64_t>{}(key.y);
    const auto hz = std::hash<std::int64_t>{}(key.z);
    return hx ^ (hy << 1U) ^ (hz << 2U);
  }
};

constexpr double kIndexResolutionM = 1.0;

CellKey cell_for(const Eigen::Vector3d & point)
{
  return CellKey{
    static_cast<std::int64_t>(std::floor(point.x() / kIndexResolutionM)),
    static_cast<std::int64_t>(std::floor(point.y() / kIndexResolutionM)),
    static_cast<std::int64_t>(std::floor(point.z() / kIndexResolutionM))};
}

Eigen::Vector3d world_half_extent(const CollisionShape & shape)
{
  Eigen::Vector3d local_half;
  if (shape.type == CollisionShape::Type::BOX) {
    local_half = 0.5 * shape.dimensions;
  } else {
    local_half = Eigen::Vector3d(
      shape.dimensions.x(), shape.dimensions.x(), 0.5 * shape.dimensions.y());
  }
  return shape.pose.linear().cwiseAbs() * local_half;
}

}  // namespace

bool RobotMeshFilter::filter(
  const Cloud & input, Cloud & output, const CollisionModel & model,
  CleaningStatistics & statistics) const
{
  output.clear();
  statistics = CleaningStatistics{};
  statistics.input_points = input.size();
  if (!model.has_lidar_transform || model.shapes.empty()) {
    return false;
  }
  output.reserve(input.size());
  for (const auto & point : input) {
    if (valid(point) && inside_any(
        model.base_from_lidar * Eigen::Vector3d(point.x, point.y, point.z), model.shapes))
    {
      ++statistics.robot_removed;
      continue;
    }
    output.push_back(point);
  }
  statistics.output_points = output.size();
  return true;
}

bool RobotMeshFilter::filter_map(
  const Cloud & input, Cloud & output, const CollisionModel & model,
  const std::vector<RigidTransform3d> & map_from_base, CleaningStatistics & statistics) const
{
  output.clear();
  statistics = CleaningStatistics{};
  statistics.input_points = input.size();
  if (model.shapes.empty() || map_from_base.empty()) {
    return false;
  }

  std::vector<CollisionShape> map_shapes;
  map_shapes.reserve(model.shapes.size() * map_from_base.size());
  for (const auto & pose : map_from_base) {
    if (!pose.has_valid_rotation()) {
      continue;
    }
    const Eigen::Isometry3d map_from_base_eigen = to_eigen(pose);
    for (const auto & base_shape : model.shapes) {
      CollisionShape map_shape = base_shape;
      map_shape.pose = map_from_base_eigen * base_shape.pose;
      map_shapes.push_back(map_shape);
    }
  }
  if (map_shapes.empty()) {
    return false;
  }

  std::unordered_map<CellKey, std::vector<std::size_t>, CellKeyHash> index;
  index.reserve(map_shapes.size() * 4U);
  for (std::size_t shape_index = 0U; shape_index < map_shapes.size(); ++shape_index) {
    const auto & shape = map_shapes[shape_index];
    const Eigen::Vector3d extent = world_half_extent(shape);
    const CellKey low = cell_for(shape.pose.translation() - extent);
    const CellKey high = cell_for(shape.pose.translation() + extent);
    for (std::int64_t z = low.z; z <= high.z; ++z) {
      for (std::int64_t y = low.y; y <= high.y; ++y) {
        for (std::int64_t x = low.x; x <= high.x; ++x) {
          index[CellKey{x, y, z}].push_back(shape_index);
        }
      }
    }
  }

  output.reserve(input.size());
  for (const auto & point : input) {
    if (valid(point)) {
      const Eigen::Vector3d map_point(point.x, point.y, point.z);
      const auto candidates = index.find(cell_for(map_point));
      if (candidates != index.end()) {
        bool inside_robot = false;
        for (const std::size_t shape_index : candidates->second) {
          if (inside(map_point, map_shapes[shape_index])) {
            inside_robot = true;
            break;
          }
        }
        if (inside_robot) {
          ++statistics.robot_removed;
          continue;
        }
      }
    }
    output.push_back(point);
  }
  statistics.output_points = output.size();
  return true;
}

}  // namespace agt_terrain_map_generator
