#include "agt_terrain_map_generator/cleaning/robot_geometry_loader.hpp"

#include <cmath>
#include <functional>

#include <urdf/model.h>

namespace agt_terrain_map_generator
{
namespace
{

Eigen::Isometry3d to_eigen(const urdf::Pose & pose)
{
  Eigen::Quaterniond rotation(
    pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  if (rotation.squaredNorm() < 1.0e-12) {
    rotation = Eigen::Quaterniond::Identity();
  } else {
    rotation.normalize();
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotation.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

bool add_collision_shape(
  const urdf::CollisionSharedPtr & collision, const Eigen::Isometry3d & base_from_link,
  CollisionModel & model)
{
  if (!collision || !collision->geometry) {
    return false;
  }
  CollisionShape shape;
  shape.pose = base_from_link * to_eigen(collision->origin);
  switch (collision->geometry->type) {
    case urdf::Geometry::BOX:
    {
      const auto box = std::dynamic_pointer_cast<urdf::Box>(collision->geometry);
      if (!box || box->dim.x <= 0.0 || box->dim.y <= 0.0 || box->dim.z <= 0.0) {
        return false;
      }
      shape.type = CollisionShape::Type::BOX;
      shape.dimensions = Eigen::Vector3d(box->dim.x, box->dim.y, box->dim.z);
      model.shapes.push_back(shape);
      return true;
    }
    case urdf::Geometry::CYLINDER:
    {
      const auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(collision->geometry);
      if (!cylinder || cylinder->radius <= 0.0 || cylinder->length <= 0.0) {
        return false;
      }
      shape.type = CollisionShape::Type::CYLINDER;
      shape.dimensions = Eigen::Vector3d(cylinder->radius, cylinder->length, 0.0);
      model.shapes.push_back(shape);
      return true;
    }
    default:
      ++model.unsupported_shape_count;
      return false;
  }
}

}  // namespace

bool RobotGeometryLoader::load(const std::string & urdf_path)
{
  return load(urdf_path, "base_link", "lidar_link");
}

bool RobotGeometryLoader::load(
  const std::string & urdf_path, const std::string & base_frame, const std::string & lidar_frame)
{
  model_ = CollisionModel{};
  model_.base_frame = base_frame;
  model_.lidar_frame = lidar_frame;
  error_.clear();

  urdf::Model urdf_model;
  if (!urdf_model.initFile(urdf_path)) {
    error_ = "could not parse rendered URDF: " + urdf_path;
    return false;
  }
  const auto base_link = urdf_model.getLink(base_frame);
  if (!base_link) {
    error_ = "base frame is absent from URDF: " + base_frame;
    return false;
  }

  std::function<void(const urdf::LinkConstSharedPtr &, const Eigen::Isometry3d &)> visit;
  visit = [&](const urdf::LinkConstSharedPtr & link, const Eigen::Isometry3d & base_from_link) {
      if (!link) {
        return;
      }
      if (link->name == lidar_frame) {
        model_.base_from_lidar = base_from_link;
        model_.has_lidar_transform = true;
      }
      for (const auto & collision : link->collision_array) {
        add_collision_shape(collision, base_from_link, model_);
      }
      for (const auto & child : link->child_links) {
        if (!child || !child->parent_joint || child->parent_joint->type != urdf::Joint::FIXED) {
          continue;
        }
        visit(child, base_from_link * to_eigen(child->parent_joint->parent_to_joint_origin_transform));
      }
    };
  visit(base_link, Eigen::Isometry3d::Identity());

  if (!model_.has_lidar_transform) {
    error_ = "lidar frame is absent from fixed base subtree: " + lidar_frame;
    return false;
  }
  if (model_.shapes.empty()) {
    error_ = "no supported box/cylinder collision shapes below " + base_frame;
    return false;
  }
  return true;
}

CollisionModel RobotGeometryLoader::getModel() const
{
  return model_;
}

const std::string & RobotGeometryLoader::error() const
{
  return error_;
}

}  // namespace agt_terrain_map_generator
