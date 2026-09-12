#include "agt_terrain_map_generator/gravity_level_patch_preprocessor.hpp"

#include <cmath>

namespace agt_terrain_map_generator
{
namespace
{

double yaw_from_transform(const RigidTransform3d & transform)
{
  const double n = std::sqrt(
    transform.qw * transform.qw + transform.qx * transform.qx +
    transform.qy * transform.qy + transform.qz * transform.qz);
  const double w = transform.qw / n;
  const double x = transform.qx / n;
  const double y = transform.qy / n;
  const double z = transform.qz / n;
  const double sin_yaw = 2.0 * (w * z + x * y);
  const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(sin_yaw, cos_yaw);
}

}  // namespace

std::string GravityLevelPatchPreprocessor::name() const
{
  return "gravity_level";
}

bool GravityLevelPatchPreprocessor::process(
  const PatchAsset & asset,
  const PointCloud & body_cloud,
  PreparedPatch & prepared,
  std::string & error) const
{
  error.clear();
  prepared.local_cloud.clear();
  prepared.map_from_local = RigidTransform3d{};
  prepared.body_from_local = RigidTransform3d{};

  if (!asset.map_from_body.has_valid_rotation()) {
    error = "patch pose has invalid quaternion: " + asset.patch_name;
    return false;
  }
  if (body_cloud.empty()) {
    error = "patch cloud is empty: " + asset.patch_name;
    return false;
  }

  const double yaw = yaw_from_transform(asset.map_from_body);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  // First rotate body points by the optimized map orientation, but without its
  // translation. Then undo only map yaw. This is equivalent to
  // R_level_body = Rz(-yaw) * R_map_body and matches the relocalization
  // candidate leveling convention.
  RigidTransform3d map_rotation = asset.map_from_body;
  map_rotation.tx = 0.0;
  map_rotation.ty = 0.0;
  map_rotation.tz = 0.0;

  prepared.local_cloud.reserve(body_cloud.size());
  for (const auto & body_point : body_cloud) {
    const Point3f map_rotated = map_rotation.transform(body_point);
    const double level_x = c * map_rotated.x + s * map_rotated.y;
    const double level_y = -s * map_rotated.x + c * map_rotated.y;
    prepared.local_cloud.push_back(Point3f{
      static_cast<float>(level_x),
      static_cast<float>(level_y),
      map_rotated.z});
  }

  // After segmentation in the yaw-neutral gravity-level frame, map points back
  // with yaw-only rotation plus the optimized keyframe translation:
  // T_map_level = [Rz(yaw), t_map_body].
  prepared.map_from_local.tx = asset.map_from_body.tx;
  prepared.map_from_local.ty = asset.map_from_body.ty;
  prepared.map_from_local.tz = asset.map_from_body.tz;
  prepared.map_from_local.qw = std::cos(0.5 * yaw);
  prepared.map_from_local.qx = 0.0;
  prepared.map_from_local.qy = 0.0;
  prepared.map_from_local.qz = std::sin(0.5 * yaw);
  prepared.body_from_local = asset.map_from_body.inverse().compose(prepared.map_from_local);
  return true;
}

}  // namespace agt_terrain_map_generator
