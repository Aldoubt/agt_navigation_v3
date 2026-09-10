#pragma once

#include <cmath>
#include <string>

#include "agt_terrain_map_generator/terrain_types.hpp"

namespace agt_terrain_map_generator
{

struct RigidTransform3d
{
  double tx{0.0};
  double ty{0.0};
  double tz{0.0};
  double qw{1.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};

  bool has_valid_rotation() const
  {
    const double n2 = qw * qw + qx * qx + qy * qy + qz * qz;
    return std::isfinite(n2) && n2 > 1.0e-12;
  }

  Point3f transform(const Point3f & p) const
  {
    const double n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    const double w = qw / n;
    const double x = qx / n;
    const double y = qy / n;
    const double z = qz / n;

    // Quaternion-vector rotation using R(q) expanded explicitly so core terrain
    // interfaces do not depend on Eigen/tf2/PCL.
    const double r00 = 1.0 - 2.0 * (y * y + z * z);
    const double r01 = 2.0 * (x * y - z * w);
    const double r02 = 2.0 * (x * z + y * w);
    const double r10 = 2.0 * (x * y + z * w);
    const double r11 = 1.0 - 2.0 * (x * x + z * z);
    const double r12 = 2.0 * (y * z - x * w);
    const double r20 = 2.0 * (x * z - y * w);
    const double r21 = 2.0 * (y * z + x * w);
    const double r22 = 1.0 - 2.0 * (x * x + y * y);

    return Point3f{
      static_cast<float>(r00 * p.x + r01 * p.y + r02 * p.z + tx),
      static_cast<float>(r10 * p.x + r11 * p.y + r12 * p.z + ty),
      static_cast<float>(r20 * p.x + r21 * p.y + r22 * p.z + tz)};
  }

  // Returns T^-1.  Keep transform operations in the terrain core independent
  // of Eigen/tf2 because offline mapping assets are plain PCD + poses.txt.
  RigidTransform3d inverse() const
  {
    const double n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    const double w = qw / n;
    const double x = -qx / n;
    const double y = -qy / n;
    const double z = -qz / n;

    const double r00 = 1.0 - 2.0 * (y * y + z * z);
    const double r01 = 2.0 * (x * y - z * w);
    const double r02 = 2.0 * (x * z + y * w);
    const double r10 = 2.0 * (x * y + z * w);
    const double r11 = 1.0 - 2.0 * (x * x + z * z);
    const double r12 = 2.0 * (y * z - x * w);
    const double r20 = 2.0 * (x * z - y * w);
    const double r21 = 2.0 * (y * z + x * w);
    const double r22 = 1.0 - 2.0 * (x * x + y * y);

    return RigidTransform3d{
      -(r00 * tx + r01 * ty + r02 * tz),
      -(r10 * tx + r11 * ty + r12 * tz),
      -(r20 * tx + r21 * ty + r22 * tz),
      w, x, y, z};
  }

  // Composition `this * rhs`: the returned transform applies rhs first.
  RigidTransform3d compose(const RigidTransform3d & rhs) const
  {
    const double lhs_n = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    const double rhs_n = std::sqrt(
      rhs.qw * rhs.qw + rhs.qx * rhs.qx + rhs.qy * rhs.qy + rhs.qz * rhs.qz);
    const double aw = qw / lhs_n;
    const double ax = qx / lhs_n;
    const double ay = qy / lhs_n;
    const double az = qz / lhs_n;
    const double bw = rhs.qw / rhs_n;
    const double bx = rhs.qx / rhs_n;
    const double by = rhs.qy / rhs_n;
    const double bz = rhs.qz / rhs_n;
    const Point3f translated = transform(Point3f{
      static_cast<float>(rhs.tx), static_cast<float>(rhs.ty), static_cast<float>(rhs.tz)});

    return RigidTransform3d{
      translated.x, translated.y, translated.z,
      aw * bw - ax * bx - ay * by - az * bz,
      aw * bx + ax * bw + ay * bz - az * by,
      aw * by - ax * bz + ay * bw + az * bx,
      aw * bz + ax * by - ay * bx + az * bw};
  }
};

struct MappingAssetSet
{
  std::string map_directory;
  std::string global_map_pcd;
  std::string poses_txt;
  std::string patches_directory;
};

struct PatchAsset
{
  std::string patch_name;
  std::string patch_pcd;
  RigidTransform3d map_from_body;
  // Mapping assets currently have no absolute sensor timestamp. These fields
  // make the deterministic poses.txt sequence explicit for terrain-only
  // provenance reconstruction.
  int patch_id{-1};
  int pose_id{-1};
  double timestamp{0.0};
};

}  // namespace agt_terrain_map_generator
