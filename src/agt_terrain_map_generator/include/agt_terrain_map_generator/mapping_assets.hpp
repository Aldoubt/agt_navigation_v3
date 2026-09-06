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
};

}  // namespace agt_terrain_map_generator
