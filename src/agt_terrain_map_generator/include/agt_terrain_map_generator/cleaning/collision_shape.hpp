#pragma once

#include <Eigen/Geometry>

namespace agt_terrain_map_generator
{

struct CollisionShape
{
  enum class Type
  {
    BOX,
    CYLINDER,
  };

  Type type{Type::BOX};
  // Shape pose in base_link coordinates.
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  // BOX: full x/y/z size. CYLINDER: x=radius, y=length, z unused.
  Eigen::Vector3d dimensions{Eigen::Vector3d::Zero()};
};

}  // namespace agt_terrain_map_generator
