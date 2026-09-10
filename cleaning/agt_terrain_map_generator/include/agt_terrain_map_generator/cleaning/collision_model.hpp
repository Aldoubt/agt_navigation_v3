#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "agt_terrain_map_generator/cleaning/collision_shape.hpp"

namespace agt_terrain_map_generator
{

struct CollisionModel
{
  std::string base_frame{"base_link"};
  std::string lidar_frame{"lidar_link"};
  Eigen::Isometry3d base_from_lidar{Eigen::Isometry3d::Identity()};
  bool has_lidar_transform{false};
  std::vector<CollisionShape> shapes;
  std::size_t unsupported_shape_count{0U};
};

}  // namespace agt_terrain_map_generator
