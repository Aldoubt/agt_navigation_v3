#pragma once

#include <string>

#include "agt_terrain_map_generator/cleaning/collision_model.hpp"

namespace agt_terrain_map_generator
{

class RobotGeometryLoader
{
public:
  // The input must be a rendered URDF XML file.  Xacro expansion remains the
  // responsibility of the description package / caller.
  bool load(const std::string & urdf_path);
  bool load(const std::string & urdf_path, const std::string & base_frame, const std::string & lidar_frame);

  CollisionModel getModel() const;
  const std::string & error() const;

private:
  CollisionModel model_;
  std::string error_;
};

}  // namespace agt_terrain_map_generator
