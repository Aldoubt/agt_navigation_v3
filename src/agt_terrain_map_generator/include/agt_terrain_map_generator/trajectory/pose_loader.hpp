#pragma once

#include <string>
#include <vector>

namespace agt_terrain_map_generator
{

struct TrajectoryPose
{
  std::string patch_name;
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

class PoseLoader final
{
public:
  bool load(const std::string & poses_txt, std::vector<TrajectoryPose> & poses, std::string & error) const;
};

}  // namespace agt_terrain_map_generator
