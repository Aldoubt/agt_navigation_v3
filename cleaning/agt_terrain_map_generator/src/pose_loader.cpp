#include "agt_terrain_map_generator/trajectory/pose_loader.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

namespace agt_terrain_map_generator
{

bool PoseLoader::load(
  const std::string & poses_txt, std::vector<TrajectoryPose> & poses, std::string & error) const
{
  poses.clear();
  error.clear();
  std::ifstream input(poses_txt);
  if (!input) {
    error = "cannot open poses file: " + poses_txt;
    return false;
  }

  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::istringstream row(line);
    TrajectoryPose pose;
    double qw = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    if (!(row >> pose.patch_name >> pose.x >> pose.y >> pose.z >> qw >> qx >> qy >> qz) ||
      !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z))
    {
      error = "invalid pose row " + std::to_string(line_number);
      poses.clear();
      return false;
    }
    poses.push_back(std::move(pose));
  }
  if (poses.empty()) {
    error = "poses file contains no poses: " + poses_txt;
    return false;
  }
  return true;
}

}  // namespace agt_terrain_map_generator
