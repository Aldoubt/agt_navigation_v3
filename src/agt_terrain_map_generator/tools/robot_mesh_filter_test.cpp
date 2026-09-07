#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "agt_terrain_map_generator/cleaning/robot_geometry_loader.hpp"
#include "agt_terrain_map_generator/cleaning/robot_mesh_filter.hpp"

namespace agt = agt_terrain_map_generator;
namespace fs = std::filesystem;

namespace
{

std::vector<agt::RigidTransform3d> load_map_poses(const fs::path & poses_path)
{
  std::ifstream input(poses_path);
  if (!input) {
    throw std::runtime_error("cannot open poses file: " + poses_path.string());
  }
  std::vector<agt::RigidTransform3d> poses;
  std::string patch_name;
  agt::RigidTransform3d pose;
  while (input >> patch_name >> pose.tx >> pose.ty >> pose.tz >> pose.qw >> pose.qx >> pose.qy >> pose.qz) {
    if (pose.has_valid_rotation()) {
      poses.push_back(pose);
    }
  }
  if (poses.empty()) {
    throw std::runtime_error("no valid poses in: " + poses_path.string());
  }
  return poses;
}

agt::Cloud read_cloud(const fs::path & path)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(path.string(), cloud) != 0) {
    throw std::runtime_error("cannot load PCD: " + path.string());
  }
  agt::Cloud result;
  result.reserve(cloud.size());
  for (const auto & point : cloud) {
    result.push_back({point.x, point.y, point.z});
  }
  return result;
}

void write_cloud(const fs::path & path, const agt::Cloud & input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(input.size());
  for (const auto & point : input) {
    cloud.push_back({point.x, point.y, point.z});
  }
  if (pcl::io::savePCDFileBinary(path.string(), cloud) != 0) {
    throw std::runtime_error("cannot write PCD: " + path.string());
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5 && argc != 7) {
    std::cerr << "usage: robot_mesh_filter_test <global_map.pcd> <poses.txt> <rendered.urdf> <output_dir> [base_link lidar_link]\n";
    return 2;
  }
  const fs::path input_pcd = fs::absolute(argv[1]);
  const fs::path poses_txt = fs::absolute(argv[2]);
  const fs::path urdf_path = fs::absolute(argv[3]);
  const fs::path output_dir = fs::absolute(argv[4]);
  const std::string base_frame = argc == 7 ? argv[5] : "base_link";
  const std::string lidar_frame = argc == 7 ? argv[6] : "lidar_link";
  fs::create_directories(output_dir);

  const auto before = read_cloud(input_pcd);
  agt::RobotGeometryLoader loader;
  if (!loader.load(urdf_path.string(), base_frame, lidar_frame)) {
    throw std::runtime_error("RobotGeometryLoader failed: " + loader.error());
  }
  const auto poses = load_map_poses(poses_txt);
  agt::RobotMeshFilter filter;
  agt::Cloud after;
  agt::CleaningStatistics statistics;
  if (!filter.filter_map(before, after, loader.getModel(), poses, statistics)) {
    throw std::runtime_error("RobotMeshFilter map-frame filtering failed");
  }

  write_cloud(output_dir / "before.pcd", before);
  write_cloud(output_dir / "after_robot_filter.pcd", after);
  std::ofstream report(output_dir / "cleaning_report.yaml");
  report << "input_points: " << statistics.input_points
         << "\nrobot_removed: " << statistics.robot_removed
         << "\noutput_points: " << statistics.output_points
         << "\npose_count: " << poses.size()
         << "\nbase_frame: " << base_frame
         << "\nlidar_frame: " << lidar_frame
         << "\ncollision_shape_count: " << loader.getModel().shapes.size()
         << "\nunsupported_shape_count: " << loader.getModel().unsupported_shape_count
         << "\n";
  std::cout << "INPUT_POINTS=" << statistics.input_points
            << " ROBOT_REMOVED=" << statistics.robot_removed
            << " OUTPUT_POINTS=" << statistics.output_points
            << " SHAPES=" << loader.getModel().shapes.size() << "\n";
  return 0;
}
