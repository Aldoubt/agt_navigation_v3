#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "agt_terrain_map_generator/map_cleaning/point_provenance.hpp"
#include "agt_terrain_map_generator/map_cleaning/voxel_persistence_filter.hpp"

namespace agt = agt_terrain_map_generator;
namespace fs = std::filesystem;

namespace
{

struct PatchPose
{
  std::string name;
  agt::RigidTransform3d map_from_body;
};

std::vector<PatchPose> load_poses(const fs::path & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open poses file: " + path.string());
  }
  std::vector<PatchPose> poses;
  PatchPose record;
  while (input >> record.name >> record.map_from_body.tx >> record.map_from_body.ty >>
    record.map_from_body.tz >> record.map_from_body.qw >> record.map_from_body.qx >>
    record.map_from_body.qy >> record.map_from_body.qz)
  {
    if (!record.map_from_body.has_valid_rotation()) {
      throw std::runtime_error("invalid pose rotation for patch: " + record.name);
    }
    poses.push_back(record);
  }
  if (poses.empty()) {
    throw std::runtime_error("no valid poses in: " + path.string());
  }
  return poses;
}

std::size_t verify_global_map(const fs::path & path)
{
  pcl::PointCloud<pcl::PointXYZ> map;
  if (pcl::io::loadPCDFile(path.string(), map) != 0) {
    throw std::runtime_error("cannot load global map: " + path.string());
  }
  return map.size();
}

std::vector<agt::ProvenancePoint> reconstruct_provenance(
  const fs::path & patches_dir, const std::vector<PatchPose> & poses)
{
  agt::PointProvenanceBuilder builder;
  std::vector<agt::ProvenancePoint> output;
  for (std::size_t pose_index = 0U; pose_index < poses.size(); ++pose_index) {
    const auto & record = poses[pose_index];
    pcl::PointCloud<pcl::PointXYZI> patch;
    const fs::path path = patches_dir / record.name;
    if (pcl::io::loadPCDFile(path.string(), patch) != 0) {
      throw std::runtime_error("cannot load patch: " + path.string());
    }
    output.reserve(output.size() + patch.size());
    for (const auto & point : patch) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      output.push_back(builder.make(
        {point.x, point.y, point.z}, point.intensity,
        static_cast<int>(pose_index), static_cast<int>(pose_index),
        static_cast<double>(pose_index), record.map_from_body));
    }
  }
  return output;
}

void write_voxels(const fs::path & path, const std::vector<agt::PersistenceVoxel> & voxels)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(voxels.size());
  for (const auto & voxel : voxels) {
    cloud.push_back({
      voxel.center.x(), voxel.center.y(), voxel.center.z(),
      static_cast<float>(voxel.observation.observation_count)});
  }
  if (pcl::io::savePCDFileBinary(path.string(), cloud) != 0) {
    throw std::runtime_error("cannot write PCD: " + path.string());
  }
}

void write_statistics(
  const fs::path & path, const agt::VoxelPersistenceStatistics & statistics,
  const std::size_t global_map_points, const std::size_t pose_count,
  const double resolution, const int minimum_observation)
{
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("cannot write: " + path.string());
  }
  out << "input_points: " << statistics.input_points
      << "\nglobal_map_points: " << global_map_points
      << "\ntotal_voxels: " << statistics.total_voxels
      << "\nstable_voxels: " << statistics.stable_voxels
      << "\nunstable_voxels: " << statistics.unstable_voxels
      << "\naverage_observation: " << statistics.average_observation
      << "\nmax_observation: " << statistics.max_observation
      << "\npose_count: " << pose_count
      << "\nresolution_m: " << resolution
      << "\nmin_observation: " << minimum_observation
      << "\ntimestamp_basis: pose_sequence_index\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 5 && argc != 7) {
    std::cerr << "usage: voxel_persistence_analyzer <global_map.pcd> <patches_dir> <poses.txt> <output_dir> [resolution_m min_observation]\n";
    return 2;
  }
  const fs::path global_map = fs::absolute(argv[1]);
  const fs::path patches_dir = fs::absolute(argv[2]);
  const fs::path poses_path = fs::absolute(argv[3]);
  const fs::path output_dir = fs::absolute(argv[4]);
  const double resolution = argc == 7 ? std::stod(argv[5]) : 0.10;
  const int min_observation = argc == 7 ? std::stoi(argv[6]) : 3;
  fs::create_directories(output_dir);

  const std::size_t global_map_points = verify_global_map(global_map);
  const auto poses = load_poses(poses_path);
  const auto provenance = reconstruct_provenance(patches_dir, poses);
  agt::VoxelPersistenceFilter filter({true, resolution, min_observation});
  agt::VoxelPersistenceResult result;
  if (!filter.analyze(provenance, result)) {
    throw std::runtime_error("invalid voxel persistence options");
  }
  write_voxels(output_dir / "stable_voxels.pcd", result.stable_voxels);
  write_voxels(output_dir / "unstable_voxels.pcd", result.unstable_voxels);
  write_statistics(
    output_dir / "voxel_statistics.yaml", result.statistics, global_map_points,
    poses.size(), resolution, min_observation);
  std::cout << "PROVENANCE_POINTS=" << result.statistics.input_points
            << " TOTAL_VOXELS=" << result.statistics.total_voxels
            << " STABLE_VOXELS=" << result.statistics.stable_voxels
            << " UNSTABLE_VOXELS=" << result.statistics.unstable_voxels << "\n";
  return 0;
}
