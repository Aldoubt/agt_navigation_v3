#include <algorithm>
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

#include "agt_terrain_map_generator/map_cleaning/static_confidence/static_confidence.hpp"

namespace agt = agt_terrain_map_generator;
namespace fs = std::filesystem;

namespace
{

agt::VoxelIndex index_for(const pcl::PointXYZI & point, const double resolution)
{
  return {
    static_cast<int>(std::floor(point.x / resolution)),
    static_cast<int>(std::floor(point.y / resolution)),
    static_cast<int>(std::floor(point.z / resolution))};
}

agt::VoxelIndex index_for(const pcl::PointXYZ & point, const double resolution)
{
  return {
    static_cast<int>(std::floor(point.x / resolution)),
    static_cast<int>(std::floor(point.y / resolution)),
    static_cast<int>(std::floor(point.z / resolution))};
}

void load_voxel_cloud(const fs::path & path, const double resolution, agt::VoxelMap & voxels)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  if (pcl::io::loadPCDFile(path.string(), cloud) != 0) {
    throw std::runtime_error("cannot load persistence cloud: " + path.string());
  }
  for (const auto & point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    auto & observation = voxels[index_for(point, resolution)];
    // Persistence PCD intensity is the distinct pose observation count.
    observation.observation_count = std::max(
      observation.observation_count, static_cast<int>(std::lround(point.intensity)));
    observation.patch_count = observation.observation_count;
  }
}

void load_ground_prior(
  const fs::path & path, const double resolution, agt::TerrainPriorMap & terrain_prior)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(path.string(), cloud) != 0) {
    throw std::runtime_error("cannot load ground prior PCD: " + path.string());
  }
  for (const auto & point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    terrain_prior[index_for(point, resolution)] = {true, 1.0F};
  }
}

pcl::PointXYZI to_point(const agt::VoxelConfidence & voxel, const double resolution)
{
  const float half = static_cast<float>(resolution * 0.5);
  return {
    static_cast<float>(voxel.index.x * resolution) + half,
    static_cast<float>(voxel.index.y * resolution) + half,
    static_cast<float>(voxel.index.z * resolution) + half,
    voxel.confidence.score};
}

pcl::PointXYZI to_point(
  const agt::VoxelConfidence & voxel, const double resolution, const float score)
{
  auto point = to_point(voxel, resolution);
  point.intensity = score;
  return point;
}

void write_cloud(const fs::path & path, const std::vector<pcl::PointXYZI> & points)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.assign(points.begin(), points.end());
  if (pcl::io::savePCDFileBinary(path.string(), cloud) != 0) {
    throw std::runtime_error("cannot write: " + path.string());
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const bool legacy = argc == 4 || argc == 7;
  const bool multiscale = argc == 5 || argc == 10;
  if (!legacy && !multiscale) {
    std::cerr << "usage (v0.3.2-b): static_confidence_exporter <stable_voxels.pcd> <unstable_voxels.pcd> <output_dir> [resolution_m high medium]\n"
              << "usage (v0.3.2-b.1): static_confidence_exporter <stable_voxels.pcd> <unstable_voxels.pcd> <ground_map.pcd> <output_dir> [fine_m structure_m scene_m high medium]\n";
    return 2;
  }
  const fs::path stable_path = fs::absolute(argv[1]);
  const fs::path unstable_path = fs::absolute(argv[2]);
  const fs::path ground_path = multiscale ? fs::absolute(argv[3]) : fs::path{};
  const fs::path output_dir = fs::absolute(argv[multiscale ? 4 : 3]);
  const double fine_resolution = legacy && argc == 7 ? std::stod(argv[4]) :
    (multiscale && argc == 10 ? std::stod(argv[5]) : 0.10);
  const double structure_resolution = multiscale && argc == 10 ? std::stod(argv[6]) : 0.25;
  const double scene_resolution = multiscale && argc == 10 ? std::stod(argv[7]) : 0.50;
  const float high_threshold = legacy && argc == 7 ? std::stof(argv[5]) :
    (multiscale && argc == 10 ? std::stof(argv[8]) : 0.80F);
  const float medium_threshold = legacy && argc == 7 ? std::stof(argv[6]) :
    (multiscale && argc == 10 ? std::stof(argv[9]) : 0.50F);
  if (!(fine_resolution > 0.0) || !(structure_resolution > 0.0) ||
    !(scene_resolution > 0.0) || high_threshold < medium_threshold || medium_threshold < 0.0F ||
    high_threshold > 1.0F)
  {
    throw std::runtime_error("invalid static confidence parameters");
  }
  fs::create_directories(output_dir);

  agt::VoxelMap voxels;
  load_voxel_cloud(stable_path, fine_resolution, voxels);
  load_voxel_cloud(unstable_path, fine_resolution, voxels);
  agt::StaticConfidenceEvaluator evaluator;
  std::vector<agt::VoxelConfidence> confidence;
  agt::TerrainPriorMap terrain_prior;
  if (multiscale) {
    load_ground_prior(ground_path, fine_resolution, terrain_prior);
  }
  const bool evaluated = multiscale ? evaluator.evaluateMultiScale(
    voxels, terrain_prior,
    {fine_resolution, structure_resolution, scene_resolution}, confidence) :
    evaluator.evaluate(voxels, confidence);
  if (!evaluated) {
    throw std::runtime_error("static confidence evaluation failed");
  }

  std::vector<pcl::PointXYZI> high;
  std::vector<pcl::PointXYZI> medium;
  std::vector<pcl::PointXYZI> low;
  std::vector<pcl::PointXYZI> all;
  std::vector<pcl::PointXYZI> fine_scores;
  std::vector<pcl::PointXYZI> structure_scores;
  std::vector<pcl::PointXYZI> scene_scores;
  std::vector<pcl::PointXYZI> ground_scores;
  std::size_t histogram[5]{};
  double score_sum = 0.0;
  double fine_sum = 0.0;
  double structure_sum = 0.0;
  double scene_sum = 0.0;
  double ground_sum = 0.0;
  for (const auto & voxel : confidence) {
    const auto point = to_point(voxel, fine_resolution);
    all.push_back(point);
    score_sum += voxel.confidence.score;
    fine_sum += voxel.confidence.fine_scale_score;
    structure_sum += voxel.confidence.structure_scale_score;
    scene_sum += voxel.confidence.scene_scale_score;
    ground_sum += voxel.confidence.ground_score;
    if (multiscale) {
      fine_scores.push_back(to_point(voxel, fine_resolution, voxel.confidence.fine_scale_score));
      structure_scores.push_back(to_point(voxel, fine_resolution, voxel.confidence.structure_scale_score));
      scene_scores.push_back(to_point(voxel, fine_resolution, voxel.confidence.scene_scale_score));
      ground_scores.push_back(to_point(voxel, fine_resolution, voxel.confidence.ground_score));
    }
    if (voxel.confidence.score >= high_threshold) {
      high.push_back(point);
    } else if (voxel.confidence.score >= medium_threshold) {
      medium.push_back(point);
    } else {
      low.push_back(point);
    }
    const int bin = std::min(4, static_cast<int>(voxel.confidence.score / 0.2F));
    ++histogram[bin];
  }
  write_cloud(output_dir / "high_static.pcd", high);
  write_cloud(output_dir / "medium_static.pcd", medium);
  write_cloud(output_dir / "low_confidence.pcd", low);
  write_cloud(output_dir / "confidence_cloud.pcd", all);
  if (multiscale) {
    fs::create_directories(output_dir / "debug");
    write_cloud(output_dir / "debug" / "fine_score.pcd", fine_scores);
    write_cloud(output_dir / "debug" / "structure_score.pcd", structure_scores);
    write_cloud(output_dir / "debug" / "scene_score.pcd", scene_scores);
    write_cloud(output_dir / "debug" / "ground_score.pcd", ground_scores);
    write_cloud(output_dir / "debug" / "final_score.pcd", all);
  }

  std::ofstream stats(output_dir / "statistics.yaml");
  if (!stats) {
    throw std::runtime_error("cannot write statistics.yaml");
  }
  stats << "total_voxels: " << confidence.size()
        << "\nhigh_static_voxels: " << high.size()
        << "\nmedium_static_voxels: " << medium.size()
        << "\nlow_confidence_voxels: " << low.size()
        << "\naverage_score: " << (confidence.empty() ? 0.0 : score_sum / confidence.size())
        << "\nscore_histogram:"
        << "\n  0-0.2: " << histogram[0]
        << "\n  0.2-0.4: " << histogram[1]
        << "\n  0.4-0.6: " << histogram[2]
        << "\n  0.6-0.8: " << histogram[3]
        << "\n  0.8-1.0: " << histogram[4]
        << "\naverage_fine_score: " << (confidence.empty() ? 0.0 : fine_sum / confidence.size())
        << "\naverage_structure_score: " << (confidence.empty() ? 0.0 : structure_sum / confidence.size())
        << "\naverage_scene_score: " << (confidence.empty() ? 0.0 : scene_sum / confidence.size())
        << "\naverage_ground_score: " << (confidence.empty() ? 0.0 : ground_sum / confidence.size())
        << "\nmode: " << (multiscale ? "multi_scale_ground_prior" : "legacy_single_scale")
        << "\nfine_resolution_m: " << fine_resolution
        << "\nstructure_resolution_m: " << structure_resolution
        << "\nscene_resolution_m: " << scene_resolution
        << "\nhigh_threshold: " << high_threshold
        << "\nmedium_threshold: " << medium_threshold << "\n";
  std::cout << "TOTAL_VOXELS=" << confidence.size()
            << " HIGH=" << high.size()
            << " MEDIUM=" << medium.size()
            << " LOW=" << low.size() << "\n";
  return 0;
}
