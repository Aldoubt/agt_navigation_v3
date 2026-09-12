#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "agt_terrain_map_generator/central_difference_slope_builder.hpp"
#include "agt_terrain_map_generator/cleaning/robot_geometry_loader.hpp"
#include "agt_terrain_map_generator/cleaning/robot_mesh_filter.hpp"
#include "agt_terrain_map_generator/map_cleaning/point_provenance.hpp"
#include "agt_terrain_map_generator/map_cleaning/voxel_persistence_filter.hpp"
#include "agt_terrain_map_generator/gravity_level_patch_preprocessor.hpp"
#include "agt_terrain_map_generator/height_obstacle_builder.hpp"
#include "agt_terrain_map_generator/export/terrain_package_exporter.hpp"
#include "agt_terrain_map_generator/mapping_assets.hpp"
#include "agt_terrain_map_generator/median_elevation_builder.hpp"
#include "agt_terrain_map_generator/grid/traversability_grid.hpp"
#include "agt_terrain_map_generator/preprocess/terrain_preprocessor.hpp"
#include "agt_terrain_map_generator/terrain/traversability_builder.hpp"
#include "agt_terrain_map_generator/trajectory/trajectory_carver.hpp"

namespace agt = agt_terrain_map_generator;

TEST(TerrainCore, MedianElevationRejectsOutlierBias)
{
  agt::GridGeometry geometry;
  geometry.resolution = 1.0;
  geometry.width = 1U;
  geometry.height = 1U;

  agt::PointCloud ground{
    {0.1F, 0.1F, 0.00F},
    {0.2F, 0.2F, 0.10F},
    {0.3F, 0.3F, 4.00F}};

  agt::MedianElevationBuilder builder({3U, 100.0});
  std::vector<agt::ElevationCell> elevation;
  ASSERT_TRUE(builder.build(ground, geometry, elevation));
  ASSERT_EQ(elevation.size(), 1U);
  EXPECT_NEAR(elevation[0].median_height, 0.10, 1.0e-6);
  EXPECT_EQ(elevation[0].point_count, 3U);
  EXPECT_GT(elevation[0].confidence, 0.9F);
}

TEST(TerrainCore, SparseCellHasReducedConfidence)
{
  agt::GridGeometry geometry;
  geometry.resolution = 1.0;
  geometry.width = 1U;
  geometry.height = 1U;

  agt::MedianElevationBuilder builder({4U, 0.04});
  std::vector<agt::ElevationCell> elevation;
  ASSERT_TRUE(builder.build({{0.2F, 0.2F, 1.0F}}, geometry, elevation));
  EXPECT_NEAR(elevation[0].median_height, 1.0, 1.0e-6);
  EXPECT_NEAR(elevation[0].confidence, 0.25, 1.0e-6);
}

TEST(TerrainCore, SlopeUsesElevationSurface)
{
  agt::GridGeometry geometry;
  geometry.resolution = 1.0;
  geometry.width = 3U;
  geometry.height = 1U;

  std::vector<agt::ElevationCell> elevation(3U);
  for (std::size_t i = 0; i < elevation.size(); ++i) {
    elevation[i].median_height = static_cast<float>(i);
    elevation[i].confidence = 1.0F;
    elevation[i].point_count = 3U;
  }

  agt::CentralDifferenceSlopeBuilder builder({0.5});
  std::vector<float> slope;
  ASSERT_TRUE(builder.build(geometry, elevation, slope));
  ASSERT_EQ(slope.size(), 3U);
  EXPECT_NEAR(slope[1], 45.0, 1.0e-4);
}

TEST(TerrainCore, LowConfidenceSlopeRemainsUnknown)
{
  agt::GridGeometry geometry;
  geometry.resolution = 1.0;
  geometry.width = 2U;
  geometry.height = 1U;

  std::vector<agt::ElevationCell> elevation(2U);
  elevation[0].median_height = 0.0F;
  elevation[0].confidence = 0.2F;
  elevation[1].median_height = 1.0F;
  elevation[1].confidence = 1.0F;

  agt::CentralDifferenceSlopeBuilder builder({0.5});
  std::vector<float> slope;
  ASSERT_TRUE(builder.build(geometry, elevation, slope));
  EXPECT_TRUE(std::isnan(slope[0]));
}

TEST(TerrainCore, ObstacleUsesHeightAboveGround)
{
  agt::GridGeometry geometry;
  geometry.resolution = 1.0;
  geometry.width = 1U;
  geometry.height = 1U;

  std::vector<agt::ElevationCell> elevation(1U);
  elevation[0].median_height = 2.0F;
  elevation[0].confidence = 1.0F;

  agt::HeightObstacleBuilder builder({0.25, 3.0, 0.5});
  std::vector<std::uint8_t> obstacle;

  ASSERT_TRUE(builder.build({{0.1F, 0.1F, 2.10F}}, geometry, elevation, obstacle));
  ASSERT_EQ(obstacle.size(), 1U);
  EXPECT_EQ(obstacle[0], 0U);

  ASSERT_TRUE(builder.build({{0.1F, 0.1F, 2.50F}}, geometry, elevation, obstacle));
  EXPECT_EQ(obstacle[0], 1U);
}

TEST(TerrainCore, MappingPoseTransformsBodyPointToMap)
{
  agt::RigidTransform3d transform;
  transform.tx = 10.0;
  transform.ty = 20.0;
  transform.tz = 1.0;
  transform.qw = std::sqrt(0.5);
  transform.qz = std::sqrt(0.5);

  const auto mapped = transform.transform({1.0F, 0.0F, 2.0F});
  EXPECT_NEAR(mapped.x, 10.0, 1.0e-5);
  EXPECT_NEAR(mapped.y, 21.0, 1.0e-5);
  EXPECT_NEAR(mapped.z, 3.0, 1.0e-5);
}

TEST(TerrainCore, GravityLevelRoundTripPreservesMapPoint)
{
  const double roll = 0.20;
  const double pitch = -0.15;
  const double yaw = 0.70;
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  agt::PatchAsset asset;
  asset.patch_name = "0.pcd";
  asset.map_from_body.tx = 4.0;
  asset.map_from_body.ty = -2.0;
  asset.map_from_body.tz = 0.5;
  asset.map_from_body.qw = cr * cp * cy + sr * sp * sy;
  asset.map_from_body.qx = sr * cp * cy - cr * sp * sy;
  asset.map_from_body.qy = cr * sp * cy + sr * cp * sy;
  asset.map_from_body.qz = cr * cp * sy - sr * sp * cy;

  const agt::Point3f body_point{1.2F, -0.4F, -0.6F};
  const auto expected_map = asset.map_from_body.transform(body_point);

  agt::GravityLevelPatchPreprocessor preprocessor;
  agt::PreparedPatch prepared;
  std::string error;
  ASSERT_TRUE(preprocessor.process(asset, {body_point}, prepared, error)) << error;
  ASSERT_EQ(prepared.local_cloud.size(), 1U);

  const auto recovered_map = prepared.map_from_local.transform(prepared.local_cloud.front());
  EXPECT_NEAR(recovered_map.x, expected_map.x, 1.0e-5);
  EXPECT_NEAR(recovered_map.y, expected_map.y, 1.0e-5);
  EXPECT_NEAR(recovered_map.z, expected_map.z, 1.0e-5);

  const auto recovered_body = prepared.body_from_local.transform(prepared.local_cloud.front());
  EXPECT_NEAR(recovered_body.x, body_point.x, 1.0e-5);
  EXPECT_NEAR(recovered_body.y, body_point.y, 1.0e-5);
  EXPECT_NEAR(recovered_body.z, body_point.z, 1.0e-5);
}

TEST(TerrainCore, TerrainPreprocessorOrdersSelfRearThenVoxel)
{
  agt::RobotBodyBox box;
  box.center_xyz = {0.0, 0.0, 0.20};
  box.size_xyz = {1.0, 1.0, 0.4};
  box.padding_m = 0.0;
  agt::StaticRobotGeometryProvider geometry(box);

  agt::TerrainPreprocessorOptions options;
  options.self_filter.enabled = true;
  options.rear_filter.enabled = true;
  options.rear_filter.box = {-2.0, -1.0, -0.8, 0.8, -0.5, 2.0};
  options.voxel.enabled = true;
  options.voxel.leaf_size_m = 0.20;
  agt::TerrainPreprocessor preprocessor(options, geometry);

  const agt::PointCloud input{
    {0.0F, 0.0F, 0.2F},       // self box
    {-1.5F, 0.0F, 0.2F},      // rear box
    {1.00F, 1.00F, 0.0F},     // first retained voxel
    {1.05F, 1.05F, 0.05F},    // duplicate retained voxel
    {2.00F, 2.00F, 0.0F}};    // second retained voxel
  agt::PointCloud output;
  agt::FilterStatistics statistics;
  ASSERT_TRUE(preprocessor.process(input, agt::RigidTransform3d{}, output, statistics));
  EXPECT_EQ(statistics.total_input_points, 5U);
  EXPECT_EQ(statistics.self_removed_points, 1U);
  EXPECT_EQ(statistics.rear_removed_points, 1U);
  EXPECT_EQ(statistics.voxel_removed_points, 1U);
  EXPECT_EQ(statistics.final_points, 2U);
  ASSERT_EQ(output.size(), 2U);
}

TEST(TerrainCore, TerrainPreprocessorUsesBodyFrameAfterGravityAlignment)
{
  agt::RobotBodyBox box;
  box.center_xyz = {0.0, 0.0, 0.2};
  box.size_xyz = {0.4, 0.4, 0.4};
  box.padding_m = 0.0;
  agt::StaticRobotGeometryProvider geometry(box);
  agt::TerrainPreprocessorOptions options;
  options.self_filter.enabled = true;
  options.rear_filter.enabled = false;
  options.voxel.enabled = false;
  agt::TerrainPreprocessor preprocessor(options, geometry);

  // `local` is deliberately not inside the body box, but body_from_local maps
  // it onto the vehicle centre.  This catches accidental filtering in the
  // gravity-level frame when the MID360/body attitude is tilted.
  agt::RigidTransform3d body_from_local;
  body_from_local.tx = -1.0;
  const agt::PointCloud local{{1.0F, 0.0F, 0.2F}, {3.0F, 0.0F, 0.2F}};
  agt::PointCloud output;
  agt::FilterStatistics statistics;
  ASSERT_TRUE(preprocessor.process(local, body_from_local, output, statistics));
  EXPECT_EQ(statistics.self_removed_points, 1U);
  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(output.front().x, 3.0, 1.0e-6);
}

TEST(TerrainV03, FlatHighConfidenceCellIsFree)
{
  agt::GridGeometry geometry;
  geometry.resolution = 1.0;
  geometry.width = 1U;
  geometry.height = 1U;
  agt::ElevationGrid elevation(1U);
  elevation[0].median_height = 0.0F;
  elevation[0].confidence = 1.0F;
  agt::SlopeGrid slope{0.0F};
  agt::ObstacleGrid obstacle{0.0F};
  agt::ConfidenceGrid confidence{1.0F};
  agt::FreeCorridorGrid corridor;
  corridor.geometry = geometry;
  corridor.confidence = {0.0F};
  agt::TraversabilityGrid output;

  agt::TraversabilityBuilder builder({});
  ASSERT_TRUE(builder.build(geometry, elevation, slope, obstacle, confidence, corridor, output));
  ASSERT_EQ(output.cells.size(), 1U);
  EXPECT_EQ(output.cells[0].state, agt::TraversabilityState::FREE);
  EXPECT_NEAR(output.cells[0].cost, 0.0, 1.0e-6);
}

TEST(TerrainV03, TwentyFiveDegreeSlopeRaisesCostButIsNotBlocked)
{
  agt::GridGeometry geometry;
  geometry.resolution = 1.0;
  geometry.width = 1U;
  geometry.height = 1U;
  agt::ElevationGrid elevation(1U);
  elevation[0].median_height = 0.0F;
  elevation[0].confidence = 1.0F;
  agt::FreeCorridorGrid corridor;
  corridor.geometry = geometry;
  corridor.confidence = {0.0F};
  agt::TraversabilityGrid output;

  agt::TraversabilityBuilder builder({});
  ASSERT_TRUE(builder.build(
    geometry, elevation, {25.0F}, {0.0F}, {1.0F}, corridor, output));
  EXPECT_EQ(output.cells[0].state, agt::TraversabilityState::FREE);
  EXPECT_GT(output.cells[0].cost, 0.0F);
  EXPECT_LT(output.cells[0].cost, 1.0F);
}

TEST(TerrainV03, OneMetreObstacleIsBlocked)
{
  agt::GridGeometry geometry;
  geometry.resolution = 1.0;
  geometry.width = 1U;
  geometry.height = 1U;
  agt::ElevationGrid elevation(1U);
  elevation[0].median_height = 0.0F;
  elevation[0].confidence = 1.0F;
  agt::FreeCorridorGrid corridor;
  corridor.geometry = geometry;
  corridor.confidence = {0.0F};
  agt::TraversabilityGrid output;

  agt::TraversabilityBuilder builder({});
  ASSERT_TRUE(builder.build(
    geometry, elevation, {0.0F}, {1.0F}, {1.0F}, corridor, output));
  EXPECT_EQ(output.cells[0].state, agt::TraversabilityState::BLOCKED);
  EXPECT_FLOAT_EQ(output.cells[0].cost, 1.0F);
}

TEST(TerrainV03, TrajectoryCarverProducesContinuousFreeCorridor)
{
  agt::GridGeometry geometry;
  geometry.resolution = 0.10;
  geometry.origin_x = -1.0;
  geometry.origin_y = -1.0;
  geometry.width = 41U;
  geometry.height = 21U;
  geometry.frame_id = "map";
  const std::vector<agt::TrajectoryPose> poses{
    {"0.pcd", 0.0, 0.0, 0.0}, {"1.pcd", 1.0, 0.0, 0.0}, {"2.pcd", 2.0, 0.0, 0.0}};
  agt::TrajectoryFreeCarver carver({true, 0.45, 0.20});
  agt::FreeCorridorGrid corridor;
  ASSERT_TRUE(carver.carve(geometry, poses, corridor));
  ASSERT_TRUE(agt::same_grid_geometry(geometry, corridor.geometry));

  const auto index_at = [&geometry](const double x, const double y) {
      const auto ix = static_cast<std::size_t>(std::floor((x - geometry.origin_x) / geometry.resolution));
      const auto iy = static_cast<std::size_t>(std::floor((y - geometry.origin_y) / geometry.resolution));
      return iy * geometry.width + ix;
    };
  EXPECT_FLOAT_EQ(corridor.confidence[index_at(0.0, 0.0)], 1.0F);
  EXPECT_FLOAT_EQ(corridor.confidence[index_at(1.0, 0.0)], 1.0F);
  EXPECT_FLOAT_EQ(corridor.confidence[index_at(2.0, 0.0)], 1.0F);
}

TEST(TerrainV03, TerrainPackageExporterWritesAllRequiredLayers)
{
  agt::GridGeometry geometry;
  geometry.resolution = 1.0;
  geometry.width = 1U;
  geometry.height = 1U;
  geometry.frame_id = "map";
  agt::ElevationGrid elevation(1U);
  elevation[0].median_height = 0.0F;
  elevation[0].confidence = 1.0F;
  agt::FreeCorridorGrid corridor;
  corridor.geometry = geometry;
  corridor.confidence = {1.0F};
  agt::TraversabilityGrid traversability;
  traversability.geometry = geometry;
  traversability.cells = {{0.0F, 1.0F, agt::TraversabilityState::FREE}};

  const auto root = std::filesystem::temp_directory_path() / "agt_terrain_v03_export_test";
  std::error_code remove_error;
  std::filesystem::remove_all(root, remove_error);
  agt::GenerationContext context;
  context.output_root = root.string();
  context.map_id = "unit_test";
  context.map_version = "v0.3";
  agt::TerrainPackageExporter exporter;
  std::string error;
  ASSERT_TRUE(exporter.export_package(
    elevation, {0.0F}, {0.0F}, {1.0F}, corridor, traversability, context, "unit_test=true", error)) << error;
  const auto package = root / "terrain_package";
  EXPECT_TRUE(std::filesystem::exists(package / "elevation.pgm"));
  EXPECT_TRUE(std::filesystem::exists(package / "slope.pgm"));
  EXPECT_TRUE(std::filesystem::exists(package / "obstacle.pgm"));
  EXPECT_TRUE(std::filesystem::exists(package / "confidence.pgm"));
  EXPECT_TRUE(std::filesystem::exists(package / "trajectory_free.pgm"));
  EXPECT_TRUE(std::filesystem::exists(package / "traversability.pgm"));
  EXPECT_TRUE(std::filesystem::exists(package / "metadata.yaml"));
  std::filesystem::remove_all(root, remove_error);
}

TEST(TerrainV031, BoxCollisionRemovesOnlyInteriorPoint)
{
  agt::CollisionModel model;
  model.has_lidar_transform = true;
  agt::CollisionShape box;
  box.type = agt::CollisionShape::Type::BOX;
  box.dimensions = Eigen::Vector3d(1.0, 1.0, 1.0);
  model.shapes.push_back(box);
  agt::Cloud output;
  agt::CleaningStatistics statistics;
  agt::RobotMeshFilter filter;
  ASSERT_TRUE(filter.filter({{0.1F, 0.1F, 0.1F}, {1.0F, 0.0F, 0.0F}}, output, model, statistics));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(output.front().x, 1.0, 1.0e-6);
  EXPECT_EQ(statistics.input_points, 2U);
  EXPECT_EQ(statistics.robot_removed, 1U);
  EXPECT_EQ(statistics.output_points, 1U);
}

TEST(TerrainV031, CylinderCollisionRemovesSupportRodPoint)
{
  agt::CollisionModel model;
  model.has_lidar_transform = true;
  agt::CollisionShape cylinder;
  cylinder.type = agt::CollisionShape::Type::CYLINDER;
  cylinder.dimensions = Eigen::Vector3d(0.20, 1.00, 0.0);
  model.shapes.push_back(cylinder);
  agt::Cloud output;
  agt::CleaningStatistics statistics;
  agt::RobotMeshFilter filter;
  ASSERT_TRUE(filter.filter({{0.10F, 0.0F, 0.0F}, {0.30F, 0.0F, 0.0F}}, output, model, statistics));
  ASSERT_EQ(output.size(), 1U);
  EXPECT_NEAR(output.front().x, 0.30, 1.0e-6);
  EXPECT_EQ(statistics.robot_removed, 1U);
}

TEST(TerrainV031, TiltedLidarTransformMapsPointIntoBaseCollision)
{
  constexpr double kDegToRad = 0.01745329251994329577;
  agt::CollisionModel model;
  model.has_lidar_transform = true;
  model.base_from_lidar = Eigen::Translation3d(0.10, 0.0, 0.30) *
    Eigen::AngleAxisd(13.0 * kDegToRad, Eigen::Vector3d::UnitY());
  const Eigen::Vector3d lidar_point(0.20, 0.0, 0.0);
  const Eigen::Vector3d expected_base = model.base_from_lidar * lidar_point;
  EXPECT_NEAR(expected_base.x(), 0.10 + std::cos(13.0 * kDegToRad) * 0.20, 1.0e-9);
  EXPECT_NEAR(expected_base.z(), 0.30 - std::sin(13.0 * kDegToRad) * 0.20, 1.0e-9);
  agt::CollisionShape box;
  box.type = agt::CollisionShape::Type::BOX;
  box.pose.translation() = expected_base;
  box.dimensions = Eigen::Vector3d(0.10, 0.10, 0.10);
  model.shapes.push_back(box);
  agt::Cloud output;
  agt::CleaningStatistics statistics;
  agt::RobotMeshFilter filter;
  ASSERT_TRUE(filter.filter({{0.20F, 0.0F, 0.0F}}, output, model, statistics));
  EXPECT_TRUE(output.empty());
  EXPECT_EQ(statistics.robot_removed, 1U);
}

TEST(TerrainV031, UrdfLoaderReadsBoxCylinderAndFixedLidarTransform)
{
  const auto path = std::filesystem::temp_directory_path() / "agt_robot_filter_test.urdf";
  {
    std::ofstream urdf(path);
    urdf << R"(<robot name="test">
      <link name="base_link"><collision><geometry><box size="1 2 3"/></geometry></collision></link>
      <link name="support"><collision><geometry><cylinder radius="0.1" length="1.0"/></geometry></collision></link>
      <link name="lidar_link"/>
      <joint name="support_joint" type="fixed"><parent link="base_link"/><child link="support"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>
      <joint name="lidar_joint" type="fixed"><parent link="base_link"/><child link="lidar_link"/><origin xyz="0.1 0 0.3" rpy="0 0.2268928028 0"/></joint>
    </robot>)";
  }
  agt::RobotGeometryLoader loader;
  ASSERT_TRUE(loader.load(path.string(), "base_link", "lidar_link")) << loader.error();
  const auto model = loader.getModel();
  EXPECT_TRUE(model.has_lidar_transform);
  EXPECT_EQ(model.shapes.size(), 2U);
  EXPECT_NEAR(model.base_from_lidar.translation().x(), 0.1, 1.0e-9);
  std::error_code error;
  std::filesystem::remove(path, error);
}

TEST(TerrainV032, ThreePatchObservationsMakeVoxelStable)
{
  const std::vector<agt::ProvenancePoint> input{
    {Eigen::Vector3f{0.01F, 0.01F, 0.01F}, 0.0F, 1, 1, 10.0},
    {Eigen::Vector3f{0.02F, 0.01F, 0.01F}, 0.0F, 2, 2, 11.0},
    {Eigen::Vector3f{0.03F, 0.01F, 0.01F}, 0.0F, 3, 3, 12.0}};
  agt::VoxelPersistenceFilter filter({true, 0.10, 3});
  agt::VoxelPersistenceResult result;
  ASSERT_TRUE(filter.analyze(input, result));
  EXPECT_EQ(result.statistics.total_voxels, 1U);
  EXPECT_EQ(result.statistics.stable_voxels, 1U);
  EXPECT_EQ(result.statistics.unstable_voxels, 0U);
  EXPECT_EQ(result.stable_voxels.front().observation.observation_count, 3);
  EXPECT_EQ(result.stable_voxels.front().observation.patch_count, 3);
}

TEST(TerrainV032, SingleObservationIsUnstable)
{
  const std::vector<agt::ProvenancePoint> input{
    {Eigen::Vector3f{0.01F, 0.01F, 0.01F}, 0.0F, 1, 1, 10.0}};
  agt::VoxelPersistenceFilter filter({true, 0.10, 3});
  agt::VoxelPersistenceResult result;
  ASSERT_TRUE(filter.analyze(input, result));
  EXPECT_EQ(result.statistics.stable_voxels, 0U);
  EXPECT_EQ(result.statistics.unstable_voxels, 1U);
  EXPECT_EQ(result.unstable_voxels.front().observation.observation_count, 1);
}

TEST(TerrainV032, SpatialVoxelsAreIndependent)
{
  const std::vector<agt::ProvenancePoint> input{
    {Eigen::Vector3f{0.01F, 0.01F, 0.01F}, 0.0F, 1, 1, 1.0},
    {Eigen::Vector3f{1.01F, 0.01F, 0.01F}, 0.0F, 2, 2, 2.0},
    {Eigen::Vector3f{1.02F, 0.01F, 0.01F}, 0.0F, 3, 3, 3.0},
    {Eigen::Vector3f{1.03F, 0.01F, 0.01F}, 0.0F, 4, 4, 4.0}};
  agt::VoxelPersistenceFilter filter({true, 0.10, 3});
  agt::VoxelPersistenceResult result;
  ASSERT_TRUE(filter.analyze(input, result));
  EXPECT_EQ(result.statistics.total_voxels, 2U);
  EXPECT_EQ(result.statistics.stable_voxels, 1U);
  EXPECT_EQ(result.statistics.unstable_voxels, 1U);
}

TEST(TerrainV032, ProvenancePreservesTimestampAndMapPosition)
{
  agt::RigidTransform3d map_from_body;
  map_from_body.tx = 5.0;
  agt::PointProvenanceBuilder builder;
  const auto point = builder.make(
    {1.0F, 2.0F, 3.0F}, 42.0F, 7, 8, 123.5, map_from_body);
  EXPECT_FLOAT_EQ(point.point.x(), 6.0F);
  EXPECT_FLOAT_EQ(point.point.y(), 2.0F);
  EXPECT_FLOAT_EQ(point.intensity, 42.0F);
  EXPECT_EQ(point.patch_id, 7);
  EXPECT_EQ(point.pose_id, 8);
  EXPECT_DOUBLE_EQ(point.timestamp, 123.5);

  agt::VoxelPersistenceFilter filter({true, 0.10, 3});
  agt::VoxelPersistenceResult result;
  ASSERT_TRUE(filter.analyze({point}, result));
  EXPECT_DOUBLE_EQ(result.unstable_voxels.front().observation.first_seen, 123.5);
  EXPECT_DOUBLE_EQ(result.unstable_voxels.front().observation.last_seen, 123.5);
}
