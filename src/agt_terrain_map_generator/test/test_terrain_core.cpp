#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "agt_terrain_map_generator/central_difference_slope_builder.hpp"
#include "agt_terrain_map_generator/height_obstacle_builder.hpp"
#include "agt_terrain_map_generator/mapping_assets.hpp"
#include "agt_terrain_map_generator/median_elevation_builder.hpp"

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
  // 90-degree yaw.
  transform.qw = std::sqrt(0.5);
  transform.qz = std::sqrt(0.5);

  const auto mapped = transform.transform({1.0F, 0.0F, 2.0F});
  EXPECT_NEAR(mapped.x, 10.0, 1.0e-5);
  EXPECT_NEAR(mapped.y, 21.0, 1.0e-5);
  EXPECT_NEAR(mapped.z, 3.0, 1.0e-5);
}
