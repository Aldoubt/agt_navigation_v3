#include <gtest/gtest.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "agt_terrain_map_generator/patchwork_adapter/patchwork_adapter.hpp"

namespace agt = agt_terrain_map_generator;

namespace
{
pcl::PointXYZ point(float x, float y, float z) {return pcl::PointXYZ{x, y, z};}

TEST(PatchworkAdapter, EmptyCloudFailsExplicitly)
{
  agt::PatchworkAdapter adapter({0.89, 1.0, 0.20});
  pcl::PointCloud<pcl::PointXYZ> input; agt::GroundCloud ground; agt::NonGroundCloud nonground;
  EXPECT_FALSE(adapter.segment(input, ground, nonground));
  EXPECT_TRUE(ground.empty()); EXPECT_TRUE(nonground.empty());
}

TEST(PatchworkAdapter, FlatGroundIsGround)
{
  pcl::PointCloud<pcl::PointXYZ> input;
  for (int x = 0; x < 4; ++x) {for (int y = 0; y < 4; ++y) {input.push_back(point(x, y, 0.0F));}}
  agt::PatchworkAdapter adapter({0.89, 1.0, 0.20}); agt::GroundCloud ground; agt::NonGroundCloud nonground;
  ASSERT_TRUE(adapter.segment(input, ground, nonground)); EXPECT_EQ(ground.size(), input.size()); EXPECT_TRUE(nonground.empty());
}

TEST(PatchworkAdapter, FlatObstacleIsNonGround)
{
  pcl::PointCloud<pcl::PointXYZ> input;
  input.push_back(point(0.1F, 0.1F, 0.0F));
  input.push_back(point(0.2F, 0.2F, 0.0F));
  input.push_back(point(0.15F, 0.15F, 0.5F));
  agt::PatchworkAdapter adapter({0.89, 1.0, 0.20}); agt::GroundCloud ground; agt::NonGroundCloud nonground;
  ASSERT_TRUE(adapter.segment(input, ground, nonground)); EXPECT_EQ(ground.size(), 2U); ASSERT_EQ(nonground.size(), 1U); EXPECT_FLOAT_EQ(nonground[0].z, 0.5F);
}

TEST(PatchworkAdapter, SlopeTerrainIsGround)
{
  pcl::PointCloud<pcl::PointXYZ> input;
  for (int x = 0; x < 6; ++x) {for (int y = 0; y < 2; ++y) {input.push_back(point(static_cast<float>(x) + 0.1F, static_cast<float>(y) + 0.1F, 0.1F * x));}}
  agt::PatchworkAdapter adapter({0.89, 1.0, 0.20}); agt::GroundCloud ground; agt::NonGroundCloud nonground;
  ASSERT_TRUE(adapter.segment(input, ground, nonground)); EXPECT_EQ(ground.size(), input.size()); EXPECT_TRUE(nonground.empty());
}
}  // namespace
