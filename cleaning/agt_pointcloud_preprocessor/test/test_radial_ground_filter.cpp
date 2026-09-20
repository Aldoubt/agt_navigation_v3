#include <cmath>
#include <vector>

#include "gtest/gtest.h"

#include "agt_pointcloud_preprocessor/radial_ground_filter.hpp"

using agt_pointcloud_preprocessor::RadialGroundFilter;
using agt_pointcloud_preprocessor::RadialGroundOptions;
using agt_pointcloud_preprocessor::TerrainPoint;

TEST(RadialGroundFilter, RemovesFlatGroundAndKeepsTreeReturns)
{
  RadialGroundFilter filter(RadialGroundOptions{});
  const std::vector<TerrainPoint> points{
    {0.5, 0.0, -0.20}, {1.0, 0.0, -0.20}, {2.0, 0.0, -0.20},
    {2.0, 0.0, 0.10}, {2.0, 0.0, 0.80}};
  const auto result = filter.classify(points);
  EXPECT_TRUE(result[0].is_ground);
  EXPECT_TRUE(result[1].is_ground);
  EXPECT_TRUE(result[2].is_ground);
  EXPECT_FALSE(result[3].is_ground);
  EXPECT_FALSE(result[4].is_ground);
  EXPECT_NEAR(result[3].ground_z, -0.20, 1e-9);
}

TEST(RadialGroundFilter, FollowsThirtyDegreeTraversableIncline)
{
  RadialGroundFilter filter(RadialGroundOptions{});
  std::vector<TerrainPoint> points;
  const double slope = std::tan(30.0 * 3.14159265358979323846 / 180.0);
  for (double range = 0.5; range <= 4.0; range += 0.25) {
    points.push_back({range, 0.0, -0.20 + slope * range});
  }
  const auto result = filter.classify(points);
  for (const auto & item : result) {
    EXPECT_TRUE(item.is_ground);
  }
}

TEST(RadialGroundFilter, MarksExcessiveSlopeAsObstacle)
{
  RadialGroundFilter filter(RadialGroundOptions{});
  std::vector<TerrainPoint> points;
  const double slope = std::tan(45.0 * 3.14159265358979323846 / 180.0);
  for (double range = 0.5; range <= 3.0; range += 0.25) {
    points.push_back({range, 0.0, -0.20 + slope * range});
  }
  const auto result = filter.classify(points);
  EXPECT_TRUE(std::any_of(result.begin(), result.end(), [](const auto & item) {
    return !item.is_ground;
  }));
}

TEST(RadialGroundFilter, KeepsStepEdge)
{
  RadialGroundOptions options;
  options.local_max_slope_deg = 25.0;
  RadialGroundFilter filter(options);
  const std::vector<TerrainPoint> points{
    {0.5, 0.0, -0.20}, {1.0, 0.0, -0.20}, {1.5, 0.0, -0.20},
    {1.55, 0.0, 0.02}};
  const auto result = filter.classify(points);
  EXPECT_FALSE(result.back().is_ground);
}
