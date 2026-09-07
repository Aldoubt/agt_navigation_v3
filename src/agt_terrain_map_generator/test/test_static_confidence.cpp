#include <gtest/gtest.h>

#include <stdexcept>

#include "agt_terrain_map_generator/map_cleaning/static_confidence/static_confidence.hpp"

namespace agt = agt_terrain_map_generator;

namespace
{

void add(agt::VoxelMap & map, const int x, const int y, const int z, const int observations)
{
  auto & voxel = map[{x, y, z}];
  voxel.observation_count = observations;
  voxel.patch_count = observations;
}

const agt::VoxelConfidence & find(
  const std::vector<agt::VoxelConfidence> & cells, const agt::VoxelIndex & index)
{
  for (const auto & cell : cells) {
    if (cell.index == index) {
      return cell;
    }
  }
  throw std::runtime_error("confidence index not found");
}

}  // namespace

TEST(StaticConfidence, ContinuousWallRaisesNeighborhoodScore)
{
  agt::VoxelMap wall;
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 10; ++y) {
      add(wall, x, y, 0, 1);
    }
  }
  agt::StaticConfidenceEvaluator evaluator;
  std::vector<agt::VoxelConfidence> confidence;
  ASSERT_TRUE(evaluator.evaluate(wall, confidence));
  const auto & center = find(confidence, {5, 5, 0});
  EXPECT_GT(center.confidence.neighborhood_score, 0.30F);
  EXPECT_GT(center.confidence.score, 0.20F);
}

TEST(StaticConfidence, IsolatedSingleObservationIsLow)
{
  agt::VoxelMap map;
  add(map, 0, 0, 0, 1);
  agt::StaticConfidenceEvaluator evaluator;
  std::vector<agt::VoxelConfidence> confidence;
  ASSERT_TRUE(evaluator.evaluate(map, confidence));
  const auto & cell = confidence.front();
  EXPECT_FLOAT_EQ(cell.confidence.observation_score, 0.20F);
  EXPECT_NEAR(cell.confidence.neighborhood_score, 1.0F / 27.0F, 1.0e-6F);
  EXPECT_LT(cell.confidence.score, 0.5F);
}

TEST(StaticConfidence, FiveObservationsHaveFullObservationScore)
{
  agt::VoxelMap map;
  add(map, 0, 0, 0, 5);
  agt::StaticConfidenceEvaluator evaluator;
  std::vector<agt::VoxelConfidence> confidence;
  ASSERT_TRUE(evaluator.evaluate(map, confidence));
  EXPECT_FLOAT_EQ(confidence.front().confidence.observation_score, 1.0F);
  EXPECT_FLOAT_EQ(confidence.front().confidence.surface_score, 0.5F);
  EXPECT_FLOAT_EQ(confidence.front().confidence.viewpoint_score, 0.5F);
}

TEST(StaticConfidence, SeparateRegionsDoNotShareNeighborhoodSupport)
{
  agt::VoxelMap map;
  add(map, 0, 0, 0, 1);
  add(map, 10, 10, 10, 1);
  agt::StaticConfidenceEvaluator evaluator;
  std::vector<agt::VoxelConfidence> confidence;
  ASSERT_TRUE(evaluator.evaluate(map, confidence));
  EXPECT_NEAR(find(confidence, {0, 0, 0}).confidence.neighborhood_score, 1.0F / 27.0F, 1.0e-6F);
  EXPECT_NEAR(find(confidence, {10, 10, 10}).confidence.neighborhood_score, 1.0F / 27.0F, 1.0e-6F);
}

TEST(StaticConfidenceB1, SparseFineWallGetsStructureSupport)
{
  agt::VoxelMap wall;
  // 0.20 m samples have no adjacent fine neighbours, yet remap continuously
  // into 0.25 m structure voxels.
  for (int x = 0; x < 10; ++x) {
    for (int y = 0; y < 10; ++y) {
      add(wall, 2 * x, 2 * y, 0, 1);
    }
  }
  agt::StaticConfidenceEvaluator evaluator;
  std::vector<agt::VoxelConfidence> confidence;
  ASSERT_TRUE(evaluator.evaluateMultiScale(wall, {}, {}, confidence));
  const auto & cell = find(confidence, {10, 10, 0});
  EXPECT_LT(cell.confidence.fine_scale_score, 0.05F);
  EXPECT_GT(cell.confidence.structure_scale_score, cell.confidence.fine_scale_score);
}

TEST(StaticConfidenceB1, SparseChangingTrajectoryStaysLow)
{
  agt::VoxelMap trajectory;
  for (int x = 0; x < 6; ++x) {
    add(trajectory, x * 20, 0, 0, 1);
  }
  agt::StaticConfidenceEvaluator evaluator;
  std::vector<agt::VoxelConfidence> confidence;
  ASSERT_TRUE(evaluator.evaluateMultiScale(trajectory, {}, {}, confidence));
  for (const auto & cell : confidence) {
    EXPECT_LT(cell.confidence.score, 0.5F);
  }
}

TEST(StaticConfidenceB1, GroundPriorRaisesFinalScore)
{
  agt::VoxelMap map;
  add(map, 0, 0, 0, 1);
  agt::StaticConfidenceEvaluator evaluator;
  std::vector<agt::VoxelConfidence> without_ground;
  std::vector<agt::VoxelConfidence> with_ground;
  ASSERT_TRUE(evaluator.evaluateMultiScale(map, {}, {}, without_ground));
  agt::TerrainPriorMap prior;
  prior[{0, 0, 0}] = {true, 1.0F};
  ASSERT_TRUE(evaluator.evaluateMultiScale(map, prior, {}, with_ground));
  EXPECT_FLOAT_EQ(with_ground.front().confidence.ground_score, 1.0F);
  EXPECT_GT(with_ground.front().confidence.score, without_ground.front().confidence.score);
}

TEST(StaticConfidenceB1, MixedWallGroundAndIsolatedPointRemainDistinguishable)
{
  agt::VoxelMap map;
  for (int x = 0; x < 6; ++x) {
    for (int y = 0; y < 6; ++y) {
      add(map, 2 * x, 2 * y, 0, 1);
    }
  }
  add(map, 100, 100, 10, 1);
  agt::TerrainPriorMap prior;
  prior[{0, 0, 0}] = {true, 1.0F};
  agt::StaticConfidenceEvaluator evaluator;
  std::vector<agt::VoxelConfidence> confidence;
  ASSERT_TRUE(evaluator.evaluateMultiScale(map, prior, {}, confidence));
  const auto & wall_ground = find(confidence, {0, 0, 0});
  const auto & isolated = find(confidence, {100, 100, 10});
  EXPECT_GT(wall_ground.confidence.score, isolated.confidence.score);
  EXPECT_GT(wall_ground.confidence.ground_score, isolated.confidence.ground_score);
}
