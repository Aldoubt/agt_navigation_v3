#include <cmath>
#include <limits>

#include "agt_base_runtime/ackermann_adapter.hpp"
#include "agt_base_runtime/bunker_adapter.hpp"
#include "gtest/gtest.h"

using agt_base_runtime::AckermannAdapter;
using agt_base_runtime::AdapterCommand;
using agt_base_runtime::ChassisProfile;
using agt_base_runtime::KinematicType;
using agt_base_runtime::BunkerAdapter;

namespace
{
ChassisProfile synthetic_ackermann_profile()
{
  ChassisProfile profile;
  profile.profile_id = "test_only_ackermann_geometry";
  profile.kinematics = KinematicType::kAckermann;
  profile.wheelbase_m = 1.0;
  profile.min_steering_angle_rad = -0.6;
  profile.max_steering_angle_rad = 0.6;
  profile.max_steering_rate_radps = 10.0;
  profile.min_turning_radius_m = 2.0;
  profile.minimum_speed_mps = 0.1;
  profile.rotate_in_place = false;
  return profile;
}

AckermannAdapter configured_ackermann()
{
  AckermannAdapter adapter;
  std::string reason;
  EXPECT_TRUE(adapter.configure(synthetic_ackermann_profile(), reason)) << reason;
  return adapter;
}
}  // namespace

TEST(BunkerAdapter, KeepsOnlyPlanarTwistComponents)
{
  BunkerAdapter adapter;
  ChassisProfile profile;
  profile.profile_id = "bunker_v1";
  profile.kinematics = KinematicType::kSkidSteer;
  std::string reason;
  ASSERT_TRUE(adapter.configure(profile, reason)) << reason;

  geometry_msgs::msg::Twist request;
  request.linear.x = 0.3;
  request.linear.y = 7.0;
  request.linear.z = 8.0;
  request.angular.x = 9.0;
  request.angular.y = 10.0;
  request.angular.z = -0.25;
  const auto command = adapter.convert(request, 0.02);

  ASSERT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.linear_x_mps, 0.3);
  EXPECT_DOUBLE_EQ(command.angular_z_radps, -0.25);
  EXPECT_DOUBLE_EQ(command.steering_angle_rad, 0.0);
}

TEST(BunkerAdapter, RejectsOtherKinematicsAndNonFiniteCommand)
{
  BunkerAdapter adapter;
  ChassisProfile wrong_profile;
  wrong_profile.kinematics = KinematicType::kAckermann;
  std::string reason;
  EXPECT_FALSE(adapter.configure(wrong_profile, reason));
  EXPECT_NE(reason.find("skid_steer"), std::string::npos);

  ChassisProfile profile;
  profile.kinematics = KinematicType::kSkidSteer;
  ASSERT_TRUE(adapter.configure(profile, reason)) << reason;
  geometry_msgs::msg::Twist request;
  request.linear.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(adapter.convert(request, 0.02).valid);
}

TEST(BunkerAdapter, PreservesWheelOdometryWithoutChangingFramesOrValues)
{
  BunkerAdapter adapter;
  nav_msgs::msg::Odometry input;
  input.header.stamp.sec = 123;
  input.header.stamp.nanosec = 456;
  input.header.frame_id = "odom";
  input.child_frame_id = "base_link";
  input.pose.pose.position.x = 1.25;
  input.pose.pose.orientation.z = 0.5;
  input.twist.twist.linear.x = 0.42;
  input.twist.twist.angular.z = -0.12;
  input.pose.covariance[0] = 3.0;
  input.twist.covariance[35] = 4.0;

  const auto output = adapter.normalize_odometry(input);
  EXPECT_EQ(output.header.stamp.sec, 123);
  EXPECT_EQ(output.header.stamp.nanosec, 456u);
  EXPECT_EQ(output.header.frame_id, "odom");
  EXPECT_EQ(output.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(output.pose.pose.position.x, 1.25);
  EXPECT_DOUBLE_EQ(output.pose.pose.orientation.z, 0.5);
  EXPECT_DOUBLE_EQ(output.twist.twist.linear.x, 0.42);
  EXPECT_DOUBLE_EQ(output.twist.twist.angular.z, -0.12);
  EXPECT_DOUBLE_EQ(output.pose.covariance[0], 3.0);
  EXPECT_DOUBLE_EQ(output.twist.covariance[35], 4.0);
}

TEST(AckermannAdapter, ConvertsStraightForwardReverseAndTurns)
{
  auto adapter = configured_ackermann();
  geometry_msgs::msg::Twist request;
  request.linear.x = 0.5;
  auto command = adapter.convert(request, 0.1);
  ASSERT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.linear_x_mps, 0.5);
  EXPECT_DOUBLE_EQ(command.steering_angle_rad, 0.0);

  request.angular.z = 0.1;
  command = adapter.convert(request, 0.1);
  EXPECT_GT(command.steering_angle_rad, 0.0);
  request.linear.x = -0.5;
  command = adapter.convert(request, 0.1);
  EXPECT_LT(command.steering_angle_rad, 0.0);
  EXPECT_DOUBLE_EQ(command.linear_x_mps, -0.5);

  request.angular.z = -0.1;
  command = adapter.convert(request, 0.1);
  EXPECT_GT(command.steering_angle_rad, 0.0);
}

TEST(AckermannAdapter, EnforcesMinimumRadiusAndSteeringRate)
{
  auto profile = synthetic_ackermann_profile();
  profile.max_steering_rate_radps = 0.5;
  AckermannAdapter adapter;
  std::string reason;
  ASSERT_TRUE(adapter.configure(profile, reason)) << reason;
  geometry_msgs::msg::Twist request;
  request.linear.x = 0.1;
  request.angular.z = 2.0;
  auto command = adapter.convert(request, 0.1);
  EXPECT_NEAR(command.steering_angle_rad, 0.05, 1.0e-12);
  command = adapter.convert(request, 1.0);
  EXPECT_NEAR(command.steering_angle_rad, std::atan(0.5), 1.0e-12);
  EXPECT_LE(std::abs(command.steering_angle_rad), 0.6);
}

TEST(AckermannAdapter, HandlesNearZeroAndRejectsInPlaceRotation)
{
  auto adapter = configured_ackermann();
  geometry_msgs::msg::Twist request;
  request.linear.x = 0.0;
  request.angular.z = 0.2;
  auto stopped = adapter.convert(request, 0.1);
  ASSERT_TRUE(stopped.valid);
  EXPECT_TRUE(stopped.in_place_rotation_rejected);
  EXPECT_DOUBLE_EQ(stopped.linear_x_mps, 0.0);
  EXPECT_TRUE(std::isfinite(stopped.steering_angle_rad));
  EXPECT_DOUBLE_EQ(stopped.angular_z_radps, 0.0);

  request.linear.x = 0.099;
  const auto below = adapter.convert(request, 1.0);
  request.linear.x = 0.101;
  const auto above = adapter.convert(request, 1.0);
  EXPECT_LT(std::abs(below.steering_angle_rad - above.steering_angle_rad), 0.01);
}

TEST(AckermannAdapter, RejectsInvalidGeometryAndSpinCapableProfile)
{
  AckermannAdapter adapter;
  auto profile = synthetic_ackermann_profile();
  profile.wheelbase_m = 0.0;
  std::string reason;
  EXPECT_FALSE(adapter.configure(profile, reason));
  EXPECT_NE(reason.find("wheelbase"), std::string::npos);

  profile = synthetic_ackermann_profile();
  profile.rotate_in_place = true;
  EXPECT_FALSE(adapter.configure(profile, reason));
  EXPECT_NE(reason.find("rotate_in_place"), std::string::npos);

  profile = synthetic_ackermann_profile();
  profile.min_turning_radius_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(adapter.configure(profile, reason));
  EXPECT_NE(reason.find("min_turning_radius"), std::string::npos);
}
