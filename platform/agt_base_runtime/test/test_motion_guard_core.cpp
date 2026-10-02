#include "agt_base_runtime/motion_guard_core.hpp"

#include <gtest/gtest.h>

using agt_base_runtime::GuardCommand;
using agt_base_runtime::GuardState;
using agt_base_runtime::MotionGuardConfig;
using agt_base_runtime::MotionGuardCore;

namespace
{
constexpr int64_t kStartNs = 1'000'000'000;

void localize(MotionGuardCore & core, int64_t stamp = kStartNs)
{
  EXPECT_FALSE(core.on_localization_status(3, true, true, stamp));
}

TEST(MotionGuardCore, StartsStoppedAndBecomesReadyOnlyAfterFreshLocalization)
{
  MotionGuardCore core;
  core.set_initial_time(kStartNs);
  auto snapshot = core.tick(kStartNs + 20'000'000);
  EXPECT_EQ(snapshot.output.linear_x, 0.0);
  EXPECT_EQ(snapshot.output.angular_z, 0.0);
  EXPECT_EQ(snapshot.state, GuardState::LOCALIZATION_BLOCKED);

  localize(core);
  snapshot = core.tick(kStartNs + 40'000'000);
  EXPECT_EQ(snapshot.state, GuardState::READY);
  EXPECT_EQ(snapshot.output.linear_x, 0.0);
}

TEST(MotionGuardCore, ClampsForwardReverseAndAngularVelocity)
{
  MotionGuardConfig config;
  config.max_linear_accel = 100.0;
  config.max_linear_decel = 100.0;
  config.max_angular_accel = 100.0;
  MotionGuardCore core(config);
  core.set_initial_time(kStartNs);
  localize(core);
  ASSERT_TRUE(core.on_navigation_command({10.0, 10.0}, kStartNs));
  auto snapshot = core.tick(kStartNs + 100'000'000);
  EXPECT_DOUBLE_EQ(snapshot.output.linear_x, 0.55);
  EXPECT_DOUBLE_EQ(snapshot.output.angular_z, 0.65);
  EXPECT_EQ(snapshot.state, GuardState::ACTIVE);

  ASSERT_TRUE(core.on_navigation_command({-10.0, -10.0}, kStartNs + 101'000'000));
  snapshot = core.tick(kStartNs + 201'000'000);
  EXPECT_DOUBLE_EQ(snapshot.output.linear_x, -0.20);
  EXPECT_DOUBLE_EQ(snapshot.output.angular_z, -0.65);
}

TEST(MotionGuardCore, PreservesPositiveNegativeSlewAndLinearDeceleration)
{
  MotionGuardCore core;
  core.set_initial_time(kStartNs);
  localize(core);
  ASSERT_TRUE(core.on_navigation_command({0.5, 0.5}, kStartNs));
  auto snapshot = core.tick(kStartNs + 100'000'000);
  EXPECT_NEAR(snapshot.output.linear_x, 0.045, 1e-9);
  EXPECT_NEAR(snapshot.output.angular_z, 0.08, 1e-9);

  for (int i = 2; i <= 5; ++i) {
    const int64_t stamp = kStartNs + i * 100'000'000;
    ASSERT_TRUE(core.on_navigation_command({0.5, 0.5}, stamp));
    snapshot = core.tick(stamp);
  }
  EXPECT_NEAR(snapshot.output.linear_x, 0.225, 1e-9);
  EXPECT_NEAR(snapshot.output.angular_z, 0.40, 1e-9);

  ASSERT_TRUE(core.on_navigation_command({0.0, 0.0}, kStartNs + 501'000'000));
  snapshot = core.tick(kStartNs + 600'000'000);
  EXPECT_NEAR(snapshot.output.linear_x, 0.145, 1e-9);  // 0.80 m/s^2 deceleration
  EXPECT_NEAR(snapshot.output.angular_z, 0.32, 1e-9);  // 0.80 rad/s^2 slew

  ASSERT_TRUE(core.on_navigation_command({-0.5, -0.5}, kStartNs + 601'000'000));
  snapshot = core.tick(kStartNs + 700'000'000);
  EXPECT_NEAR(snapshot.output.linear_x, 0.10, 1e-9);  // negative-direction slew
  EXPECT_NEAR(snapshot.output.angular_z, 0.24, 1e-9);
}

TEST(MotionGuardCore, FreshZeroCommandSlewsToZero)
{
  MotionGuardCore core;
  core.set_initial_time(kStartNs);
  localize(core);
  ASSERT_TRUE(core.on_navigation_command({0.0, 0.0}, kStartNs));
  core.on_control_mode("manual");
  ASSERT_TRUE(core.on_manual_command({0.3, 0.2}, kStartNs + 10'000'000));
  auto snapshot = core.tick(kStartNs + 30'000'000);
  EXPECT_GT(snapshot.output.linear_x, 0.0);
  ASSERT_TRUE(core.on_manual_command({0.0, 0.0}, kStartNs + 40'000'000));
  snapshot = core.tick(kStartNs + 140'000'000);
  EXPECT_NEAR(snapshot.output.linear_x, 0.0, 1e-9);
  EXPECT_NEAR(snapshot.output.angular_z, 0.0, 1e-9);
}

TEST(MotionGuardCore, StaleCommandStopsAndIsDiagnosable)
{
  MotionGuardCore core;
  core.set_initial_time(kStartNs);
  localize(core);
  ASSERT_TRUE(core.on_navigation_command({0.3, 0.2}, kStartNs));
  auto snapshot = core.tick(kStartNs + 20'000'000);
  EXPECT_GT(snapshot.output.linear_x, 0.0);
  snapshot = core.tick(kStartNs + 260'000'001);
  EXPECT_EQ(snapshot.output.linear_x, 0.0);
  EXPECT_EQ(snapshot.output.angular_z, 0.0);
  EXPECT_EQ(snapshot.state, GuardState::STALE_COMMAND);
  EXPECT_STREQ(agt_base_runtime::to_string(snapshot.state), "STALE_COMMAND");
}

TEST(MotionGuardCore, LocalizationLossHardStopsAndRequiresFreshCommandAfterRecovery)
{
  MotionGuardCore core;
  core.set_initial_time(kStartNs);
  localize(core);
  ASSERT_TRUE(core.on_navigation_command({0.3, 0.2}, kStartNs));
  auto snapshot = core.tick(kStartNs + 20'000'000);
  EXPECT_GT(snapshot.output.linear_x, 0.0);

  EXPECT_TRUE(core.on_localization_status(5, false, false, kStartNs + 30'000'000));
  snapshot = core.snapshot(kStartNs + 30'000'000);
  EXPECT_EQ(snapshot.output.linear_x, 0.0);
  EXPECT_EQ(snapshot.state, GuardState::LOCALIZATION_BLOCKED);
  EXPECT_FALSE(core.on_navigation_command({0.3, 0.2}, kStartNs + 40'000'000));

  localize(core, kStartNs + 50'000'000);
  snapshot = core.tick(kStartNs + 60'000'000);
  EXPECT_EQ(snapshot.state, GuardState::READY);
  EXPECT_EQ(snapshot.output.linear_x, 0.0);
  ASSERT_TRUE(core.on_navigation_command({0.3, 0.2}, kStartNs + 70'000'000));
  snapshot = core.tick(kStartNs + 90'000'000);
  EXPECT_GT(snapshot.output.linear_x, 0.0);
}

TEST(MotionGuardCore, MissingStaleAndDeniedPayloadPermissionBlockMotion)
{
  MotionGuardConfig config;
  config.require_payload_drive_permission = true;
  MotionGuardCore core(config);
  core.set_initial_time(kStartNs);
  localize(core);
  EXPECT_FALSE(core.on_navigation_command({0.3, 0.2}, kStartNs));
  auto snapshot = core.tick(kStartNs + 20'000'000);
  EXPECT_EQ(snapshot.state, GuardState::HEALTH_BLOCKED);
  EXPECT_EQ(snapshot.output.linear_x, 0.0);

  core.on_payload_permission(true, kStartNs + 30'000'000);
  ASSERT_TRUE(core.on_navigation_command({0.3, 0.2}, kStartNs + 40'000'000));
  snapshot = core.tick(kStartNs + 60'000'000);
  EXPECT_GT(snapshot.output.linear_x, 0.0);

  EXPECT_TRUE(core.on_payload_permission(false, kStartNs + 70'000'000));
  snapshot = core.snapshot(kStartNs + 70'000'000);
  EXPECT_EQ(snapshot.state, GuardState::HEALTH_BLOCKED);
  EXPECT_EQ(snapshot.output.linear_x, 0.0);
  EXPECT_FALSE(snapshot.payload_hold);  // hold is asserted on the next timed publication
  snapshot = core.tick(kStartNs + 90'000'000);
  EXPECT_TRUE(snapshot.payload_hold);

  core.on_payload_permission(true, kStartNs + 100'000'000);
  snapshot = core.tick(kStartNs + 120'000'000);
  EXPECT_EQ(snapshot.state, GuardState::READY);
  EXPECT_EQ(snapshot.output.linear_x, 0.0);

  ASSERT_TRUE(core.on_navigation_command({0.3, 0.2}, kStartNs + 130'000'000));
  snapshot = core.tick(kStartNs + 150'000'000);
  EXPECT_GT(snapshot.output.linear_x, 0.0);
  snapshot = core.tick(kStartNs + 630'000'001);
  EXPECT_EQ(snapshot.state, GuardState::HEALTH_BLOCKED);
  EXPECT_EQ(snapshot.output.linear_x, 0.0);
  EXPECT_FALSE(snapshot.payload_hold);  // stale permission is not a denial acknowledgement
}

TEST(MotionGuardCore, InvalidLocalizationHealthIsClassifiedAndStops)
{
  MotionGuardCore core;
  core.set_initial_time(kStartNs);
  EXPECT_TRUE(core.on_localization_status(3, false, true, kStartNs));
  auto snapshot = core.snapshot(kStartNs);
  EXPECT_EQ(snapshot.state, GuardState::HEALTH_BLOCKED);
  EXPECT_EQ(snapshot.output.linear_x, 0.0);
  EXPECT_TRUE(core.on_localization_status(3, true, false, kStartNs + 10'000'000));
  snapshot = core.snapshot(kStartNs + 10'000'000);
  EXPECT_EQ(snapshot.state, GuardState::HEALTH_BLOCKED);
}

TEST(MotionGuardCore, ModeHandoffStopsAndRejectsOldSource)
{
  MotionGuardCore core;
  core.set_initial_time(kStartNs);
  localize(core);
  ASSERT_TRUE(core.on_navigation_command({0.3, 0.2}, kStartNs));
  auto snapshot = core.tick(kStartNs + 20'000'000);
  EXPECT_GT(snapshot.output.linear_x, 0.0);
  EXPECT_TRUE(core.on_control_mode("manual"));
  snapshot = core.snapshot(kStartNs + 30'000'000);
  EXPECT_EQ(snapshot.output.linear_x, 0.0);
  EXPECT_FALSE(core.on_navigation_command({0.3, 0.2}, kStartNs + 40'000'000));
  ASSERT_TRUE(core.on_manual_command({0.2, 0.0}, kStartNs + 50'000'000));
  snapshot = core.tick(kStartNs + 70'000'000);
  EXPECT_GT(snapshot.output.linear_x, 0.0);
  EXPECT_FALSE(core.on_control_mode("unsupported"));
}

}  // namespace
