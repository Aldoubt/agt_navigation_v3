#include "agt_base_runtime/motion_guard_core.hpp"

#include <algorithm>
#include <cmath>

namespace agt_base_runtime
{

namespace
{
constexpr uint8_t kStateLocalized = 3;
constexpr uint8_t kStateDegraded = 4;
}

MotionGuardCore::MotionGuardCore(MotionGuardConfig config)
: config_(config)
{
}

bool MotionGuardCore::on_navigation_command(const GuardCommand & command, int64_t now_ns)
{
  if (control_mode_ != "navigation") {
    return false;
  }
  if (!motion_gate(now_ns).allowed) {
    target_ = {};
    last_rx_ns_ = 0;
    return false;
  }
  target_ = command;
  last_rx_ns_ = now_ns;
  return true;
}

bool MotionGuardCore::on_manual_command(const GuardCommand & command, int64_t now_ns)
{
  if (control_mode_ != "manual") {
    return false;
  }
  if (!motion_gate(now_ns).allowed) {
    manual_target_ = {};
    last_manual_rx_ns_ = 0;
    return false;
  }
  manual_target_ = command;
  last_manual_rx_ns_ = now_ns;
  return true;
}

bool MotionGuardCore::on_control_mode(const std::string & mode)
{
  if (mode != "navigation" && mode != "manual") {
    return false;
  }
  if (mode == control_mode_) {
    return false;
  }
  control_mode_ = mode;
  target_ = {};
  manual_target_ = {};
  last_rx_ns_ = 0;
  last_manual_rx_ns_ = 0;
  hard_stop();
  return true;
}

bool MotionGuardCore::on_localization_status(
  uint8_t state, bool local_odom_fresh, bool global_correction_valid, int64_t now_ns)
{
  have_localization_status_ = true;
  localization_state_ = state;
  local_odom_fresh_ = local_odom_fresh;
  global_correction_valid_ = global_correction_valid;
  last_localization_rx_ns_ = now_ns;
  if (!localization_gate(now_ns).allowed) {
    target_ = {};
    manual_target_ = {};
    last_rx_ns_ = 0;
    last_manual_rx_ns_ = 0;
    hard_stop();
    return true;
  }
  return false;
}

bool MotionGuardCore::on_payload_permission(bool permitted, int64_t now_ns)
{
  payload_permission_ = permitted;
  last_payload_rx_ns_ = now_ns;
  if (config_.require_payload_drive_permission && !payload_permission_) {
    target_ = {};
    manual_target_ = {};
    last_rx_ns_ = 0;
    last_manual_rx_ns_ = 0;
    hard_stop();
    return true;
  }
  return false;
}

GuardSnapshot MotionGuardCore::tick(int64_t now_ns)
{
  double dt = std::max((now_ns - last_tick_ns_) / 1e9, 1e-4);
  last_tick_ns_ = now_ns;

  const GuardCommand & active_target = control_mode_ == "manual" ? manual_target_ : target_;
  const int64_t active_rx_ns = control_mode_ == "manual" ? last_manual_rx_ns_ : last_rx_ns_;
  const bool stale = active_rx_ns <= 0 ||
    (now_ns - active_rx_ns) / 1e9 > config_.command_timeout_sec;
  const GateResult gate = motion_gate(now_ns);

  if (!gate.allowed && gate.reason.rfind("payload_", 0) == 0) {
    target_ = {};
    manual_target_ = {};
    last_rx_ns_ = 0;
    last_manual_rx_ns_ = 0;
  }

  if (stale || !gate.allowed) {
    output_ = {};
  } else {
    const double target_x = clamp(active_target.linear_x, -config_.max_reverse_x, config_.max_linear_x);
    const double target_w = clamp(active_target.angular_z, -config_.max_angular_z, config_.max_angular_z);
    const double linear_limit = std::abs(target_x) < std::abs(output_.linear_x) ?
      config_.max_linear_decel : config_.max_linear_accel;
    output_.linear_x = slew(output_.linear_x, target_x, linear_limit, dt);
    output_.angular_z = slew(output_.angular_z, target_w, config_.max_angular_accel, dt);
  }

  GuardSnapshot result = snapshot(now_ns);
  const GateResult payload = payload_gate(now_ns);
  result.payload_hold = config_.require_payload_drive_permission &&
    payload.reason == "payload_drive_permission_false" &&
    std::abs(output_.linear_x) < 1e-9 && std::abs(output_.angular_z) < 1e-9;
  return result;
}

GuardSnapshot MotionGuardCore::snapshot(int64_t now_ns) const
{
  return snapshot_for_gate(motion_gate(now_ns), now_ns);
}

void MotionGuardCore::hard_stop()
{
  target_ = {};
  manual_target_ = {};
  last_rx_ns_ = 0;
  last_manual_rx_ns_ = 0;
  output_ = {};
}

MotionGuardCore::GateResult MotionGuardCore::localization_gate(int64_t now_ns) const
{
  if (!config_.require_localization_status) {
    return {true, "localization_gate_disabled", GuardState::LOCALIZATION_BLOCKED};
  }
  if (!have_localization_status_ || last_localization_rx_ns_ <= 0) {
    return {false, "localization_status_missing", GuardState::LOCALIZATION_BLOCKED};
  }
  const double age = std::max(0.0, (now_ns - last_localization_rx_ns_) / 1e9);
  if (age > config_.localization_status_timeout_sec) {
    return {false, "localization_status_stale", GuardState::LOCALIZATION_BLOCKED};
  }
  const bool state_ok = localization_state_ == kStateLocalized ||
    (config_.allow_degraded_localization && localization_state_ == kStateDegraded);
  if (!state_ok) {
    return {false, "localization_state", GuardState::LOCALIZATION_BLOCKED};
  }
  if (!local_odom_fresh_) {
    return {false, "local_odom_not_fresh", GuardState::HEALTH_BLOCKED};
  }
  if (!global_correction_valid_) {
    return {false, "global_correction_invalid", GuardState::HEALTH_BLOCKED};
  }
  return {true, "localized", GuardState::READY};
}

MotionGuardCore::GateResult MotionGuardCore::payload_gate(int64_t now_ns) const
{
  if (!config_.require_payload_drive_permission) {
    return {true, "payload_gate_disabled", GuardState::HEALTH_BLOCKED};
  }
  if (last_payload_rx_ns_ <= 0) {
    return {false, "payload_drive_permission_missing", GuardState::HEALTH_BLOCKED};
  }
  const double age = std::max(0.0, (now_ns - last_payload_rx_ns_) / 1e9);
  if (age > config_.payload_drive_permission_timeout_sec) {
    return {false, "payload_drive_permission_stale", GuardState::HEALTH_BLOCKED};
  }
  if (!payload_permission_) {
    return {false, "payload_drive_permission_false", GuardState::HEALTH_BLOCKED};
  }
  return {true, "payload_drive_permitted", GuardState::READY};
}

MotionGuardCore::GateResult MotionGuardCore::motion_gate(int64_t now_ns) const
{
  const GateResult localization = localization_gate(now_ns);
  if (!localization.allowed) {
    return localization;
  }
  return payload_gate(now_ns);
}

GuardSnapshot MotionGuardCore::snapshot_for_gate(const GateResult & gate, int64_t now_ns) const
{
  GuardSnapshot result;
  result.output = output_;
  result.reason = gate.reason;
  if (!gate.allowed) {
    result.state = gate.blocked_state;
    return result;
  }
  const int64_t active_rx_ns = control_mode_ == "manual" ? last_manual_rx_ns_ : last_rx_ns_;
  if (active_rx_ns <= 0) {
    result.state = GuardState::READY;
  } else if ((now_ns - active_rx_ns) / 1e9 > config_.command_timeout_sec) {
    result.state = GuardState::STALE_COMMAND;
    result.reason = "command_timeout";
  } else {
    result.state = GuardState::ACTIVE;
  }
  return result;
}

double MotionGuardCore::clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(hi, value));
}

double MotionGuardCore::slew(double current, double target, double limit, double dt)
{
  const double delta = target - current;
  if (std::abs(delta) <= 1e-9) {
    return target;
  }
  const double step = std::max(limit * dt, 0.0);
  return current + std::max(-step, std::min(step, delta));
}

const char * to_string(GuardState state)
{
  switch (state) {
    case GuardState::READY: return "READY";
    case GuardState::ACTIVE: return "ACTIVE";
    case GuardState::STALE_COMMAND: return "STALE_COMMAND";
    case GuardState::LOCALIZATION_BLOCKED: return "LOCALIZATION_BLOCKED";
    case GuardState::HEALTH_BLOCKED: return "HEALTH_BLOCKED";
  }
  return "HEALTH_BLOCKED";
}

}  // namespace agt_base_runtime
