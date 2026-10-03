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
  if (config_.research_enabled || !std::isfinite(command.linear_x) || !std::isfinite(command.angular_z)) {
    return false;
  }
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
  if (config_.research_enabled || !std::isfinite(command.linear_x) || !std::isfinite(command.angular_z)) {
    return false;
  }
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
  uint8_t state, bool local_odom_fresh, bool global_correction_valid, int64_t now_ns,
  int64_t source_stamp_ns)
{
  have_localization_status_ = true;
  localization_state_ = state;
  local_odom_fresh_ = local_odom_fresh;
  global_correction_valid_ = global_correction_valid;
  last_localization_rx_ns_ = now_ns;
  last_localization_source_ns_ = source_stamp_ns;
  if (!(config_.research_enabled ? research_gate(now_ns) : localization_gate(now_ns)).allowed) {
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
  if (config_.research_enabled && now_ns < last_tick_ns_) {invalidate_research_clock();}
  double dt = std::max((now_ns - last_tick_ns_) / 1e9, 1e-4);
  if (config_.research_enabled) {dt = std::min(dt, 0.1);}
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
    const double max_x = config_.research_enabled ?
      std::min(config_.max_linear_x, research_authorization_.max_linear_mps) : config_.max_linear_x;
    const double max_w = config_.research_enabled ?
      std::min(config_.max_angular_z, research_authorization_.max_angular_rps) : config_.max_angular_z;
    const double target_x = clamp(active_target.linear_x, config_.research_enabled ? 0.0 : -config_.max_reverse_x, max_x);
    const double target_w = clamp(active_target.angular_z, -max_w, max_w);
    const double linear_limit = std::abs(target_x) < std::abs(output_.linear_x) ?
      config_.max_linear_decel : config_.max_linear_accel;
    output_.linear_x = slew(output_.linear_x, target_x, linear_limit, dt);
    output_.angular_z = slew(output_.angular_z, target_w, config_.max_angular_accel, dt);
    if (config_.research_enabled && output_.linear_x <= 1e-9) {output_.angular_z = 0.0;}
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

void MotionGuardCore::invalidate_research_clock()
{
  clock_blocked_control_epoch_ = std::max(clock_blocked_control_epoch_, research_authorization_.control_epoch);
  hard_stop();
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
  if (config_.research_enabled && (last_localization_source_ns_ <= 0 ||
    last_localization_source_ns_ > now_ns + 10000000 ||
    (now_ns - last_localization_source_ns_) / 1e9 > config_.localization_status_timeout_sec)) {
    return {false, "localization_source_stale", GuardState::LOCALIZATION_BLOCKED};
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
  const GateResult localization = config_.research_enabled ? research_gate(now_ns) : localization_gate(now_ns);
  if (!localization.allowed) {
    return localization;
  }
  return payload_gate(now_ns);
}

bool MotionGuardCore::on_research_authorization(const ResearchAuthorization & value, int64_t now_ns)
{
  if (!config_.research_enabled || value.owner_id != config_.research_owner_id) {return false;}
  if (value.control_epoch < research_authorization_.control_epoch ||
    (value.control_epoch == research_authorization_.control_epoch && value.stamp_ns < research_authorization_.stamp_ns)) {
    return false;
  }
  const bool changed = value.control_epoch != research_authorization_.control_epoch ||
    value.odom_epoch != research_authorization_.odom_epoch || value.plan_stamp_ns != research_authorization_.plan_stamp_ns ||
    value.task_id != research_authorization_.task_id;
  research_authorization_ = value; research_mode_rx_ns_ = now_ns;
  if (changed) {research_command_ = {};}
  if (changed || !research_gate(now_ns).allowed) {hard_stop(); return true;}
  return false;
}

bool MotionGuardCore::on_research_row(const ResearchQuality & value, int64_t now_ns)
{
  if (value.stamp_ns < research_row_.stamp_ns && value.odom_epoch == research_row_.odom_epoch) {return false;}
  research_row_ = value; research_row_rx_ns_ = now_ns;
  if (config_.research_enabled && !research_gate(now_ns).allowed) {hard_stop(); return true;}
  return false;
}

bool MotionGuardCore::on_research_odom(const ResearchQuality & value, int64_t now_ns)
{
  if (value.stamp_ns < research_odom_.stamp_ns && value.odom_epoch == research_odom_.odom_epoch) {return false;}
  research_odom_ = value; research_odom_rx_ns_ = now_ns;
  if (config_.research_enabled && !research_gate(now_ns).allowed) {hard_stop(); return true;}
  return false;
}

bool MotionGuardCore::on_research_command(const ResearchCommand & value, int64_t now_ns)
{
  const auto & mode = research_authorization_;
  const double age = (now_ns - value.stamp_ns) / 1e9;
  if (!config_.research_enabled || control_mode_ != "navigation" ||
    !std::isfinite(value.velocity.linear_x) || !std::isfinite(value.velocity.angular_z) ||
    value.velocity.linear_x < 0.0 ||
    (value.velocity.linear_x <= 1e-9 && std::abs(value.velocity.angular_z) > 1e-9) ||
    value.stamp_ns <= 0 || age < -0.01 || age > config_.command_timeout_sec ||
    value.stamp_ns < research_command_.stamp_ns || value.control_epoch != mode.control_epoch ||
    value.odom_epoch != mode.odom_epoch || value.plan_stamp_ns != mode.plan_stamp_ns ||
    value.owner_id != mode.owner_id || value.task_id != mode.task_id || !motion_gate(now_ns).allowed) {
    hard_stop(); return false;
  }
  research_command_ = value; target_ = value.velocity; last_rx_ns_ = now_ns;
  if (!motion_gate(now_ns).allowed) {hard_stop(); return false;}
  return true;
}

bool MotionGuardCore::on_research_base_state(const ResearchBaseState & value, int64_t now_ns)
{
  const bool unsafe = !value.source_valid || !value.drive_permitted || value.emergency_stop ||
    value.manual_override || value.fault_active || !value.measured_velocity_valid;
  if (value.stamp_ns < research_base_.stamp_ns && !unsafe) {return false;}
  research_base_ = value; research_base_rx_ns_ = now_ns;
  if (config_.research_enabled && !research_gate(now_ns).allowed) {hard_stop(); return true;}
  return false;
}

MotionGuardCore::GateResult MotionGuardCore::research_gate(int64_t now_ns) const
{
  const auto blocked = [](const char * reason) {return GateResult{false, reason, GuardState::HEALTH_BLOCKED};};
  const auto fresh = [now_ns](int64_t stamp, int64_t receipt, double timeout) {
      return stamp > 0 && receipt > 0 && stamp <= now_ns + 10000000 && receipt <= now_ns + 10000000 &&
        (now_ns - stamp) / 1e9 <= timeout && (now_ns - receipt) / 1e9 <= timeout;
    };
  const auto & mode = research_authorization_;
  if (!config_.research_field_verified) {return blocked("research_field_unverified");}
  if (mode.control_epoch <= clock_blocked_control_epoch_) {return blocked("research_clock_epoch_revoked");}
  if (control_mode_ != "navigation") {return blocked("research_manual_mode");}
  if (!fresh(research_base_.stamp_ns, research_base_rx_ns_, config_.research_quality_timeout_sec) ||
    research_base_.robot_profile != config_.expected_robot_profile || !research_base_.source_valid ||
    !research_base_.drive_permitted || research_base_.emergency_stop || research_base_.manual_override ||
    research_base_.fault_active || !research_base_.measured_velocity_valid ||
    !std::isfinite(research_base_.measured_linear_mps) ||
    !std::isfinite(research_base_.measured_angular_rps)) {
    return blocked("research_base_state_or_drive_permission_invalid");
  }
  if (!fresh(mode.stamp_ns, research_mode_rx_ns_, config_.research_mode_timeout_sec) ||
    mode.valid_until_ns <= now_ns || !mode.authorized || mode.owner_id != config_.research_owner_id ||
    mode.task_id.empty() || mode.control_epoch == 0 || mode.odom_epoch == 0 || mode.plan_stamp_ns <= 0 ||
    !std::isfinite(mode.max_linear_mps) || mode.max_linear_mps <= 0.0 ||
    !std::isfinite(mode.max_angular_rps) || mode.max_angular_rps <= 0.0) {
    return blocked("research_authorization_invalid");
  }
  if (!fresh(research_odom_.stamp_ns, research_odom_rx_ns_, config_.research_quality_timeout_sec) ||
    !research_odom_.valid || research_odom_.odom_epoch != mode.odom_epoch ||
    !std::isfinite(research_odom_.quality) || research_odom_.quality < config_.research_min_odom_quality ||
    !std::isfinite(research_odom_.position_std_m) || research_odom_.position_std_m < 0.0 ||
    research_odom_.position_std_m > config_.research_max_position_std_m ||
    !std::isfinite(research_odom_.yaw_std_rad) || research_odom_.yaw_std_rad < 0.0 ||
    research_odom_.yaw_std_rad > config_.research_max_yaw_std_rad) {return blocked("research_odom_invalid");}
  if (last_rx_ns_ > 0 && !fresh(research_command_.stamp_ns, last_rx_ns_, config_.command_timeout_sec)) {
    return blocked("research_source_command_expired");
  }
  if (mode.mode != 0 && mode.mode != 2 && mode.mode != 3) {return blocked("research_mode_not_executable");}
  if (!fresh(research_row_.stamp_ns, research_row_rx_ns_, config_.research_quality_timeout_sec) ||
    !research_row_.clearance_valid || research_row_.odom_epoch != mode.odom_epoch) {
    return blocked("research_visibility_invalid");
  }
  const double speed = std::max({std::abs(output_.linear_x), std::abs(target_.linear_x),
    std::abs(research_base_.measured_linear_mps)});
  const double stop = speed * config_.research_stop_latency_sec +
    speed * speed / (2.0 * config_.research_braking_decel_mps2) + config_.research_clearance_margin_m;
  if (!std::isfinite(stop) || !std::isfinite(research_row_.clearance_front_m) ||
    !std::isfinite(research_row_.clearance_left_m) || !std::isfinite(research_row_.clearance_right_m) ||
    research_row_.clearance_front_m < stop || research_row_.clearance_left_m < config_.research_clearance_margin_m ||
    research_row_.clearance_right_m < config_.research_clearance_margin_m) {
    return blocked("research_stopping_clearance");
  }
  if (mode.mode == 0) {return localization_gate(now_ns);}
  if (!std::isfinite(mode.remaining_distance_m) || mode.remaining_distance_m < stop ||
    !std::isfinite(mode.remaining_time_sec) || mode.remaining_time_sec <= 0.0) {
    return blocked("research_budget_exhausted");
  }
  if (!research_row_.valid || !std::isfinite(research_row_.quality) ||
    research_row_.quality < config_.research_min_row_quality) {return blocked("research_row_invalid");}
  return {true, "research_local_authorized", GuardState::READY};
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
