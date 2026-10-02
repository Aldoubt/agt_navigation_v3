#pragma once

#include <cstdint>
#include <string>

namespace agt_base_runtime
{

struct MotionGuardConfig
{
  double command_timeout_sec{0.25};
  bool require_localization_status{true};
  double localization_status_timeout_sec{0.75};
  bool allow_degraded_localization{false};
  bool require_payload_drive_permission{false};
  double payload_drive_permission_timeout_sec{0.5};
  double max_linear_x{0.55};
  double max_reverse_x{0.20};
  double max_angular_z{0.65};
  double max_linear_accel{0.45};
  double max_linear_decel{0.80};
  double max_angular_accel{0.80};
};

struct GuardCommand
{
  double linear_x{0.0};
  double angular_z{0.0};
};

enum class GuardState
{
  READY,
  ACTIVE,
  STALE_COMMAND,
  LOCALIZATION_BLOCKED,
  HEALTH_BLOCKED,
};

struct GuardSnapshot
{
  GuardCommand output;
  GuardState state{GuardState::READY};
  std::string reason{"startup"};
  bool payload_hold{false};
};

class MotionGuardCore
{
public:
  explicit MotionGuardCore(MotionGuardConfig config = {});
  void set_initial_time(int64_t now_ns) {last_tick_ns_ = now_ns;}

  bool on_navigation_command(const GuardCommand & command, int64_t now_ns);
  bool on_manual_command(const GuardCommand & command, int64_t now_ns);
  bool on_control_mode(const std::string & mode);
  bool on_localization_status(
    uint8_t state, bool local_odom_fresh, bool global_correction_valid, int64_t now_ns);
  bool on_payload_permission(bool permitted, int64_t now_ns);

  GuardSnapshot tick(int64_t now_ns);
  GuardSnapshot snapshot(int64_t now_ns) const;
  void hard_stop();
  const GuardCommand & output() const {return output_;}
  const std::string & control_mode() const {return control_mode_;}

private:
  struct GateResult
  {
    bool allowed{false};
    std::string reason;
    GuardState blocked_state{GuardState::LOCALIZATION_BLOCKED};
  };

  GateResult localization_gate(int64_t now_ns) const;
  GateResult payload_gate(int64_t now_ns) const;
  GateResult motion_gate(int64_t now_ns) const;
  GuardSnapshot snapshot_for_gate(const GateResult & gate, int64_t now_ns) const;
  static double clamp(double value, double lo, double hi);
  static double slew(double current, double target, double limit, double dt);

  MotionGuardConfig config_;
  GuardCommand target_;
  GuardCommand manual_target_;
  GuardCommand output_;
  int64_t last_rx_ns_{0};
  int64_t last_manual_rx_ns_{0};
  int64_t last_tick_ns_{0};
  int64_t last_localization_rx_ns_{0};
  int64_t last_payload_rx_ns_{0};
  uint8_t localization_state_{0};
  bool have_localization_status_{false};
  bool local_odom_fresh_{false};
  bool global_correction_valid_{false};
  bool payload_permission_{false};
  std::string control_mode_{"navigation"};
};

const char * to_string(GuardState state);

}  // namespace agt_base_runtime
