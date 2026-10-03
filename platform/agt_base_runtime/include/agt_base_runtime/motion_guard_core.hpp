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
  bool research_enabled{false};
  bool research_field_verified{false};
  std::string research_owner_id{"agt_task_continuity"};
  double research_mode_timeout_sec{0.25};
  double research_quality_timeout_sec{0.3};
  double research_min_row_quality{0.0};
  double research_min_odom_quality{0.0};
  double research_max_position_std_m{0.0};
  double research_max_yaw_std_rad{0.0};
  double research_braking_decel_mps2{0.0};
  double research_stop_latency_sec{0.0};
  double research_clearance_margin_m{0.0};
  std::string expected_robot_profile{"bunker_v1"};
};

struct GuardCommand
{
  double linear_x{0.0};
  double angular_z{0.0};
};

struct ResearchAuthorization
{
  int64_t stamp_ns{0}, valid_until_ns{0}, plan_stamp_ns{0};
  uint8_t mode{4};
  std::string owner_id, task_id;
  uint64_t control_epoch{0}, odom_epoch{0};
  bool authorized{false};
  double max_linear_mps{0.0}, max_angular_rps{0.0};
  double remaining_distance_m{0.0}, remaining_time_sec{0.0};
};

struct ResearchQuality
{
  int64_t stamp_ns{0};
  uint64_t odom_epoch{0};
  bool valid{false}, clearance_valid{false};
  double quality{0.0}, position_std_m{0.0}, yaw_std_rad{0.0};
  double clearance_left_m{0.0}, clearance_right_m{0.0}, clearance_front_m{0.0};
};

struct ResearchCommand
{
  GuardCommand velocity;
  int64_t stamp_ns{0}, plan_stamp_ns{0};
  std::string owner_id, task_id;
  uint64_t control_epoch{0}, odom_epoch{0};
};

struct ResearchBaseState
{
  int64_t stamp_ns{0};
  std::string robot_profile;
  bool source_valid{false}, drive_permitted{false};
  bool emergency_stop{false}, manual_override{false}, fault_active{false};
  bool measured_velocity_valid{false};
  double measured_linear_mps{0.0}, measured_angular_rps{0.0};
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
    uint8_t state, bool local_odom_fresh, bool global_correction_valid, int64_t now_ns,
    int64_t source_stamp_ns = 0);
  bool on_payload_permission(bool permitted, int64_t now_ns);
  bool on_research_authorization(const ResearchAuthorization &, int64_t now_ns);
  bool on_research_row(const ResearchQuality &, int64_t now_ns);
  bool on_research_odom(const ResearchQuality &, int64_t now_ns);
  bool on_research_command(const ResearchCommand &, int64_t now_ns);
  bool on_research_base_state(const ResearchBaseState &, int64_t now_ns);

  GuardSnapshot tick(int64_t now_ns);
  GuardSnapshot snapshot(int64_t now_ns) const;
  void hard_stop();
  void invalidate_research_clock();
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
  GateResult research_gate(int64_t now_ns) const;
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
  int64_t last_localization_source_ns_{0};
  int64_t last_payload_rx_ns_{0};
  uint8_t localization_state_{0};
  bool have_localization_status_{false};
  bool local_odom_fresh_{false};
  bool global_correction_valid_{false};
  bool payload_permission_{false};
  std::string control_mode_{"navigation"};
  ResearchAuthorization research_authorization_;
  ResearchQuality research_row_, research_odom_;
  ResearchCommand research_command_;
  ResearchBaseState research_base_;
  int64_t research_mode_rx_ns_{0}, research_row_rx_ns_{0}, research_odom_rx_ns_{0};
  int64_t research_base_rx_ns_{0};
  uint64_t clock_blocked_control_epoch_{0};
};

const char * to_string(GuardState state);

}  // namespace agt_base_runtime
