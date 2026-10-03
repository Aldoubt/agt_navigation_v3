#include "agt_base_runtime/motion_guard_core.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "agt_robot_interfaces/msg/localization_status.hpp"
#include "agt_robot_interfaces/msg/base_state.hpp"
#include "agt_navigation_interfaces/msg/navigation_mode.hpp"
#include "agt_navigation_interfaces/msg/local_row_state.hpp"
#include "agt_navigation_interfaces/msg/odom_quality.hpp"
#include "agt_navigation_interfaces/msg/velocity_command.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace agt_base_runtime
{

class MotionGuardNode final : public rclcpp::Node
{
public:
  MotionGuardNode()
  : Node("agt_cmd_vel_guard")
  {
    const std::string input_topic = declare_parameter<std::string>("input_topic", "/cmd_vel_smoothed");
    const std::string manual_topic = declare_parameter<std::string>("manual_input_topic", "/agt/hmi/cmd_vel");
    const std::string mode_topic = declare_parameter<std::string>("control_mode_topic", "/agt/control/mode");
    const std::string default_mode = normalize_mode(
      declare_parameter<std::string>("default_control_mode", "navigation"));
    const std::string output_topic = declare_parameter<std::string>(
      "output_topic", "/agt/base/cmd_vel");
    const std::string localization_topic = declare_parameter<std::string>(
      "localization_status_topic", "/agt/localization/status");
    const std::string permission_topic = declare_parameter<std::string>(
      "payload_drive_permission_topic", "/agt/payload/drive_permission");
    const std::string hold_topic = declare_parameter<std::string>(
      "payload_hold_topic", "/agt/cmd_vel_guard/payload_hold");
    const std::string state_topic = declare_parameter<std::string>(
      "state_topic", "/agt/cmd_vel_guard/state");

    const double rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);
    MotionGuardConfig config;
    config.command_timeout_sec = declare_parameter<double>("command_timeout_sec", 0.25);
    config.require_localization_status = declare_parameter<bool>("require_localization_status", true);
    config.localization_status_timeout_sec = declare_parameter<double>(
      "localization_status_timeout_sec", 0.75);
    config.allow_degraded_localization = declare_parameter<bool>("allow_degraded_localization", false);
    config.require_payload_drive_permission = declare_parameter<bool>(
      "require_payload_drive_permission", false);
    config.payload_drive_permission_timeout_sec = declare_parameter<double>(
      "payload_drive_permission_timeout_sec", 0.5);
    config.max_linear_x = declare_parameter<double>("max_linear_x", 0.55);
    config.max_reverse_x = declare_parameter<double>("max_reverse_x", 0.20);
    config.max_angular_z = declare_parameter<double>("max_angular_z", 0.65);
    config.max_linear_accel = declare_parameter<double>("max_linear_accel", 0.45);
    config.max_linear_decel = declare_parameter<double>("max_linear_decel", 0.80);
    config.max_angular_accel = declare_parameter<double>("max_angular_accel", 0.80);
    config.research_enabled = declare_parameter<bool>("research_enabled", false);
    config.research_field_verified = declare_parameter<bool>("research_field_verified", false);
    config.research_owner_id = declare_parameter<std::string>("research_owner_id", "agt_task_continuity");
    config.research_mode_timeout_sec = declare_parameter<double>("research_mode_timeout_sec", 0.25);
    config.research_quality_timeout_sec = declare_parameter<double>("research_quality_timeout_sec", 0.3);
    config.research_min_row_quality = declare_parameter<double>("research_min_row_quality", 0.0);
    config.research_min_odom_quality = declare_parameter<double>("research_min_odom_quality", 0.0);
    config.research_max_position_std_m = declare_parameter<double>("research_max_position_std_m", 0.0);
    config.research_max_yaw_std_rad = declare_parameter<double>("research_max_yaw_std_rad", 0.0);
    config.research_braking_decel_mps2 = declare_parameter<double>("research_braking_decel_mps2", 0.0);
    config.research_stop_latency_sec = declare_parameter<double>("research_stop_latency_sec", 0.0);
    config.research_clearance_margin_m = declare_parameter<double>("research_clearance_margin_m", 0.0);
    config.expected_robot_profile = declare_parameter<std::string>("expected_robot_profile", "bunker_v1");
    research_enabled_ = config.research_enabled;
    research_source_timeout_ = config.command_timeout_sec;
    research_mode_timeout_ = config.research_mode_timeout_sec;
    research_quality_timeout_ = config.research_quality_timeout_sec;
    require_payload_ = config.require_payload_drive_permission;
    payload_timeout_ = config.payload_drive_permission_timeout_sec;
    localization_timeout_ = config.localization_status_timeout_sec;

    validate_config(rate_hz, config);
    core_ = MotionGuardCore(config);
    core_.set_initial_time(now_ns());
    last_clock_ns_ = now_ns();
    clock_progress_wall_ = std::chrono::steady_clock::now();
    if (default_mode == "manual") {
      core_.on_control_mode(default_mode);
    }

    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, 20);
    hold_pub_ = create_publisher<std_msgs::msg::Bool>(hold_topic, 20);
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic, 20);
    nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic, 20, [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        core_.on_navigation_command(to_command(*message), now_ns());
      });
    manual_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      manual_topic, 20, [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        core_.on_manual_command(to_command(*message), now_ns());
      });
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      mode_topic, 10, [this](std_msgs::msg::String::ConstSharedPtr message) {
        const std::string mode = normalize_mode(message->data);
        if (mode != "navigation" && mode != "manual") {
          RCLCPP_WARN(get_logger(), "ignoring unsupported control mode '%s'", message->data.c_str());
          return;
        }
        if (core_.on_control_mode(mode)) {
          publish_immediate_stop(now_ns());
          RCLCPP_INFO(get_logger(), "control mode changed to %s; fresh command required", mode.c_str());
        }
      });
    localization_sub_ = create_subscription<agt_robot_interfaces::msg::LocalizationStatus>(
      localization_topic, 20,
      [this](agt_robot_interfaces::msg::LocalizationStatus::ConstSharedPtr message) {
        localization_wall_ = std::chrono::steady_clock::now();
        const int64_t stamp = now_ns();
        const bool stopped = core_.on_localization_status(
          message->state, message->local_odom_fresh, message->global_correction_valid, stamp,
          rclcpp::Time(message->stamp).nanoseconds());
        if (stopped) {
          publish_immediate_stop(stamp);
        } else {
          publish_state(core_.snapshot(stamp));
        }
      });
    permission_sub_ = create_subscription<std_msgs::msg::Bool>(
      permission_topic, 20, [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        const int64_t stamp = now_ns();
        payload_wall_ = std::chrono::steady_clock::now();
        if (core_.on_payload_permission(message->data, stamp)) {
          publish_immediate_stop(stamp);
        }
      });
    if (config.research_enabled) {
      research_mode_sub_ = create_subscription<agt_navigation_interfaces::msg::NavigationMode>(
        declare_parameter<std::string>("research_mode_topic", "/agt/research/navigation_mode"), rclcpp::QoS(1).reliable(),
        [this](agt_navigation_interfaces::msg::NavigationMode::ConstSharedPtr m) {
          research_mode_wall_ = std::chrono::steady_clock::now();
          ResearchAuthorization value;
          value.stamp_ns = rclcpp::Time(m->header.stamp).nanoseconds();
          value.valid_until_ns = rclcpp::Time(m->valid_until).nanoseconds();
          value.plan_stamp_ns = rclcpp::Time(m->plan_stamp).nanoseconds();
          value.mode = m->mode; value.owner_id = m->owner_id; value.task_id = m->task_id;
          value.control_epoch = m->control_epoch; value.odom_epoch = m->odom_epoch;
          value.authorized = m->authorized; value.max_linear_mps = m->max_linear_mps;
          value.max_angular_rps = m->max_angular_rps; value.remaining_distance_m = m->remaining_distance_m;
          value.remaining_time_sec = m->remaining_time_sec;
          if (core_.on_research_authorization(value, now_ns())) {publish_immediate_stop(now_ns());}
        });
      research_row_sub_ = create_subscription<agt_navigation_interfaces::msg::LocalRowState>(
        declare_parameter<std::string>("research_row_topic", "/agt/local_row/state"), rclcpp::QoS(1).best_effort(),
        [this](agt_navigation_interfaces::msg::LocalRowState::ConstSharedPtr m) {
          research_row_wall_ = std::chrono::steady_clock::now();
          ResearchQuality q; q.stamp_ns = rclcpp::Time(m->header.stamp).nanoseconds(); q.odom_epoch = m->odom_epoch;
          q.valid = m->row_valid && m->ground_valid; q.clearance_valid = m->clearance_valid; q.quality = m->quality;
          q.clearance_left_m = m->clearance_left_m; q.clearance_right_m = m->clearance_right_m;
          q.clearance_front_m = m->clearance_front_m;
          if (core_.on_research_row(q, now_ns())) {publish_immediate_stop(now_ns());}
        });
      research_odom_sub_ = create_subscription<agt_navigation_interfaces::msg::OdomQuality>(
        declare_parameter<std::string>("research_odom_topic", "/agt/odometry/quality"), rclcpp::QoS(1).reliable(),
        [this](agt_navigation_interfaces::msg::OdomQuality::ConstSharedPtr m) {
          research_odom_wall_ = std::chrono::steady_clock::now();
          ResearchQuality q; q.stamp_ns = rclcpp::Time(m->header.stamp).nanoseconds(); q.odom_epoch = m->odom_epoch;
          q.valid = m->valid; q.quality = m->quality; q.position_std_m = m->position_std_m; q.yaw_std_rad = m->yaw_std_rad;
          if (core_.on_research_odom(q, now_ns())) {publish_immediate_stop(now_ns());}
        });
      research_command_sub_ = create_subscription<agt_navigation_interfaces::msg::VelocityCommand>(
        declare_parameter<std::string>("research_command_topic", "/agt/research/cmd_vel_smoothed"), rclcpp::QoS(1).reliable(),
        [this](agt_navigation_interfaces::msg::VelocityCommand::ConstSharedPtr m) {
          research_command_wall_ = std::chrono::steady_clock::now();
          ResearchCommand c; c.velocity = to_command(m->velocity); c.stamp_ns = rclcpp::Time(m->header.stamp).nanoseconds();
          c.plan_stamp_ns = rclcpp::Time(m->plan_stamp).nanoseconds(); c.owner_id = m->owner_id; c.task_id = m->task_id;
          c.control_epoch = m->control_epoch; c.odom_epoch = m->odom_epoch;
          if (!core_.on_research_command(c, now_ns())) {publish_immediate_stop(now_ns());}
        });
      research_base_sub_ = create_subscription<agt_robot_interfaces::msg::BaseState>(
        declare_parameter<std::string>("base_state_topic", "/agt/base/state"), rclcpp::QoS(1).reliable(),
        [this](agt_robot_interfaces::msg::BaseState::ConstSharedPtr m) {
          research_base_wall_ = std::chrono::steady_clock::now();
          ResearchBaseState s; s.stamp_ns = rclcpp::Time(m->stamp).nanoseconds(); s.robot_profile = m->robot_profile;
          s.source_valid = m->source_valid; s.drive_permitted = m->drive_permitted;
          s.emergency_stop = m->emergency_stop; s.manual_override = m->manual_override; s.fault_active = m->fault_active;
          s.measured_velocity_valid = m->measured_velocity_valid;
          s.measured_linear_mps = m->measured_velocity.linear.x;
          s.measured_angular_rps = m->measured_velocity.angular.z;
          if (core_.on_research_base_state(s, now_ns())) {publish_immediate_stop(now_ns());}
        });
    }

    if (research_enabled_) {
      timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), std::bind(&MotionGuardNode::on_tick, this));
    } else {
      timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<double>(1.0 / std::max(rate_hz, 1.0)),
        std::bind(&MotionGuardNode::on_tick, this));
    }
    RCLCPP_INFO(
      get_logger(), "C++ motion guard: %s, manual=%s -> %s @ %.1f Hz; fail-closed localization gate%s",
      input_topic.c_str(), manual_topic.c_str(), output_topic.c_str(), rate_hz,
      config.require_payload_drive_permission ? "; payload permission required" : "");
  }

  void publish_shutdown_zeros()
  {
    const auto zero = geometry_msgs::msg::Twist();
    for (int i = 0; i < 3; ++i) {
      output_pub_->publish(zero);
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
  }

private:
  static GuardCommand to_command(const geometry_msgs::msg::Twist & message)
  {
    return {message.linear.x, message.angular.z};
  }

  static std::string normalize_mode(std::string mode)
  {
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    const auto first = mode.find_first_not_of(" \t\r\n");
    const auto last = mode.find_last_not_of(" \t\r\n");
    return first == std::string::npos ? std::string() : mode.substr(first, last - first + 1);
  }

  static void validate_config(double rate_hz, const MotionGuardConfig & config)
  {
    const double positive_values[] = {
      rate_hz, config.command_timeout_sec, config.localization_status_timeout_sec,
      config.payload_drive_permission_timeout_sec, config.max_linear_x,
      config.max_angular_z, config.max_linear_accel, config.max_linear_decel,
      config.max_angular_accel};
    for (const double value : positive_values) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("motion guard rates, timeouts and positive limits must be finite and > 0");
      }
    }
    if (!std::isfinite(config.max_reverse_x) || config.max_reverse_x < 0.0) {
      throw std::invalid_argument("motion guard max_reverse_x must be finite and >= 0");
    }
    if (config.research_enabled && config.research_field_verified) {
      const double values[] = {config.research_mode_timeout_sec, config.research_quality_timeout_sec,
        config.research_min_row_quality, config.research_min_odom_quality, config.research_max_position_std_m,
        config.research_max_yaw_std_rad, config.research_braking_decel_mps2,
        config.research_stop_latency_sec, config.research_clearance_margin_m};
      for (double value : values) {
        if (!std::isfinite(value) || value <= 0.0) {throw std::invalid_argument("verified research guard needs explicit positive quality and measured safety parameters");}
      }
      if (!config.require_localization_status || config.research_owner_id.empty() || config.expected_robot_profile.empty() ||
        config.research_min_row_quality > 1.0 || config.research_min_odom_quality > 1.0) {
        throw std::invalid_argument("research guard requires baseline global localization gate and valid owner/quality limits");
      }
      const double minimum_latency = std::max({config.command_timeout_sec,
        config.research_mode_timeout_sec, config.research_quality_timeout_sec,
        config.localization_status_timeout_sec,
        config.require_payload_drive_permission ? config.payload_drive_permission_timeout_sec : 0.0});
      if (config.research_stop_latency_sec < minimum_latency) {
        throw std::invalid_argument("measured research stop latency must include the configured watchdog detection intervals");
      }
    }
  }

  int64_t now_ns()
  {
    return get_clock()->now().nanoseconds();
  }

  void on_tick()
  {
    if (research_enabled_) {
      const auto wall_now = std::chrono::steady_clock::now();
      const auto clock_now = now_ns();
      if (clock_now < last_clock_ns_) {core_.invalidate_research_clock();}
      if (clock_now != last_clock_ns_) {clock_progress_wall_ = wall_now;}
      if (std::chrono::duration<double>(wall_now - clock_progress_wall_).count() > research_quality_timeout_) {
        core_.invalidate_research_clock();
      }
      last_clock_ns_ = clock_now;
      const auto fresh = [wall_now](std::chrono::steady_clock::time_point time, double timeout) {
          return time.time_since_epoch().count() > 0 && std::chrono::duration<double>(wall_now - time).count() <= timeout;
        };
      const auto mode = core_.snapshot(now_ns());
      if (!fresh(research_mode_wall_, research_mode_timeout_) ||
        !fresh(research_command_wall_, research_source_timeout_) ||
        !fresh(research_odom_wall_, research_quality_timeout_) ||
        !fresh(research_base_wall_, research_quality_timeout_) ||
        !fresh(research_row_wall_, research_quality_timeout_) ||
        (mode.reason == "localized" && !fresh(localization_wall_, localization_timeout_)) ||
        (require_payload_ && !fresh(payload_wall_, payload_timeout_))) {
        core_.hard_stop();
      }
    }
    const GuardSnapshot snapshot = core_.tick(now_ns());
    geometry_msgs::msg::Twist output;
    output.linear.x = snapshot.output.linear_x;
    output.angular.z = snapshot.output.angular_z;
    output_pub_->publish(output);
    std_msgs::msg::Bool hold;
    hold.data = snapshot.payload_hold;
    hold_pub_->publish(hold);
    publish_state(snapshot);
  }

  void publish_immediate_stop(int64_t stamp)
  {
    geometry_msgs::msg::Twist zero;
    output_pub_->publish(zero);
    publish_state(core_.snapshot(stamp));
  }

  void publish_state(const GuardSnapshot & snapshot)
  {
    std_msgs::msg::String state;
    state.data = to_string(snapshot.state);
    state_pub_->publish(state);
  }

  MotionGuardCore core_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hold_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<agt_robot_interfaces::msg::LocalizationStatus>::SharedPtr localization_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permission_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<agt_navigation_interfaces::msg::NavigationMode>::SharedPtr research_mode_sub_;
  rclcpp::Subscription<agt_navigation_interfaces::msg::LocalRowState>::SharedPtr research_row_sub_;
  rclcpp::Subscription<agt_navigation_interfaces::msg::OdomQuality>::SharedPtr research_odom_sub_;
  rclcpp::Subscription<agt_navigation_interfaces::msg::VelocityCommand>::SharedPtr research_command_sub_;
  rclcpp::Subscription<agt_robot_interfaces::msg::BaseState>::SharedPtr research_base_sub_;
  bool research_enabled_{false}, require_payload_{false};
  double research_source_timeout_{0.25}, research_mode_timeout_{0.25}, research_quality_timeout_{0.3}, payload_timeout_{0.5};
  double localization_timeout_{0.75};
  int64_t last_clock_ns_{0};
  std::chrono::steady_clock::time_point research_mode_wall_, research_row_wall_, research_odom_wall_,
    research_command_wall_, research_base_wall_, payload_wall_, localization_wall_, clock_progress_wall_;
};

}  // namespace agt_base_runtime

namespace
{
volatile std::sig_atomic_t shutdown_requested = 0;

void request_shutdown(int)
{
  shutdown_requested = 1;
}
}

int main(int argc, char ** argv)
{
  rclcpp::init(
    argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_shutdown);
  std::signal(SIGTERM, request_shutdown);
  try {
    auto node = std::make_shared<agt_base_runtime::MotionGuardNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && shutdown_requested == 0) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
    node->publish_shutdown_zeros();
    executor.remove_node(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("agt_cmd_vel_guard"), "motion guard failed: %s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
