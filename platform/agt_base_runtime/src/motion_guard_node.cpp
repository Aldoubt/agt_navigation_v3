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
    const std::string output_topic = declare_parameter<std::string>("output_topic", "/mux/cmd_vel");
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

    validate_config(rate_hz, config);
    core_ = MotionGuardCore(config);
    core_.set_initial_time(now_ns());
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
        const int64_t stamp = now_ns();
        const bool stopped = core_.on_localization_status(
          message->state, message->local_odom_fresh, message->global_correction_valid, stamp);
        if (stopped) {
          publish_immediate_stop(stamp);
        } else {
          publish_state(core_.snapshot(stamp));
        }
      });
    permission_sub_ = create_subscription<std_msgs::msg::Bool>(
      permission_topic, 20, [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        const int64_t stamp = now_ns();
        if (core_.on_payload_permission(message->data, stamp)) {
          publish_immediate_stop(stamp);
        }
      });

    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / std::max(rate_hz, 1.0)),
      std::bind(&MotionGuardNode::on_tick, this));
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
  }

  int64_t now_ns()
  {
    return get_clock()->now().nanoseconds();
  }

  void on_tick()
  {
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
