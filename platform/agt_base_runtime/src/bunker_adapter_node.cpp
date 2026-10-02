#include <algorithm>
#include <chrono>
#include <csignal>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "agt_base_runtime/bunker_adapter.hpp"
#include "bunker_msgs/msg/bunker_rc_state.hpp"
#include "bunker_msgs/msg/bunker_status.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agt_base_runtime
{
namespace
{
using SteadyClock = std::chrono::steady_clock;
using Seconds = std::chrono::duration<double>;

diagnostic_msgs::msg::KeyValue key_value(std::string key, std::string value)
{
  diagnostic_msgs::msg::KeyValue entry;
  entry.key = std::move(key);
  entry.value = std::move(value);
  return entry;
}
}  // namespace

class BunkerAdapterNode final : public rclcpp::Node
{
public:
  BunkerAdapterNode()
  : Node("agt_bunker_base_adapter"), last_command_rx_(SteadyClock::now())
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/agt/base/cmd_vel");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/mux/cmd_vel");
    const auto odom_input_topic = declare_parameter<std::string>(
      "odom_input_topic", "/wheel/odom");
    const auto odom_output_topic = declare_parameter<std::string>(
      "odom_output_topic", "/agt/base/odom");
    const auto status_topic = declare_parameter<std::string>(
      "driver_status_topic", "/bunker_status");
    const auto rc_topic = declare_parameter<std::string>(
      "remote_status_topic", "/bunker_rc_state");
    const auto state_topic = declare_parameter<std::string>(
      "state_topic", "/agt/base/state");
    const auto health_topic = declare_parameter<std::string>(
      "health_topic", "/agt/base/health");
    const double rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.15);
    driver_timeout_sec_ = declare_parameter<double>("driver_timeout_sec", 0.5);

    if (!std::isfinite(rate_hz) || rate_hz <= 0.0 ||
      !std::isfinite(command_timeout_sec_) || command_timeout_sec_ <= 0.0 ||
      !std::isfinite(driver_timeout_sec_) || driver_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("adapter rate and timeouts must be finite and positive");
    }

    ChassisProfile profile;
    profile.profile_id = "bunker_v1";
    profile.kinematics = KinematicType::kSkidSteer;
    std::string reason;
    if (!adapter_.configure(profile, reason)) {
      throw std::invalid_argument("Bunker adapter configuration rejected: " + reason);
    }

    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, 20);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_output_topic, 20);
    state_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(state_topic, 10);
    health_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(health_topic, 10);

    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic, 20, std::bind(&BunkerAdapterNode::on_command, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_input_topic, 20,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        odom_received_ = true;
        last_odom_rx_ = SteadyClock::now();
        odom_pub_->publish(adapter_.normalize_odometry(*message));
      });
    status_sub_ = create_subscription<bunker_msgs::msg::BunkerStatus>(
      status_topic, 10, [this](bunker_msgs::msg::BunkerStatus::ConstSharedPtr message) {
        latest_status_ = *message;
        status_received_ = true;
        last_status_rx_ = SteadyClock::now();
      });
    rc_sub_ = create_subscription<bunker_msgs::msg::BunkerRCState>(
      rc_topic, 10, [this](bunker_msgs::msg::BunkerRCState::ConstSharedPtr message) {
        latest_rc_ = *message;
        rc_received_ = true;
        last_rc_rx_ = SteadyClock::now();
      });

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    command_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&BunkerAdapterNode::publish_command, this));
    diagnostic_timer_ = create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&BunkerAdapterNode::publish_diagnostics, this));

    RCLCPP_INFO(
      get_logger(), "Bunker adapter: guarded %s -> %s @ %.1f Hz; odom %s -> %s; TF disabled",
      input_topic.c_str(), output_topic.c_str(), rate_hz,
      odom_input_topic.c_str(), odom_output_topic.c_str());
  }

  void publish_shutdown_zeros()
  {
    const geometry_msgs::msg::Twist zero;
    for (int i = 0; i < 3; ++i) {
      command_pub_->publish(zero);
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
  }

private:
  bool fresh(const bool received, const SteadyClock::time_point last, const double timeout) const
  {
    return received && Seconds(SteadyClock::now() - last).count() <= timeout;
  }

  void on_command(const geometry_msgs::msg::Twist & request)
  {
    const auto converted = adapter_.convert(request, 0.02);
    latest_command_ = geometry_msgs::msg::Twist();
    if (converted.valid) {
      latest_command_.linear.x = converted.linear_x_mps;
      latest_command_.angular.z = converted.angular_z_radps;
    } else {
      ++invalid_command_total_;
    }
    last_command_rx_ = SteadyClock::now();
    command_received_ = true;
  }

  void publish_command()
  {
    const bool command_is_fresh = fresh(
      command_received_, last_command_rx_, command_timeout_sec_);
    command_pub_->publish(command_is_fresh ? latest_command_ : geometry_msgs::msg::Twist());
  }

  diagnostic_msgs::msg::DiagnosticStatus status(
    const std::string & name, const uint8_t level, const std::string & message,
    std::vector<diagnostic_msgs::msg::KeyValue> values = {}) const
  {
    diagnostic_msgs::msg::DiagnosticStatus result;
    result.name = name;
    result.level = level;
    result.message = message;
    result.hardware_id = "bunker";
    result.values = std::move(values);
    return result;
  }

  void publish_diagnostics()
  {
    const bool command_is_fresh = fresh(command_received_, last_command_rx_, command_timeout_sec_);
    const bool odom_is_fresh = fresh(odom_received_, last_odom_rx_, driver_timeout_sec_);
    const bool status_is_fresh = fresh(status_received_, last_status_rx_, driver_timeout_sec_);
    const bool rc_is_fresh = fresh(rc_received_, last_rc_rx_, driver_timeout_sec_);
    diagnostic_msgs::msg::DiagnosticArray state;
    state.header.stamp = get_clock()->now();
    std::vector<diagnostic_msgs::msg::KeyValue> state_values{
      key_value("kinematics", "skid_steer"),
      key_value("command_input", command_is_fresh ? "fresh" : "stale"),
      key_value("wheel_odom", odom_is_fresh ? "fresh" : "stale"),
      key_value("remote_feedback", rc_is_fresh ? "fresh" : "stale"),
      key_value("invalid_command_total", std::to_string(invalid_command_total_)),
    };
    if (status_received_) {
      state_values.push_back(key_value(
        "driver_vehicle_state_code", std::to_string(latest_status_.vehicle_state)));
      state_values.push_back(key_value(
        "driver_control_mode_code", std::to_string(latest_status_.control_mode)));
      state_values.push_back(key_value(
        "driver_error_code", std::to_string(latest_status_.error_code)));
    }
    if (rc_received_) {
      state_values.push_back(key_value("remote_switch_a", std::to_string(latest_rc_.swa)));
      state_values.push_back(key_value("remote_switch_b", std::to_string(latest_rc_.swb)));
      state_values.push_back(key_value("remote_switch_c", std::to_string(latest_rc_.swc)));
      state_values.push_back(key_value("remote_switch_d", std::to_string(latest_rc_.swd)));
    }
    const auto state_level = (status_is_fresh && rc_is_fresh) ?
      diagnostic_msgs::msg::DiagnosticStatus::OK : diagnostic_msgs::msg::DiagnosticStatus::WARN;
    state.status.push_back(status(
      "agt_base_runtime/bunker/state", state_level,
      "raw driver/remote state is reported for diagnostics; software does not arbitrate it",
      std::move(state_values)));
    state_pub_->publish(state);

    diagnostic_msgs::msg::DiagnosticArray health;
    health.header.stamp = state.header.stamp;
    const auto warn = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    const auto ok = diagnostic_msgs::msg::DiagnosticStatus::OK;
    health.status.push_back(status(
      "agt_base_runtime/bunker/command_input", command_is_fresh ? ok : warn,
      command_is_fresh ? "guarded command stream fresh" : "guarded command absent or stale"));
    health.status.push_back(status(
      "agt_base_runtime/bunker/wheel_odom", odom_is_fresh ? ok : warn,
      odom_is_fresh ? "wheel odometry fresh" : "wheel odometry absent or stale"));
    health.status.push_back(status(
      "agt_base_runtime/bunker/driver_status", status_is_fresh ? ok : warn,
      status_is_fresh ? "driver status fresh; raw error code is diagnostic only" :
      "driver status absent or stale", status_received_ ?
      std::vector<diagnostic_msgs::msg::KeyValue>{key_value(
        "driver_error_code", std::to_string(latest_status_.error_code))} :
      std::vector<diagnostic_msgs::msg::KeyValue>{}));
    health.status.push_back(status(
      "agt_base_runtime/bunker/remote_feedback", rc_is_fresh ? ok : warn,
      rc_is_fresh ? "remote feedback fresh; arbitration remains in the chassis" :
      "remote feedback absent or stale"));
    health_pub_->publish(health);
  }

  BunkerAdapter adapter_;
  double command_timeout_sec_{0.15};
  double driver_timeout_sec_{0.5};
  bool command_received_{false};
  bool odom_received_{false};
  bool status_received_{false};
  bool rc_received_{false};
  uint64_t invalid_command_total_{0};
  geometry_msgs::msg::Twist latest_command_;
  bunker_msgs::msg::BunkerStatus latest_status_;
  bunker_msgs::msg::BunkerRCState latest_rc_;
  SteadyClock::time_point last_command_rx_;
  SteadyClock::time_point last_odom_rx_;
  SteadyClock::time_point last_status_rx_;
  SteadyClock::time_point last_rc_rx_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerRCState>::SharedPtr rc_sub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace agt_base_runtime

namespace
{
volatile std::sig_atomic_t shutdown_requested = 0;

void request_shutdown(int)
{
  shutdown_requested = 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_shutdown);
  std::signal(SIGTERM, request_shutdown);
  try {
    auto node = std::make_shared<agt_base_runtime::BunkerAdapterNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && shutdown_requested == 0) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
    node->publish_shutdown_zeros();
    executor.remove_node(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("agt_bunker_base_adapter"), "Bunker adapter failed: %s", error.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
