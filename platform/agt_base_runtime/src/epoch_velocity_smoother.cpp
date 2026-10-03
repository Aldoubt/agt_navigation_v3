#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "agt_navigation_interfaces/msg/navigation_mode.hpp"
#include "agt_navigation_interfaces/msg/velocity_command.hpp"

// The envelope's source stamp/plan/epochs are copied unchanged. A timer tick is
// never evidence that the controller produced a new command.
class EpochVelocitySmoother final : public rclcpp::Node
{
  using Mode = agt_navigation_interfaces::msg::NavigationMode;
  using Command = agt_navigation_interfaces::msg::VelocityCommand;
public:
  EpochVelocitySmoother() : Node("agt_epoch_velocity_smoother")
  {
    enabled_ = declare_parameter("research_enabled", false);
    verified_ = declare_parameter("field_verified", false);
    owner_ = declare_parameter<std::string>("owner_id", "agt_task_continuity");
    timeout_ = declare_parameter("source_timeout_sec", 0.25);
    mode_timeout_ = declare_parameter("mode_timeout_sec", 0.25);
    const double rate = declare_parameter("smoothing_frequency", 50.0);
    accel_ = declare_parameter("max_linear_accel", 0.0);
    decel_ = declare_parameter("max_linear_decel", 0.0);
    angular_accel_ = declare_parameter("max_angular_accel", 0.0);
    if (!std::isfinite(rate) || rate <= 0.0 || !std::isfinite(timeout_) || timeout_ <= 0.0 ||
      !std::isfinite(mode_timeout_) || mode_timeout_ <= 0.0 || owner_.empty()) {
      throw std::invalid_argument("epoch smoother rate/timeouts/owner are invalid");
    }
    if (verified_ && (!std::isfinite(accel_) || accel_ <= 0.0 || !std::isfinite(decel_) || decel_ <= 0.0 ||
      !std::isfinite(angular_accel_) || angular_accel_ <= 0.0)) {
      throw std::invalid_argument("verified epoch smoother requires measured acceleration parameters");
    }
    publisher_ = create_publisher<Command>(declare_parameter<std::string>(
      "output_topic", "/agt/research/cmd_vel_smoothed"), rclcpp::QoS(1).reliable());
    mode_sub_ = create_subscription<Mode>(declare_parameter<std::string>(
      "mode_topic", "/agt/research/navigation_mode"), rclcpp::QoS(1).reliable(), [this](Mode::ConstSharedPtr m) {
        if (m->owner_id != owner_ || (have_mode_ && (m->control_epoch < mode_.control_epoch ||
          (m->control_epoch == mode_.control_epoch && rclcpp::Time(m->header.stamp) < rclcpp::Time(mode_.header.stamp))))) {return;}
        const bool changed = !have_mode_ || m->control_epoch != mode_.control_epoch ||
          m->odom_epoch != mode_.odom_epoch || m->plan_stamp != mode_.plan_stamp || m->task_id != mode_.task_id;
        mode_ = *m; have_mode_ = true; mode_received_ = std::chrono::steady_clock::now();
        if (changed || !authorized()) {clear(); publish_zero();}
      });
    command_sub_ = create_subscription<Command>(declare_parameter<std::string>(
      "input_topic", "/agt/research/cmd_vel_source"), rclcpp::QoS(1).reliable(), [this](Command::ConstSharedPtr c) {
        if (!authorized() || !matches(*c) || !source_fresh(*c) ||
          (have_command_ && rclcpp::Time(c->header.stamp) < rclcpp::Time(command_.header.stamp)) ||
          !std::isfinite(c->velocity.linear.x) || !std::isfinite(c->velocity.angular.z)) {
          clear(); publish_zero(); return;
        }
        if (c->velocity.linear.x < 0.0 ||
          (c->velocity.linear.x <= 1e-9 && std::abs(c->velocity.angular.z) > 1e-9)) {
          clear(); publish_zero(); return;
        }
        command_ = *c; have_command_ = true; command_received_ = std::chrono::steady_clock::now();
      });
    last_tick_ = std::chrono::steady_clock::now();
    last_ros_ns_ = get_clock()->now().nanoseconds();
    clock_progress_ = last_tick_;
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {tick();});
  }
private:
  bool authorized()
  {
    const auto now = get_clock()->now();
    const double age = (now - rclcpp::Time(mode_.header.stamp)).seconds();
    return enabled_ && verified_ && have_mode_ && mode_.authorized && mode_.control_epoch > 0 &&
      mode_.odom_epoch > 0 && !mode_.task_id.empty() && rclcpp::Time(mode_.plan_stamp).nanoseconds() > 0 &&
      mode_.control_epoch > clock_blocked_epoch_ &&
      age >= -0.01 && age <= mode_timeout_ && rclcpp::Time(mode_.valid_until) > now &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - mode_received_).count() <= mode_timeout_ &&
      (mode_.mode == Mode::GLOBAL || mode_.mode == Mode::LOCAL_TASK || mode_.mode == Mode::RECOVERING) &&
      std::isfinite(mode_.max_linear_mps) && mode_.max_linear_mps > 0.0 &&
      std::isfinite(mode_.max_angular_rps) && mode_.max_angular_rps > 0.0;
  }
  bool source_fresh(const Command & c)
  {
    const double age = (get_clock()->now() - rclcpp::Time(c.header.stamp)).seconds();
    return rclcpp::Time(c.header.stamp).nanoseconds() > 0 && age >= -0.01 && age <= timeout_;
  }
  bool matches(const Command & c) const
  {return c.owner_id == owner_ && c.task_id == mode_.task_id && c.control_epoch == mode_.control_epoch &&
      c.odom_epoch == mode_.odom_epoch && c.plan_stamp == mode_.plan_stamp;}
  static double slew(double value, double target, double amount)
  {return value + std::clamp(target - value, -amount, amount);}
  void clear() {have_command_ = false; linear_ = 0.0; angular_ = 0.0;}
  void publish_zero()
  {
    Command zero = command_; zero.velocity = geometry_msgs::msg::Twist(); publisher_->publish(zero);
  }
  void tick()
  {
    const auto now = std::chrono::steady_clock::now();
    const auto ros_ns = get_clock()->now().nanoseconds();
    if (ros_ns < last_ros_ns_) {clock_blocked_epoch_ = std::max(clock_blocked_epoch_, mode_.control_epoch);}
    if (ros_ns != last_ros_ns_) {clock_progress_ = now;}
    if (std::chrono::duration<double>(now - clock_progress_).count() > mode_timeout_) {
      clock_blocked_epoch_ = std::max(clock_blocked_epoch_, mode_.control_epoch);
    }
    last_ros_ns_ = ros_ns;
    const double dt = std::min(0.1, std::chrono::duration<double>(now - last_tick_).count()); last_tick_ = now;
    if (!authorized() || !have_command_ || !matches(command_) || !source_fresh(command_) ||
      std::chrono::duration<double>(now - command_received_).count() > timeout_) {
      clear(); publish_zero(); return;
    }
    const double target = std::clamp(command_.velocity.linear.x, 0.0, mode_.max_linear_mps);
    linear_ = slew(linear_, target, (std::abs(target) < std::abs(linear_) ? decel_ : accel_) * dt);
    angular_ = slew(angular_, std::clamp(command_.velocity.angular.z, -mode_.max_angular_rps,
      mode_.max_angular_rps), angular_accel_ * dt);
    if (linear_ <= 1e-9) {angular_ = 0.0;}
    auto output = command_; output.velocity.linear.x = linear_; output.velocity.angular.z = angular_;
    output.velocity.linear.y = output.velocity.linear.z = 0.0;
    output.velocity.angular.x = output.velocity.angular.y = 0.0;
    publisher_->publish(output);
  }
  bool enabled_{false}, verified_{false}, have_mode_{false}, have_command_{false};
  std::string owner_;
  double timeout_, mode_timeout_, accel_, decel_, angular_accel_, linear_{0.0}, angular_{0.0};
  Mode mode_; Command command_;
  int64_t last_ros_ns_{0};
  uint64_t clock_blocked_epoch_{0};
  std::chrono::steady_clock::time_point mode_received_, command_received_, last_tick_, clock_progress_;
  rclcpp::Publisher<Command>::SharedPtr publisher_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<EpochVelocitySmoother>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("agt_epoch_velocity_smoother"), "%s", e.what());
    rclcpp::shutdown(); return 2;}
  rclcpp::shutdown(); return 0;
}
