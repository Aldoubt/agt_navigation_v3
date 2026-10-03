#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "agt_robot_interfaces/msg/base_state.hpp"

// A common SI Twist boundary for independently configured tracked drivers.
// Vendor CAN encoding remains inside the selected driver/protocol bridge.
class TrackedTwistAdapter : public rclcpp::Node
{
  using Clock = std::chrono::steady_clock;
public:
  TrackedTwistAdapter() : Node("agt_tracked_twist_adapter")
  {
    profile_ = declare_parameter<std::string>("robot_profile", "");
    verified_ = declare_parameter<bool>("field_verified", false);
    const auto input = declare_parameter<std::string>("input_topic", "/agt/base/cmd_vel");
    const auto output = declare_parameter<std::string>("driver_command_topic", "");
    const auto raw_odom = declare_parameter<std::string>("driver_odom_topic", "/wheel/odom");
    const auto state = declare_parameter<std::string>("base_state_topic", "/agt/base/state");
    const auto out_odom = declare_parameter<std::string>("output_odom_topic", "/agt/base/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    max_forward_ = declare_parameter<double>("max_forward_mps", 0.0);
    max_reverse_ = declare_parameter<double>("max_reverse_mps", 0.0);
    max_yaw_ = declare_parameter<double>("max_angular_rps", 0.0);
    timeout_ = declare_parameter<double>("command_timeout_sec", 0.15);
    state_timeout_ = declare_parameter<double>("state_timeout_sec", 0.3);
    const auto rate = declare_parameter<double>("publish_rate_hz", 50.0);
    if (!std::isfinite(rate) || rate < 50 || !std::isfinite(timeout_) || timeout_ <= 0 ||
      !std::isfinite(state_timeout_) || state_timeout_ <= 0)
    {throw std::runtime_error("finite watchdogs and at least 50 Hz are required");}
    if (verified_) {
      if (profile_.empty() || output.empty() || output == input || output[0] != '/' ||
        !std::isfinite(max_forward_) || max_forward_ <= 0 ||
        !std::isfinite(max_yaw_) || max_yaw_ <= 0 ||
        !std::isfinite(max_reverse_) || max_reverse_ < 0)
      {throw std::runtime_error("verified driver/profile/positive motion limits required");}
      pub_ = create_publisher<geometry_msgs::msg::Twist>(output, 10);
    } else {
      RCLCPP_WARN(get_logger(), "unverified tracked profile: no driver command publisher created");
    }
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(out_odom, 10);
    if (out_odom == raw_odom) {throw std::runtime_error("odometry normalization loop rejected");}
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(input, 10,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!finite(*msg) || std::abs(msg->linear.y) > 1e-9 ||
          std::abs(msg->linear.z) > 1e-9 || std::abs(msg->angular.x) > 1e-9 ||
          std::abs(msg->angular.y) > 1e-9)
        {target_ = geometry_msgs::msg::Twist(); have_command_ = false; return;}
        target_ = *msg; command_rx_ = Clock::now(); have_command_ = true;
      });
    state_sub_ = create_subscription<agt_robot_interfaces::msg::BaseState>(state, 10,
      [this](agt_robot_interfaces::msg::BaseState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = *msg; state_rx_ = Clock::now(); have_state_ = true;
        if (!state_permitted()) {target_ = geometry_msgs::msg::Twist(); have_command_ = false; if (pub_) pub_->publish(geometry_msgs::msg::Twist());}
      });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(raw_odom, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto & p = msg->pose.pose.position;
        const auto & q = msg->pose.pose.orientation;
        const auto stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
        const double age = (now()-stamp).seconds();
        const double norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        if (msg->header.frame_id != odom_frame_ || msg->child_frame_id != base_frame_ ||
          !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(norm) || norm < 1e-12 || !finite(msg->twist.twist) ||
          age < -0.05 || age > state_timeout_ || stamp.nanoseconds() <= last_odom_ns_)
        {return;}
        last_odom_ns_ = stamp.nanoseconds();
        // A frame mismatch is rejected, never fixed by relabelling a lever arm.
        odom_pub_->publish(*msg);
      });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0/rate), [this]() {tick();});
  }
  ~TrackedTwistAdapter() override {if (pub_) pub_->publish(geometry_msgs::msg::Twist());}

private:
  static bool finite(const geometry_msgs::msg::Twist & t)
  {return std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.linear.z) &&
      std::isfinite(t.angular.x) && std::isfinite(t.angular.y) && std::isfinite(t.angular.z);}
  bool state_permitted() const
  {
    if (!have_state_) return false;
    const double age = (now()-rclcpp::Time(state_.stamp, get_clock()->get_clock_type())).seconds();
    return state_.robot_profile == profile_ && state_.source_valid && state_.drive_permitted &&
           !state_.emergency_stop && !state_.manual_override && !state_.fault_active &&
           age >= -0.05 && age <= state_timeout_ &&
           std::chrono::duration<double>(Clock::now()-state_rx_).count() <= state_timeout_;
  }
  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pub_) return;
    geometry_msgs::msg::Twist out;
    if (have_command_ && state_permitted() &&
      std::chrono::duration<double>(Clock::now()-command_rx_).count() <= timeout_)
    {out.linear.x = std::clamp(target_.linear.x, -max_reverse_, max_forward_);
     out.angular.z = std::clamp(target_.angular.z, -max_yaw_, max_yaw_);}
    else {have_command_ = false; target_ = geometry_msgs::msg::Twist();}
    pub_->publish(out);
  }
  std::string profile_, odom_frame_, base_frame_;
  bool verified_{false}, have_command_{false}, have_state_{false};
  double max_forward_, max_reverse_, max_yaw_, timeout_, state_timeout_;
  int64_t last_odom_ns_{0};
  std::mutex mutex_;
  geometry_msgs::msg::Twist target_;
  agt_robot_interfaces::msg::BaseState state_;
  Clock::time_point command_rx_, state_rx_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<agt_robot_interfaces::msg::BaseState>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<TrackedTwistAdapter>());}
  catch (const std::exception & e) {RCLCPP_FATAL(rclcpp::get_logger("tracked_adapter"), "%s", e.what()); rclcpp::shutdown(); return 1;}
  rclcpp::shutdown(); return 0;
}
