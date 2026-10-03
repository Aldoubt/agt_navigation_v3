#include "agt_epoch_rpp_controller/epoch_rpp_controller.hpp"
#include <algorithm>
#include <cmath>
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace agt_epoch_rpp_controller
{
void EpochRppController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap)
{
  RegulatedPurePursuitController::configure(parent, name, tf, costmap);
  plugin_name_ = name;
  epoch_node_ = parent;
  auto node = parent.lock();
  last_ros_ns_ = node->now().nanoseconds();
  clock_progress_ = std::chrono::steady_clock::now();
  const auto declare = [&node, &name](const std::string & key, const rclcpp::ParameterValue & value) {
      if (!node->has_parameter(name + "." + key)) {node->declare_parameter(name + "." + key, value);}
    };
  declare("research_enabled", rclcpp::ParameterValue(false));
  declare("field_verified", rclcpp::ParameterValue(false));
  declare("mode_timeout_sec", rclcpp::ParameterValue(0.25));
  declare("owner_id", rclcpp::ParameterValue("agt_task_continuity"));
  declare("mode_topic", rclcpp::ParameterValue("/agt/research/navigation_mode"));
  declare("command_topic", rclcpp::ParameterValue("/agt/research/cmd_vel_source"));
  enabled_ = node->get_parameter(name + ".research_enabled").as_bool();
  field_verified_ = node->get_parameter(name + ".field_verified").as_bool();
  mode_timeout_sec_ = node->get_parameter(name + ".mode_timeout_sec").as_double();
  owner_id_ = node->get_parameter(name + ".owner_id").as_string();
  if (enabled_ && field_verified_ &&
    (node->get_parameter(name + ".use_rotate_to_heading").as_bool() ||
    node->get_parameter(name + ".allow_reversing").as_bool())) {
    throw std::invalid_argument("forward visibility research controller forbids reversing and rotate_to_heading");
  }
  if (!std::isfinite(mode_timeout_sec_) || mode_timeout_sec_ <= 0.0 || owner_id_.empty()) {
    throw std::invalid_argument("epoch controller requires a finite mode timeout and owner_id");
  }
  mode_sub_ = node->create_subscription<Mode>(node->get_parameter(name + ".mode_topic").as_string(),
    rclcpp::QoS(1).reliable(), [this](Mode::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(epoch_mutex_);
      if (message->owner_id != owner_id_) {return;}
      if (have_mode_ && (message->control_epoch < mode_.control_epoch ||
        (message->control_epoch == mode_.control_epoch &&
        rclcpp::Time(message->header.stamp) < rclcpp::Time(mode_.header.stamp)))) {return;}
      mode_ = *message; have_mode_ = true; received_ = std::chrono::steady_clock::now();
    });
  command_pub_ = node->create_publisher<agt_navigation_interfaces::msg::VelocityCommand>(
    node->get_parameter(name + ".command_topic").as_string(), rclcpp::QoS(1).reliable());
}

EpochRppController::Mode EpochRppController::validate_locked() const
{
  auto node = epoch_node_.lock();
  if (!enabled_ || !field_verified_ || !active_ || !node || !have_mode_) {
    throw nav2_core::PlannerException("research controller is disabled or unverified");
  }
  if (node->get_parameter(plugin_name_ + ".use_rotate_to_heading").as_bool() ||
    node->get_parameter(plugin_name_ + ".allow_reversing").as_bool()) {
    throw nav2_core::PlannerException("forward visibility does not authorize reversing or pivot turns");
  }
  const auto clock_now = node->now().nanoseconds();
  const auto wall_now = std::chrono::steady_clock::now();
  if (clock_now < last_ros_ns_) {clock_blocked_epoch_ = std::max(clock_blocked_epoch_, mode_.control_epoch);}
  if (clock_now != last_ros_ns_) {clock_progress_ = wall_now;}
  if (std::chrono::duration<double>(wall_now - clock_progress_).count() > mode_timeout_sec_) {
    clock_blocked_epoch_ = std::max(clock_blocked_epoch_, mode_.control_epoch);
  }
  last_ros_ns_ = clock_now;
  const double age = (node->now() - rclcpp::Time(mode_.header.stamp)).seconds();
  if (age < -0.01 || age > mode_timeout_sec_ ||
    std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() > mode_timeout_sec_ ||
    rclcpp::Time(mode_.valid_until) <= node->now() || !mode_.authorized || mode_.control_epoch == 0 ||
    mode_.odom_epoch == 0 || mode_.task_id.empty() || mode_.control_epoch <= clock_blocked_epoch_ ||
    (mode_.mode != Mode::GLOBAL && mode_.mode != Mode::LOCAL_TASK && mode_.mode != Mode::RECOVERING) ||
    !std::isfinite(mode_.max_linear_mps) || mode_.max_linear_mps <= 0.0 ||
    !std::isfinite(mode_.max_angular_rps) || mode_.max_angular_rps <= 0.0)
  {throw nav2_core::PlannerException("research authorization missing, expired or invalid");}
  return mode_;
}

void EpochRppController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(epoch_mutex_);
  have_plan_ = false;
  const auto mode = validate_locked();
  if (path.header.stamp != mode.plan_stamp || rclcpp::Time(path.header.stamp).nanoseconds() <= 0 ||
    path.poses.size() < 2 || (mode.mode == Mode::GLOBAL ? path.header.frame_id != "map" :
    path.header.frame_id != "odom")) {
    throw nav2_core::PlannerException("path stamp/frame does not match authorization");
  }
  for (const auto & stamped : path.poses) {
    const auto & p = stamped.pose.position;
    const auto & q = stamped.pose.orientation;
    if ((!stamped.header.frame_id.empty() && stamped.header.frame_id != path.header.frame_id) ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
      std::abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) > 0.01) {
      throw nav2_core::PlannerException("authorized path contains invalid poses");
    }
  }
  RegulatedPurePursuitController::setPlan(path);
  plan_epoch_ = mode.control_epoch; plan_odom_epoch_ = mode.odom_epoch;
  plan_task_id_ = mode.task_id; plan_stamp_ = path.header.stamp; have_plan_ = true;
}

geometry_msgs::msg::TwistStamped EpochRppController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * checker)
{
  Mode mode;
  {
    std::lock_guard<std::mutex> lock(epoch_mutex_); mode = validate_locked();
    if (!have_plan_ || plan_epoch_ != mode.control_epoch || plan_odom_epoch_ != mode.odom_epoch ||
      plan_task_id_ != mode.task_id || plan_stamp_ != mode.plan_stamp) {
      throw nav2_core::PlannerException("accepted plan belongs to a revoked epoch");
    }
  }
  auto command = RegulatedPurePursuitController::computeVelocityCommands(pose, velocity, checker);
  std::lock_guard<std::mutex> lock(epoch_mutex_);
  const auto current = validate_locked();
  if (current.control_epoch != mode.control_epoch || current.plan_stamp != mode.plan_stamp ||
    current.odom_epoch != mode.odom_epoch || current.task_id != mode.task_id) {
    throw nav2_core::PlannerException("authorization changed during control calculation");
  }
  if (!std::isfinite(command.twist.linear.x) || !std::isfinite(command.twist.angular.z)) {
    throw nav2_core::PlannerException("nonfinite controller output");
  }
  if (command.twist.linear.x < 0.0 ||
    (command.twist.linear.x <= 1e-9 && std::abs(command.twist.angular.z) > 1e-9)) {
    throw nav2_core::PlannerException("controller requires unobserved reverse or pivot motion");
  }
  command.twist.linear.x = std::clamp(command.twist.linear.x, 0.0, current.max_linear_mps);
  command.twist.angular.z = std::clamp(command.twist.angular.z, -current.max_angular_rps, current.max_angular_rps);
  command.twist.linear.y = command.twist.linear.z = 0.0;
  command.twist.angular.x = command.twist.angular.y = 0.0;
  auto node = epoch_node_.lock();
  command.header.stamp = node->now();
  agt_navigation_interfaces::msg::VelocityCommand tagged;
  tagged.header = command.header; tagged.owner_id = owner_id_; tagged.task_id = plan_task_id_;
  tagged.control_epoch = plan_epoch_; tagged.odom_epoch = plan_odom_epoch_; tagged.plan_stamp = plan_stamp_;
  tagged.velocity = command.twist;
  command_pub_->publish(tagged);
  return command;
}
void EpochRppController::activate()
{std::lock_guard<std::mutex> lock(epoch_mutex_);
  RegulatedPurePursuitController::activate(); command_pub_->on_activate(); active_ = true;}
void EpochRppController::deactivate()
{std::lock_guard<std::mutex> lock(epoch_mutex_); active_ = false; have_plan_ = false;
  command_pub_->on_deactivate(); RegulatedPurePursuitController::deactivate();}
void EpochRppController::cleanup()
{mode_sub_.reset(); command_pub_.reset(); have_mode_ = false; have_plan_ = false;
  RegulatedPurePursuitController::cleanup();}
}
PLUGINLIB_EXPORT_CLASS(agt_epoch_rpp_controller::EpochRppController, nav2_core::Controller)
