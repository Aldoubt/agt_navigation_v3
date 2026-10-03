#pragma once
#include <chrono>
#include <mutex>
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "agt_navigation_interfaces/msg/navigation_mode.hpp"
#include "agt_navigation_interfaces/msg/velocity_command.hpp"

namespace agt_epoch_rpp_controller
{
class EpochRppController : public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string,
    std::shared_ptr<tf2_ros::Buffer>, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path &) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker *) override;
private:
  using Mode = agt_navigation_interfaces::msg::NavigationMode;
  Mode validate_locked() const;
  mutable std::mutex epoch_mutex_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr epoch_node_;
  rclcpp::Subscription<Mode>::SharedPtr mode_sub_;
  rclcpp_lifecycle::LifecyclePublisher<agt_navigation_interfaces::msg::VelocityCommand>::SharedPtr command_pub_;
  Mode mode_;
  std::chrono::steady_clock::time_point received_;
  mutable std::chrono::steady_clock::time_point clock_progress_;
  mutable int64_t last_ros_ns_{0};
  mutable uint64_t clock_blocked_epoch_{0};
  bool have_mode_{false}, enabled_{false}, field_verified_{false}, have_plan_{false};
  bool active_{false};
  uint64_t plan_epoch_{0}, plan_odom_epoch_{0};
  std::string owner_id_, plan_task_id_, plugin_name_;
  builtin_interfaces::msg::Time plan_stamp_;
  double mode_timeout_sec_{0.25};
};
}
