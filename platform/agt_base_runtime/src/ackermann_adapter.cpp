#include "agt_base_runtime/ackermann_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace agt_base_runtime
{
namespace
{
constexpr double kZeroEpsilon = 1.0e-9;
}

bool AckermannAdapter::configure(const ChassisProfile & profile, std::string & reason)
{
  if (profile.kinematics != KinematicType::kAckermann) {
    reason = "Ackermann adapter requires ackermann kinematics";
    return false;
  }
  if (profile.rotate_in_place) {
    reason = "Ackermann profile must set rotate_in_place=false";
    return false;
  }
  if (!std::isfinite(profile.wheelbase_m) || profile.wheelbase_m <= 0.0) {
    reason = "wheelbase_m must be finite and positive";
    return false;
  }
  if (!std::isfinite(profile.min_steering_angle_rad) ||
    !std::isfinite(profile.max_steering_angle_rad) ||
    profile.min_steering_angle_rad >= 0.0 || profile.max_steering_angle_rad <= 0.0 ||
    profile.min_steering_angle_rad >= profile.max_steering_angle_rad)
  {
    reason = "steering angle bounds must straddle zero and be ordered";
    return false;
  }
  if (!std::isfinite(profile.max_steering_rate_radps) ||
    profile.max_steering_rate_radps <= 0.0)
  {
    reason = "max_steering_rate_radps must be finite and positive";
    return false;
  }
  if (!std::isfinite(profile.min_turning_radius_m) ||
    profile.min_turning_radius_m <= 0.0)
  {
    reason = "min_turning_radius_m must be finite and positive";
    return false;
  }
  if (!std::isfinite(profile.minimum_speed_mps) || profile.minimum_speed_mps <= 0.0) {
    reason = "minimum_speed_mps must be finite and positive";
    return false;
  }
  profile_ = profile;
  configured_ = true;
  previous_steering_angle_rad_ = 0.0;
  reason.clear();
  return true;
}

AdapterCommand AckermannAdapter::convert(
  const geometry_msgs::msg::Twist & request, const double elapsed_sec)
{
  AdapterCommand result;
  if (!configured_ || !std::isfinite(request.linear.x) ||
    !std::isfinite(request.angular.z) || !std::isfinite(elapsed_sec) || elapsed_sec <= 0.0)
  {
    return result;
  }

  const double speed = request.linear.x;
  const double yaw_rate = request.angular.z;
  const double speed_sign = speed < 0.0 ? -1.0 : 1.0;
  const double effective_speed = speed_sign * std::max(
    std::abs(speed), profile_.minimum_speed_mps);
  const double curvature_limit = 1.0 / profile_.min_turning_radius_m;
  const double desired_curvature = std::clamp(
    yaw_rate / effective_speed, -curvature_limit, curvature_limit);
  const double desired_angle = std::clamp(
    std::atan(profile_.wheelbase_m * desired_curvature),
    profile_.min_steering_angle_rad, profile_.max_steering_angle_rad);
  const double max_angle_delta = profile_.max_steering_rate_radps * elapsed_sec;
  const double steering = std::clamp(
    desired_angle,
    previous_steering_angle_rad_ - max_angle_delta,
    previous_steering_angle_rad_ + max_angle_delta);

  result.linear_x_mps = speed;
  result.steering_angle_rad = std::clamp(
    steering, profile_.min_steering_angle_rad, profile_.max_steering_angle_rad);
  result.in_place_rotation_rejected = std::abs(speed) <= kZeroEpsilon &&
    std::abs(yaw_rate) > kZeroEpsilon;
  result.valid = true;
  previous_steering_angle_rad_ = result.steering_angle_rad;
  return result;
}

nav_msgs::msg::Odometry AckermannAdapter::normalize_odometry(
  const nav_msgs::msg::Odometry & input) const
{
  return input;
}

}  // namespace agt_base_runtime
