#include "agt_base_runtime/bunker_adapter.hpp"

#include <cmath>

namespace agt_base_runtime
{

bool BunkerAdapter::configure(const ChassisProfile & profile, std::string & reason)
{
  if (profile.kinematics != KinematicType::kSkidSteer) {
    reason = "Bunker adapter requires skid_steer kinematics";
    return false;
  }
  reason.clear();
  return true;
}

AdapterCommand BunkerAdapter::convert(
  const geometry_msgs::msg::Twist & request, const double /*elapsed_sec*/)
{
  AdapterCommand result;
  if (!std::isfinite(request.linear.x) || !std::isfinite(request.angular.z)) {
    return result;
  }
  result.linear_x_mps = request.linear.x;
  result.angular_z_radps = request.angular.z;
  result.valid = true;
  return result;
}

nav_msgs::msg::Odometry BunkerAdapter::normalize_odometry(
  const nav_msgs::msg::Odometry & input) const
{
  // Bunker /wheel/odom is already a standard Odometry message. Preserve it
  // byte-for-byte at the message-field level and do not create TF here.
  return input;
}

}  // namespace agt_base_runtime
