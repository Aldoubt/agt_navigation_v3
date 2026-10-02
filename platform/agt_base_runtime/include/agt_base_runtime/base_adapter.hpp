#pragma once

#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace agt_base_runtime
{

enum class KinematicType
{
  kSkidSteer,
  kDifferential,
  kAckermann,
  kFourWheelSteer,
  kOmni,
  kSwerve,
};

struct ChassisProfile
{
  std::string profile_id;
  KinematicType kinematics{KinematicType::kSkidSteer};
  double wheelbase_m{0.0};
  double min_steering_angle_rad{0.0};
  double max_steering_angle_rad{0.0};
  double max_steering_rate_radps{0.0};
  double min_turning_radius_m{0.0};
  double minimum_speed_mps{0.0};
  bool rotate_in_place{true};
};

// A vendor-neutral adapter result. A skid/differential adapter uses linear_x
// and angular_z; Ackermann uses linear_x and steering_angle_rad. The adapter
// boundary is the only place that converts these values to driver messages.
struct AdapterCommand
{
  double linear_x_mps{0.0};
  double angular_z_radps{0.0};
  double steering_angle_rad{0.0};
  bool valid{false};
  bool in_place_rotation_rejected{false};
};

class BaseAdapter
{
public:
  virtual ~BaseAdapter() = default;

  virtual bool configure(const ChassisProfile & profile, std::string & reason) = 0;
  virtual AdapterCommand convert(
    const geometry_msgs::msg::Twist & request, double elapsed_sec) = 0;
  virtual nav_msgs::msg::Odometry normalize_odometry(
    const nav_msgs::msg::Odometry & input) const = 0;
};

}  // namespace agt_base_runtime
