#pragma once

#include <string>

#include "agt_base_runtime/base_adapter.hpp"

namespace agt_base_runtime
{

class AckermannAdapter final : public BaseAdapter
{
public:
  bool configure(const ChassisProfile & profile, std::string & reason) override;
  AdapterCommand convert(
    const geometry_msgs::msg::Twist & request, double elapsed_sec) override;
  nav_msgs::msg::Odometry normalize_odometry(
    const nav_msgs::msg::Odometry & input) const override;

  double previous_steering_angle_rad() const {return previous_steering_angle_rad_;}

private:
  ChassisProfile profile_;
  bool configured_{false};
  double previous_steering_angle_rad_{0.0};
};

}  // namespace agt_base_runtime
