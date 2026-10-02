#pragma once

#include "agt_base_runtime/base_adapter.hpp"

namespace agt_base_runtime
{

class BunkerAdapter final : public BaseAdapter
{
public:
  bool configure(const ChassisProfile & profile, std::string & reason) override;
  AdapterCommand convert(
    const geometry_msgs::msg::Twist & request, double elapsed_sec) override;
  nav_msgs::msg::Odometry normalize_odometry(
    const nav_msgs::msg::Odometry & input) const override;
};

}  // namespace agt_base_runtime
