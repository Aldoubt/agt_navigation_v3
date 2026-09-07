#pragma once

#include <array>
#include <cmath>

#include "agt_terrain_map_generator/terrain_types.hpp"

namespace agt_terrain_map_generator
{

struct RobotBodyBox
{
  std::array<double, 3U> center_xyz{0.0, 0.0, 0.20};
  std::array<double, 3U> size_xyz{1.023, 0.778, 0.400};
  double padding_m{0.05};

  bool valid() const
  {
    return padding_m >= 0.0 && size_xyz[0] > 0.0 && size_xyz[1] > 0.0 && size_xyz[2] > 0.0;
  }

  bool contains(const Point3f & body_point) const
  {
    const double hx = 0.5 * size_xyz[0] + padding_m;
    const double hy = 0.5 * size_xyz[1] + padding_m;
    const double hz = 0.5 * size_xyz[2] + padding_m;
    return std::abs(static_cast<double>(body_point.x) - center_xyz[0]) <= hx &&
           std::abs(static_cast<double>(body_point.y) - center_xyz[1]) <= hy &&
           std::abs(static_cast<double>(body_point.z) - center_xyz[2]) <= hz;
  }
};

class RobotGeometryProvider
{
public:
  virtual ~RobotGeometryProvider() = default;
  virtual RobotBodyBox body_box() const = 0;
};

class StaticRobotGeometryProvider final : public RobotGeometryProvider
{
public:
  explicit StaticRobotGeometryProvider(const RobotBodyBox & box)
  : box_(box) {}

  RobotBodyBox body_box() const override { return box_; }

private:
  RobotBodyBox box_;
};

}  // namespace agt_terrain_map_generator
