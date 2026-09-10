#pragma once

#include <string>

#include "agt_terrain_map_generator/terrain_types.hpp"

namespace agt_terrain_map_generator
{

class GroundSegmenter
{
public:
  virtual ~GroundSegmenter() = default;

  virtual std::string name() const = 0;

  virtual bool process(
    const PointCloud & input,
    PointCloud & ground,
    PointCloud & non_ground) = 0;
};

}  // namespace agt_terrain_map_generator
