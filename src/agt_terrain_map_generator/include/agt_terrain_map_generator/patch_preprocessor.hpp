#pragma once

#include <string>

#include "agt_terrain_map_generator/mapping_assets.hpp"

namespace agt_terrain_map_generator
{

struct PreparedPatch
{
  PointCloud local_cloud;
  RigidTransform3d map_from_local;
  // Converts the gravity-level local frame back to the source body frame.
  // Terrain filters use this only to evaluate robot-relative geometry; points
  // retained for segmentation remain in the gravity-level local frame.
  RigidTransform3d body_from_local;
};

class PatchPreprocessor
{
public:
  virtual ~PatchPreprocessor() = default;

  virtual std::string name() const = 0;

  virtual bool process(
    const PatchAsset & asset,
    const PointCloud & body_cloud,
    PreparedPatch & prepared,
    std::string & error) const = 0;
};

}  // namespace agt_terrain_map_generator
