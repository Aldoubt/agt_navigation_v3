#pragma once

#include "agt_terrain_map_generator/patch_preprocessor.hpp"

namespace agt_terrain_map_generator
{

class GravityLevelPatchPreprocessor final : public PatchPreprocessor
{
public:
  std::string name() const override;

  bool process(
    const PatchAsset & asset,
    const PointCloud & body_cloud,
    PreparedPatch & prepared,
    std::string & error) const override;
};

}  // namespace agt_terrain_map_generator
