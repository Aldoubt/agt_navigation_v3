#pragma once

#include <string>

#include "agt_terrain_map_generator/mapping_assets.hpp"

namespace agt_terrain_map_generator
{

class PatchSetSource
{
public:
  virtual ~PatchSetSource() = default;

  virtual bool open(const MappingAssetSet & assets, std::string & error) = 0;
  virtual void reset() = 0;

  // Returns false both at end-of-stream and on error. `error` is empty at a
  // clean end-of-stream and non-empty on failure.
  virtual bool next(
    PatchAsset & asset,
    PointCloud & body_cloud,
    std::string & error) = 0;
};

}  // namespace agt_terrain_map_generator
