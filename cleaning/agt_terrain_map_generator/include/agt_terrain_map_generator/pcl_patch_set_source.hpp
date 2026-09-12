#pragma once

#include <memory>
#include <string>

#include "agt_terrain_map_generator/patch_set_source.hpp"

namespace agt_terrain_map_generator
{

class PclPatchSetSource final : public PatchSetSource
{
public:
  PclPatchSetSource();
  ~PclPatchSetSource() override;

  PclPatchSetSource(const PclPatchSetSource &) = delete;
  PclPatchSetSource & operator=(const PclPatchSetSource &) = delete;

  bool open(const MappingAssetSet & assets, std::string & error) override;
  void reset() override;
  bool next(PatchAsset & asset, PointCloud & body_cloud, std::string & error) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace agt_terrain_map_generator
