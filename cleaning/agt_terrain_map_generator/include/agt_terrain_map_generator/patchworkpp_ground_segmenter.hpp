#pragma once

#include <memory>
#include <string>

#include "agt_terrain_map_generator/ground_segmenter.hpp"

namespace agt_terrain_map_generator
{

struct PatchworkppOptions
{
  double sensor_height_m{0.0};
  double min_range_m{1.0};
  double max_range_m{80.0};
  bool verbose{false};
  bool enable_rnr{false};
  bool enable_rvpf{true};
  bool enable_tgr{true};
};

class PatchworkppGroundSegmenter final : public GroundSegmenter
{
public:
  explicit PatchworkppGroundSegmenter(const PatchworkppOptions & options);
  ~PatchworkppGroundSegmenter() override;

  PatchworkppGroundSegmenter(const PatchworkppGroundSegmenter &) = delete;
  PatchworkppGroundSegmenter & operator=(const PatchworkppGroundSegmenter &) = delete;

  std::string name() const override;

  bool process(
    const PointCloud & input,
    PointCloud & ground,
    PointCloud & non_ground) override;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace agt_terrain_map_generator
