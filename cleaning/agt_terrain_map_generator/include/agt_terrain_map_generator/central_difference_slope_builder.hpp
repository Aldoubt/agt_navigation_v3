#pragma once

#include "agt_terrain_map_generator/slope_builder.hpp"

namespace agt_terrain_map_generator
{

struct CentralDifferenceSlopeOptions
{
  double min_confidence{0.5};
};

class CentralDifferenceSlopeBuilder final : public SlopeBuilder
{
public:
  explicit CentralDifferenceSlopeBuilder(const CentralDifferenceSlopeOptions & options);

  bool build(
    const GridGeometry & geometry,
    const std::vector<ElevationCell> & elevation,
    std::vector<float> & slope_deg) override;

private:
  CentralDifferenceSlopeOptions options_;
};

}  // namespace agt_terrain_map_generator
