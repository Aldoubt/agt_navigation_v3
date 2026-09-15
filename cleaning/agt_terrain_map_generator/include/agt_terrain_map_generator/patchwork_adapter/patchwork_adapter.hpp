#pragma once

#include "agt_terrain_map_generator/patchwork_adapter/patchwork_interface.hpp"

namespace agt_terrain_map_generator
{

struct PatchworkAdapterOptions
{
  double sensor_height_m{0.89};
  double cell_size_m{0.20};
  double ground_tolerance_m{0.20};
};

// MQ3-A deterministic offline adapter.  It is intentionally a small,
// dependency-free implementation of the PatchworkInterface contract, not a
// substitute for the native Patchwork++ algorithm.
class PatchworkAdapter final : public PatchworkInterface
{
public:
  explicit PatchworkAdapter(PatchworkAdapterOptions options);

  bool segment(
    const pcl::PointCloud<pcl::PointXYZ> & input,
    GroundCloud & ground,
    NonGroundCloud & nonground) override;

private:
  PatchworkAdapterOptions options_;
};

}  // namespace agt_terrain_map_generator
