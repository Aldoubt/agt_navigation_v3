#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace agt_terrain_map_generator
{

using GroundCloud = pcl::PointCloud<pcl::PointXYZ>;
using NonGroundCloud = pcl::PointCloud<pcl::PointXYZ>;

// Deliberately owns no Patchwork++ types.  Native Patchwork++, Patchwork++
// variants, and Patchwork-LIO can each implement this boundary later.
class PatchworkInterface
{
public:
  virtual ~PatchworkInterface() = default;

  virtual bool segment(
    const pcl::PointCloud<pcl::PointXYZ> & input,
    GroundCloud & ground,
    NonGroundCloud & nonground) = 0;
};

}  // namespace agt_terrain_map_generator
