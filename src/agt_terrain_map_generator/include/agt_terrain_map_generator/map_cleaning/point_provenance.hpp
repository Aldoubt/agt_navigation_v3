#pragma once

#include <Eigen/Core>

#include "agt_terrain_map_generator/mapping_assets.hpp"

namespace agt_terrain_map_generator
{

// A terrain-only record reconstructed from a body-frame mapping patch.  The
// source mapping assets do not contain absolute acquisition timestamps, so the
// offline source uses a monotonic patch/pose sequence as timestamp.
struct ProvenancePoint
{
  Eigen::Vector3f point{Eigen::Vector3f::Zero()};
  float intensity{0.0F};
  int patch_id{-1};
  int pose_id{-1};
  double timestamp{0.0};
};

class PointProvenanceBuilder
{
public:
  ProvenancePoint make(
    const Point3f & body_point,
    float intensity,
    int patch_id,
    int pose_id,
    double timestamp,
    const RigidTransform3d & map_from_body) const;
};

}  // namespace agt_terrain_map_generator
