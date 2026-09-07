#include "agt_terrain_map_generator/map_cleaning/point_provenance.hpp"

namespace agt_terrain_map_generator
{

ProvenancePoint PointProvenanceBuilder::make(
  const Point3f & body_point,
  const float intensity,
  const int patch_id,
  const int pose_id,
  const double timestamp,
  const RigidTransform3d & map_from_body) const
{
  const Point3f mapped = map_from_body.transform(body_point);
  return ProvenancePoint{
    Eigen::Vector3f{mapped.x, mapped.y, mapped.z}, intensity,
    patch_id, pose_id, timestamp};
}

}  // namespace agt_terrain_map_generator
