# Livox data format policy and BBS + GICP relocalization primer

## V1 data format policy

### Mapping / navigation odometry: Livox CustomMsg is canonical

Use `livox_ros_driver2/msg/CustomMsg` on `/livox/lidar` together with `/livox/imu` for:

- robotics-laboratory/fast-lio2 mapping;
- Functionhx/Batch-LIO navigation odometry.

Reason: LIO deskew depends on per-point timing. CustomMsg explicitly carries a message `timebase` and every point carries `offset_time`; it also carries Livox `tag` and `line` metadata.

Do not replace the raw CustomMsg LIO path with a generic PointCloud2 unless the PointCloud2 still contains reconstructable per-point time.

### Relocalization / local perception: PointCloud2 is canonical

Use `sensor_msgs/msg/PointCloud2` on `/agt/livox/points` for:

- 3D-BBS global coarse localization;
- small_gicp refinement;
- self/range/voxel filtering for Nav2 local obstacles;
- RViz/debug/PCD export.

BBS/GICP only need geometric point coordinates for registration. The original per-point Livox timing is not required once a stationary/deskewed query scan has been formed.

## Conversion policy

`agt_livox_tools/livox_format_bridge` supports both directions.

### CustomMsg -> PointCloud2

The AGT bridge preserves:

- x/y/z;
- reflectivity as float `intensity`;
- `tag`;
- `line`;
- `offset_time`;
- float64 per-point `timestamp`;
- the Livox `timebase` in `PointCloud2.header.stamp`.

This representation is intended to support a bridge round trip without discarding Livox timing information.

### PointCloud2 -> CustomMsg

Strict mode is the default. The source PointCloud2 must contain at least one of:

- `offset_time` (preferred), or
- per-point `timestamp`.

If neither exists, the bridge refuses the conversion because reconstructing a CustomMsg with `offset_time=0` for every point is not equivalent to raw Livox data and invalidates LIO deskew assumptions.

`require_per_point_time:=false` / `--allow-missing-point-time` exists only for geometry-only/debug conversion and must not be treated as a valid Fast-LIO2/Batch-LIO raw input.

## Offline rosbag conversion

Build/source the workspace, then use:

```bash
ros2 run agt_livox_tools convert_livox_bag_format \
  --input /data/custom_bag \
  --output /data/custom_as_pc2 \
  --mode custom_to_pointcloud2
```

The converted lidar topic defaults to `/livox/lidar_pc2`; `/livox/imu` is copied unchanged.

PointCloud2 to CustomMsg:

```bash
ros2 run agt_livox_tools convert_livox_bag_format \
  --input /data/pc2_bag \
  --output /data/pc2_as_custom \
  --mode pointcloud2_to_custom
```

The converted lidar topic defaults to `/livox/lidar_custom`.

When a downstream package expects `/livox/lidar`, replay with a remap, for example:

```bash
ros2 bag play /data/custom_as_pc2 \
  --remap /livox/lidar_pc2:=/agt/livox/points
```

For a converted CustomMsg bag:

```bash
ros2 bag play /data/pc2_as_custom \
  --remap /livox/lidar_custom:=/livox/lidar
```

## Which of the user's two bags should be used?

1. Prefer the CustomMsg bag for Fast-LIO2 mapping and Batch-LIO odometry tuning.
2. Both bags can be used for BBS/GICP geometry-only relocalization tests after they are normalized to PointCloud2.
3. A PointCloud2 bag may also be used for mapping only if inspection confirms that it contains valid per-point `offset_time` or `timestamp`, and the strict PointCloud2 -> CustomMsg conversion succeeds.
4. If the PointCloud2 bag contains only x/y/z/intensity, keep it as a relocalization/local-perception dataset. Do not claim that converting it to CustomMsg restores the lost raw timing.

## BBS + GICP: why two stages?

The two algorithms solve different problems.

### Stage 1 — 3D-BBS: global coarse search

Input:

- a prebuilt global point cloud map;
- one stationary/gravity-aligned query scan.

Goal:

- search a large pose space without an initial pose;
- return a coarse transform `T_map_query`.

Conceptually, the map is voxelized into a hierarchy. At coarse resolution, many nearby poses share an upper bound on how well the query could overlap occupied map voxels. Branch-and-bound explores pose-space cells in best-first order:

1. create a coarse pose-space branch (translation plus angle ranges);
2. score it using a coarse voxel level, producing an optimistic upper bound;
3. discard a branch when its upper bound cannot beat the best known solution;
4. subdivide promising branches into finer translation/rotation cells;
5. repeat until the configured resolution/score/timeout is reached.

This avoids evaluating every fine 6DoF pose exhaustively. The AGT V1 removes the known MID360 installation tilt first and keeps the robot stationary, so BBS mainly searches XYZ + full yaw with only a bounded roll/pitch residual.

Important consequence: BBS is good at answering **"roughly where in the whole map am I?"**, but its discretized voxel score is not the final centimeter-level pose.

### Stage 2 — GICP: local continuous refinement

GICP starts from the BBS transform and solves a much smaller local optimization problem.

Ordinary point-to-point ICP repeatedly:

1. transforms the source scan using the current pose estimate;
2. finds nearest target points;
3. minimizes point-to-point residuals;
4. updates the pose until convergence.

GICP improves the residual model by estimating a local covariance/shape around points. Instead of treating every point as an isotropic sphere, it models local surface uncertainty; points on a wall or ground plane therefore constrain the pose differently from points in an unstructured cluster.

A simplified residual for a correspondence is:

```text
r_i = p_target_i - T * p_source_i
cost_i = r_i^T (C_target_i + R C_source_i R^T)^(-1) r_i
```

The optimizer repeatedly updates the 6DoF transform to minimize the sum of these Mahalanobis-like residuals. `max_correspondence_distance` rejects implausibly distant matches.

In AGT, GICP is not run against the complete large map. After BBS finds a coarse location, the backend crops a local submap around that location and refines against it. This makes the fine stage faster and reduces unrelated correspondences.

### Why not use GICP alone for global localization?

GICP is a local optimizer. With a good initial pose it converges accurately; with a far-away initial pose it can:

- converge to the wrong repeated structure;
- get trapped in a local minimum;
- fail because no valid correspondences are within the correspondence radius.

Therefore V1 uses:

```text
whole-map no-initial-pose search
          3D-BBS
             ↓ coarse T_map_base
local continuous optimization
         small_gicp
             ↓ refined T_map_base
quality gates
 score + fitness + overlap
             ↓
/agt/relocalization/pose
```

## What the quality values mean in the current backend

- `bbs_score`: coarse voxel-overlap score from the global search;
- `overlap`: GICP inlier count divided by the query point count;
- `fitness`: GICP accumulated registration error divided by inlier count;
- `score`: current AGT composite `0.60 * bbs_score + 0.40 * overlap`.

These values are engineering gates, not a mathematically calibrated probability of correctness. Tune them using the rosbag benchmark and prioritize zero false-positive global localizations over maximum nominal success rate.

## Recommended learning/test sequence

1. Use the CustomMsg bag to build `map.pcd` and save PGO patches/poses.
2. Build BBS voxel assets from the final map.
3. Run `agt_relocalization_benchmark` against PGO patches to understand score/fitness/overlap distributions.
4. Replay the same raw bag through the complete Batch-LIO + PointCloud2 + BBS/GICP ROS chain.
5. Validate the selected parameters with the second bag, preferably a different route/start/time.
