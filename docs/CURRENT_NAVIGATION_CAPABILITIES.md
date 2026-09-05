# Current Navigation Capabilities (runtime-v1)

This document records the current implemented capability boundary for the RViz patrol demo.

## 1. MID360 input and message policy

The official livox_ros_driver2 uses `/livox/lidar` for the LiDAR stream, but the message type depends on `xfer_format`:

- `xfer_format=0`: `sensor_msgs/msg/PointCloud2` with Livox fields (`x y z intensity tag line timestamp`)
- `xfer_format=1`: `livox_ros_driver2/msg/CustomMsg`
- MID360 IMU: `/livox/imu` (`sensor_msgs/msg/Imu`)
- typical publish frequency: 10 Hz unless configured otherwise
- official MID360 launch default frame is commonly `livox_frame`

AGT policy:

```text
/livox/lidar CustomMsg -------------------------------> FAST-LIO2
       |
       +-> agt_livox_tools custom_to_pointcloud2
                |
                +-> /agt/livox/points PointCloud2
                         |
                         +-> agt_pointcloud_preprocessor
                                  |
                                  +-> /agt/navigation/points_obstacles
```

If the driver is already configured to output PointCloud2, remap that PointCloud2 directly to `/agt/livox/points`; the bridge is not mandatory.

Do not put voxel/self filtering in front of FAST-LIO2 unless the selected FAST-LIO2 fork and per-point timing behavior have been explicitly validated.

## 2. Livox format bridge

Package: `agt_livox_tools`

Modes:

```text
custom_to_pointcloud2
pointcloud2_to_custom
```

CustomMsg -> PointCloud2 preserves:

- x/y/z
- reflectivity as `intensity`
- tag
- line
- offset_time
- generated per-point `timestamp`

The PointCloud2 header timestamp is set to the Livox `timebase` so bridge-generated PointCloud2 can reconstruct CustomMsg point offsets on the reverse path.

PointCloud2 -> CustomMsg uses `offset_time` when present, otherwise reconstructs it from a `timestamp` field relative to `header.stamp`. `lidar_id` is a parameter because a generic PointCloud2 does not carry the Livox device id.

The reverse conversion is a compatibility/debug tool. It is not a replacement for native CustomMsg input when FAST-LIO2 expects CustomMsg.

## 3. Local perception

Package: `agt_pointcloud_preprocessor`

Current pipeline:

```text
/agt/livox/points
  -> finite xyz validation
  -> range gate
  -> TF-aware robot self box filter
  -> optional rear-sector mask
  -> 3D voxel downsample
  -> /agt/navigation/points_obstacles
```

Current default filters:

- minimum range: 0.5 m
- maximum range: 120 m
- robot self box: approximately Bunker body envelope + padding
- rear sector: disabled by default until measured against rosbag
- voxel leaf: 0.20 m

Nav2 local costmap consumes `/agt/navigation/points_obstacles` through a `VoxelLayer` in `odom` frame. Current local window is 8 m x 8 m, with 0.05 m costmap resolution.

Not yet implemented in live local perception:

- ground segmentation / explicit ground removal
- online slope/roughness estimation
- obstacle clustering or tracking
- semantic classes
- dynamic-object velocity prediction
- terrain confidence fusion

For the first demo this is intentional: static map handles global traversability; current MID360 cloud handles newly appearing local obstacles.

## 4. Global path planning

Current Nav2 global planner: `nav2_smac_planner/SmacPlanner2D`.

Current role:

- cost-aware 2D A* search over the global costmap
- respects the static occupancy map and inflation costs
- can steer paths away from high-cost cells using `cost_travel_multiplier`
- replanning is handled by Nav2 behavior-tree/controller flow

Current AGT parameters include:

- planner frequency expectation: 5 Hz
- tolerance: 0.25 m
- `allow_unknown: true`
- planning timeout: 2 s
- cost travel multiplier: 2.0

Limitations:

SmacPlanner2D is a grid-space planner and does not enforce tracked-vehicle kinematic feasibility in the path itself. For the first Bunker demo this is acceptable because the chassis is differential/skid-steer and the local controller handles heading/curvature. If field tests show excessive in-place rotation, corner cutting or infeasible tight turns, compare against `SmacPlannerLattice` rather than immediately writing a custom planner.

Current local controller baseline is Regulated Pure Pursuit at 50 Hz. Gazebo
closed-loop navigation has reached `NavigateToPose=SUCCEEDED`; the controller is
still not frozen for the tracked field vehicle. `PoseProgressChecker` is used so
valid in-place heading correction counts as progress. RPP's internal angular
acceleration bootstrap is `10.0 rad/s^2`, while the downstream smoother/guard
continue to enforce the current physical `0.8 rad/s^2` angular acceleration
limit.

## 5. Global automatic relocalization implementation status

Implemented:

### Batch-LIO navigation adapter

`agt_batch_lio_adapter` converts Batch-LIO `camera_init -> body` odometry into:

```text
/agt/odometry/local
```

The `body -> base_link` conversion uses a versioned calibrated extrinsic by
default instead of requiring a TF-buffer lookup on every odometry message. This
makes live and rosbag behavior deterministic. TF remains available for the rest
of the ROS graph and as an optional adapter fallback.

Batch-LIO's source odometry currently reports zero twist. The adapter therefore
derives `base_link`-frame linear/angular velocity from consecutive adapted poses
for Nav2 consumers. The derivative has dt bounds, deadbands and maximum-norm
guards; quaternion delta is used instead of Euler-angle differentiation.

### Localization Manager

`agt_localization_manager` is implemented and is the only intended `map -> odom` owner.

Inputs:

```text
/agt/odometry/local
/agt/relocalization/pose
```

Output/state:

```text
map -> odom
/agt/localization/status
```

Manual relocalization request:

```text
/agt/localization/relocalize (Trigger)
    -> clears current map correction
    -> publishes /agt/relocalization/request
```

When a global pose arrives at time `t`, the manager finds the nearest local odometry sample and computes:

```text
T_map_odom = T_map_base(t) * inverse(T_odom_base(t))
```

Acceptance gates already implemented:

- correct `map` frame
- non-zero timestamp
- covariance required by default
- maximum global position uncertainty
- maximum global yaw uncertainty
- maximum time skew to local odometry

Runtime states already implemented include WAIT_LOCAL_ODOM, WAIT_GLOBAL, LOCALIZED, DEGRADED, LOST and RELOCALIZING behavior.

### Production global relocalization backend

The no-initial-pose backend is now implemented in production code:

```text
static MID360 query in base_link
  -> AGT Polar Context descriptor
  -> Top-K keyframe candidates from patches + poses.txt
  -> descriptor yaw seed + candidate pose seed
  -> candidate-local CPU 3D-BBS coarse registration
  -> small_gicp 6-DoF refinement
  -> score / fitness / overlap gates
  -> /agt/relocalization/pose
  -> Localization Manager
  -> map -> odom
```

Current CPU V1 baseline:

```text
BBS assets             0.5 m minimum level, 5 levels
descriptor prefilter   40
candidate Top-K        2
candidate XY radius    +/- 4 m
candidate Z radius     +/- 2 m
BBS residual angles    0 deg (descriptor/map pose supplies orientation seed)
per-candidate timeout  8 s
BBS coarse threshold   0.05
backend timeout        18 s
```

The deliberately low BBS threshold is not the final acceptance threshold. BBS
is used to obtain a geometrically useful coarse seed; GICP and ROS-side
`score/fitness/overlap` gates decide whether the pose is safe to publish.

Offline end-to-end replay with `bag_mapping_current` + `211105` passed on
2026-09-05 and reached `LOCALIZED` with a valid `map -> odom` correction.

A clean single-stack Gazebo gate also passed on 2026-09-05 without
`/initialpose`. Representative cold-start localization was
`score=0.926627`, `overlap=0.991720`, `fitness=0.110056`, with candidate BBS
finishing in `453.279 ms`; a subsequent known-free PGO target returned
`NavigateToPose=SUCCEEDED` in `45.322 s`.

The `cmd_vel_guard` fail-closed transition has a repeatable ROS pub/sub
acceptance test in `src/agt_base_control/test/guard_fail_closed_acceptance.py`.
The measured LOST-to-zero latency was `1.4 ms`, and stale command replay after
localization recovery measured exactly `0.000000` in the test.

Still pending before field freeze:

- at least 5 materially different start positions/headings on the real map;
- tree-shadow and repeated-tree false-positive testing;
- rough tracked-chassis vibration testing;
- restart/power-cycle recovery;
- field tuning of failure/retry timing;

The previously deferred vehicle-runtime blockers were closed on 2026-09-05:

- `cmd_vel_guard` now fail-closes on invalid/stale `LocalizationStatus` and
  discards motion commands received while localization is unsafe;
- `mission_runtime` now requires pose-delta + twist stationary evidence for a
  continuous hold period before camera capture;
- `camera_gimbal_interfaces` is compiled inside the same `ros2_ws` dependency
  overlay for deterministic runtime imports.

See `docs/RVIZ_FIELD_ACCEPTANCE.md` for the bench evidence and first vehicle-test
sequence.

Do not duplicate `map -> odom` publication in any relocalization backend.
