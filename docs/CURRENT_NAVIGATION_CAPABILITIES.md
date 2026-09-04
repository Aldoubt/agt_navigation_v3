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

Current local controller baseline is Regulated Pure Pursuit at 50 Hz. It is not frozen as the final controller.

## 5. Global automatic relocalization implementation status

Implemented:

### FAST-LIO2 adapter

`agt_fastlio_adapter` validates frame/timestamp contract and republishes local odometry as:

```text
/agt/odometry/local
```

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

Not yet implemented:

The actual **3D global search backend** that consumes `/agt/relocalization/request`, builds the live scan descriptor/candidates, performs Scan Context/coarse matching/GICP or calls the existing 3D Map Localization SDK, validates candidate consistency, and publishes the final `/agt/relocalization/pose` is not yet landed as production code in this repository.

Therefore current status is:

```text
local odom / TF manager / acceptance / handoff    IMPLEMENTED
3D no-initial-pose search backend                 NOT YET LANDED
existing external SDK first-step                  AVAILABLE FOR INTEGRATION
```

For the current RViz demo, the next localization milestone is to wrap the already-working 3D Map Localization SDK behind the fixed contract:

```text
/agt/relocalization/request
      -> SDK adapter
      -> /agt/relocalization/pose PoseWithCovarianceStamped
```

Do not duplicate map->odom publication inside the SDK adapter.
