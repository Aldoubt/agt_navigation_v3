# R1.1 Debug Navigation Runtime Validation

## Build result

`agt_system_bringup` build: PASS. Native BBS/GICP and its relocalization wrapper:
PASS after restoring existing external build prefixes. Whole workspace: NOT PASS
because the unrelated ROS 1 `ins` catkin package fails configuration after the
symlink-cache issues are resolved. See `build_baseline_report.md`.

## Debug launch contract

```bash
ros2 launch agt_system_bringup navigation_debug.launch.py \
  localization_map:=/absolute/path/global_map.pcd \
  navigation_map:=/absolute/path/map.yaml \
  enable_camera:=false enable_demo_task:=false
```

The two assets are explicit debug inputs. This path does not resolve
`active_map.yaml`, validate SHA-256, or publish a map version. Production
`hmi_field_demo` remains unchanged. Camera and Nav2-success capture demo are
independent optional nodes, both disabled by default.

## Intended topic and TF graph

```text
MID360 CustomMsg + IMU -> Batch-LIO -> adapter -> /agt/odometry/local
secondary PointCloud2 -> BBS/GICP -> localization_manager -> map -> odom
navigation_map YAML/PGM -> Nav2 -> /cmd_vel -> guard -> /mux/cmd_vel
```

The expected authoritative TF chain is `map -> odom -> base_link`; the
localization manager remains the only `map -> odom` publisher. No TF or topic
names were changed in R1.1.

## Rosbag evidence

Inspected `src/rosbag/bunker_mid360_mapping_20260901_205036` (482.3 s,
268427 messages). It contains `/agt/sensors/lidar/custom` (9578),
`/agt/sensors/imu/data` (96018), chassis odometry (47934) and one `/tf_static`.
It has zero dynamic `/tf` messages. Asset/input availability is therefore
verified, but a full playback runtime test was not run: hardware-oriented debug
bringup needs a supplied static robot TF and a controlled non-hardware playback
launch to establish `map -> odom` and Nav2 lifecycle safely.

## Minimal navigation demo runbook

1. Start debug navigation with matching explicit PCD and YAML assets.
2. In RViz, wait for a single `map -> odom` authority and Nav2 lifecycle active.
3. Use **2D Goal Pose**; inspect `/cmd_vel` and guarded `/mux/cmd_vel`.
4. Cancel using RViz/Nav2 cancel before any physical motion test.
5. Only when desired, restart with `enable_camera:=true enable_demo_task:=true`.
   The demo task invokes capture only after Nav2 success; it is not required for
   navigation-only testing.

## Unverified

- Full rosbag playback from LIO through `map -> odom` and Nav2.
- Live Nav2 goal/cancel and non-zero command output.
- Camera hardware capability and post-success capture.
- Real robot safety and chassis command behavior.
