# AGT Navigation V3 current-architecture audit

Audit date: 2026-09-10.  Scope is this repository only, at baseline commit
`29c26e6` on `feature/map-lifecycle-hmi-release`; no algorithm or calibration
was changed while collecting this inventory.

## Packages and responsibilities

| Current package(s) | Observed responsibility | Target domain |
|---|---|---|
| `agt_livox_tools`, `ros2_livox_simulation`, external `livox_ros_driver2` | Livox format bridge and simulation | sensor |
| `agt_batch_lio_adapter`, `agt_fastlio_adapter`, external Batch-LIO/FAST-LIO2 | local LIO output adaptation | navigation/state_estimation |
| `agt_global_relocalization`, `agt_global_relocalization_native`, `agt_localization_manager`, `agt_map_tracker`, `agt_relocalization_benchmark` | BBS/GICP, map correction, tracking and benchmarks | navigation/localization |
| `agt_nav2_bringup`, `agt_pointcloud_preprocessor` | Nav2 wrapper/config and obstacle cloud | navigation/nav2 (preprocessor remains cleaning-owned) |
| `agt_map_manager` | map package registry, validation, active-map binding and edit lifecycle | map_data_manager |
| `agt_mapping_bringup`, `agt_mapping_session`, `agt_map_converter` | mapping orchestration, session control, PCD-to-Nav2 map conversion | mapping (converter is also a cleaning/export boundary) |
| `agt_terrain_map_generator` | offline terrain/map cleaning and export | cleaning |
| `agt_system_bringup` | field/system orchestration | bringup |
| `agt_base_control`, `agt_navigation_runtime`, `agt_demo_task`, `agt_rviz_patrol`, `agt_capability_camera`, `agt_operator_console` | command safety, mission/runtime, HMI/camera | bringup/runtime-safety boundary |
| `agt_rtk_manager` | RTK quality/record only | sensor |
| `agt_robot_interfaces` | AGT messages/services/actions | interfaces |
| `agt_gazebo_sim` | simulation fixtures | tests/simulation |

## Running chain and interface ownership

The protected field chain is:

```text
/livox/lidar + /livox/imu
  -> Batch-LIO -> /aft_mapped_to_init
  -> agt_batch_lio_adapter -> /agt/odometry/local, odom -> base_link

/livox/lidar -> agt_livox_tools -> /agt/livox/points
  -> global relocalization (Polar Context, 3D-BBS, small_gicp)
  -> agt_localization_manager -> map -> odom

active Map Package: localization/global_map.pcd + navigation/map.yaml/map.pgm
  -> global localization + Nav2 -> /cmd_vel -> cmd_vel_guard -> /mux/cmd_vel
```

`agt_localization_manager` is the sole intended authority for `map -> odom`.
The adapter normalizes Batch-LIO local odometry to `odom -> base_link` and
publishes `/agt/odometry/local`.  RTK is diagnostic metadata only.  Important
compatible interfaces include `/livox/lidar`, `/livox/imu`, `/agt/livox/points`,
`/agt/odometry/local`, `/agt/map/status`, `/agt/localization/status`, `/map`,
`/cmd_vel`, `/mux/cmd_vel`, `/navigate_to_pose`, `/agt/localization/relocalize`,
and `/camera_gimbal/acquire_view`.

## Launch chain

The present field entrance is `agt_system_bringup/rviz_field_demo.launch.py`;
`navigation_debug.launch.py` adds `sensor_session.launch.py`, C1 capability and
the demonstration task.  The field demo includes mapping LIO navigation mode,
Livox bridge, obstacle cloud, relocalization, localization manager, optional
map tracker/RTK, Nav2, guard, runtime and RViz patrol. `hmi_field_demo` is the
active-map lifecycle path. `system.launch.py` is a selective legacy aggregator.

## Map and third-party governance

`agt_map_manager` already validates a versioned package using `metadata.yaml`,
SHA-256, `localization_map`, `navigation_map`, optional relocalization assets,
and an `active_map.yaml` pointer. Its expected assets match
`<map_id>/<version>/{localization,navigation,...}` without forcing migration of
existing data under `/home/yangxuan/ros2_ws/agt_data/maps`.

Upstream code is not nested in this repository: `dependencies/agt_navigation.repos`
and `field_demo.repos` pin Livox SDK/driver, FAST-LIO2, Batch-LIO, PGO/HBA,
3D-BBS, small_gicp and Sophus. They are governed as external dependencies and
must not be moved in this refactor.

## Mixed responsibilities, migration candidates, and protected areas

`agt_mapping_bringup/navigation_lio.launch.py` is a state-estimation wrapper
currently named as mapping; launch/config ownership should move first, retaining
its installed launch compatibility. `agt_system_bringup` mixes product bringup
with sensor-session and debug entrypoints. `agt_pointcloud_preprocessor` is an
online cleaning node consumed by Nav2. `agt_map_converter` bridges mapping and
map-lifecycle assets and should retain its CLI/package name while its ownership
is documented explicitly.

Do not modify FAST-LIO2, Batch-LIO, BBS/GICP implementation, Nav2 tuning,
MID360 extrinsics, map data, raw LIO point timing, or the TF/topic names above.
Directory relocation must preserve ROS package names and installed launch paths.

## Risks

Moving a package changes source paths but not ROS names; stale relative paths,
CI globs, and `.repos` instructions are the main migration risk.  Launches can
start duplicate sensor/RTK or TF publishers if composed incorrectly. High-rate
PointCloud2 uses DDS between independent processes; no executor/composition
change is authorized in this phase.
