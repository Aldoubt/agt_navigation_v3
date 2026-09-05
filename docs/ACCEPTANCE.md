# Acceptance plan

## Offline replay gate — PASSED on 2026-09-05

The first end-to-end automatic relocalization replay gate is now passed with:

- optimized map package: `agt_data/maps/bag_mapping_current`
- matching source replay: `bunker_mid360_mapping_20260901_211105`
- no `/initialpose`
- Batch-LIO local odometry
- configured `body -> base_link` calibrated extrinsic
- AGT Polar Context candidate retrieval
- candidate-local CPU 3D-BBS coarse registration
- small_gicp 6-DoF refinement
- `agt_localization_manager` as the only `map -> odom` owner

Observed successful state sequence:

```text
QUERY_READY
  -> BBS_SEARCHING
  -> BBS_COARSE_FOUND
  -> GICP_REFINING
  -> SUCCEEDED
  -> global_pose_accepted
  -> LOCALIZED
```

Representative accepted replay result:

```text
score                 0.874344
fitness               0.128999
overlap               0.973186
position_std_m        0.231676
yaw_std_deg           4.507872
candidate_patch       35.pcd
global_correction     valid
local_odom_fresh      true
```

`map -> odom` was also observed continuously after acceptance. This passes the
**offline functional gate**, not the full field-product gate. Multi-location,
multi-heading and real-vehicle tests below are still required before freezing V1.

## Gazebo closed-loop navigation gate — PASSED on 2026-09-05

A clean, single-stack Gazebo run also passed the boot-to-motion software gate:

- ROS domain: `148`
- no `/initialpose`
- one `navigation_demo.launch.py` stack only
- automatic Polar Context -> candidate BBS -> GICP relocalization
- `agt_localization_manager` accepted the correction and opened `cmd_vel_guard`
- Nav2 loaded `PoseProgressChecker`
- SmacPlanner2D + RPP produced a real closed-loop chassis trajectory
- a minimal `rclpy` action client received final `NavigateToPose=SUCCEEDED`

Cold-start relocalization evidence from this run:

```text
score                   0.926627
fitness                 0.110056
overlap                 0.991720
position_std_m          0.197924
yaw_std_deg             3.884748
candidate_patch         18.pcd
BBS elapsed             453.279 ms
global_correction       accepted
cmd_vel_guard           OPEN
```

Navigation acceptance used a known-free PGO keypose and completed with:

```text
NAV_ACCEPTANCE status=SUCCEEDED elapsed_sec=45.322
```

RPP remains configured at `50 Hz`. Its controller-side
`FollowPath.max_angular_accel` is `10.0 rad/s^2` only to avoid a bootstrap lock
when the 10 Hz pose-derived LIO twist reports zero at the start of a turn. The
physical output is still bounded downstream by the velocity smoother and
`cmd_vel_guard` (`0.8 rad/s^2` angular acceleration limit in the current demo
baseline).

This Gazebo run still emitted some missed-50-Hz warnings under the current
desktop/ToDesk/Gazebo load. Therefore it proves functional closed-loop behavior,
not hard real-time scheduling performance. Do not lower the field controller
frequency just to make the simulator quiet.

### Fail-closed velocity gate acceptance

The guard was also tested in an isolated ROS domain with real ROS pub/sub:

```text
LOCALIZED + fresh cmd
  -> non-zero /mux/cmd_vel
LOST + continued non-zero upstream cmd
  -> immediate hard zero
LOCALIZED again, but no new cmd
  -> remains zero (no stale replay)
new post-recovery cmd
  -> motion resumes
```

Measured result:

```text
GUARD_ACCEPTANCE PASS
opened_peak             0.300
LOST stop latency       1.4 ms
LOST peak after 80 ms   0.000000
stale replay peak       0.000000
fresh resume peak       0.200
```

Repeat with:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ROS_DOMAIN_ID=149 python3 \
  ~/ros2_ws/src/agt_navigation_v3/src/agt_base_control/test/guard_fail_closed_acceptance.py \
  --ros-args --params-file \
  ~/ros2_ws/src/agt_navigation_v3/src/agt_base_control/config/cmd_vel_guard.yaml
```

## V1 core acceptance

The acceptance target is:

> **3D LiDAR global localization with no initial pose, followed by stable FAST-LIO2 continuous tracking.**

### Test A — static startup

1. Start the system with no `/initialpose`.
2. Load a known 3D map.
3. Present a static MID360 scan.
4. Expect `GLOBAL_SEARCH -> ... -> LOCALIZED`.

### Test B — multiple map locations

Repeat from at least 5 materially different locations and headings. Do not seed the real pose.

### Test C — rough tracked motion

Repeat localization/continuous tracking over rough terrain with the actual tracked chassis vibration.

### Test D — tree shade

Repeat in the known tree-shadow environment. Record success rate and time to localization.

### Test E — rear vehicle geometry

Compare self-filter OFF/ON and confirm chassis rods disappear while environmental points remain.

## Initial target metrics

These are V1 engineering targets, not frozen product specifications:

- global relocalization success: >= 90%
- nominal time to accepted pose: <= 10 s when the first descriptor candidate is correct
- fallback budget may extend to ~18 s while the vehicle remains stopped
- planar position error: <= 0.5 m where ground truth is available
- yaw error: <= 5 deg where ground truth is available
- post-handoff FAST-LIO2 tracking: stable for the test trajectory

## V2 system acceptance

Only after V1 is stable:

- software restart without initial pose
- persisted last-pose acceleration
- complete robot reboot
- power-cycle recovery
- RTK/INS prior/fallback integration
- Nav2 mission continuation

Power-cycle tests should verify the whole boot-to-navigation chain separately from the localization algorithm.
