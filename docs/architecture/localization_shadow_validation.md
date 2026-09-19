# AGT Localization v1 Patch 2 — Shadow Validation

## Scope

Patch 2 adds `agt_localization_ros` as a standalone compatibility/shadow layer. It is not included by any existing field, navigation, offline replay, or FAST-LIO launch.

The shadow node subscribes to the legacy observation contract:

```text
/agt/odometry/local
/agt/relocalization/pose
/agt/map_tracking/pose
/agt/map_tracking/status
/agt/localization/status
```

It also read-only subscribes to `/agt/localization/metrics` to calculate a legacy-versus-shadow correction delta when both corrections exist.

It publishes only ordinary namespaced ROS topics:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/agt/localization/v1/shadow/state` | `agt_localization_interfaces/msg/LocalizationState` | public v1 state projection |
| `/agt/localization/v1/shadow/map_odom` | `geometry_msgs/msg/TransformStamped` | candidate correction as a normal topic; never `/tf` |
| `/agt/localization/v1/shadow/diagnostics` | `std_msgs/msg/String` | JSON legacy/v1 state, correction delta, innovation, decision, reason |

`shadow_manager_node.py` has no `TransformBroadcaster`, no `/tf` publisher, and no `/agt/relocalization/request` publisher. The legacy `agt_localization_manager` remains the sole `map -> odom` owner.

## Build and unit validation

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths src/agt_navigation_v3 \
  --packages-select agt_localization_interfaces agt_localization_core agt_localization_ros
source install/setup.bash
colcon test --base-paths src/agt_navigation_v3 \
  --packages-select agt_localization_interfaces agt_localization_core agt_localization_ros
```

Observed result:

```text
agt_localization_core: 10 tests, 0 failures, 0 errors
agt_localization_ros:  7 tests, 0 failures, 0 errors
```

The shadow tests cover global correction computation, innovation-gate rejection, recovery as a diagnostic-only `WOULD_REQUEST_RECOVERY`, legacy wire-state mapping, and a static assertion that the shadow source does not reference a TF broadcaster.

## Offline navigation bag replay

Input bag:

```text
~/ros2_ws/nav_full_baseline_20260917_114327
duration: 265.213664838 s
```

The replay used isolated `ROS_DOMAIN_ID=73`, so it did not interact with a live robot or another ROS graph:

```bash
export ROS_DOMAIN_ID=73
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch agt_localization_ros localization_shadow.launch.py use_sim_time:=true

ros2 bag record -o \
  ~/ros2_ws/experiments/results/localization_shadow_validation/nav_full_baseline_20260917_114327_shadow_clean \
  /agt/localization/v1/shadow/state \
  /agt/localization/v1/shadow/map_odom \
  /agt/localization/v1/shadow/diagnostics

ros2 bag play ~/ros2_ws/nav_full_baseline_20260917_114327 --clock --rate 40 \
  --topics /agt/odometry/local /agt/localization/status /agt/localization/metrics
```

Result bag:

```text
~/ros2_ws/experiments/results/localization_shadow_validation/
  nav_full_baseline_20260917_114327_shadow_clean/
```

| Output topic | Count |
| --- | ---: |
| `/agt/localization/v1/shadow/state` | 265 |
| `/agt/localization/v1/shadow/diagnostics` | 265 |
| `/agt/localization/v1/shadow/map_odom` | 0 |

### Input completeness and comparison result

The selected navigation bag contains:

| Legacy input | Count |
| --- | ---: |
| `/agt/odometry/local` | 2600 |
| `/agt/localization/status` | 1325 |
| `/agt/localization/metrics` | 1325 |
| `/agt/relocalization/pose` | 0 |
| `/agt/map_tracking/pose` | 0 |
| `/agt/map_tracking/status` | 0 |

Legacy recorded states were `LOCALIZED=982`, `LOST=340`, `DEGRADED=3`. Since no global or tracking observation was recorded, the shadow model had no valid input from which it could reconstruct `T_map_odom`. Its output was therefore:

```text
v1 core state: WAIT_GLOBAL = 265
v1 public state: SEARCHING = 261, UNINITIALIZED = 4
shadow correction present: 0
legacy metrics received: 261
```

This is the expected safe behavior: the shadow path does **not** derive a correction from legacy status/metrics and does **not** emit an identity or stale `map -> odom` transform when an observation is absent.

## Validation conclusion

The replay verifies the non-invasive properties of Patch 2:

- no TF message was emitted by shadow;
- no shadow correction was fabricated without global/tracking observations;
- namespaced state and diagnostics were recorded successfully;
- legacy status/metrics can be observed without changing their types or publishers;
- all replay processes exited cleanly after the second run.

It does **not** validate numerical correction parity because this navigation bag did not record `/agt/relocalization/pose` or `/agt/map_tracking/pose`. A parity-qualified replay bag must contain, with non-zero counts:

```text
/agt/odometry/local
/agt/relocalization/pose
/agt/map_tracking/pose          # optional when tracking is disabled, required for tracker parity
/agt/map_tracking/status        # same condition
/agt/localization/status
/agt/localization/metrics
```

When such a bag is available, compare each shadow diagnostic's `legacy_shadow_translation_delta_m`, `legacy_shadow_yaw_delta_rad`, decision, and state against the nearest legacy metrics/status sample. Only then may a future patch propose a v1 active manager profile.
