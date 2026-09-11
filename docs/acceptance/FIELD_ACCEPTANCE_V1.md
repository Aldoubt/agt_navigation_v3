# Field Acceptance V1

Branch: `release/field-acceptance-v1`

This branch freezes the functional-domain architecture at `8d0ab16` and turns the repository into an acceptance-oriented delivery baseline. Large package moves, package renames, and new architecture work are out of scope until the acceptance release is merged.

## Acceptance modes

The acceptance workflow has three runtime modes:

- `offline_replay`: deterministic rosbag/map regression. No hardware command output is allowed.
- `field_debug`: real sensors and localization, but explicit map assets and manual initialization are allowed.
- `field_acceptance`: fixed map, fixed start/targets, fixed Nav2 profile, evidence logging, camera capture, and return-home mission.

## Map workflow

The acceptance map pipeline is intentionally simple and reproducible:

```text
global_map.raw.pcd
        |
        +--> optional manual PCD cleanup
        |      (remove people/vehicles/ghosts; preserve static geometry)
        v
global_map.cleaned.pcd
        |
        +--> pcd_to_pgm acceptance converter
        v
map.raw.pgm + map.yaml
        |
        +--> optional manual PGM cleanup
        v
map.acceptance.pgm + map.yaml
        |
        +--> validate + rebuild hashes/manifest
        v
acceptance map package
```

Do not disable integrity checks globally. Manual edits become a new accepted asset by regenerating the manifest/hash after validation.

For navigation-only defects, editing the PGM is allowed while keeping the localization PCD unchanged. If dynamic clutter materially affects GICP/localization, clean the PCD and rebuild localization assets from the cleaned PCD.

## Localization modes

The existing `agt_localization_manager` remains the exclusive `map -> odom` owner.

Formal/global mode:
```text
3D-BBS -> GICP -> /agt/relocalization/pose -> localization_manager
```

Acceptance/manual-seed mode:
```text
RViz /initialpose or named start pose
        -> local GICP refinement
        -> /agt/relocalization/pose
        -> localization_manager
```

The manual path is an acceptance shortcut, not a replacement for global relocalization. Keyboard XY/yaw nudging is not the primary UI; use RViz 2D Pose Estimate or a named pose from YAML. The test field should define one or more surveyed start poses.

## Mission mode

Add an acceptance mission alongside the existing `navigate_capture_task` instead of overloading the existing demo bridge.

State flow:

```text
PRECHECK
 -> LOCALIZED
 -> NAVIGATE waypoint_i
 -> MEASURED_STOP
 -> CAPTURE
 -> RECORD
 -> NEXT
 -> RETURN_HOME
 -> COMPLETE
```

The mission owns the waypoint sequence and uses Nav2 `NavigateToPose`. Each target records the requested map pose, final map/base pose, local odometry pose, timestamps, Nav2 result, camera output, and operator-entered physical error.

`RETURN_HOME` sends the stored home pose as a normal absolute Nav2 goal. It must not replay velocity commands or reverse the recorded trajectory.

## Navigation precision profile

The current general baseline uses `xy_goal_tolerance=0.15 m`. That is too loose for the 5 m acceptance target whose mean error limit is 0.12 m.

The acceptance overlay should start with:

```yaml
controller_server:
  ros__parameters:
    progress_checker:
      required_movement_radius: 0.05
      required_movement_angle: 0.05
      movement_time_allowance: 15.0
    general_goal_checker:
      xy_goal_tolerance: 0.08
      yaw_goal_tolerance: 0.0872665

planner_server:
  ros__parameters:
    GridBased:
      tolerance: 0.05
```

Keep the RPP controller, 50 Hz controller frequency, measured-stop gate, and downstream acceleration limits. Tune terminal approach behavior before increasing global speed. If 0.08 m causes repeated near-goal oscillation, diagnose localization/controller bias before relaxing the tolerance.

The return-home leg can use the same profile for simplicity. If return-home is not part of the scored position test, a later task-level completion radius may be relaxed without changing the scored waypoint profile.

## Official acceptance targets

- Speed: physical 30 m measured interval, two runs, average speed >= 1.0 m/s.
- Slope: loaded chassis climbs >= 30 degree slope.
- Endurance: all required devices powered, commanded running condition, >= 3 h.
- Navigation accuracy: 5 m / 7 m / 10 m targets, five runs each, mean physical error <= 0.12 / 0.15 / 0.20 m.
- Data collection: multiple fixed collection points, operator confirms required data are present and usable.
- Vision: offline test set evaluation, mAP@0.5 >= 0.80.

Physical ruler/laser measurements are the acceptance ground truth for navigation accuracy. Nav2 pose error is supporting telemetry, not the final truth source.

## Offline pre-acceptance

A test-field rosbag and acceptance map are sufficient to run deterministic software gates offline:

```text
rosbag
 -> sensor/LIO replay
 -> map load
 -> manual or GICP-seeded localization
 -> localization state/TF checks
 -> Nav2 planning checks
 -> mission state-machine tests
 -> camera/data mocks
 -> report generation
```

Offline replay cannot prove physical 30 m speed, 30 degree climbing, 3 h battery endurance, or physical 5/7/10 m navigation error. It is the pre-acceptance gate used to catch software failures before field testing.

Nav2 closed-loop controller behavior also requires a simulated/mocked moving base. A rosbag alone provides a recorded trajectory and does not respond to new `cmd_vel` commands.

## Release rule

Only bug fixes, acceptance tooling, parameter overlays, logging, hardware adaptation, and safety-gate changes are allowed on this branch. After all official acceptance items pass, merge this branch to `main` and tag the field-acceptance release.


## Current implementation entry points

### 1. Reproducible map cleanup

Generate the initial Nav2 map from the cleaned/frozen PCD:

```bash
ros2 run agt_map_converter pcd_to_nav_map \
  /path/to/global_map.cleaned.pcd \
  --output /path/to/acceptance_map/navigation \
  --resolution 0.10 \
  --max-step 0.22 \
  --max-slope-deg 20.0

ros2 run agt_map_converter validate_nav_map \
  /path/to/acceptance_map/navigation
```

For a manual occupancy correction, write a patch YAML in map-frame meters:

```yaml
edits:
  - mode: free
    note: remove temporary parked vehicle
    polygon_m:
      - [12.1, 3.2]
      - [14.0, 3.2]
      - [14.0, 4.8]
      - [12.1, 4.8]
```

Then create a new edited map directory instead of overwriting the source:

```bash
ros2 run agt_map_converter patch_nav_map \
  /path/to/acceptance_map/navigation \
  /path/to/remove_vehicle.yaml \
  --output /path/to/acceptance_map/navigation-edited
```

The tool edits `map.pgm` (and `obstacle.pgm` when present), records the patch in
`converter_metadata.yaml`, and runs map validation. The patch YAML is the audit
record of the manual edit.

### 2. Offline rosbag pre-acceptance

```bash
ros2 launch agt_system_bringup acceptance_offline_replay.launch.py \
  bag:=/path/to/test_field_bag \
  navigation_map:=/path/to/navigation/map.yaml \
  localization_map:=/path/to/global_map.cleaned.pcd \
  relocalization_assets:=/path/to/relocalization
```

The offline entry point starts no Bunker/CAN/camera hardware driver. In RViz use
**2D Pose Estimate** to publish `/initialpose`; the acceptance relocalizer crops
a local submap and runs `map_gicp_tracker` (small_gicp) before publishing the
validated pose to `/agt/relocalization/pose`.

### 3. Field acceptance

```bash
ros2 launch agt_system_bringup acceptance_field.launch.py \
  navigation_map:=/path/to/navigation/map.yaml \
  localization_map:=/path/to/global_map.cleaned.pcd \
  relocalization_assets:=/path/to/relocalization \
  waypoint_file:=/path/to/field_waypoints.yaml
```

After sensor/LIO readiness:

1. In RViz set the approximate pose with **2D Pose Estimate**.
2. Wait for `/agt/localization/status` to report a valid localized state and
   visually confirm `map -> odom -> base_link`.
3. Start the acceptance mission:

```bash
ros2 service call /agt/acceptance/start std_srvs/srv/Trigger "{}"
```

Cancel at any time with:

```bash
ros2 service call /agt/acceptance/cancel std_srvs/srv/Trigger "{}"
```

The mission executes:

```text
target -> measured stop -> camera capture -> next target -> ... -> return home
```

and writes JSONL evidence under `~/.ros/agt_acceptance/runs`.

### 4. Acceptance Nav2 profile

Formal field acceptance uses the complete installed parameter file:

```text
agt_nav2_bringup/config/nav2_acceptance_params.yaml
```

It is a full copy of the normal baseline with only terminal-accuracy settings
tightened. Do not pass a partial YAML overlay directly to Nav2.
