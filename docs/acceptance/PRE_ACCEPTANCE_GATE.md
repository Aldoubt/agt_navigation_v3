# Pre-Acceptance Gate

Branch: `release/field-acceptance-v1`

This gate removes software uncertainty before the vehicle acceptance run. It is
ordered deliberately:

```text
MAP -> LIO -> LOCALIZATION -> MAP TRACKER -> NAV2 PLANNING -> CLOSED-LOOP SIM -> FIELD
```

Do not tune a downstream stage while an upstream stage is failing.

## Gate 1 - MAP READY

Required source assets:

- final mapping PCD
- `poses.txt`
- `patches/*.pcd` when relocalization candidate assets are required

Generate a candidate Nav2 map with trajectory evidence:

```bash
ros2 run agt_map_converter pcd_to_nav_map \
  /path/to/global_map.cleaned.pcd \
  --output /path/to/acceptance_map/navigation \
  --trajectory-poses /path/to/poses.txt \
  --resolution 0.10 \
  --max-step 0.22 \
  --max-slope-deg 20.0
```

Inspect `converter_metadata.yaml`:

- `trajectory_qa_status=PASS`: no generated non-free cell intersected the recorded swept footprint.
- `trajectory_qa_status=REVIEW`: one or more non-free cells intersected a physically traversed corridor and were cleared. Review the corresponding area in RViz for people/vehicle ghosts, canopy, self returns, or projection artifacts.
- `trajectory_qa_status=NOT_RUN`: no trajectory was supplied; the map is not accepted by this gate.

If a reproducible manual occupancy fix is needed, use `patch_nav_map` and keep
the patch YAML as the audit record. Do not overwrite the source map manually.

The localization PCD and Nav2 occupancy map are separate products from the same
mapping session. Navigation-only occupancy edits do not require changing the
localization PCD. If dynamic clutter materially affects GICP, clean the
localization PCD and rebuild relocalization assets.

## Gate 2 - DETERMINISTIC ROSBAG REPLAY

The offline launch intentionally starts no CAN/base/camera hardware.

Manual-seed baseline:

```bash
ros2 launch agt_system_bringup acceptance_offline_replay.launch.py \
  bag:=/path/to/bag \
  navigation_map:=/path/to/navigation/map.yaml \
  localization_map:=/path/to/localization/global_map.cleaned.pcd \
  relocalization_assets:=/path/to/relocalization \
  relocalization_executable:=manual_seed_relocalization \
  auto_relocalize:=false \
  enable_map_tracking:=true
```

Use RViz 2D Pose Estimate to provide the approximate start pose.

Global-relocalization replay:

```bash
ros2 launch agt_system_bringup acceptance_offline_replay.launch.py \
  bag:=/path/to/bag \
  navigation_map:=/path/to/navigation/map.yaml \
  localization_map:=/path/to/localization/global_map.cleaned.pcd \
  relocalization_assets:=/path/to/relocalization \
  relocalization_executable:=global_relocalization \
  auto_relocalize:=true \
  enable_map_tracking:=true
```

Replay checks:

1. Batch-LIO odometry remains current; processing latency must not grow with replay duration.
2. `map -> odom -> base_link` is continuous after a valid global anchor.
3. Nav2 loads the map and can produce a global path.
4. The 0.5 Hz tracker reports `TRACKING_OK`, `HOLD`, `DEGRADED`, or
   `RECOVERY_REQUIRED`; a rejected local registration must never directly move
   `map->odom`.
5. A global relocalization result remains the hard anchor; local tracking only
   updates the correction target.

A rosbag cannot prove closed-loop controller behavior because recorded odometry
does not react to new `cmd_vel` commands.

## Gate 3 - MAP TRACKER ROBUSTNESS

The current low-rate tracker uses:

- fitness gate
- overlap gate
- translation/yaw innovation against the Batch-LIO prediction
- consecutive reject hysteresis
- temporal consistency in `agt_localization_manager`
- GICP Hessian observability diagnostics

Initial failure policy:

```text
bad sample
  -> HOLD

repeated bad samples
  -> DEGRADED

persistent bad samples
  -> RECOVERY_REQUIRED
```

Automatic global recovery is disabled by default until offline replay has
established safe thresholds. Enable
`tracking_recovery_auto_request=true` only after replay validation.

Hessian rejection is also disabled initially. Collect
`hessian_eigenvalues` and `hessian_condition_number` from representative
normal, repeated-geometry, sparse, and dynamic scenes before freezing a field
threshold.

Required replay cases:

- normal structured area
- repeated trees/poles or corridor-like geometry
- sparse/open area
- people/vehicle interference
- perturbed initial pose

For localization safety, false rejection is preferable to false acceptance.

## Gate 4 - CLOSED-LOOP SIMULATION

Use a simulated/mocked moving base for controller validation:

```text
Nav2 -> cmd_vel -> simulated base -> odom -> Nav2
```

Validate:

- 50 Hz controller path
- terminal approach
- no near-goal oscillation
- stop gate
- NavigateToPose result handling
- capture mission sequencing
- return-home behavior

A rosbag alone is insufficient for this gate.

## Gate 5 - FIELD READY

Only enter formal vehicle acceptance when the dashboard/report can show:

```text
MAP          PASS
LIO          PASS
LOCALIZATION PASS
MAP TRACKER  PASS
NAV2 PLAN    PASS
SIM CONTROL  PASS
MISSION      PASS
CAMERA       PASS/WARN
---------------------
FIELD READY
```

The following remain physical field measurements:

- 30 m speed test, average >= 1.0 m/s
- loaded slope >= 30 degrees
- endurance >= 3 h
- navigation physical mean error:
  - 5 m <= 0.12 m
  - 7 m <= 0.15 m
  - 10 m <= 0.20 m
  - five repeats for each target
- final data-acquisition usability confirmation

Physical ruler/laser measurements are the acceptance ground truth for navigation
accuracy. ROS pose error is supporting telemetry.
