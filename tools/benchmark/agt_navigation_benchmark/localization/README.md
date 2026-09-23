# Localization shadow offline validation

This directory contains read-only tools for Patch 3A.  It never starts a
localization node, publishes TF, or enables v1 as the active localization
path.

## Patch 3B: check the replay contract first

Before replaying a bag into legacy plus shadow, run the strict contract check:

```bash
python3 localization/check_localization_replay_contract.py \
  --bag /path/to/replay_bag \
  --report /path/to/localization_replay_contract.md
```

It prints `OK` only when every required topic has the baseline ROS type and a
nonzero message count. Otherwise it prints `BLOCKED_INPUT_CONTRACT` and lists
missing, zero-message, and type-mismatched topics. A zero-count topic is not
valid evidence even if it appears in rosbag metadata.

## Record a complete comparison bag

Run the legacy manager and the shadow manager together, while the legacy
manager remains the only `map -> odom` TF owner.  In another terminal record:

```bash
cd /home/yangxuan/ros2_ws/src/agt_navigation_benchmark
./localization/record_shadow_validation.sh \
  /home/yangxuan/ros2_ws/experiments/results/localization_shadow_validation/<run_id>
```

The run is complete only if it contains nonzero messages for local odometry,
global relocalization pose, and map-tracking pose, in addition to legacy
metrics/status and shadow state/diagnostics.

## Analyze

One combined recording can be passed as both inputs.  Separate legacy and
shadow bags are also supported.

```bash
source /opt/ros/humble/setup.bash
source /home/yangxuan/ros2_ws/install/setup.bash
cd /home/yangxuan/ros2_ws/src/agt_navigation_benchmark
python3 localization/analyze_shadow_validation.py \
  --legacy-bag /path/to/legacy_or_combined_bag \
  --shadow-bag /path/to/shadow_or_combined_bag \
  --output-dir /home/yangxuan/ros2_ws/experiments/results/localization_shadow_validation/<run_id>/metrics \
  --report /home/yangxuan/ros2_ws/src/agt_navigation_v3/docs/architecture/localization_shadow_metrics.md
```

The CSV schema is fixed for downstream comparison:

```text
timestamp,legacy_state,shadow_state,translation_error,yaw_error,innovation,correction_accept
```

`innovation` is a compact JSON value with `translation_m` and `yaw_rad`.  The
report blocks numeric parity when the recording has no global or tracking pose
observations instead of treating unavailable correction values as zeros.
