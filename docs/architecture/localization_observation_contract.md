# Localization Replay Observation Contract

## Purpose and scope

This contract is the admission gate for an offline **legacy versus v1 shadow**
localization replay. It is derived from the frozen code-level baseline in
[`localization_contract_baseline_audit.md`](localization_contract_baseline_audit.md)
and the evidence gap found in
[`localization_shadow_validation.md`](localization_shadow_validation.md).

It defines recording evidence only. It does not change a runtime topic, TF
owner, localization algorithm, launch profile, or map package.

## Required replay topics

Every required topic must be present in rosbag metadata, have the exact type
below, and contain at least one message. A declared topic with zero messages
is an observation absence and blocks the replay.

| Topic | Type | Why it is required |
| --- | --- | --- |
| `/agt/odometry/local` | `nav_msgs/msg/Odometry` | Timestamped `T_odom_base` used to reconstruct `T_map_odom`. Baseline semantic frames are `odom` and `base_link`. |
| `/agt/localization/status` | `agt_robot_interfaces/msg/LocalizationStatus` | Legacy typed wire state. This topic is not a JSON string. |
| `/agt/relocalization/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Global `T_map_base` observation and its covariance. |
| `/agt/map_tracking/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Local tracking correction observation in `map`. |
| `/agt/map_tracking/status` | `std_msgs/msg/String` | JSON tracker health, rejection, and recovery context. |
| `/tf` | `tf2_msgs/msg/TFMessage` | Records the legacy dynamic chain, including legacy-owned `map -> odom`. |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Records the static `base_link` to sensor-frame chain required for frame interpretation. |

The strict Patch 3B contract applies to a profile in which global
relocalization and map tracking are both under test. A profile that deliberately
disables the tracker must use a separate, explicitly versioned contract; it may
not claim full tracker parity through this one.

## Optional but recommended evidence

These topics are not admission requirements, but should be recorded whenever
available:

| Topic | Purpose |
| --- | --- |
| `/agt/localization/metrics` | Legacy map->odom correction, innovation, reason, and quality used for numeric shadow comparison. |
| `/agt/relocalization/status` | JSON legacy debug state and recovery context. |
| `/agt/global_relocalization/status` | Backend progress and acceptance context. |
| `/agt/localization/v1/shadow/state` | v1 public state projection. |
| `/agt/localization/v1/shadow/map_odom` | Shadow correction on a normal topic, never `/tf`. |
| `/agt/localization/v1/shadow/diagnostics` | Shadow decision, reason, innovation, and legacy/shadow delta. |
| `/clock` | Required when using simulated-time bag replay. |

## Acceptance rule

The checker result is one of:

```text
OK
BLOCKED_INPUT_CONTRACT
```

`OK` means only that the required recording evidence is available. It does not
prove correction parity, state parity, numerical accuracy, or vehicle safety.
`BLOCKED_INPUT_CONTRACT` means one or more required topic is missing, has zero
messages, or has a different ROS message type. In that case the bag must not
be used to accept a shadow-to-active migration.

## Tooling

The read-only checker is located at:

```text
~/ros2_ws/src/agt_navigation_benchmark/localization/check_localization_replay_contract.py
```

Run it before starting an offline replay:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/agt_navigation_benchmark

python3 localization/check_localization_replay_contract.py \
  --bag /path/to/localization_replay_bag \
  --report /path/to/localization_replay_contract.md
```

The Patch 3A recorder records all required topics plus optional metrics and
shadow diagnostics:

```bash
./localization/record_shadow_validation.sh \
  ~/ros2_ws/experiments/results/localization_shadow_validation/<run_id>
```

Neither tool publishes TF or starts a ROS node. The legacy
`agt_localization_manager` remains the only `map -> odom` owner; shadow output
remains a namespaced non-TF diagnostic topic.
