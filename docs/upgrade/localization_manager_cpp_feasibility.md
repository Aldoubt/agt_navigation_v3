# P5 Localization Manager C++ Feasibility Audit

Date: 2026-10-02

Branch: `refactor/navigation-runtime-v4`

Decision: **`KEEP_PYTHON_THIS_RELEASE`**

This is the P5 decision required by `PLATFORM_RUNTIME_CPP_IMPLEMENTATION_HANDOFF.md`.
The C++-heavy runtime target does not by itself require translating this node.
The current Python node remains the sole `map -> odom` owner. No C++ candidate,
second broadcaster, live graph, bag replay, or hardware session was started for
this phase.

## Ownership and runtime contract

| Item | Current contract |
|---|---|
| Node/package | `agt_localization_manager`, Python entry point `localization_manager` |
| Production launch | `agt_system_bringup/localization.launch.py` includes `agt_localization_manager/localization_manager.launch.py` once |
| TF authority | Sole intended dynamic publisher of `map -> odom` on `/tf`; the selected LIO adapter owns `odom -> base_footprint`; its `/agt/odometry/local` message is `odom -> base_link`. The Bunker driver has `publish_odom_tf=false`. |
| Other publishers | `/agt/localization/status`, `/agt/localization/metrics`, `/agt/relocalization/status`, and `/agt/relocalization/request` are published by this node only in the production localization launch. |
| Command path | Does not publish velocity, motor, or actuator commands. Nav2 consumes its TF and localization status, so a wrong or stale transform can still affect navigation safety. |
| Executor | `rclpy.spin(node)` with the default single-threaded executor; state callbacks and timers are serialized and no explicit locks are used. |

## Rates, queues, and time behavior

- `/agt/odometry/local` subscription depth is 100. Accepted messages must have
  `header.frame_id=odom` and `child_frame_id=base_link`. Samples are kept in a
  30-second timestamp buffer; a global or tracking pose uses the nearest sample
  only when the timestamp skew is at most 0.10 seconds.
- `/agt/relocalization/pose` and `/agt/map_tracking/pose` subscriptions have
  depth 10. Tracking and backend status subscriptions have depth 20; map events
  have depth 10. The publishers use the default ROS publisher QoS with depths
  10 (status/metrics/debug/request).
- The TF timer is configured for 30 Hz (minimum effective parameter value is
  clamped to 1 Hz). Status and metrics timers run at 5 Hz. TF stamps use the
  node clock at publication time, rather than copying the source pose stamp.
- TF is sent only when a valid correction exists and the state is `LOCALIZED`,
  `TRACKING`, or `DEGRADED`. `LOST`, `WAIT_GLOBAL`, and relocalization states
  stop refreshing the dynamic transform so stale data can expire. The
  visualization-only identity transform parameter is false in the production
  YAML.
- Current source/config target: 30 Hz TF, 5 Hz status/metrics. Historical,
  read-only `NAV_TEST_002` bag audit records 5,392 `map -> odom` samples and
  1,788 local-odometry messages over a 180.948-second bag span (about 29.8 Hz
  and 9.9 Hz when divided by the full span). This is an observation from an
  archived recording, not a replay or a benchmark of this branch's CPU,
  callback deadline, or jitter.

## API and correction math

Inputs:

| Interface | Type | Meaning |
|---|---|---|
| `/agt/odometry/local` | `nav_msgs/Odometry` | Timestamped local `odom -> base_link` pose |
| `/agt/relocalization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Global `map -> base_link` anchor proposal |
| `/agt/map_tracking/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Low-rate tracking proposal; never owns TF |
| `/agt/map_tracking/status`, `/agt/global_relocalization/status`, `/agt/map/events` | `std_msgs/String` | Tracker/backend health and active-map invalidation events |
| `/agt/localization/relocalize` | `std_srvs/Trigger` service | Invalidates the active correction and requests global relocalization |

Outputs:

| Interface | Type | Meaning |
|---|---|---|
| `/tf`: `map -> odom` | `geometry_msgs/TransformStamped` | Sole authoritative global correction |
| `/agt/localization/status` | `agt_robot_interfaces/LocalizationStatus` | Frozen wire state, odom freshness, correction validity, map identity and reason |
| `/agt/localization/metrics` | `agt_robot_interfaces/LocalizationMetrics` | Correction delta, tracking innovation and uncertainty diagnostics |
| `/agt/relocalization/status` | `std_msgs/String` JSON | Internal state, backend/tracker state and recovery counters |
| `/agt/relocalization/request` | `std_msgs/Empty` | Requests the global relocalization worker |

There is no action server in this node. A global pose is validated for map
frame, nonzero timestamp, covariance presence, position standard deviation
(default max 1.0 m), and yaw standard deviation (default max 20 degrees). The
correction is computed from timestamp-aligned poses:

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

A global result is a hard anchor. Accepted tracking results require the
innovation gate (default 0.50 m / 5 degrees) and two consistent measurements
(default consistency 0.20 m / 2 degrees), then update a target correction.
The 30 Hz timer smooths tracking changes with a 3-second time constant and
limits correction change to 0.10 m/s and 2 degrees/s. Global relocalization
replaces the current correction immediately.

## State transitions

Internal states are `BOOT`, `WAIT_GLOBAL`, `LOCALIZED`, `TRACKING`,
`DEGRADED`, `LOST`, `RECOVERY_REQUESTED`, and `RELOCALIZING`. The frozen
`LocalizationStatus` wire enum maps `TRACKING` to `LOCALIZED` and both recovery
states to `RELOCALIZING`; the debug JSON reports the full internal state.

- Startup/no local odometry or no valid correction stays in `WAIT_GLOBAL`.
- A valid global pose plus a time-aligned local sample establishes the hard
  correction and enters `LOCALIZED`.
- Accepted, consistent tracker poses enter `TRACKING`; innovation or tracker
  health problems enter `DEGRADED`.
- Local odometry older than 0.30 seconds enters `DEGRADED`; older than 1.00
  second enters `LOST`. A transient local-odometry loss can recover if the
  existing correction remains available.
- Tracker `RECOVERY_REQUIRED` clears the correction and enters `LOST`, then
  requests global relocalization if enabled and the 5-second cooldown permits.
  The request transitions through `RECOVERY_REQUESTED` to `RELOCALIZING` in one
  callback sequence.
- Manual Trigger service calls clear the correction and publish an Empty
  relocalization request. A mismatching `MAP_ACTIVATED` event also invalidates
  the correction. Terminal backend failure with no correction returns to
  `WAIT_GLOBAL` and preserves a visible failure reason.

## Python dependencies and test coverage

The node uses ROS Python APIs (`rclpy`, `tf2_ros`, generated ROS messages) and
Python standard-library modules (`math`, `json`, `collections.deque`,
`dataclasses`, `enum`, and `typing`). No Python-only numerical or platform
library is required. The current implementation does not have a language
dependency that makes C++ necessary.

`test/test_correction_math.py` has 13 unit cases covering SE(3) compose/inverse,
the map-to-odom equation, smoothing, selected tracking/recovery helpers,
metrics serialization, and backend-status behavior. It does not exercise a
live ROS graph, all callback-to-state transitions, TF broadcaster timing, or
real-time behavior. P4's static launch ownership test verifies one manager
launch inclusion in source; it is not runtime graph evidence.

## Decision and limits

Keep the manager in Python for this release. The configured and historically
observed publication rates are modest, the archived recording is consistent
with the configured TF rate, and no profiler or deadline evidence identifies a
CPU or determinism problem. C++ would reimplement the most authoritative and
stateful TF owner without a measured benefit, increasing the chance of changing
timestamp matching, uncertainty rejection, tracking gates, correction
smoothing, failure visibility, or recovery behavior. This node is
navigation-safety relevant because it owns global TF, but it is not the
50 Hz motion command or final actuator safety loop.

No C++ parity is claimed. A later migration proposal must first add frozen
math/state tests, then run a shadow candidate from rosbag with outputs isolated
from `/tf`; only the Python node may broadcast `map -> odom` during that work.
The authority switch requires numeric/state parity, graph inspection, and a
documented return to the Python launch. Reconsider migration only if profiling
shows a measured deadline miss, CPU constraint, or equivalent concrete need.

## P5 evidence classes and rollback

| Evidence class | P5 result |
|---|---|
| Unit/mock | **PASS** — package build succeeded and all 13 correction-math unit tests passed. No ROS node was started. |
| Rosbag replay | **NOT_RUN** — no replay or shadow comparison was run. The archived `NAV_TEST_002` audit was read-only historical observation only. |
| Real hardware | **NOT_RUN** — no launch, CAN operation, base, camera, or other physical device test was run. |

Runtime rollback is straightforward because this phase changes no runtime
code: keep `agt_system_bringup/localization.launch.py` selecting the existing
Python `localization_manager.launch.py`, which remains the only `map -> odom`
publisher. If future work creates an experimental C++ candidate, keep it
off `/tf` and revert its candidate launch/config before any production use.
This P5 documentation-only decision can be reverted independently without
changing runtime behavior.
