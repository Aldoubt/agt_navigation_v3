# Platform Runtime C++ Source Ownership and Migration Audit

Date: 2026-10-02. Target checkout: `/home/yangxuan/ros2_ws/src/agt_navigation_v3`, branch `refactor/navigation-runtime-v4`, source snapshot `9a93b91` before this audit's documentation changes. The fetched branch was fast-forwarded from a clean tree. No user modifications were present. Other local source repositories listed below were inspected read-only and left untouched.

This document is the P0 source audit. It records source-declared contracts and intended owners; it does **not** claim a live ROS graph or hardware observation. No build, test, ROS launch, CAN command, robot motion, or arm action was run while assembling this inventory. Static checks run after the document was written are recorded in the acceptance matrix.

## Scope and classification

The navigation checkout contains 35 tracked-source `package.xml` files and 31 `*.launch.py` files. The inventory below covers production ROS nodes in the target flow, physical platform/sensor owners that feed it, optional state producers, and legacy nodes that can collide with the target flow. Offline analyzers, benchmarks, map tools, test nodes, and RViz utilities are listed separately so they are not mistaken for real-time state authorities.

Every node has exactly one required migration classification:

- `KEEP_PYTHON`: retain as Python orchestration, policy, mission, diagnostics, or low-rate state logic.
- `MIGRATE_CPP`: C++ is the required target for a high-rate/safety/authoritative path. If the node is already C++, this means preserve the existing native C++ implementation; it is not a request to rewrite it.
- `TOOL_ONLY`: test, simulation, diagnostics, operator UI, or offline command-line tool; not part of the production runtime core.
- `REMOVE_AFTER_PARITY`: legacy implementation or duplicate entry point to retire only after the replacement has proven parity.
- `UNKNOWN_NEEDS_AUDIT`: insufficient evidence to choose a safe implementation or physical contract. This classification blocks enabling the affected profile/path.

The audit does not prescribe a mass Python-to-C++ conversion. In particular, Localization Manager remains `UNKNOWN_NEEDS_AUDIT` until the required P5 feasibility audit; this document makes no migration decision for it.

## Production node inventory

| Package / node | Language | Owner and startup path | Interfaces and TF | Authoritative state / single publisher | Rate and real-time or safety relevance | Classification and reason |
|---|---|---|---|---|---|---|
| `robot_state_publisher` | C++ upstream | `agt_robot_bringup/robot_hardware.launch.py` includes `agt_robot_description/display.launch.py` once when description is enabled | Reads `robot_description`; publishes fixed robot/sensor transforms on `/tf_static`; model chain is `base_footprint -> base_link -> chassis/sensors` | Sole intended owner of static vehicle/sensor links. No dynamic odometry TF | Startup/static; frame correctness is navigation-critical | `MIGRATE_CPP` — already native C++; preserve calibrated URDF/extrinsics, no port required |
| `livox_ros_driver2_node` / MID360 | C++ vendor driver | `agt_robot_bringup/robot_hardware.launch.py` in `vendor_launch` mode; mapping launch and `scripts/start_mid360.sh` are alternate entry points | `/livox/lidar` `CustomMsg`, `/livox/imu`; no nav TF authority | One physical MID360 publisher in the active ROS domain | Sensor-rate; raw timing is LIO-critical | `MIGRATE_CPP` — preserve vendor raw data/timing path |
| FAST-LIO2 `lio_node` | C++ upstream | `fastlio_navigation_lio.launch.py`, selected only for `lio_backend=fastlio2` | Inputs `/livox/lidar`, `/livox/imu`; raw output `/fastlio2/lio_odom`; navigation TF and canonical odom are owned by adapter | Selected frontend only; adapter owns `/agt/odometry/local` and navigation `odom -> base_footprint` | LIO-rate; authoritative input to navigation odometry | `MIGRATE_CPP` — existing native front-end; preserve its selected runtime YAML as calibration authority |
| Batch-LIO frontend | C++ upstream | `navigation_lio.launch.py`, selected only for `lio_backend=batch_lio` | Inputs raw `/livox/lidar`, `/livox/imu`; source output `/aft_mapped_to_init`; no preprocessing before frontend | Mutually exclusive with FAST-LIO2; adapter owns canonical output/TF | LIO-rate; authoritative input to navigation odometry | `MIGRATE_CPP` — existing native front-end; preserve its own runtime calibration YAML |
| `agt_fastlio_adapter` | Python | Included by `fastlio_navigation_lio.launch.py` | `/fastlio2/lio_odom` -> `/agt/odometry/local`, `/agt/odometry/adapter_status`; broadcasts `odom -> base_footprint`, derives base twist from consecutive pose | Sole selected FAST-LIO canonical odometry and navigation TF owner | Per-odom callback, status 5 Hz; authoritative state/TF path | `MIGRATE_CPP` — high-rate frame/extrinsic conversion owns navigation odometry and TF; migrate only with calibration and replay parity |
| `agt_batch_lio_adapter` | Python | Included by `navigation_lio.launch.py` | `/aft_mapped_to_init` -> `/agt/odometry/local`, `/agt/odometry/adapter_status`; broadcasts `odom -> base_footprint`; optional parent alias is disabled | Sole selected Batch-LIO canonical odometry and navigation TF owner | Per-odom callback, status 5 Hz; authoritative state/TF path | `MIGRATE_CPP` — same authoritative high-rate boundary; preserve Batch YAML semantics and never copy FAST-LIO signs |
| `livox_format_bridge` | C++ | `agt_system_bringup/localization.launch.py` includes `agt_livox_tools/livox_format_bridge.launch.py` | Raw `/livox/lidar` `CustomMsg` -> secondary `/agt/livox/points` `PointCloud2`; preserves `timebase`, `offset_time`, line/tag and per-point timestamp in forward mode | One secondary-cloud producer; does not replace the raw LIO input | Per scan; sensor-layer correctness, but not a motion command path | `MIGRATE_CPP` — already native C++; preserve the raw/secondary branch split and timing contract |
| `agt_pointcloud_preprocessor` | C++ | Currently instantiated directly by `agt_system_bringup/navigation.launch.py`; standalone `pointcloud_preprocessor.launch.py` is another entry point | `/agt/livox/points` -> `/agt/navigation/points_obstacles`; TF-transforms to `base_link`, range/self/rear/ground/voxel processing; clearing retains unfiltered `/agt/livox/points` | One intended `/agt/navigation/points_obstacles` producer | Per scan; local obstacle marking is motion-safety relevant | `MIGRATE_CPP` — already native C++; never move this filter in front of LIO |
| `agt_localization_manager` | Python | Included once by `agt_system_bringup/localization.launch.py` | Inputs `/agt/odometry/local`, `/agt/relocalization/pose`, optional `/agt/map_tracking/pose` and status; outputs `/agt/localization/status`, metrics/debug/request, service `/agt/localization/relocalize`; broadcasts `map -> odom` | Sole intended authoritative `map -> odom` owner; exact correction is `T_map_base * inverse(T_odom_base)` | TF 30 Hz; status/metrics 5 Hz; authoritative global localization state | P0: `UNKNOWN_NEEDS_AUDIT`; P5: `KEEP_PYTHON_THIS_RELEASE` — rates and authoritative state logic are audited; no measured C++ need |
| `agt_global_relocalization` | Python | Included by `localization.launch.py`; executable selected by launch | Inputs `/agt/livox/points`, `/agt/odometry/local`, request/map state; outputs `/agt/relocalization/pose`, backend status and debug clouds; calls native BBS/GICP executable | Produces pose proposals only; no TF authority | On request or 0.5 s auto poll; navigation initialization/recovery, not high-rate | `KEEP_PYTHON` — map snapshot, process and lifecycle orchestration; compute is already native C++ |
| `candidate_bbs_gicp_localizer`, `bbs_gicp_localizer`, `map_gicp_tracker` | C++ CLI executables | Spawned by Python global-relocalization/tracker processes; not independent ROS graph owners | Consume saved query/map assets and emit candidate/refined poses; no ROS topics or TF | Propose poses; Localization Manager remains `map -> odom` authority | On explicit localization/tracking work; CPU-heavy, not command loop | `TOOL_ONLY` — native worker utilities; retain and audit their CLI contract, do not translate to ROS nodes |
| `initialization_relocalization` / `manual_seed_relocalization` | Python | Standalone alternate executable in `agt_global_relocalization`; not selected by `agt_system_bringup/localization.launch.py` | Subscribes `/initialpose` when explicitly launched, requests scans and publishes pose proposal/status; no TF | Proposal only; no TF ownership | Initialization-only experimental tool, excluded from V1 field runtime | `TOOL_ONLY` — manual-seed code remains available only as an explicit standalone experiment; V1 system bringup hard-rejects non-auto modes and always selects `global_relocalization`. |
| `agt_map_tracker` | Python | Optional include from `localization.launch.py`; disabled by default | `/agt/odometry/local`, `/agt/livox/points`, localization status -> `/agt/map_tracking/pose` and status; no TF | Tracker proposes corrections; manager owns any accepted `map -> odom` correction | Configured low-rate tracking; optional localization feature | `KEEP_PYTHON` — optional policy/registration orchestration; do not create a TF authority |
| `agt_navigation_supervisor` | Python | Direct `Node` in `agt_system_bringup/navigation.launch.py` | Watches `/livox/lidar`, `/livox/imu`, `/wheel/odom`, `/agt/odometry/local`, localization status, TF/Nav2 lifecycle; publishes transient-local `/navigation/health` and reads `/navigation/goal_active` | Sole intended `/navigation/health` publisher; wheel reception appears as `base_alive` diagnostics and is not an input gate in `health_model.decide` | 2 Hz health publication, 0.5 Hz lifecycle checks; health feeds the safety/capability gate but this node does not own commands | `KEEP_PYTHON` — low-rate diagnostics/state policy is readable and testable in Python; no evidence calls for C++ |
| `agt_navigation_capability` | Python | Direct `Node` in `agt_system_bringup/navigation.launch.py` | Actions `/navigation/navigate_to`, `/navigation/follow_route`; consumes `/navigation/health` and optional `/agt/payload/drive_permission`; forwards/cancels Nav2 `/navigate_to_pose` and FollowPath | Owns action policy only; no command or TF publisher | Event-driven plus 10 Hz watchdogs; motion permission gate, not high-rate actuator loop | `KEEP_PYTHON` — asynchronous policy/action orchestration; guard remains the command-safety authority |
| `agt_cmd_vel_guard` (`cmd_vel_guard`) | Python | Direct `Node` in `agt_system_bringup/navigation.launch.py`; standalone `cmd_vel_guard.launch.py` also exists | `/cmd_vel`, `/agt/hmi/cmd_vel`, control mode, localization status, optional payload permission -> `/mux/cmd_vel`, hold status | Sole intended software publisher of `/mux/cmd_vel`; Bunker driver subscribes after remap | 50 Hz refresh, 0.25 s command/status timeouts; safety-critical final software command gate | `MIGRATE_CPP` — high-rate final command/safety path; only after Python shadow/parity and rollback proof |
| Nav2 map server and lifecycle manager | C++ upstream | `agt_nav2_bringup/navigation.launch.py` | Map YAML -> `/map`; lifecycle services; no hardware/TF owner | One map server and map lifecycle manager per navigation launch | Map startup/lifecycle; required Nav2 state | `MIGRATE_CPP` — existing upstream C++; keep as-is |
| Nav2 planner/controller/BT/behavior/waypoint/smoother/costmaps | C++ upstream | `agt_nav2_bringup/navigation.launch.py` includes Humble `nav2_bringup/navigation_launch.py` with `use_composition=False` | `/navigate_to_pose`, planner/controller/costmap topics; controller output `/cmd_vel_nav`, 50 Hz smoother output `/cmd_vel`; consumes `/agt/odometry/local`, `/map`, `/agt/navigation/points_obstacles`, `/agt/livox/points`; no localization TF publisher | Nav2 action authority is isolated from command output; final `/mux/cmd_vel` belongs to guard | Controller/smoother/costmaps configured at 20/50/10 Hz; control-adjacent but not final safety authority | `MIGRATE_CPP` — existing upstream C++; retain configuration and interface rather than rewrite |
| `agt_rviz_path_tool` (`rviz_path_tool`) | Python | Direct `Node` in system navigation launch; offline preview launch is separate | Publishes map-frame route/trails and sends navigation FollowPath action; reads local/wheel odometry; no TF or raw velocity publication | Route UI only | Human-interactive; outside real-time core | `KEEP_PYTHON` — RViz editing and preview are UI/orchestration |
| `agt_rviz_patrol` (`rviz_patrol`) | Python | Optional `agt_rviz_patrol.launch.py`, included by legacy inspection mode | RViz goal queue, markers/status, `/agt/rviz_patrol/{start,clear,cancel}`; delegates to mission runtime; no velocity/TF | Legacy waypoint/task UI; not a physical command owner | Human-interactive task flow | `REMOVE_AFTER_PARITY` — retire only after new queue/capture route has same recorded field behavior |
| `agt_navigation_runtime` (`mission_runtime`) | Python | Optional `runtime.launch.py`; included by `agt_mission_bringup` when `enable_legacy_inspection=true` | `/agt/mission/execute`, camera action `/camera_gimbal/acquire_view`, local odom measured-stop gate, record/status and RTK metadata; no command/TF output | Legacy stop-and-capture task owner | Task-rate async orchestration; stationary gate is safety relevant but it does not command motion | `REMOVE_AFTER_PARITY` — preserve until target queue/capture parity is demonstrated; duplicate task interface risk when run with the V4 mission node |
| `agt_mission_runtime` (`mission_runtime_v4`) | Python | `agt_mission_bringup/mission.launch.py` always includes one node | Mission route action/status, Nav2/Capability calls, measured-stop gate, camera action and synchronized record; no direct velocity/TF output | New mission orchestration owner when mission launch is used | Task-rate; stop/capture sequencing is safety relevant | `KEEP_PYTHON` — action and record orchestration is not a real-time command loop |
| `mission_route_runner` | Python CLI | Optional node from `agt_mission_bringup/mission.launch.py` only when a route file is supplied | Validates/runs a selected route through mission interfaces; no TF or velocity publisher | Route ingress/runner only | Task-rate | `KEEP_PYTHON` — CLI/task input, not runtime safety path |
| `agt_rtk_manager` | Python | Optional standalone `rtk_manager.launch.py` | `/ins/navsatfix`, `/ins/status` -> `/agt/rtk/status`, `/agt/rtk/map_pose`; no TF or odometry output | Metadata/quality observation only | 5 Hz; explicitly not navigation authority | `KEEP_PYTHON` — low-rate record/quality policy; RTK never moves `map -> odom` in V1 |
| `agt_asensing_driver` / `asensing_driver` | C++ | External `agt_robot_bringup` includes `agt_asensing_driver/asensing.launch.py` if enabled | `/ins/navsatfix`, `/ins/imu`, `/ins/status`, compatibility `/ins/pose`, `/ins/velocity`, `/ins/odom`, raw frame; no TF broadcast | Physical INS producer; RTKManager is a separate metadata consumer | Serial poll every 10 ms; sensor path, not navigation authority | `MIGRATE_CPP` — existing native driver; preserve R1/R2 receive-time stamp semantics and no frame authority |
| `camera_gimbal_capability` | Python | Active C1 launch `autolabor_c1.launch.py` | Image `/cv_camera0/image_raw`, gimbal feedback/actions; serves `/camera_gimbal/acquire_view`, publishes health | Current active acquire-view action owner | Event-driven capture; capture correctness depends on measured stop before task invokes it | `KEEP_PYTHON` — multi-step capability/capture policy, no motion command or TF ownership |
| USB camera, pan/tilt serial driver, gimbal state manager | C++ upstream/local | Active C1 launch includes `usb_cam_node_exe`, `pantilt_camera_serial_node`, and optional manager launch | Image `/cv_camera0/image_raw`; pan/tilt feedback/action; manager publishes `/camera_gimbal/state`; fixed camera/gimbal transforms belong to robot description | One device driver each when C1 launch is started once | Camera configured 30 FPS; state manager 20 Hz; sensor/action path | `MIGRATE_CPP` — retain current sensor/serial implementations |
| `camera_capture_manager` | C++ | Separate legacy `camera_capture_manager.launch.py`; not part of active C1 launch | Alternative `/camera_gimbal/acquire_view` action server; reads image and state, controls pan/tilt; no TF | Would compete with Python `camera_gimbal_capability` if both launches run | Capture action path | `REMOVE_AFTER_PARITY` — duplicate action-server implementation; active C1 launch currently selects Python capability only |

## Tool-only and optional nodes

| Node / executable | Interfaces and owner | Classification |
|---|---|---|
| `agt_tf_authority_probe` | Read-only `/tf`/`/tf_static` authority inspection; standalone tool | `TOOL_ONLY` |
| `demo_preflight`, `wait_navigation_ready`, replay/bag benchmarks, relocalization sweep/capture/analyze, map validators/converters | CLI/test/analysis paths; no physical command output | `TOOL_ONLY` |
| `agt_sim_odometry_adapter`, Gazebo sensor plugins, simulation acceptance nodes | Simulation-only `/sim/ground_truth_odom` adapter can publish simulated `/agt/odometry/local` and `odom -> base_link`; must never coexist with physical LIO adapter | `TOOL_ONLY` |
| `agt_yaw_constraint` | Optional matched LIO/wheel yaw diagnostic publisher; explicitly never publishes TF/corrections; not started by field launch | `TOOL_ONLY` |
| Python map package/editor/catalog utilities | Offline/map-lifecycle code; do not own TF or motion command | `TOOL_ONLY` |

Python launch files, `run_field_stack.sh`, `run_bunker_hardware.sh`, `localization_initialization.sh`, and `start_mid360.sh` remain orchestration/entry points, not candidates for translation into runtime C++.

## Bunker V1 command, odometry, and TF baseline

```text
Nav2 controller /cmd_vel_nav
  -> Nav2 velocity_smoother (50 Hz), output /cmd_vel
  -> agt_cmd_vel_guard (50 Hz; localization/payload/mode/timeouts)
  -> /mux/cmd_vel
  -> Bunker driver subscription (internal /cmd_vel remapped to /mux/cmd_vel)

Bunker driver -> /wheel/odom, /bunker_status, /bunker_rc_state
Selected LIO frontend -> exactly one selected adapter -> /agt/odometry/local
Selected LIO adapter -> odom -> base_footprint
agt_localization_manager -> map -> odom
robot_state_publisher -> fixed base/sensor links on /tf_static
URDF fixed joint -> base_footprint -> base_link
```

| Contract | Current static owner | Expected publisher count | Notes |
|---|---|---:|---|
| `/mux/cmd_vel` (`geometry_msgs/Twist`) | `agt_cmd_vel_guard` | 1 | It is the sole intended software output to the physical base. HMI and Nav2 are inputs selected by guard policy, not direct driver publishers. |
| Bunker command subscriber | `bunker_base_node` | 1 | Driver's absolute `/cmd_vel` subscription is remapped to `/mux/cmd_vel`. Do not start a second base driver. |
| `/wheel/odom` (`nav_msgs/Odometry`) | Bunker driver | 1 | Diagnostic only in navigation; not a Nav2 localization/readiness/stop authority. |
| `/agt/odometry/local` (`nav_msgs/Odometry`) | One selected FAST-LIO or Batch-LIO adapter | 1 | Both adapters publish the same canonical topic; launcher validation selects exactly one. |
| `odom -> base_footprint` on `/tf` | Selected LIO adapter | 1 | The simulation adapter is an alternate simulation-only owner and must not run with a physical backend. |
| `map -> odom` on `/tf` | `agt_localization_manager` | 1 | Global relocalization and map tracker propose poses; neither broadcasts this transform. |
| `odom -> base_link` from Bunker driver | None in navigation | 0 | Bunker SDK driver can broadcast this edge by default; its field launch explicitly sets `publish_odom_tf=false`. |
| `base_footprint -> base_link` and fixed sensor links | `robot_state_publisher` via URDF | 1 per fixed child | Do not add a second static transform publisher for the same child. |
| `/agt/livox/points` secondary PointCloud2 | `livox_format_bridge` | 1 | Separate from raw `/livox/lidar` used by LIO. |
| `/agt/navigation/points_obstacles` | `agt_pointcloud_preprocessor` | 1 | Marking cloud only; clearing uses the unfiltered secondary cloud. |
| `/navigation/health` | `agt_navigation_supervisor` | 1 | Capability consumes this; it does not publish velocity. |
| `/navigate_to_pose` action | Nav2 BT navigator | 1 server | Capability is the user-facing policy/action gate; direct RViz Nav2 action remains a debugging path. |

The Bunker source calls `EnableCommandedMode()` before creating the velocity subscription. `EnableCommandedMode()` sends a `CONTROL_MODE_CAN` request. The local SDK exposes RC status and the vendor README says to keep the remote controller available, but source inspection does not prove physical RC-over-CAN priority or E-stop behavior for the fitted firmware. No safety arbitration was changed or physically tested. Treat remote priority as an external chassis safety invariant and require K01 supervised validation before field release.

## Launch and duplicate-owner audit

The table below records the P0 baseline before launch changes. P4 has since
introduced the owners and composition paths in the follow-up table.

| Resource | Intended active owner | Alternate path / collision risk |
|---|---|---|
| Whole physical hardware and robot description | `agt_robot_bringup/robot_hardware.launch.py`, reached by `scripts/run_bunker_hardware.sh` | `agt_robot_description/display.launch.py`, vendor device launches, direct Bunker launch, and driver-specific launches can be started separately; do not combine with the aggregate. |
| MID360 | Hardware launch's vendor `msg_MID360_launch.py` | `scripts/start_mid360.sh` and mapping framework's custom live launch are alternate physical sensor starts. Mapping and navigation hardware must not share a ROS domain/session. |
| Bunker | Hardware launch includes one `bunker_base.launch.py`; profile resolves `publish_odom_tf=false`, 50 Hz, `/wheel/odom`, `can0` 500 kbit/s | Direct `bunker_base.launch.py` or a second hardware launch can create duplicate subscribers/odom. `publish_odom_tf=true` would add a conflicting `odom -> base_link`. |
| LIO and localization | `agt_system_bringup/localization.launch.py` chooses FAST-LIO2 or Batch-LIO then includes bridge, relocalizer, one manager, optional tracker | Atomic adapter launches can be started manually beside the system launch; mapping launch has its own LIO. The launcher only guarantees backend exclusivity within one invocation. |
| Navigation nodes | `agt_system_bringup/navigation.launch.py` starts perception, supervisor, capability, guard and path tool, then includes Nav2 wrapper | The required P4 aggregator contract is not met: the navigation aggregator directly instantiates these `Node` objects rather than including their atomic launch files. Do not launch both the aggregator and an atomic package launch for the same node. |
| Pointcloud preprocessor | Currently direct `Node` in system navigation launch | Standalone `pointcloud_preprocessor.launch.py` is a duplicate path if combined with system navigation. |
| Command guard | Currently direct `Node` in system navigation launch | Standalone `cmd_vel_guard.launch.py` is a duplicate path if combined with system navigation. Duplicate guard outputs would violate the single command publisher rule. |
| Camera acquire action | Active C1 aggregate includes Python `camera_gimbal_capability` | Separate C++ `camera_capture_manager.launch.py` serves the same action name; never start it alongside the active C1 capability. |
| Mission/capture | `agt_mission_bringup/mission.launch.py` includes navigation and `mission_runtime_v4`; legacy mode additionally includes `agt_navigation_runtime/runtime.launch.py` and `rviz_patrol.launch.py` | Legacy and V4 task nodes can coexist and create competing task/action/UI surfaces. The legacy path stays until parity is recorded, then retire. |
| `run_field_stack.sh` | Preflight, then separate process groups for localization and navigation; expects `/robot_state_publisher`, `/bunker`, `/livox_lidar_publisher` to exist | It does not start physical hardware. `run_bunker_hardware.sh` is a separate owner. Inspection selects external `agt_mission_bringup` with legacy inspection enabled. |
| YHS | Current whole-robot config is `BLOCKED` and refuses to start with unknown gear | If later enabled, driver + `agt_yhs_adapter` must be the only YHS base/command adapter; physical protocol audit is still blocked in `YHS_BASE_PROTOCOL_AUDIT.md`. |

### P4 launch ownership after atomic split

| Responsibility | Atomic owner | Aggregation / single-owner rule |
|---|---|---|
| Robot description and fixed TF | `agt_robot_description/description.launch.py` | Platform `robot_hardware.launch.py` includes it once when enabled; legacy `display.launch.py` composes the description and RViz atomics. |
| MID360 driver | `agt_robot_bringup/lidar.launch.py` | `robot_hardware.launch.py` or `sensors.launch.py` includes the same atomic; the raw CustomMsg path stays direct to LIO. |
| Bunker physical driver | `agt_robot_bringup/base.launch.py` | Hardware aggregate includes one driver launch with `publish_odom_tf=false`; remote/manual arbitration remains in the chassis. |
| C1 and RTK drivers | `agt_robot_bringup/camera.launch.py`, `rtk.launch.py` | Optional only when enabled by the validated whole-robot config; `sensors.launch.py` composes sensor atomics only. |
| Navigation LIO | `agt_navigation_runtime/lio.launch.py` | A mutually exclusive branch includes either `fastlio_navigation_lio.launch.py` or `navigation_lio.launch.py`; localization includes the selector and no Nav2. |
| Local obstacle cloud | `agt_pointcloud_preprocessor/local_perception.launch.py` | `navigation.launch.py` includes one atomic; its PointCloud2 input remains separate from raw LIO. |
| Global relocalization / manager | Existing `global_relocalization.launch.py`, `localization_manager.launch.py` | `localization.launch.py` includes each once, along with the LIO and bridge atomics; only the manager broadcasts `map -> odom`. |
| Motion guard | `agt_base_runtime/motion_guard.launch.py` | One selected backend: C++ by default or Python rollback. Both publish only `/agt/base/cmd_vel`; navigation includes one guard launch. |
| Bunker command adapter | `agt_base_runtime/base_adapter.launch.py` | One runtime adapter forwards guarded commands to `/mux/cmd_vel`; the physical Bunker driver remains separately owned by the hardware atomic. |
| Nav2 map/planning stack | `agt_nav2_bringup/nav2.launch.py` | `navigation.launch.py` includes Nav2; this launch starts no physical driver. `navigation.launch.py` in the same package remains a compatibility include. |
| RViz UI / path editor / services | `agt_rviz_patrol/rviz.launch.py`, `path_tool.launch.py`; supervisor/capability package launches | Each is independently invocable and owns no TF; navigation aggregates them by include. |
| Convenience layers | platform `sensors.launch.py`; system `localization.launch.py`, `navigation.launch.py`, `system.launch.py`; `hardware.launch.py` remains a wrapper | Launch-source tests reject direct `Node` actions in these aggregators. Do not start an atomic and its aggregate together in one ROS domain. |

`test_launch_ownership_static.py` parses production launch sources and checks
the known singleton node declarations and aggregator boundaries. Profile-based
hardware selection remains fail-closed. YHS is now blocked in the adapter
registry and `robot_hardware_components.py` explicitly refuses YHS startup;
the physical YHS driver is not an enabled launch path.

P4 rollback is a reverse-order revert after stopping the runtime: first the
single P4 commit at the current `agt_navigation_v3` HEAD, then
`agt_robot_platform` commit
`079f8663629f3a11bf5464a68bffc406ff9264da`, then
`agt_robot_description` commit `a0e91268aed0f39cab6e1959b014f74fade2b750`.
The Python Motion Guard remains independently selectable with
`motion_guard_backend:=python` / `AGT_MOTION_GUARD_BACKEND=python`. The old
`hardware.launch.py`, platform `robot_hardware.launch.py`, description
`display.launch.py`, pointcloud `pointcloud_preprocessor.launch.py`, and Nav2
`navigation.launch.py` compatibility entries remain available.

P4 evidence is static launch/show-args and software test evidence only. No
atomic launch was run against a hardware driver; runtime graph ownership and
shutdown/restart remain unverified.

## Python/C++ migration rationale

| Candidate | Decision | Evidence needed before switch |
|---|---|---|
| Motion Guard | Migrate to C++; preserve current Python as the fallback until equivalent behavior is established | Freeze old behavior; unit parity for clamps, signed slew, timeouts, localization/payload interlocks, startup/recovery; 50 Hz mock/shadow comparison; exactly-one-publisher inspection; explicit rollback command/config. |
| FAST-LIO and Batch-LIO adapters | Migrate to C++ as two alternative implementations of the same adapter API | Independent adapter tests; per-backend config ownership; bag replay with identical `/agt/odometry/local`, twist and child TF; stale frame/timestamp rejection; ensure only one adapter broadcasts the child edge. |
| Localization Manager | P0: `UNKNOWN_NEEDS_AUDIT`; P5: `KEEP_PYTHON_THIS_RELEASE` | P5 audit records rates, APIs, TF timing, correction/recovery state, current test coverage, and the absence of a measured C++ need. No C++ candidate or false parity claim. See `localization_manager_cpp_feasibility.md`. |
| Supervisor, Capability, relocalization coordinator, mission, RTK manager, launch files | Keep Python | They are low-rate health/action/process/policy/record orchestration. No real-time evidence justifies translation. |
| LIO frontends, Bunker driver, sensor drivers, Livox bridge, point-cloud preprocessor, Nav2 | Keep current C++ implementations | They already use native C++; audit their contracts and do not rewrite upstream/vendor/field-tuned behavior as part of this task. |
| RViz editors, map tools, replay/benchmark tools, preflight and diagnostics | Keep as Python/YAML/CLI tools | They are not command or TF authorities and are outside the real-time core. |
| YHS adapter/profile | Blocked; no motion adapter decision | Close the vehicle-specific protocol/kinematics audit first; current ROS 2 message, README and manual disagree. |

## V1 invariants and open findings

1. **Raw LiDAR timing:** FAST-LIO2 and Batch-LIO subscribe to `/livox/lidar` CustomMsg directly. The C++ format bridge and C++ point-cloud preprocessor are on a separate `/agt/livox/points` branch. No generic filter is between the physical MID360 and LIO.
2. **TF singleton design:** manager is the only intended `map -> odom` publisher; selected LIO adapter is the only intended `odom -> base_footprint` publisher; Bunker driver field config disables `odom -> base_link`; robot model owns fixed edges.
3. **Wheel odom:** supervisor displays/checks arrival for diagnostic state, but `health_model.decide` does not use `base_alive` as a readiness gate. Navigation odometry and stop checks use `/agt/odometry/local`.
4. **RTK:** INS topics and `/agt/rtk/map_pose` are metadata/observation only; neither RTK driver nor RTK manager publishes TF.
5. **Initialization conflict found and closed in P0:** the source audit found `run_field_stack.sh` defaulted to `auto_then_manual` and exposed RViz `/initialpose`. The field launcher/helper now accept only `auto`, exit when global attempts fail, no longer launch the initialization view, and no longer accept a start-hint seed. System `localization.launch.py` hard-rejects non-auto modes and always selects `global_relocalization`. The manual-seed executable and RViz view remain standalone tools outside the V1 launch path. Automatic 3D-BBS/local-submap GICP is the only V1 production path.
6. **Hardware remote priority:** driver source requests CAN commanded mode; physical RC/E-stop priority is not established by this software audit and no physical validation was run.
7. **Atomic aggregator:** the P0 baseline directly instantiated runtime nodes. P4 now composes the atomic launch files documented above; static source tests and show-args pass, while graph-level owner inspection and shutdown/restart remain unrun.
8. **YHS:** see `YHS_BASE_PROTOCOL_AUDIT.md`; all physical YHS support stays `BLOCKED`.
9. **Ackermann:** no production Ackermann profile/conversion was found in this runtime inventory. Future software tests may use explicitly synthetic geometry only; never reuse test geometry as a robot profile.

## P5 Localization Manager disposition

The P0 inventory left the manager `UNKNOWN_NEEDS_AUDIT`. P5 completed the
feasibility audit and resolves its current disposition to
`KEEP_PYTHON_THIS_RELEASE`; the P0 record above is retained as historical
context. The detailed rates, API, math, states, evidence limits, decision, and
rollback path are in `localization_manager_cpp_feasibility.md`. The Python
manager remains the sole `map -> odom` owner. P5 did not run a rosbag replay,
runtime graph inspection, C++ parity test, or hardware test.

## P0 result, remediation, and commands

The complete source inventory and intended-owner evidence are recorded above. The audit surfaced a production `/initialpose` fallback contrary to the V1 contract. Before P0 exit, the field startup path was narrowed to automatic global relocalization only; the standalone experimental manual-seed node remains available outside the V1 path. Targeted package build and tests were run after that correction.

Exact build/test commands and results are recorded in `docs/acceptance/PLATFORM_RUNTIME_CPP_ACCEPTANCE.md`. P0 provides unit/mock and static-source evidence only. It does not claim runtime graph, rosbag replay, real-hardware parity, or field readiness; those remain unrun or blocked as recorded in the matrix.
