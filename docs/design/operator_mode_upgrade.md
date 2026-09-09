# Operator Mode Upgrade Plan and Task Log

## Scope

This document is the single execution record for the operator-mode upgrade.
Every completed substantial task must append a time-stamped entry to
`Task Log`; unfinished work remains in `Plan`.

The upgrade provides separate operator terminals: a persistent sensor session
with terminal preflight, a mapping save flow, and a navigation/HMI mode that
reuses the running ROS graph but never owns the sensor session.
MCU remote-control arbitration remains outside ROS and is not duplicated here.

## Plan

1. Create an `agt_operator_console` package with a declarative sensor profile
   and a terminal preflight command. It will report `READY`, `WARN`, or
   `BLOCKED` per sensor and per requested mode.
2. Create a sensor-session launch that starts configured MID360, CAN chassis,
   RTK, and camera/gimbal driver launches in a dedicated `sensors` terminal.
   Mapping/navigation terminals only inspect the ROS graph.
3. Create the interactive operator console. Default mode is sensors; mapping's first
   Ctrl-C prompts to continue, save, or discard. Save requests PGO output,
   waits for declared artifacts, then invokes the existing Map Package
   generation interface. The implementation must not forward this first signal
   to the mapping child.
4. Add navigation mode that validates/selects one active Map Package and starts
   the existing HMI field demo without starting sensor drivers again.
5. Add focused unit tests for profile parsing, readiness policy, and process
   command construction. Run package tests/build checks, then record results.
6. Keep HMI in terminal-selected-map compatibility mode for this stage. A
   separate future task will expose list/load/edit APIs in the external HMI.

## Acceptance Criteria

- The `sensors` terminal starts one persistent sensor session for all four
  driver groups and cleanly stops it only when that terminal exits. Mapping and
  navigation do not start or stop sensor drivers.
- Preflight observes fresh required ROS topics/actions and prints a concise
  terminal table. Mapping/navigation refuse to start when their required checks
  are blocked; RTK remains advisory.
- Mapping Ctrl-C presents an interactive choice before mapping processes stop.
- Saving uses `/pgo/save_maps`, requires explicit stable artifact paths, and
  publishes a non-active Map Package through the existing generator action or
  CLI-compatible pipeline.
- Navigation starts no driver launch and uses the active Map Package through
  `hmi_field_demo.launch.py`; automatic relocalization remains enabled.
- No MCU remote-control policy, Nav2 map hot reload, Terrain default switch, or
  external HMI source change is included in this stage.

## Task Log

### 2026-09-09 10:08:38 +0800 - Started

- Confirmed the operator decision: MCU owns remote-control priority and ROS
  will not add release-token logic.
- Confirmed existing in-workspace launches for Bunker CAN, Asensing RTK, and
  Autolabor C1 camera/gimbal. MID360 remains the separately maintained
  `livox_ros_driver2` dependency and is configured explicitly.
- Confirmed that ordinary `ros2 launch` Ctrl-C cannot provide the required save
  prompt because it propagates shutdown to child processes. A parent console
  must own the first SIGINT and manage mode child process groups.
- Started implementation of tasks 1 through 5.

### 2026-09-09 10:15:40 +0800 - Sensor Profile and Mapping Save Orchestration Complete

- Added `agt_operator_console` as the process-boundary owner. It keeps the
  sensor session separate from mapping/navigation children, so replacing a mode
  does not implicitly stop MID360, CAN, RTK, or camera/gimbal drivers.
- Added a versioned operator profile with deployed-driver launch commands and
  seven topic/action checks. MID360, IMU, and Bunker CAN feedback block mapping
  and navigation; RTK is visible but advisory; camera/gimbal become mandatory
  only for inspection mode.
- Added mapping-mode static MID360 IMU preflight. The mapping command explicitly
  binds existing FAST-LIO2/PGO launch arguments to `/livox/lidar` and
  `/livox/imu`.
- Added first-SIGINT handling for mapping. It prompts continue/save/discard
  before stopping the mapping process; save calls `/pgo/save_maps`, requires
  stable `map.pcd` and `poses.txt`, builds relocalization assets from that PCD,
  and invokes the existing package generator without activating the result.
- Added six unit tests for profile policy and command construction. Source-tree
  tests and Python compile checks passed. Colcon install/build verification is
  the next task.

### 2026-09-09 10:17:45 +0800 - Persistent Sensor Launch and Install Verification Complete

- Added `agt_system_bringup/sensor_session.launch.py`. It launches MID360,
  Bunker CAN, Asensing RTK, RTK status management, and the C1 camera/gimbal
  capability once for the entire operator session.
- Changed the operator profile to own that one persistent sensor launch. Its
  navigation and inspection commands pass `enable_rtk:=false` to the HMI field
  demo because RTK status management is already part of the persistent session.
- Built `agt_operator_console` and `agt_system_bringup` successfully with
  `colcon build --symlink-install`; verified the installed console entry point
  and installed profile. The six source-tree tests remain passing.
- Did not launch hardware. The driver packages resolve after the full workspace
  is built and sourced; preflight will still report missing or stale live ROS
  interfaces honestly on an unconnected robot.

### 2026-09-09 10:19:49 +0800 - Navigation Selection Compatibility Complete

- Added explicit `--map-id` and `--map-version` options to the operator console.
  Navigation uses the existing exact-version package selector before validating
  the active binding and launching HMI; omitted options retain the already
  selected active package.
- Confirmed `sensor_session.launch.py --show-args` resolves against the sourced
  workspace driver packages without starting hardware.
- Expanded focused unit coverage to seven tests. All tests, Python compile
  checks, and both affected package builds pass.
- Deferred external HMI's native map chooser and edit-session UI. The safe
  compatibility path is terminal exact-map selection followed by the existing
  HMI field launch; HMI API integration remains a separate, explicitly scoped
  task rather than a hidden filesystem-path workaround.

### 2026-09-09 10:20:21 +0800 - Final Verification Complete

- Rebuilt `agt_operator_console` and `agt_system_bringup` after the final
  profile and launch changes using `colcon build --symlink-install`.
- Re-ran the installed console help and the sensor-session launch argument
  expansion without starting hardware. Both succeeded.
- Re-ran all seven focused tests and `git diff --check`; both passed.

### 2026-09-09 10:21:35 +0800 - Mode Handoff Lifecycle Corrected

- Kept the operator console alive after a saved mapping run. It now preserves
  the sensor session while the operator completes edit/version publication, then
  can start navigation in the same process without relaunching hardware.
- Added a focused regression test for navigation reuse of an existing sensor
  session. All eight focused tests and the package rebuild passed afterward.

### 2026-09-09 10:29:28 +0800 - Separate Sensor and Mode Terminals Complete

- Superseded the same-console handoff in the preceding entry. The dedicated
  `operator_console sensors` terminal is now the only owner allowed to start
  and stop `sensor_session.launch.py`; it refreshes the terminal connection
  table until its own Ctrl-C.
- `operator_console mapping` and `operator_console navigation` now only
  preflight the existing ROS graph. Mapping Ctrl-C controls only the
  FAST-LIO2/PGO child and cannot signal the independently running sensor
  terminal; navigation similarly cannot relaunch or stop sensor drivers.
- Changed the default command mode to `sensors`, and added regression coverage
  that both mapping and navigation reject any accidental sensor-session start.
- Ran nine focused tests, rebuilt `agt_operator_console` and
  `agt_system_bringup` with `--symlink-install`, checked the installed console
  help, expanded `sensor_session.launch.py --show-args`, and ran
  `git diff --check`. No hardware drivers were launched during verification.

### 2026-09-09 10:32:31 +0800 - MID360 Driver Install Repair Complete

- Diagnosed the first field launch failure as stale `livox_ros_driver2` install
  contents: the source package had `launch_ROS2/msg_MID360_launch.py` and
  `config/MID360_config.json`, but neither existed below its install share path.
- Rebuilt `livox_ros_driver2` with `--symlink-install`. Confirmed both assets
  are now installed and that `sensor_session.launch.py --show-args` resolves
  after sourcing the workspace. No physical driver process was started during
  this repair.

### 2026-09-09 10:34:50 +0800 - MID360 Launch Directory Compatibility Complete

- Corrected the sensor-session integration after field execution exposed that
  Livox installs its ROS 2 sample under `launch_ROS2/`, not the conventional
  `launch/` directory. This was a composition-path error; a full workspace
  rebuild could not correct it.
- Updated only the Livox include to use `launch_ROS2/msg_MID360_launch.py`;
  all other driver includes retain the standard `launch/` convention.
- Rebuilt `agt_system_bringup`, compiled the launch source, and expanded the
  launch substitution with a ROS `LaunchContext`. The resulting path exists at
  the installed MID360 launch file. No physical driver process was started.

### 2026-09-09 10:36:49 +0800 - ASENSING Install Overlay Repair Complete

- Diagnosed the next field-launch failure as a stale symlink-install overlay:
  `install/agt_asensing_driver` pointed to a separate external worktree whose
  launch requires an uninstalled `rtk_indicator_node`.
- Rebuilt the workspace-owned `src/drivers/agt_ins_driver/agt_asensing_driver`.
  Its installed launch now resolves to the workspace source and starts only
  the deployed `asensing_node`; `agt_rtk_manager` remains the single RTK
  health/status publisher in the sensor session.
- Confirmed the package exposes only `asensing_node`, its launch resolves, and
  the complete sensor-session launch starts cleanly with every physical driver
  disabled. No hardware was opened during verification.

### 2026-09-09 10:38:59 +0800 - Persistent Bunker SocketCAN Diagnosis Complete

- Field preflight reached the Bunker check and correctly blocked mapping:
  `can0` exists but is `DOWN / STOPPED`, so the persistent Bunker driver cannot
  receive `/bunker_status` frames.
- Confirmed the repository already provides the correct system boundary:
  `agt-bunker-can.service` supervises `can0@500000` across boot and reconnects.
  It configures SocketCAN only; it sends no velocity command and does not alter
  MCU remote/manual priority. The service is not installed on this host yet.
- The required privileged one-time service installation is left for the field
  operator. ROS modes will remain correctly blocked until the physical CAN bus
  is up and produces chassis status frames.

### 2026-09-09 10:43:04 +0800 - C1 Camera and Gimbal Session Binding Complete

- Diagnosed the camera warning on the active sensor session: the C1 default
  `/dev/video4` does not exist on this host, so `usb_cam_node_exe` exited and
  `/cv_camera0/image_raw` had zero publishers. The attached Wasintek camera is
  available on `/dev/video0` and supports the configured 1920x1080 MJPEG mode.
- Found and fixed a launch-argument collision: Bunker and C1 both used the
  generic `port_name`, allowing Bunker's `can0` value to reach the gimbal. The
  C1 node consequently attempted to open `can0` as a serial device even though
  its action server was discoverable.
- Added isolated top-level bindings for `bunker_can_port`,
  `camera_device_path`, and `gimbal_port_name`. The C1 defaults now bind to
  `/dev/video0` and the stable C1 `/dev/serial/by-id` link. Rebuilt
  `agt_system_bringup`, compiled its launch source, and verified expanded child
  arguments without restarting any physical driver process.

### 2026-09-09 10:45:43 +0800 - Mapping IMU Preflight Entry Point Repaired

- Diagnosed the mapping-mode failure after all sensor checks passed:
  `mid360_imu_preflight.py` was included in the CMake install rule but lacked
  its executable permission bit, so ROS 2 omitted it from `ros2 run`.
- Restored executable metadata, rebuilt `agt_mapping_bringup`, and confirmed
  `ros2 pkg executables agt_mapping_bringup` now lists
  `mid360_imu_preflight.py`.
- Ran the read-only field preflight against `/livox/imu`: 1928 samples passed;
  median acceleration was `0.992 g` and mean gyro magnitude was `0.033 rad/s`.
  Both Batch-LIO and the pinned Fast-LIO2 mapping compatibility gates passed.

### 2026-09-09 10:50:33 +0800 - Mapping Sensor Topic Binding Repaired

- Diagnosed the first mapping run after the IMU check: raw MID360 topics were
  healthy, but FAST-LIO2 still subscribed to the obsolete
  `/agt/sensors/...` defaults and therefore published neither odometry nor
  `body_cloud`; the downstream OctoMap baseline correctly remained empty.
- Replaced the fragile text substitution in `mapping_mode.launch.py` with a
  structured YAML overlay. The requested `lidar_topic` and `imu_topic` now
  always replace the FAST-LIO2 config keys, while the PGO config is copied
  unchanged.
- Added the YAML runtime dependency, rebuilt `agt_mapping_bringup`, and ran a
  short isolated-domain launch. Its generated run-local YAML was verified to
  contain `/livox/lidar` and `/livox/imu`; no field sensor or active mapping
  process was interrupted by verification.

### 2026-09-09 14:34:00 +0800 - Map Edit, Selection, and Motion Handoff Complete

- Added a standalone HMI map-editor launch that validates one exact immutable
  Map Package, starts `agt_map_manager`, and opens Qt without starting Nav2.
  The HMI now creates a Map Manager edit session before enabling drawing; its
  publish command saves only the staging occupancy/topology assets and asks
  Map Manager to create a new immutable version. It never overwrites a
  released package and does not activate the new version implicitly.
- Navigation launch now starts Map Manager with the field runtime. The operator
  console gained `edit` mode and navigation now requires an explicit numbered
  Map Package selection unless exact `--map-id` and `--map-version` are given.
  Mapping, editing, and navigation remain independent of the sensor terminal.
- Separated Qt manual velocity output (`/agt/hmi/cmd_vel`) from Nav2
  (`/cmd_vel`). `agt_cmd_vel_guard` defaults to navigation control and accepts
  HMI motion only after an explicit `/agt/control/mode: manual` handoff. Every
  handoff hard-stops and clears cached commands; the MCU remote-control
  priority is unchanged. Normal HMI shutdown explicitly returns to navigation;
  an abnormal HMI loss remains a zero-motion manual-mode fail-safe.
- Repaired the HMI launcher environment: it now retains `ROS_DOMAIN_ID` and
  adds the workspace interface type-support library. It also initializes
  rclcpp before constructing lifecycle clients. This was verified by launching
  the editor offscreen in an isolated ROS domain: Map Manager, both HMI
  lifecycle clients, the editor frame, and all edit services were discovered.
- Rebuilt `agt_base_control`, `agt_operator_console`, `agt_system_bringup`, and
  `agt_robot_hmi`. Ran 36 focused tests, the isolated guard acceptance test,
  Python/launch and shell syntax checks, and a real StartMapEdit/CancelMapEdit
  service round trip against a temporary edit root. No physical sensors or
  nonzero chassis command were used during these tests.

### 2026-09-09 15:02:00 +0800 - HMI Map Edit Rendering Race Repaired

- Investigated the editor crash after publishing an immutable navigation-map
  edit. The edit session and `v001-edited` package were valid; the crash came
  from `DisplayOccMap` rebuilding a shared `QImage` and updating its
  `QGraphicsItem` from `QtConcurrent` while the GUI thread painted, edited,
  and saved the same image.
- Routed occupancy-map callbacks through the display object's queued Qt event
  loop and removed the worker-thread rendering path. `QImage`, `QPainter`,
  graphics-item geometry, and repaint requests now execute only on the GUI
  thread. Map Manager publication and active-map selection behavior are
  unchanged.
- Rebuilt and installed `agt_robot_hmi` successfully. The existing HMI test
  suite has no rendering-concurrency coverage; a no-chassis editor regression
  remains required: open a map, edit, publish, and keep the editor open while
  confirming no Qt painter or cross-thread warnings appear.

### 2026-09-09 15:43:34 +0800 - Persistent Robot Description Integration Complete

- Field navigation audit found that the persistent sensor session started the
  drivers but not `robot_state_publisher`. As a result, the live graph had no
  `base_link -> livox_frame` transform and automatic global relocalization
  rejected otherwise healthy MID360 clouds before registration.
- `sensor_session.launch.py` now includes the existing
  `tracked_chassis_description` Xacro launcher with RViz disabled. It is
  enabled by default and exposes an explicit calibration-file argument, so the
  calibrated fixed sensor TF chain is owned by the same long-lived terminal as
  the physical drivers rather than by mapping or navigation.
- Added the description package as an explicit runtime dependency. Rebuilt
  `tracked_chassis_description` and `agt_system_bringup` successfully with
  `--symlink-install`.
- In an isolated ROS domain, started `sensor_session.launch.py` with all
  physical drivers disabled and confirmed `robot_state_publisher` publishes
  `base_link <- livox_frame`: translation `(0.406, 0.000, 0.727)` m and pitch
  `13.000` degrees. No live driver, mapping, navigation, or chassis-control
  process was changed during this verification.

### 2026-09-09 17:03:06 +0800 - Map Package Candidate Database and Unknown-Cell Repair Complete

- Found the active `v002-edited` package had valid BBS voxel assets but no
  `polar_context.db`; the PGO source map still contained all 292 keyframe
  patches. This selected the expensive whole-map BBS/GICP fallback at runtime
  and prevented the candidate relocalization backend from being used.
- The mapping save path now builds `polar_context.db` and
  `polar_context.yaml` from the same PGO output after BBS assets are built.
  Package creation and package validation now require both files, so an
  incomplete relocalization package cannot be released or selected silently.
- Repaired the occupancy-map semantic contract: PGM unknown (`205`) has an
  occupancy probability of `50/255`, so the converter and HMI now use
  `free_thresh: 0.196`. Converter and package validation reject a map where
  that threshold would decode unknown cells as free.
- Created immutable `bunker_mid360_mapping_20260901_205036/v003-indexed`
  from the frozen `v002-edited` localization PCD and the matching formal PGO
  trajectory. It contains a rebuilt 292-entry candidate database, restored
  975482 incorrectly-freed unknown cells, retained 2576 explicit HMI
  obstacle-to-free edits, and copied the topology asset unchanged.
- Verified 22 focused Python tests, rebuilt `agt_map_converter`,
  `agt_map_manager`, `agt_operator_console`, and `agt_robot_hmi`, then ran
  installed `validate_nav_map` plus full package/lifecycle validation. All
  required checks passed. The active map remains `v002-edited` deliberately;
  select `v003-indexed` only after the current navigation stack has stopped
  and is being started again.
