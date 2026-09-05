# RViz field acceptance — current stage

This is the only acceptance path for the current design stage. HMI integration,
power-cycle mission resume and per-point camera optionality are deliberately out
of scope until this flow is repeatable on the Bunker.

## Acceptance behavior

```text
RViz goal queue P001 ... PN
  -> Nav2 NavigateToPose
  -> measured base stop
  -> fixed front-left / front-center / front-right C1 capture template
  -> record image_stamp + map pose + RTK metadata + actual gimbal joints
  -> next point
  -> RETURN_HOME
  -> measured stop
  -> standby
```

RTK is not a navigation or relocalization input in this stage.

## Hardware processes started separately

Start and verify these before the AGT software launch:

1. `livox_ros_driver2` for MID360 (`/livox/lidar` CustomMsg + `/livox/imu`).
2. Bunker v1 CAN driver; it consumes `/mux/cmd_vel` and must not publish an odom TF.
3. `robot_state_publisher` / URDF static transforms, including the real tilted MID360 mount.
4. Autolabor C1 driver/capability with `/camera_gimbal/acquire_view` available.
5. `agt_ins_driver` when RTK metadata recording is required. Missing/poor RTK must not move the robot map frame.

## P0 IMU check before mapping or navigation

Run with the robot fully stationary:

```bash
ros2 run agt_mapping_bringup mid360_imu_preflight.py --ros-args \
  -p duration_sec:=10.0
```

For Batch-LIO:

- raw acceleration norm near `1.0` -> use `mapping.acc_norm=1.0`;
- raw acceleration norm near `9.81` -> use `mapping.acc_norm=9.81`;
- neither -> stop and inspect driver units/data.

The tool now reports both `batch_lio_unit_ready` and
`pinned_fastlio2_mapping_unit_ready` so the two consumers are not confused.

Before using the pinned mapping front-end, run the stricter check:

```bash
ros2 run agt_mapping_bringup mid360_imu_preflight.py --ros-args \
  -p duration_sec:=10.0 \
  -p require_fastlio2_mapping_compatible:=true
```

The currently pinned `robotics-laboratory/fast-lio2` source multiplies incoming
Livox linear acceleration by `10.0`. Therefore the unmodified mapping baseline is
only accepted when raw stationary `/livox/imu` acceleration norm is approximately
`1.0`. If the driver reports approximately `9.81 m/s^2`, fix/replace that upstream
scaling before mapping; do not compensate by guessing noise parameters.

## Build smoke on the target Humble workspace

After all pinned algorithm and hardware-interface dependencies are present:

```bash
cd ~/ros2_ws
bash src/agt_navigation_v3/scripts/field_build_smoke.sh
```

Expected final line:

```text
FIELD BUILD SMOKE PASS
```

This performs the full current software-chain build, selected pure-software tests,
and launch-description parsing. It is not a hardware acceptance result.

## Mapping

The AGT wrapper now launches exactly one FAST-LIO2 node and one optional PGO node.
It intentionally does not include the upstream `lio_launch.py` and `pgo_launch.py`
together because the upstream PGO launch starts another FAST-LIO2 instance.

```bash
ros2 launch agt_mapping_bringup mapping_mode.launch.py
```

Explicit versioned configs used by default:

```text
agt_mapping_bringup/config/fastlio2_mid360.yaml
agt_mapping_bringup/config/pgo_mid360.yaml
```

After mapping, save/refine the final PCD, then run `agt_map_converter` and optionally
`build_relocalization_assets` as documented in the main README.

## Navigation / field demo startup

The external hardware processes above remain running. Start the complete AGT
software chain with matching 2D and 3D maps:

```bash
ros2 launch agt_system_bringup rviz_field_demo.launch.py \
  map:=/data/site_A/navigation/map.yaml \
  global_map:=/data/site_A/global_map.pcd \
  relocalization_assets:=/data/site_A/relocalization \
  map_id:=site_A_v1
```

For the field-demo launch, `auto_relocalize:=true` is now the default and the
live relocalization query is `/agt/livox/points` from the secondary
CustomMsg->PointCloud2 bridge. Use `auto_relocalize:=false` only when deliberately
debugging the manual `/agt/localization/relocalize` service path.

Scope note for the first vehicle run: automatic **startup** relocalization is in
scope and must work. Automatic mission recovery after an injected runtime `LOST`
event is not yet an accepted feature. The safety gate will stop the chassis; for
this first test, keep it stopped and use the manual relocalization service below
before deciding whether to resume. Do not use the first field run to validate
automatic LOST->relocalize->mission-resume behavior.

If BBS assets have not been generated yet, omit `relocalization_assets`; the native
backend keeps the slower PCD-build fallback for initial debugging.

This launch starts:

```text
Batch-LIO
agt_batch_lio_adapter
Livox CustomMsg -> PointCloud2 secondary bridge
local obstacle preprocessor
Polar Context + candidate 3D-BBS + small_gicp relocalization
Localization Manager
optional RTK manager
Nav2
50 Hz Bunker cmd_vel guard
inspection mission runtime
RViz patrol queue
RViz
```

It does **not** start MID360, Bunker, URDF or C1 hardware drivers and does **not**
automatically decide readiness.

## Manual relocalization

Keep the robot stationary, verify `/agt/odometry/local` and `/agt/livox/points`, then:

```bash
ros2 service call /agt/localization/relocalize std_srvs/srv/Trigger "{}"
```

Verify in RViz that `map -> odom -> base_link` is plausible and stable before sending
a navigation point. Do not continue if the BBS/GICP solution is visually wrong.

## Manual preflight

Run preflight after global relocalization. By default it now requires
`LocalizationStatus.STATE_LOCALIZED`, fresh local odometry and a valid global
correction in addition to Nav2/C1/obstacle cloud/TF checks:

```bash
ros2 run agt_navigation_runtime demo_preflight
```

For the normal V1 acceptance RTK is not required. Only add `require_rtk:=true` when
specifically checking the RTK record path.

## Pre-field runtime safety fixes — PASSED on 2026-09-05

The two runtime blockers that were intentionally deferred during relocalization
development are now implemented and bench-accepted.

### 1. `cmd_vel_guard` localization safety gate

`agt_base_control/cmd_vel_guard.py` now subscribes to
`/agt/localization/status` and is fail-closed by default.

Motion is allowed only when all of the following are true:

```text
LocalizationStatus == LOCALIZED
local_odom_fresh == true
global_correction_valid == true
LocalizationStatus age <= 0.75 s
fresh upstream cmd_vel received while the gate is open
```

`BOOT`, `WAIT_*`, `DEGRADED`, `LOST`, `RELOCALIZING`, invalid correction,
missing status or stale status all force `/mux/cmd_vel` to zero. A localization
fault also clears the cached upstream command; commands received while the gate
is closed are discarded and cannot be replayed when localization recovers.

Topic-level acceptance result:

```text
missing LocalizationStatus       -> max output 0.0
LOCALIZED + valid correction     -> 0.4 m/s command passed
LOST                             -> max output 0.0
reopen without a fresh command   -> max output 0.0
status stale > 0.75 s            -> max output 0.0
GUARD_ACCEPTANCE_PASS
```

### 2. `mission_runtime` measured-stop gate

`agt_navigation_runtime/mission_runtime.py` no longer trusts odometry twist by
itself. Capture readiness requires both:

- reported twist below the configured thresholds; and
- pose-derived planar translation/yaw rate below independent thresholds.

Current initial thresholds:

```text
twist linear       <= 0.03 m/s
twist angular      <= 0.05 rad/s
pose-delta linear  <= 0.04 m/s
pose-delta yaw     <= 0.06 rad/s
continuous hold    >= 0.8 s
odom freshness     <= 0.5 s
```

Bench acceptance explicitly reproduced the Batch-LIO failure mode:

```text
twist = 0, pose continuously moving
  -> stationary check timed out / rejected

small stationary pose jitter
  -> accepted after hold time
  -> pose_delta ~= 0.010 m/s, 0.010 rad/s

MISSION_STATIONARY_ACCEPTANCE_PASS
```

The C1 `camera_gimbal_interfaces` package is now also present and compiled in
the same `ros2_ws` overlay through the repository's existing
`drivers/Autolabor-C1-ROS2` dependency layout, so `mission_runtime` no longer
depends on manually sourcing a second workspace just to import the action type.

These two fixes remove the known software blockers for the first controlled
vehicle patrol test. They do **not** replace real Bunker watchdog, emergency-stop
or physical field-safety procedures.

## First controlled vehicle test order

For the first run, keep the route short and verify each gate before expanding it:

```text
1. Start MID360 / Bunker / URDF / C1 hardware processes.
2. Start rviz_field_demo; confirm /mux/cmd_vel stays zero before LOCALIZED.
3. Wait for automatic relocalization; visually verify map/scan alignment.
4. Run demo_preflight and require PASS.
5. Send one nearby RViz inspection point.
6. Verify Nav2 success -> measured stop -> 3 captures.
7. Verify RETURN_HOME -> measured stop -> standby.
8. Inspect captures.csv/jsonl and images before attempting three points.
```

Do not skip step 2: it is the real-vehicle confirmation that the localization
safety gate is actually in the Bunker command path.

## Acceptance sequence

Start with one point:

```text
P001 -> measured stop -> 3 images -> RETURN_HOME -> measured stop
```

Then three points:

```text
P001 -> 3 images
P002 -> 3 images
P003 -> 3 images
RETURN_HOME -> standby
```

Validate the record directory:

```bash
ros2 run agt_navigation_runtime validate_records /path/to/mission_dir --expected-points 3
ros2 run agt_navigation_runtime generate_demo_report /path/to/mission_dir
```

Only after repeated three-point runs are stable should the project add HMI mission
dispatch, power-cycle continuation or camera-enable/disable policy.
