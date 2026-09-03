# agt_navigation_v3

ROS 2 Humble navigation and inspection integration stack for the AGT tracked robot.

Target platform: Bunker v1 tracked base + tilted MID360 + FAST-LIO2 + no-initial-pose 3D global relocalization + Nav2 + auxiliary RTK/INS + Qt HMI + Autolabor C1 camera-gimbal.

> Active integration branch: `runtime-v1`
>
> Code is being landed continuously. Nothing is considered hardware-ready until it passes ROS 2 Humble `colcon build`, rosbag regression and real Bunker acceptance tests.

## Design principles

### 1. LiDAR localization is primary

The system must remain navigable under trees where RTK can degrade or disappear. FAST-LIO2 owns high-rate local motion estimation. Global LiDAR relocalization owns global anchoring. RTK/INS is an auxiliary global reference, health signal and future optimization constraint, not a hard navigation dependency.

### 2. Exactly one owner per TF edge

Target TF ownership:

```text
map
 |
 +-- odom                 localization manager / global relocalization
      |
      +-- base_link       FAST-LIO2 continuous local odometry
           |
           +-- physical static frames from URDF
```

Bunker wheel odometry may support velocity/control diagnostics, but `agt_bunker_base` must keep `publish_odom_tf=false`. It must not compete with FAST-LIO2 for `odom -> base_*`.

The measured MID360 tilt stays in TF. Do not level the incoming point cloud in software.

### 3. Preserve LiDAR timing for FAST-LIO2

Do not place generic voxel/downsample processing in front of FAST-LIO2 unless all per-point timing required for deskew is proven to survive.

Preferred split:

```text
MID360 raw/custom message + IMU
        |
        +----------------------> FAST-LIO2 time-preserving input
        |
        +--> self-filter/downsample --> relocalization / Nav2 obstacle cloud / debug
```

Mapping and live registration geometry must be compatible, but that does not require destructive preprocessing before the LIO frontend.

### 4. Map package is the product asset

A map is more than `map.pgm`:

```text
maps/<map_id>/
├── metadata.yaml
├── localization/
│   ├── global_map.pcd
│   ├── submaps/
│   └── scan_context.db
└── navigation/
    ├── map.yaml
    ├── map.pgm
    ├── height.pgm
    ├── slope.pgm
    └── obstacle.pgm
```

Inspection tasks and geographic anchors must be bound to `map_id` + `map_version`, so stale task data cannot silently run against another map.

### 5. HMI is a client, not the robot brain

`agt_robot_hmi` stays as the product UI. It displays robot/map/task state and edits inspection tasks, but it does not directly control Nav2, CAN, camera serial protocols or localization internals.

Current compatibility endpoints remain supported:

```text
/agt/task/request
/agt/task/start
/agt/task/pause
/agt/task/cancel
/agt/task/status
```

`agt_navigation_runtime` bridges these placeholders while typed contracts in `agt_robot_interfaces` are adopted.

### 6. Camera-gimbal is a capability

The runtime consumes the frozen C1 public action:

```text
/camera_gimbal/acquire_view
```

Success means the gimbal really reached and stabilized and a new image was captured afterward. `image_stamp` is the synchronization anchor for map pose, RTK, actual gimbal angles and image metadata.

### 7. 50 Hz is an end-to-end control requirement

```text
Nav2 controller @ 50 Hz
       |
velocity_smoother @ 50 Hz
       |
    /cmd_vel
       |
agt_cmd_vel_guard @ 50 Hz
 clamp + slew limit + stale timeout
       |
 /mux/cmd_vel
       |
agt_bunker_base -> CAN
```

The controller plugin is replaceable. `runtime-v1` uses Regulated Pure Pursuit as a baseline, not a frozen final selection. RPP / MPPI / DWB or other candidates must be selected using Bunker field benchmarks.

## Repository layout

```text
agt_navigation_v3/
├── config/                         design-stage common configuration
├── docs/                           architecture and acceptance documents
└── src/
    ├── agt_robot_interfaces/       shared typed ROS interfaces
    ├── agt_navigation_runtime/     HMI / Nav2 / C1 inspection runtime
    ├── agt_rtk_manager/            RTK/INS quality gate
    ├── agt_nav2_bringup/           Nav2 without AMCL ownership conflict
    ├── agt_base_control/           Bunker 50 Hz cmd_vel guard
    └── agt_system_bringup/         staged top-level launch composition
```

## Runtime architecture

```text
                         Qt HMI
                           |
                    /agt/task/*
                           |
                  agt_navigation_runtime
                    /             \
                   /               \
          NavigateToPose        AcquireView
                |                   |
              Nav2            Autolabor C1
                |
             /cmd_vel
                |
        agt_cmd_vel_guard
                |
          /mux/cmd_vel
                |
        agt_bunker_base / CAN

MID360 + IMU -> FAST-LIO2 -> odom -> base_link
       |              ^
       |              |
       +-> global relocalization -> map -> odom

agt_ins_driver -> agt_rtk_manager -> /agt/rtk/status
       |
       +---------------------------> mission capture metadata
```

## Build

Target environment:

- Ubuntu 22.04
- ROS 2 Humble
- Nav2 Humble
- external repos/workspaces for MID360, FAST-LIO2, Bunker, INS and C1

Typical workspace:

```text
~/agt_ws/src/
├── agt_navigation_v3/
├── agt_ins_driver/
├── agt_bunker_base/
├── agt_chassis_description/
└── Autolabor-C1-ROS2/
```

Build:

```bash
source /opt/ros/humble/setup.bash
cd ~/agt_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Passing Python syntax checks is not the merge gate. The branch still requires a real Humble build and runtime tests.

## Usage

### Staged system bringup

The top-level launch defaults to a safe integration stage: RTK manager and base command guard enabled; Nav2 and mission runtime disabled until localization prerequisites are ready.

```bash
ros2 launch agt_system_bringup system.launch.py
```

Enable Nav2 after a valid map and localization TF chain are available:

```bash
ros2 launch agt_system_bringup system.launch.py \
  enable_nav2:=true \
  map:=/absolute/path/to/maps/site_A/navigation/map.yaml
```

Enable the inspection runtime only after Nav2 and C1 `AcquireView` are ready:

```bash
ros2 launch agt_system_bringup system.launch.py \
  enable_nav2:=true \
  enable_runtime:=true \
  map:=/absolute/path/to/maps/site_A/navigation/map.yaml
```

The current top-level launch composes implemented packages only. A readiness state machine that automatically gates `sensor -> map -> localization -> Nav2 -> mission` is still pending.

### RTK quality manager

Start `agt_ins_driver`, then:

```bash
ros2 launch agt_rtk_manager rtk_manager.launch.py
ros2 topic echo /agt/rtk/status
```

The manager consumes `/ins/navsatfix` and `/ins/status`. Freshness is judged from ROS receive time so stale FIX data cannot remain falsely usable after a data outage.

Default quality thresholds are initial test values, not final field-calibrated values. `map_origin.example.yaml` is the intended geographic-anchor schema; in `runtime-v1` it is metadata only. Earth/ENU/map conversion and RTK correction injection are not implemented yet.

### Nav2 baseline

Nav2 intentionally does **not** start AMCL because global LiDAR localization is expected to own `map -> odom`.

```bash
ros2 launch agt_nav2_bringup navigation.launch.py \
  map:=/absolute/path/to/maps/site_A/navigation/map.yaml
```

Prerequisites before sending a goal:

- valid `map -> odom`
- valid continuous local TF from FAST-LIO2
- `/wheel/odom` available for velocity/control reference with `publish_odom_tf=false`
- `/agt/navigation/points_obstacles` PointCloud2 available for the local 3D voxel costmap

The Bunker footprint, controller values and acceleration limits are conservative starting points and must be tuned on the real chassis.

### Bunker 50 Hz guard

```bash
ros2 launch agt_base_control cmd_vel_guard.launch.py
```

The guard receives the Humble Nav2 final `/cmd_vel`, applies tracked-base velocity/slew limits, refreshes at 50 Hz and publishes `/mux/cmd_vel`. If upstream commands become stale, the target becomes zero.

This is a software command guard, not a safety-rated emergency stop. Physical E-stop and hardware safety remain independent.

### Inspection runtime

```bash
ros2 launch agt_navigation_runtime runtime.launch.py
```

Example mission:

```text
src/agt_navigation_runtime/config/mission_example.yaml
```

Execution chain:

```text
NAVIGATE
 -> ARRIVAL
 -> BASE SETTLE
 -> GIMBAL MOVE / STABLE
 -> NEW IMAGE CAPTURE
 -> image_stamp TF lookup
 -> associate RTK
 -> write record
 -> next view / next point
```

Capture records contain mission/map/point/view IDs, image path/timestamp, map pose, RTK and actual gimbal angles.

## Integration progress

| Area | Status | Notes |
| --- | --- | --- |
| Architecture / TF policy | designed | must still be verified with the selected FAST-LIO2 ROS 2 port |
| Shared robot interfaces | implemented | mission, inspection and RTK status messages/actions |
| HMI compatibility bridge | implemented | keeps existing `/agt/task/*` boundary |
| Inspection mission runtime | implemented | Nav2 + C1 orchestration and capture records |
| C1 integration | integrated by public Action | C1 hardware acceptance stays in its own repo |
| RTK quality manager | implemented | quality/freshness gate; no navigation ownership |
| Geographic map anchor | schema only | ENU/earth/map conversion pending |
| Nav2 bringup | baseline implemented | no AMCL; RPP baseline; Humble build + field tuning pending |
| Bunker 50 Hz command path | implemented | `/cmd_vel -> /mux/cmd_vel`; CAN timing acceptance pending |
| Wheel odometry policy | integrated | velocity/control reference only; no odom TF ownership |
| Staged system bringup | implemented | manual enable gates; automatic readiness state machine pending |
| Point-cloud preprocessing node | not implemented yet | must preserve FAST-LIO2 timing path |
| Global relocalization backend | design/adapter stage | Scan Context + coarse registration + GICP still to land |
| Localization Manager | not implemented yet | single `map -> odom` owner and runtime LOST recovery pending |
| Map Manager V1 | design stage | discovery/version/atomic switch pending |
| Terrain converter | design stage | elevation/slope/roughness/traversability pending |
| Power-cycle mission resume | next | checkpoint + relocalize + map-version validation + continue |
| Hardware acceptance | not yet passed | requires target robot and Humble environment |

## Immediate landing order

1. Timing-safe point-cloud split/preprocessing adapter.
2. Localization Manager with `LOCALIZED -> DEGRADED -> LOST -> RELOCALIZING` runtime handling.
3. Global relocalization backend adapters and MID360 rosbag benchmark.
4. Map Manager V1 and map-package version validation.
5. Terrain converter: elevation + slope + roughness + traversability -> Nav2 representation.
6. Automatic system readiness state machine.
7. Power-cycle mission checkpoint/resume.
8. Bunker controller benchmark and parameter freeze.

## External repositories

- `Aldoubt/agt_ins_driver` — ASENSING RTK/INS abstraction
- `Aldoubt/agt_robot_hmi` — Qt operator HMI
- `Aldoubt/Autolabor-C1-ROS2` — camera-gimbal capability
- `Aldoubt/agt_bunker_base` — Bunker ROS 2 CAN driver
- `Aldoubt/agt_chassis_description` — physical URDF / static TF source of truth

Dependency branches are intentionally not frozen yet. Freeze exact commits only after the end-to-end demo and acceptance datasets are stable.
