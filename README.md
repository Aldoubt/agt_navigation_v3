# agt_navigation_v3

ROS 2 Humble navigation and inspection integration stack for the AGT tracked robot.

The target system is an outdoor Bunker v1 robot using a tilted MID360, FAST-LIO2 continuous LiDAR-inertial odometry, no-initial-pose 3D global relocalization, Nav2 navigation, auxiliary RTK/INS, Qt HMI inspection-task editing and Autolabor C1 camera-gimbal acquisition.

> Current integration branch: `runtime-v1`
>
> Code in this branch is being landed continuously. It is not considered hardware-ready until it passes ROS 2 Humble `colcon build`, rosbag regression and real Bunker acceptance tests.

## Design principles

### 1. LiDAR localization is primary

The robot must remain navigable under trees where RTK can degrade or disappear. FAST-LIO2 owns high-rate local motion estimation. Global LiDAR relocalization owns global anchoring. RTK/INS is an auxiliary global reference, health signal and future optimization constraint; it must not become a hard navigation dependency.

### 2. Exactly one owner per TF edge

Target TF ownership:

```text
map
 |
 +-- odom                 global localization / localization manager
      |
      +-- base_link       FAST-LIO2 continuous local odometry
           |
           +-- physical sensor/static frames from URDF
```

Bunker wheel odometry may be used for velocity/control diagnostics, but the Bunker driver is configured with `publish_odom_tf=false`. It must not compete with FAST-LIO2 for `odom -> base_*`.

The measured MID360 tilt stays in TF. Do not level the point cloud in software.

### 3. Preserve LiDAR timing for FAST-LIO2

Do not place a generic voxel/downsample filter in front of FAST-LIO2 unless every per-point timing field required for deskew is proven to be preserved.

Preferred split:

```text
MID360 raw/custom message + IMU
        |
        +----------------------> FAST-LIO2 time-preserving input
        |
        +--> self-filter/downsample --> relocalization / Nav2 obstacle cloud / debug
```

Map/live registration geometry must be compatible, but that does not require destructive preprocessing before the LIO frontend.

### 4. Map package is the product asset

A map is not only `map.pgm`. The intended versioned package contains at least:

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

Future inspection points and geographic anchors must also be tied to `map_id` + `map_version` so stale tasks cannot silently run against a different map.

### 5. HMI is a client, not the robot brain

`agt_robot_hmi` remains the product UI. It displays state and edits tasks, but does not directly control Nav2, CAN, camera serial protocols or localization internals.

Current compatibility endpoints remain supported:

```text
/agt/task/request
/agt/task/start
/agt/task/pause
/agt/task/cancel
/agt/task/status
```

They are bridged inside `agt_navigation_runtime` while the stable typed interfaces in `agt_robot_interfaces` are adopted.

### 6. Camera-gimbal is a capability

The navigation runtime uses the already-frozen C1 public action:

```text
/camera_gimbal/acquire_view
```

A successful capture means the gimbal actually reached and stabilized, and a new image was acquired after stabilization. The image timestamp is the synchronization anchor for robot pose / RTK / gimbal / image metadata.

### 7. 50 Hz is an end-to-end control requirement

The tracked base control chain is intentionally explicit:

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

The controller plugin remains replaceable. `runtime-v1` uses Regulated Pure Pursuit as a baseline, not as a frozen final choice. RPP / MPPI / DWB or other candidates must be selected by Bunker field benchmarks.

## Repository layout

```text
agt_navigation_v3/
├── config/                         design-stage common configuration
├── docs/                           architecture and acceptance documents
└── src/
    ├── agt_robot_interfaces/       shared typed ROS interfaces
    ├── agt_navigation_runtime/     mission / HMI / Nav2 / C1 integration runtime
    ├── agt_rtk_manager/            RTK/INS quality gate
    ├── agt_nav2_bringup/           Nav2 bringup without AMCL ownership conflict
    └── agt_base_control/           Bunker 50 Hz cmd_vel guard
```

## Current runtime integration

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
- external workspaces/repos for MID360, FAST-LIO2, Bunker, INS and C1 camera-gimbal

Example workspace layout:

```text
~/agt_ws/src/
├── agt_navigation_v3/
├── agt_ins_driver/
├── agt_bunker_base/
├── agt_chassis_description/
└── Autolabor-C1-ROS2/   # or source/install this capability in an underlay
```

Build:

```bash
source /opt/ros/humble/setup.bash
cd ~/agt_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Do not treat the repository as validated merely because Python files parse. The merge gate is a real Humble build plus runtime tests.

## Usage

### RTK quality manager

Start the ASENSING driver first, then:

```bash
ros2 launch agt_rtk_manager rtk_manager.launch.py
ros2 topic echo /agt/rtk/status
```

The manager consumes `/ins/navsatfix` and `/ins/status`. Freshness is judged using ROS receive time so stale FIX data cannot remain falsely usable after a serial/GNSS outage.

The default thresholds in `agt_rtk_manager/config/rtk_manager.yaml` are initial test values, not final field-calibrated numbers.

`map_origin.example.yaml` defines the intended versioned geographic-anchor asset. In `runtime-v1` it is metadata only: earth/ENU/map TF publication and RTK correction injection are intentionally not implemented yet.

### Nav2 baseline

The Nav2 launch intentionally does **not** start AMCL because global LiDAR localization is expected to own `map -> odom`.

```bash
ros2 launch agt_nav2_bringup navigation.launch.py \
  map:=/absolute/path/to/maps/site_A/navigation/map.yaml
```

Important prerequisites before sending a goal:

- valid `map -> odom` from localization manager
- valid continuous local TF from FAST-LIO2
- `/wheel/odom` available for velocity/control reference, with Bunker `publish_odom_tf=false`
- `/agt/navigation/points_obstacles` PointCloud2 available for the local 3D voxel costmap

The current Bunker footprint and velocity/acceleration limits are conservative starting values and must be measured on the actual chassis.

### Bunker 50 Hz command guard

```bash
ros2 launch agt_base_control cmd_vel_guard.launch.py
```

It receives `/cmd_vel`, applies tracked-base velocity limits and slew limits, refreshes commands at 50 Hz, and publishes `/mux/cmd_vel`. If upstream commands become stale, the target automatically becomes zero.

This is a software command guard, not a safety-rated emergency stop. Physical E-stop and hardware safety remain independent.

### Inspection runtime

```bash
ros2 launch agt_navigation_runtime runtime.launch.py
```

Example mission:

```text
src/agt_navigation_runtime/config/mission_example.yaml
```

The runtime executes:

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

Capture records are written beneath the configured record root and include mission/map/point/view IDs, image path/timestamp, map pose, RTK and actual gimbal angles.

## Integration progress

| Area | Status | Notes |
| --- | --- | --- |
| Architecture / TF policy | designed | docs present; runtime ownership must still be verified with real FAST-LIO2 port |
| Shared robot interfaces | implemented in `runtime-v1` | mission, inspection and RTK status messages/actions |
| HMI compatibility bridge | implemented in `runtime-v1` | keeps existing `/agt/task/*` placeholder boundary |
| Inspection mission runtime | implemented in `runtime-v1` | Nav2 + C1 orchestration and capture records |
| C1 camera-gimbal integration | integrated by public Action | C1 hardware acceptance remains in its own repo |
| RTK quality manager | implemented in `runtime-v1` | quality/freshness gate; no direct navigation ownership |
| Geographic map anchor | schema only | ENU/earth/map conversion still to implement and validate |
| Nav2 bringup | baseline implemented | no AMCL; RPP baseline; needs Humble build + field tuning |
| Bunker 50 Hz command path | implemented in `runtime-v1` | guard outputs `/mux/cmd_vel`; real CAN timing acceptance pending |
| Wheel odometry policy | integrated | `/wheel/odom` can support control; no odom TF ownership |
| Point-cloud preprocessing node | not implemented yet | must split LIO timing-preserving path from obstacle/relocalization path |
| Global relocalization backend | design/adapter stage | Scan Context + coarse registration + GICP backend still to land |
| Localization manager | not implemented yet | single `map -> odom` owner and runtime LOST recovery still to land |
| Map Manager V1 | design stage | map discovery/version/atomic switch still to land |
| Terrain converter | design stage | elevation/slope/roughness/traversability generation still to land |
| System bringup / readiness state machine | next | ordered sensor -> map -> localization -> Nav2 -> mission startup |
| Power-cycle mission resume | next | checkpoint + relocalize + map-version validation + continue |
| Hardware acceptance | not yet passed | requires target robot and ROS 2 Humble environment |

## Immediate landing order

1. Point-cloud split and timing-safe preprocessing adapter.
2. Localization Manager with runtime `LOCALIZED -> DEGRADED -> LOST -> RELOCALIZING` handling.
3. Global relocalization backend adapters and MID360 rosbag benchmark.
4. Map Manager V1 and map-package schema/version validation.
5. Terrain converter: elevation + slope + roughness + traversability -> Nav2 representation.
6. System bringup/readiness manager.
7. Power-cycle mission checkpoint/resume.
8. Bunker controller benchmark and parameter freeze.

## External repositories

- `Aldoubt/agt_ins_driver` — ASENSING RTK/INS abstraction
- `Aldoubt/agt_robot_hmi` — Qt operator HMI
- `Aldoubt/Autolabor-C1-ROS2` — camera-gimbal capability
- `Aldoubt/agt_bunker_base` — Bunker ROS 2 CAN driver
- `Aldoubt/agt_chassis_description` — physical robot URDF / static TF source of truth

Dependency branches are intentionally not frozen yet. Freeze exact commits only after the end-to-end demo and acceptance datasets are stable.
