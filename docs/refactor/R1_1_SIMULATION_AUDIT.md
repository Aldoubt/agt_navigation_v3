# R1.1 Simulation Navigation Runtime Audit

Audit baseline: `3c9d083` (2026-09-10). This audit is read-only; no LIO,
global-localization, TF naming, or Nav2 core source was changed.

## Existing simulation capability

| Area | Existing implementation | Reuse value |
|---|---|---|
| Gazebo Classic | `agt_gazebo_sim/sim_world.launch.py` | Starts Gazebo, expands the Bunker/MID360 Xacro, robot_state_publisher and spawn. |
| Robot description | `agt_gazebo_sim/urdf/bunker_mid360_sim.urdf.xacro` | Canonical `base_link`, IMU and Livox fixed-frame tree. |
| Base simulation | `libgazebo_ros_planar_move.so` | Accepts `/mux/cmd_vel`, publishes `/sim/ground_truth_odom` at 50 Hz. |
| Nav2 wrapper | `agt_nav2_bringup/navigation.launch.py` | Existing map-server/lifecycle/Nav2 servers and AGT parameters. |
| Safety chain | `agt_base_control/cmd_vel_guard.py` | Existing `/cmd_vel` -> `/mux/cmd_vel` production contract. |
| Manual/automated goal support | RViz config and `agt_gazebo_sim/nav_goal_probe.py` | Existing goal action client and Gazebo truth acceptance check. |

No `ros2_control` controller or Gazebo diff-drive plugin is present. The current
planar-move plugin is therefore the simulation chassis interface; it must remain
fed from `/mux/cmd_vel`, not directly from `/cmd_vel`.

## Existing launch paths

- `rviz_field_demo.launch.py`: field navigation includes Batch-LIO, point-cloud
  bridge, BBS/GICP, localization manager, Nav2 and the guard.
- `navigation_debug.launch.py`: hardware-oriented explicit PCD/YAML debug path;
  camera and demo are optional.
- `agt_gazebo_sim/sim_world.launch.py`: Gazebo/description/spawn only.
- `agt_gazebo_sim/navigation_demo.launch.py`: starts Gazebo but then includes
  `rviz_field_demo`; it currently starts the real LIO/global-localization chain
  and is not an R1.1 simulation runtime gate.

## Gaps

1. The planar-move plugin publishes `sim_world -> base_link` odometry message
   data but has `publish_odom_tf=false`, `odometry_frame=sim_world`, and publishes
   no canonical `odom -> base_link` TF.
2. There is no `map -> odom` simulation localization source and no AMCL launch.
3. Nav2 consumes `/agt/odometry/local`; Gazebo currently only offers
   `/sim/ground_truth_odom`.
4. The existing simulation world/map provenance is not established as a matching
   Nav2 map package in the current launch.

## Minimal modification plan

Do not modify Gazebo, Nav2, LIO, BBS/GICP, or production localization. Add one
small simulation-only adapter in `agt_gazebo_sim` that repackages Gazebo truth
odometry as `/agt/odometry/local` and is the only simulation `odom -> base_link`
publisher. Add a static identity `map -> odom` only for the controlled Gazebo
map/world frame; this is a simulation localization substitute, not a replacement
for `agt_localization_manager` in a real system. Then add
`agt_system_bringup/simulation_debug.launch.py` to compose sim world, adapter,
Nav2, guard and RViz while defaulting camera/demo off.

Before implementation, validate that the chosen `navigation_map` has the same
origin/geometry as the Gazebo world. Without that evidence, planner/controller,
TF and velocity claims cannot be accepted.
