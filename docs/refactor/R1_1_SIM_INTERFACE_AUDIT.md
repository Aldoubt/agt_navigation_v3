# R1.1 Simulation Interface Audit

Baseline examined: `3c9d083`; implementation audit updated 2026-09-10.

## Current Gazebo contract

`agt_gazebo_sim` expands `bunker_mid360_sim.urdf.xacro` and starts
`libgazebo_ros_planar_move.so`. The plugin subscribes to `/mux/cmd_vel` and
publishes `/sim/ground_truth_odom` (`nav_msgs/Odometry`) at 50 Hz. Its configured
frame is `sim_world`, and `publish_odom_tf=false`; it does not publish any TF.

Robot state publisher owns only the fixed `base_link -> imu_link -> livox_frame`
sensor chain. There is no `ros2_control`, diff-drive plugin, simulation
localization package, or AMCL sensor input in the existing harness.

## Existing launch roles

| Launch | Role | R1.1 suitability |
|---|---|---|
| `agt_gazebo_sim/sim_world.launch.py` | Gazebo, robot description, spawn | reusable |
| `agt_gazebo_sim/navigation_demo.launch.py` | Gazebo plus field demo | unsuitable: includes real LIO/BBS chain |
| `agt_system_bringup/navigation_debug.launch.py` | hardware debug explicit assets | intentionally unchanged |
| `agt_system_bringup/rviz_field_demo.launch.py` | production field chain | intentionally unchanged |

## Missing interface and implemented minimum

The missing boundary was Gazebo truth odometry to AGT local odometry. New
`agt_sim_odometry_adapter` subscribes only to `/sim/ground_truth_odom`, publishes
the existing `/agt/odometry/local` contract with `odom`/`base_link` frames, and
is the sole simulation publisher of `odom -> base_link`. It never publishes
`map -> odom`.

`simulation_debug.launch.py` provides two mutually exclusive global-frame modes:
identity `map -> odom` for controlled world/map tests, or standard Nav2 AMCL.
AMCL is structurally available but requires a `/scan` source; current Gazebo
Livox CustomMsg is not a LaserScan and is therefore not a validated AMCL mode.

The base command contract remains `/cmd_vel -> agt_cmd_vel_guard -> /mux/cmd_vel
-> Gazebo planar-move`; Nav2 never consumes `/sim/ground_truth_odom` directly.
