# R1.1 Simulation Validation

## Build

PASS — `colcon build --symlink-install --packages-select agt_sim_odometry_adapter
agt_gazebo_sim agt_system_bringup` completed (3/3). Package discovery initially
exposed a metadata cycle; removing the reverse `agt_gazebo_sim ->
agt_system_bringup` runtime declaration restored a valid package graph.

## Gazebo startup

FAIL in this execution environment. `simulation_debug.launch.py` started
gzserver, robot_state_publisher, spawn, adapter, identity publisher, Nav2 and
guard, but gzserver reported `X_GLXCreateContext BadValue`; spawn_entity exited
afterwards and no simulated robot was created. Log:
`~/.ros/log/2026-09-10-22-49-07-768504-yangxuan-Default-string-1175289/launch.log`.

## TF

FAIL / not observable. Identity `map -> odom` node started, but no robot entity
meant no incoming truth odometry and therefore no adapter `odom -> base_link`.
Expected ownership after a successful spawn is:

```text
map -> odom                 agt_sim_identity_map_to_odom OR AMCL
odom -> base_link           agt_sim_odometry_adapter
base_link -> sensor frames  robot_state_publisher
```

## odom adapter

PASS — package built and launch node started. Runtime message forwarding remains
unverified because Gazebo failed before publishing `/sim/ground_truth_odom`.

## map->odom

PASS for identity publisher process startup; transform delivery is unverified
in the failed Gazebo run. AMCL: NOT TESTED; current simulation provides no
`/scan`, so AMCL cannot localize honestly.

## Nav2 planner

NOT TESTED — planner server started, but no spawned robot/TF chain existed to
submit a valid goal.

## Nav2 controller

NOT TESTED — controller server started, but no spawned robot/TF chain existed.

## cmd_vel output

NOT TESTED — topic graph contains `/cmd_vel`, `/cmd_vel_nav`, and `/mux/cmd_vel`;
no goal was sent while Gazebo failed. The intended chain is unchanged:
`/cmd_vel -> agt_cmd_vel_guard -> /mux/cmd_vel -> planar-move`.

## RViz goal test

NOT TESTED — RViz was deliberately disabled for headless launch. No screenshot
was generated. Rerun with `gui:=true launch_rviz:=true` on a host with a working
Gazebo GLX/EGL context, then send and cancel a 2D Goal Pose.

## Known issues

1. This host's Gazebo Classic server cannot create the required GLX context.
2. AMCL mode needs a simulated `/scan`; converting Livox CustomMsg to LaserScan
   is outside this R1.1 control-chain gate and has not been added.
3. The Gazebo world and default navigation map origin have not been measured for
   equivalence. Do not claim planner/controller acceptance until that relation is
   documented; the minimal fix is a dedicated map matching the world, not a map
   edit or a TF offset hidden in the launch.

## R1.1 Simulation Baseline

NOT ACHIEVED. The software interface boundary, launch composition, package build
and Nav2/guard process startup are in place; Gazebo entity spawn, TF delivery,
Nav2 goal/cancel, velocity output and robot motion require a graphics-capable
Gazebo execution environment and a verified world/map pair.
