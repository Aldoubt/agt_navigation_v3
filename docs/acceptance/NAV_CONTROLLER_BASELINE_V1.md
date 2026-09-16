# NAV-C0 Controller / Local Costmap / cmd_vel Baseline

This is a read-only production baseline for `feature/nav2-mppi`. No Nav2,
map, perception, or chassis parameter is changed here.

## Controller and MPPI status

The production controller is `controller_server` plugin `FollowPath`:
`nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`.
`controller_frequency` is **50 Hz** and its odometry source is
`/agt/odometry/local`.

RPP parameters: desired linear velocity 0.45 m/s; velocity-scaled lookahead
0.30--1.00 m with 0.60 m nominal and 1.5 s lookahead time; approach minimum
0.05 m/s at 0.8 m; collision detection enabled with 1.0 s TTC horizon;
turn-radius regulation 0.8 m / 0.10 m/s; cost regulation distance/gain
0.6 m / 1.0; rotate-to-heading enabled at 0.60 rad, 0.55 rad/s; maximum
angular acceleration 10.0 rad/s²; reversing disabled. Progress checking is
`PoseProgressChecker` (0.20 m, 0.10 rad, 12 s); goal tolerance is 0.15 m and
0.0872665 rad.

`nav2_mppi_controller` is installed in the Humble environment, but is **not
integrated**: it is absent from repository package dependencies, configuration,
and launch references. Status: `MPPI_NOT_INTEGRATED`.

## Command chain

```text
controller_server (FollowPath)
  └─ /cmd_vel (Nav2 controller output; verify live publisher count)
     velocity_smoother (50 Hz, OPEN_LOOP)
       └─ /cmd_vel_smoothed (default Nav2 smoother output; verify live)
          agt_cmd_vel_guard (50 Hz)
            └─ /mux/cmd_vel
               Bunker base driver (external `bunker_base`; 50 Hz control)
```

The guard is the only intended software publisher to `/mux/cmd_vel`; the
Bunker driver consumes it. The physical remote/manual arbitration is outside
this software chain and remains higher priority. The guard also accepts
`/agt/hmi/cmd_vel` only after an explicit `/agt/control/mode` switch; this is
not a second navigation publisher. Live audit must use `ros2 topic info -v`
on every command topic, rather than infer publisher multiplicity from names.

The smoother can clamp velocity to `[0.55, 0, 0.65]` and
`[-0.20, 0, -0.65]`, apply 50 Hz open-loop acceleration/deceleration limits
`[0.45, 0, 0.80]` / `[-0.55, 0, -0.90]`, deadband `[0.01, 0, 0.01]`, and
zero a stale input after 0.25 s. The guard then clamps to ±0.55 m/s forward,
0.20 m/s reverse, ±0.65 rad/s, and applies 0.45 m/s² acceleration, 0.80 m/s²
deceleration, and 0.80 rad/s² angular slew limits. It hard-stops on stale
commands, unsafe/stale localization, or mode handoff. Thus a downstream
change in `cmd_wz` can be a smoother/guard effect rather than RPP oscillation.

## Local obstacle and costmap chain

```text
MID360 CustomMsg -> Batch-LIO (timing-preserving LIO branch)
                 -> Livox format bridge -> /agt/livox/points
                 -> agt_obstacle_cloud_preprocessor
                 -> /agt/navigation/points_obstacles
                 -> local_costmap VoxelLayer (marking + clearing)
                 -> InflationLayer -> controller collision checking
```

The preprocessor accepts PointCloud2 in its real source frame and looks up
`base_link <- cloud_frame` at the cloud timestamp. It drops clouds on TF
failure, removes invalid points and 0.5--120 m range outliers, removes the
self box centered at `(0,0,0.20)` with size `(1.023,0.778,0.400)` m plus
0.05 m padding, then applies 0.20 m voxel reduction. The rear-sector filter
is disabled in the production baseline. Output retains the source cloud frame.

The rolling local costmap is `odom` frame, 8 m × 8 m at 0.05 m resolution,
updates at 10 Hz and publishes at 5 Hz. Its VoxelLayer observes
`/agt/navigation/points_obstacles`, marks and clears, accepts 0.10--2.0 m
heights, observes 0.30--12 m and raytraces 0.30--15 m. Inflation radius is
0.55 m. Self-body leakage, rear hardware outside the self box, ground returns
above 0.10 m, vegetation, sparse returns, TF drops (which suppress clearing),
and marking/clearing imbalance can all create time-varying left/right costs.
These are hypotheses for bag evidence, not tuning authorization.

## Odometry baseline

`Functionhx/Batch-LIO -> agt_batch_lio_adapter -> /agt/odometry/local` is
the canonical navigation odometry source. The adapter publishes `odom` to
`base_link`, copies/converts pose, and derives twist from successive LIO poses
when enabled; it publishes Odometry with queue depth 50. The controller runs
at 50 Hz but does not imply 50 Hz LIO pose/twist updates. Existing
`NAV_TEST_002_AUDIT` recorded 1,788 local-odom messages in 180.95 s
(approximately 9.88 Hz), so current field bags must measure both odom message
rate and twist update rate before attributing a 50 Hz command oscillation to
RPP.

## Required rosbag contract

Record actual live names after `ros2 topic list`/`ros2 topic info -v`:

```text
/tf  /tf_static
/agt/odometry/local  /wheel/odom
/plan  /local_plan  /navigate_to_pose/_action/feedback
/cmd_vel  /cmd_vel_smoothed  /mux/cmd_vel
/agt/navigation/points_obstacles
/local_costmap/costmap  /local_costmap/costmap_updates
/local_costmap/costmap_raw  /local_costmap/published_footprint
/global_costmap/costmap
```

Also capture controller trajectory/debug topics if exposed by the running
Humble Nav2 version, localization status, and Bunker driver feedback/status.
The existing `p3_1_self_obstacle_replay_analyzer.py` can analyze obstacle
cloud count, local-costmap footprint costs, `/agt/odometry/local`, and one
`/cmd_vel` stream. `agt_operator_console/replay_audit.py` checks local odom
freshness/rate and frame contracts. Neither currently separates raw RPP,
smoother, guard, and final commands, nor computes cross-track/heading error,
curvature, command zero crossings, RMS, or oscillation spectra. Those are
the controller-benchmark TODOs; do not create a parallel benchmark package.

## Field baseline tests and attribution order

Each test requires at least three repeats: (A) straight 8--10 m; (B) gentle
turn; (C) 90-degree turn; (D) approach a static obstacle and stop. For the
straight test calculate cross-track and heading error, raw/smoothed/final
`cmd_wz`, `cmd_vx`, odom yaw/yaw-rate, local-path curvature, `cmd_wz`
zero-crossing frequency and RMS, heading-error RMS, and cross-track p50/p90/max.

Diagnose in this fixed order: (A) global path straightness, (B) odom/yaw
stability, (C) local-costmap stability, (D) RPP raw command behavior,
(E) smoother/guard transformation, then (F) chassis tracking. Only with
A--C normal and D oscillatory may the cause be assigned to the controller.

Highest-priority evidence gaps are: (1) distinguish raw/smoothed/guard/final
commands in one bag, (2) measure LIO pose/twist/yaw-rate cadence against the
50 Hz command chain, and (3) correlate local-costmap/obstacle-cloud changes
with left/right command reversals and footprint costs.
