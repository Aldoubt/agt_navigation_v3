# AGENTS.md — AGT navigation V1 working rules

This repository is in rapid field-validation mode. `main` is the normal development branch.
The current acceptance target is deliberately narrow:

```text
RViz waypoint queue
 -> Nav2 NavigateToPose
 -> measured base stop
 -> fixed C1 three-view capture
 -> synchronized record
 -> next point
 -> RETURN_HOME
```

HMI integration, power-cycle mission resume, automatic RTK relocalization and per-point camera optionality are out of scope until the RViz flow is repeatable on the real Bunker.

## Hardware baseline

- ROS: Ubuntu 22.04 / ROS 2 Humble.
- LiDAR: Livox MID360, lidar IP `192.168.1.117`, intentionally tilted in the robot URDF.
- MID360 internal LiDAR/IMU translation baseline: `[0.011, 0.02329, -0.04412]`; `extrinsic_est_en=false` for V1.
- IMU: MID360 built-in IMU first. Run `mid360_imu_preflight.py` before freezing `acc_norm`.
- Bunker v1 CAN: interface `can0`, bitrate `50000` bit/s.
- Bunker remote controller remains higher priority. Software may bring CAN up automatically but must not bypass the chassis/manual safety arbitration.
- Bunker ROS driver consumes `/mux/cmd_vel`, publishes `/wheel/odom`, `publish_odom_tf=false`, control rate 50 Hz.
- RTK/INS: record/quality only in V1; never use it to move `map->odom` or seed automatic relocalization.
- C1 camera/gimbal action: `/camera_gimbal/acquire_view`.

Canonical physical constants are also stored in `config/field_hardware.env`.

## Architecture invariants — do not casually break these

1. `agt_localization_manager` is the only owner of `map -> odom`.
2. Navigation local odometry is `Functionhx/Batch-LIO -> agt_batch_lio_adapter -> /agt/odometry/local`.
3. Mapping is `robotics-laboratory/fast-lio2 + PGO`, optional HBA afterward.
4. Global relocalization is `3D-BBS coarse -> local-submap small_gicp fine`; no `/initialpose` and no RTK seed in V1.
5. FAST-LIO2/Batch-LIO raw LiDAR path must preserve Livox point timing. Do not put generic voxel/self filters before the LIO front-end.
6. Navigation/relocalization use a separate `CustomMsg -> PointCloud2 -> /agt/livox/points` branch.
7. Nav2 success does not mean camera-ready. The mission runtime must pass the measured-stop gate first.
8. C1 image timestamp is the association anchor for map pose / RTK metadata / actual gimbal angle.
9. Nav2 controller, velocity smoother and Bunker command guard target a 50 Hz command chain.
10. RTK V1 is metadata only. Poor RTK must not move the robot navigation frame.

## First commands after cloning

```bash
mkdir -p ~/agt_ws/src
git clone https://github.com/Aldoubt/agt_navigation_v3.git ~/agt_ws/src/agt_navigation_v3
cd ~/agt_ws
bash src/agt_navigation_v3/scripts/bootstrap_humble.sh --smoke
```

Field diagnostics:

```bash
cd ~/agt_ws
bash src/agt_navigation_v3/scripts/field_diagnostics.sh
```

Install automatic SocketCAN bring-up once:

```bash
sudo bash ~/agt_ws/src/agt_navigation_v3/scripts/install_bunker_can_service.sh
systemctl status agt-bunker-can.service
```

## Normal field startup order

1. Verify MID360 host Ethernet is in the same subnet as `192.168.1.117` and ping succeeds.
2. Verify `agt-bunker-can.service` and `ip -details link show can0`.
3. Start MID360 driver; verify `/livox/lidar` and `/livox/imu`.
4. Start robot_state_publisher / URDF; verify tilted MID360 TF.
5. Start Bunker driver and C1 driver; keep remote/manual priority available.
6. Launch the current software chain with `agt_system_bringup/rviz_field_demo.launch.py`.
7. Keep robot stationary and manually request global relocalization.
8. Verify `map -> odom -> base_link` visually and run `demo_preflight`.
9. Only then enable autonomous motion and queue RViz patrol points.

## Important topics / actions / services

Sensor/input:
- `/livox/lidar` — Livox CustomMsg in the V1 raw path.
- `/livox/imu` — MID360 IMU.
- `/agt/livox/points` — secondary PointCloud2 branch.
- `/wheel/odom` — Bunker measured odometry.
- `/agt/odometry/local` — canonical Batch-LIO odometry.
- `/ins/navsatfix`, `/ins/status` — RTK record/quality.

Navigation/visualization:
- `/map`
- `/plan`
- `/local_plan`
- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/agt/navigation/points_obstacles`
- `/agt/rviz_patrol/markers`

Actions/services:
- `/navigate_to_pose`
- `/camera_gimbal/acquire_view`
- `/agt/localization/relocalize`
- `/agt/rviz_patrol/start`
- `/agt/rviz_patrol/clear`
- `/agt/rviz_patrol/cancel`

## Failure triage

### MID360 unavailable

```bash
ping -c 2 192.168.1.117
ip addr
ip route get 192.168.1.117
ros2 topic list -t | grep livox
```

If ping fails, fix Ethernet/subnet before debugging ROS. Do not change LiDAR algorithm parameters to compensate for a network problem.

### CAN unavailable

```bash
systemctl status agt-bunker-can.service
ip -details link show can0
journalctl -u agt-bunker-can.service -b --no-pager
candump can0
```

Expected bitrate is 50000. Bringing CAN up does not mean the chassis is in autonomous-control mode; the physical remote/controller arbitration still applies.

### Batch-LIO unstable

Check `/livox/imu` units first with `mid360_imu_preflight.py`, then timestamps, point timing, vibration/clipping, and only then covariance/noise parameters. Keep the V1 MID360 translation baseline unless data proves it wrong.

### Relocalization wrong

Stop the robot. Confirm `/agt/odometry/local` is fresh, `/agt/livox/points` is fresh, LiDAR->base_link TF is correct, active PCD/BBS assets match the Nav2 map version, and the query scan contains only post-stop frames. Never paper over a visually wrong BBS/GICP pose by relaxing all gates.

### Nav2 moves but robot does not

Check `/cmd_vel`, then guard output `/mux/cmd_vel`, then Bunker driver/CAN, then physical remote/manual priority. Do not bypass the remote priority in software during V1 testing.

### Robot reaches goal but camera starts while moving

Check `/agt/odometry/local` stop thresholds and vibration. Tune the measured-stop thresholds from real bag data; do not delete the stop gate.

## Map generation V1

`agt_map_converter` projects the final 3D PCD into XY cells. For each cell it computes point count, `min_z`, `max_z`, vertical span, a locally filled elevation surface and gradient-derived slope. A valid cell becomes occupied when either vertical span exceeds `max_step` or slope exceeds `max_slope_deg`; valid non-obstacle cells are free and unsampled cells remain unknown. This is a first-pass traversability projection, not final semantic terrain understanding.

See `docs/RVIZ_FIELD_ACCEPTANCE.md` for the current acceptance procedure and `README.md` for the whole system flow.
