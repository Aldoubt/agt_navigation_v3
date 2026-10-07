# Real robot acceptance — sequential gates

Record operator/date/profile hash/map bundle digest and evidence for each gate. All gates initially PENDING. Stop at a failed gate; use STOP ALL and physical estop. Clear the area and maintain physical remote priority. Never change safety gates to make a check pass.

| Gate | Procedure | Evidence needed |
|---|---|---|
| R0 Docker/GUI | install.sh, ./agt up, desktop launcher; doctor --report | processes healthy, Qt visible, volumes survive runtime recreation |
| R1 CAN only, wheels stationary | physical estop engaged; ./agt can up; ip -details link show measured interface | correct measured bitrate, no wheel movement, chassis no faults |
| R2 YHS state + odom | launch audited ROS1 driver; command path remains disabled | chassis/estop fresh; wheel odom rate/scale/sign; no competing TF |
| R3 low-speed direction | wheels safely raised/test area; apply measured low limits; commands enter Motion Guard (never direct YHS) | forward/reverse/yaw direction, remote priority and measured stop; see note below |
| R4 TF/URDF | load mounted URDF; inspect map/odom/base/footprint/lidar/imu chain | physical rotation center and sensor pitch/translation match; sole map->odom owner |
| R5 MID360 + IMU | System -> Start MID360 -> Test Connection/Preflight | Ethernet packets, raw Livox timing, IMU units, frequencies and timestamps |
| R6 static localization | known validated map, Start Navigation Mode while stationary | fresh LIO odom, correction validity; no motion until READY |
| R7 mapping | new ID/version; Start Mapping; drive with physical remote; Stop & Build Map | raw bag closed, clean lifecycle, PGO/nonempty map/keyframes/hash verified |
| R8 bundle build | native localization assets, grid, MapStudio Review -> Confirm & Save -> close editor -> Confirm & Seal | mapping/localization provenance, raster/review hashes; READY and Activate |
| R9 3D-BBS + GICP | cold start from multiple distinct poses without initialpose | accurate coarse+fine pose, retained artifacts and quality gates, tracking continuity |
| R10 low-speed Nav2 | single reachable point under measured limits | obstacle avoidance, stable TF, guard chain and measured-stop gate |
| R11 multi waypoint/dwell | save bound route P1=5 s, P2=20 s, P3=0 | action results advance, dwell timestamps, COMPLETED |
| R12 pause/resume/cancel | pause while navigating and dwelling; cancel; localization loss; unplug gateway | cancellation barrier, no old goal replay, hard stops and fresh command recovery |
| R13 30-minute endurance | repetitive route + recording + disconnect/reconnect + stop all | memory/disk/CPU, timing, watchdog deadlines, bags closed, no orphan processes |

R3 note: existing V3 Motion Guard refuses manual commands without fresh valid localization. This invariant is retained. Before a known map/localization is available, use the **physical YHS remote** for direction/odom tests; defer software low-speed teleop until R6/R9. Never manufacture LocalizationStatus=READY or disable require_localization_status to pass R3. Document the deferred software test explicitly, then repeat R3 software command chain before R10.

Kill ROS2 runtime: ROS1 gateway must publish zero within configured timeout. Kill ROS1 gateway: actual driver watchdog must stop the vehicle within its separately verified deadline. Unplug CAN/MID360: mission must ERROR, guard must close, physical stop confirmed. Cloud mock socket timing does not verify these deadlines.

After each gate: ./agt doctor --report; keep diagnostics in ~/agt/diagnostics. Include relevant small logs or bag references. Do not package huge bags in default diagnostic report.
