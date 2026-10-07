# REAL_ROBOT_TODO — only physical facts and acceptance

All are PENDING until measured on the real vehicle. Configuration template: ~/agt/profiles/yhs/. Never copy the old Bunker constants.

| Required fact | File / validation |
|---|---|
| Exact YHS model and ROS1 driver repo/commit, launch and status message type | robot.yaml; record driver audit in this document |
| CAN interface and bitrate | base.yaml can_interface/can_bitrate; do not guess |
| ROS1 master and driver environment | base.yaml ros1_master_uri; host Noetic + installed driver overlay |
| ROS1 cmd_vel, odom, chassis JSON status, estop Bool topics | topics.yaml ros1_*; adapt actual vendor status type if needed |
| Wheel odom scale, wheel directions and angular sign | measure R2/R3; record verified values/driver config |
| Driver watchdog behavior and stop deadline | base.yaml driver_watchdog_sec/driver_watchdog_verified; kill BOTH gateway endpoints separately |
| Base link, footprint frame, rotation center | robot.yaml; calibration/base_geometry.yaml |
| Footprint/padding/self-filter center and size, ground reference | navigation.yaml; no Bunker geometry allowed |
| URDF and meshes | profiles/yhs/robot_description/urdf and meshes (mounted); robot.yaml urdf relative path |
| MID360 mounting pose, IMU mounting pose, lidar/imu frames | robot.yaml lidar_frame/imu_frame + URDF TF + calibration YAML |
| LiDAR and IMU extrinsics/version | calibration/lidar_extrinsics.yaml, imu_extrinsics.yaml, calibration_version.yaml; VERIFIED only after measurement |
| Actual FAST-LIO calibration | localization.yaml fastlio_config points to external YAML, shared by mapping and navigation; contains measured IMU/LiDAR and body/base transform contract |
| Host network interface, host IP, MID360 sensor IP | sensors.yaml and external livox_config JSON; Ethernet/UDP only |
| Terrain/footprint projection parameters | mapping.yaml projection_config points to externally reviewed exporter YAML |
| Maximum linear/reverse/angular velocities and acceleration/deceleration | navigation.yaml motion_limits; low-speed acceptance first |
| Measured stop thresholds and maximum settling time | base.yaml stop_linear_mps/stop_angular_radps/stop_timeout_sec |
| Physical emergency stop, remote control priority | test hardware estop and ROS1 Bool adapter; watchdog is not an estop |

Before production motion, audit actual driver: distro, CAN protocol/interface/bitrate, odom rate, TF publisher settings (must not compete with LIO odom/base ownership), cmd_vel timeout, estop source and chassis heartbeat. The software cannot infer whether a vendor driver maintains its last speed after process death. `driver_watchdog_verified` is a field attestation, not an automatic test result.

Real sensor point-time units, IMU units/noise, network packet loss, TF consistency, 3D-BBS/GICP accuracy, Nav2 controller behavior, and physical stopping distance require bags/real hardware. No real-hardware PASS is asserted by cloud tests.
