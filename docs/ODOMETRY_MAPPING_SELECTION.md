# Mapping / Odometry Selection for Bunker + MID360

## Recommended split

Do not force one monolithic SLAM process to own both mapping and navigation runtime.

### Mapping mode

Preferred candidate for evaluation: `robotics-laboratory/fast-lio2` (ROS 2 Humble).

Why it is interesting for this project:
- FAST-LIO2 front-end;
- loop closure + GTSAM PGO;
- online re-localization module;
- consistency-map optimization;
- HBA option intended for large scenes;
- Ubuntu 22.04 / ROS 2 Humble and livox_ros_driver2 dependency.

For the first field benchmark, use it as an offline/engineering mapping tool and export a globally consistent PCD. The AGT map converter then creates Nav2 layers from the exported map.

Alternative legacy/community references include FAST_LIO_SAM / FAST_LIO_SLAM variants, but many are ROS 1 first and should not become the product runtime baseline without a port/maintenance review.

### Navigation mode

The navigation runtime should own only a low-latency local odometry source. Loop closure is not required in the high-rate control path because the global map alignment is handled separately by `agt_global_relocalization + agt_localization_manager`.

Two candidates should be benchmarked on the same MID360 rosbag and Bunker field run:

1. FAST-LIO2 baseline
   - familiar and already aligned with the existing architecture;
   - use original per-point timing / CustomMsg path;
   - good default if vibration does not saturate or strongly bias the IMU.

2. Point-LIO vibration candidate
   - specifically designed for high-bandwidth odometry under severe vibration/aggressive motion and IMU saturation;
   - requires correct per-point timestamps, IMU saturation parameters, time synchronization and calibrated extrinsics;
   - preferred experiment if the tracked chassis vibration causes FAST-LIO2 drift/jumps.

## Architecture

```text
MAPPING MODE
MID360 + IMU
    -> mapping SLAM (FAST-LIO2 + PGO/HBA candidate)
    -> globally consistent global_map.pcd
    -> agt_map_converter
    -> Nav2 map + terrain layers

NAVIGATION MODE
MID360 + IMU
    -> FAST-LIO2 OR Point-LIO benchmark winner
    -> odom -> base_link

MID360 current scan + global_map.pcd
    -> agt_global_relocalization
    -> T_map_base measurement
    -> agt_localization_manager
    -> map -> odom
```

This permits the mapping SLAM to change without changing Nav2 or the mission runtime, and permits the odometry front-end to change without regenerating the map as long as calibration/frame conventions stay consistent.

## High-vibration priorities

Before changing algorithms, verify these physical/data issues:
1. rigid LiDAR/IMU mount with no relative micro-motion;
2. mechanical isolation that does not introduce a low-frequency flexible mode;
3. correct IMU range/saturation configuration;
4. LiDAR/IMU hardware or accurately calibrated time synchronization;
5. fixed, measured LiDAR-IMU extrinsics (disable online extrinsic estimation in normal runtime once calibrated);
6. inspect raw gyro/acceleration PSD and saturation count on the actual tracked chassis;
7. benchmark stationary, straight, spin-in-place, rough-ground and slope runs.

If the onboard MID360 IMU saturates or is dominated by chassis resonance, software tuning alone is not sufficient. A better-range external IMU rigidly mounted to the LiDAR assembly can be evaluated, with LiDAR-IMU temporal/extrinsic calibration performed before runtime.

## Acceptance benchmark

Compare FAST-LIO2 and Point-LIO using identical data:
- no-map local trajectory continuity;
- stationary drift after vibration;
- yaw drift during repeated track turns;
- odometry jumps / invalid frames;
- IMU saturation count;
- CPU load and latency;
- relocalization alignment residual against the same global map.

Freeze the runtime odometry only after this benchmark.
