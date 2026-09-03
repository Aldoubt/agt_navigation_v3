# agt_navigation_v3

ROS 2 Humble navigation stack focused on the core localization problem:

> **3D LiDAR Global Localization without an initial pose + FAST-LIO2 continuous tracking**

## Scope

The first implementation intentionally does **not** depend on RTK/INS, camera fusion, Bunker control, or Nav2. Those systems are integration layers added after localization is stable.

### Core pipeline

```text
MID360 PointCloud2
        |
        v
+-----------------------+
| PointCloud Preprocess  |
| - NaN/range            |
| - robot self-filter    |
| - optional rear mask   |
| - voxel downsample     |
+-----------+-----------+
            |
            +--------------------+
            |                    |
            v                    v
       FAST-LIO2          Global Relocalization
       continuous         Scan Context -> coarse
       odometry           registration -> GICP
            |                    |
            +---------+----------+
                      v
             Localization Manager
                      |
                      v
                  map -> odom
```

## Coordinate-frame policy

The MID360 is intentionally mounted with its measured tilt. The sensor must **not** be artificially levelled in the incoming point cloud. The installation transform belongs in TF:

```text
base_footprint
  -> base_link
  -> chassis_cad_link
  -> lidar_mount_link
  -> lidar_link
```

The current chassis description is the source of truth for the physical installation transform. `imu_link` must only represent the actual IMU convention used by the selected FAST-LIO2 input; do not silently use it as an external INS frame.

## Point-cloud filtering policy

The first filter is a geometry-based robot self-filter, using `base_link` collision geometry. It removes vehicle structure while preserving environmental points behind the robot. A narrow rear angular mask is available only as a fallback for known structures that cannot be represented by the self-filter geometry.

This preprocessing must be shared by mapping/localization and must be configured identically enough that the map and live scan have compatible geometry.

## Repository layout

```text
agt_navigation_v3/
├── docs/
├── config/
├── launch/
├── agt_navigation_v3/
├── scripts/
└── test/
```

## Current status

This repository starts with the ROS 2 integration skeleton and the deterministic point-cloud preprocessing layer. The global relocalization backend is deliberately an adapter boundary so Scan Context / global registration / GICP can be selected and pinned only after the first rosbag benchmark.

## First milestone

1. Replay the existing MID360 rosbag.
2. Run FAST-LIO2 with the tilted MID360 TF unchanged.
3. Verify timestamp/IMU/deskew behaviour on rough tracked motion.
4. Enable self-filter and verify vehicle rods disappear without deleting useful rear environment.
5. Build a clean 3D map and localization database.
6. Run global relocalization with **no `/initialpose`**.
7. Hand the recovered pose to FAST-LIO2 and verify continuous tracking.

## Dependency policy

External repositories are referenced by repository URL only in this stage. **No branch is frozen yet.** Dependency commits/branches are frozen only after the end-to-end demo and acceptance dataset are stable.
