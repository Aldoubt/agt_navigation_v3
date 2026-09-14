# TF Convention

The repository assumes the chassis description from `tracked_chassis_description` is the physical sensor-mount TF source of truth.

```text
map
 |
 +-- odom                 # continuous local odometry frame
      |
      +-- base_footprint  # planar vehicle reference
           |
           +-- base_link
                |
                +-- chassis_cad_link
                     |
                     +-- lidar_mount_link
                          |
                          +-- lidar_link
```

## Important rules

- Keep the measured MID360 mounting tilt in TF.
- Do not level the MID360 point cloud in software.
- `lidar_link` is the navigation/localization LiDAR frame.
- `livox_frame` is only a driver/compatibility frame when supplied by the driver.
- `imu_link` must describe the actual IMU used by FAST-LIO2. Do not assume it represents the external INS.
- External INS/GNSS frames are a later integration and require measured lever-arm/extrinsic calibration.
- Exactly one component owns `map -> odom` during runtime: the localization manager.
- The continuous odometry backend provides `odom -> base_link` semantics to the navigation stack.

## Batch-LIO `body` and robot `base_link`

Batch-LIO publishes `camera_init -> body`, where `body` is the LIO/IMU state
frame. `body` is **not** the robot chassis frame and must not be aliased to
`base_link` with an identity transform.

V3 no longer stores an independent fixed `T_body_base` number in the odometry
adapter or relocalization configuration. The runtime relation is derived from
two independent authorities:

```text
Batch-LIO runtime YAML
  mapping.extrinsic_R/T
  = T_body_lidar

tracked_chassis_description
  robot_state_publisher
  = T_base_lidar
```

The runtime composes:

```text
T_body_base = T_body_lidar * T_lidar_base
```

`agt_batch_lio_adapter` and `agt_global_relocalization` must use the same
Batch-LIO config path and the same physical `base_link <-> livox_frame` TF.
The candidate BBS backend receives the inverse `T_base_body` generated from
that same resolved transform.

This removes the former duplicated body/base constants and prevents online and
offline paths from silently using different mount calibrations. Offline replay
must start the same `tracked_chassis_description` calibration instead of
publishing its own hard-coded base-to-lidar transform.

The mapping-era `mapping_body_livox_*` query transform is a separate frozen
map/query contract. Do not change it as part of chassis-mount cleanup unless a
dedicated relocalization-frame experiment proves that contract wrong.

## Tilted sensor

The current chassis description contains the physical CAD-to-LiDAR mounting tilt. Preserve it. The global map and live scans must use the same TF convention.

## Future INS branch

When RTK/INS is introduced, use a separate branch of the TF tree:

```text
base_link
  +-- ins_link
       +-- gnss_link
```

Do not overload the MID360 `imu_link` frame with external INS semantics.
