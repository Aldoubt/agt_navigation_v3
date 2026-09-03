# TF Convention

The repository assumes the chassis description from `agt_chassis_description` is the physical TF source of truth.

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
- The continuous odometry backend owns `odom -> base_footprint`.

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
