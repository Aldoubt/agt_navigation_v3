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
- The continuous odometry backend provides `odom -> base_link` semantics to the navigation stack.

## Batch-LIO `body` and robot `base_link`

Batch-LIO publishes `camera_init -> body`, where `body` is the LIO/IMU state
frame. `body` is **not** the robot chassis frame and must not be aliased to
`base_link` with an identity transform.

Current calibrated relation:

```text
T_base_body:
  translation  [ 0.25960014, -0.02326770,  0.45244230 ]
  quaternion   [-0.00047700,  0.100267018, 0.00159200, 0.994959177]  # xyzw

T_body_base:
  translation  [-0.16403417,  0.02439982, -0.49511119 ]
  quaternion   [ 0.00047700, -0.100267018,-0.00159200, 0.994959177]  # xyzw
```

`agt_batch_lio_adapter` uses the versioned `T_body_base` parameter directly by
default. This fixed calibration should not depend on TF-buffer timing during
rosbag playback. A matching static `body -> base_link` TF is still published for
RViz/Nav2 and other graph consumers.

The relocalization backend uses the inverse `T_base_body` to convert mapping
keyframe `T_map_body` poses into runtime `T_map_base` candidate poses. Keep both
configurations synchronized.

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
