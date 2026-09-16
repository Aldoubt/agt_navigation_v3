# Field Sensor Baseline V1

## MID360 / IMU

V1 uses the MID360 built-in IMU. Do not add an external IMU until a recorded Bunker vibration dataset demonstrates clipping, unacceptable bias instability, or LIO degradation.

Batch-LIO initial factory baseline:

```yaml
extrinsic_est_en: false
extrinsic_T: [0.011, 0.02329, -0.04412]
extrinsic_R: [1,0,0, 0,1,0, 0,0,1]
time_diff_lidar_to_imu: 0.0
```

The translation is the MID360 value already documented by the selected Batch-LIO upstream. Treat it as the V1 factory-consistency baseline, not as a mathematically proven calibration of this individual unit.


## Vehicle mount authority

Do not copy a second `body_to_base` calibration into navigation/localization
configs.

The two sources of truth are:

- MID360 LiDAR/IMU internal relation: the exact Batch-LIO runtime YAML
  `mapping.extrinsic_R/T`;
- MID360-to-chassis physical mount: `tracked_chassis_description` published by
  `robot_state_publisher`.

The runtime derives:

```text
T_body_base = T_body_lidar(Batch-LIO)
              * T_lidar_base(robot_state_publisher)
```

Both `agt_batch_lio_adapter` and `agt_global_relocalization` use this
derived relation. Offline relocalization starts the same chassis description;
it must not publish a separate hard-coded base-to-lidar transform.

If the physical damping mount is changed, update/verify the chassis calibration
first. Do not compensate for a chassis mounting error by editing Batch-LIO's
internal LiDAR/IMU extrinsic.

## Mandatory acc_norm test

Before the first mapping/navigation field run and after any driver/firmware change:

1. Place the robot on a reasonably level surface and keep it completely static.
2. Start the MID360 driver.
3. Run:

Run the MID360 IMU preflight supplied by the separate mapping-producer
workspace with the sensor stationary before freezing the mapping configuration.

Expected output is JSON with `result: PASS` and `recommended_batch_lio_acc_norm`.

Interpretation:

- static acceleration magnitude near `1.0` -> Batch-LIO `acc_norm: 1.0`;
- near `9.81` -> Batch-LIO `acc_norm: 9.81`;
- anything ambiguous -> stop field testing and inspect driver units / sensor state.

This check is mandatory because copying an `acc_norm` value from another Livox configuration can silently corrupt the inertial model.

## Vibration dataset

Use `record_vibration_bag.sh` to record at least:

- static before motion;
- powered static;
- low/medium straight motion;
- left/right in-place rotation;
- grass/gravel/rough terrain;
- static after motion.

Use the bag to freeze `satu_acc`, `satu_gyro`, IMU noise/covariance and stationary thresholds. Do not freeze those values from upstream examples alone.

## LI-Init

Pinned optional tool: `morte2025/LiDAR_IMU_Init_ROS2` at the exact revision in `dependencies/field_demo.repos`.

LI-Init is **not** a normal boot-time dependency. Use it when one of these is observed:

- repeatable map distortion that suggests LiDAR/IMU extrinsic error;
- time-alignment symptoms during acceleration/rotation;
- Batch-LIO degradation not explained by IMU saturation/noise;
- the MID360/IMU rigid installation changes.

Calibration should use a dedicated excitation bag with significant roll, pitch, yaw and translation. A normal tracked-vehicle patrol bag is not sufficient excitation for a trustworthy full extrinsic calibration.

If LI-Init results are used, repeat the calibration multiple times and require repeatability before replacing the factory baseline. Store the accepted result as a versioned calibration asset and keep `extrinsic_est_en: false` during normal navigation.

## RTK/INS

`agt_ins_driver` remains enabled for:

- geographic record during mapping;
- inspection image/gimbal location records;
- RTK quality/health display;
- future surveyed map/ENU tooling.

V1 explicitly forbids RTK from:

- seeding automatic LiDAR relocalization;
- correcting Batch-LIO local odometry;
- publishing or modifying `map -> odom`.

This prevents imperfect RTK calibration or tree-shadow drift from moving the navigation frame.
