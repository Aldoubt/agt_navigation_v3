# MID360 Mount Yaw Audit

Status: active investigation on `fix/lidar-mount-yaw`.

## Goal

Determine whether the MID360-to-chassis mounting relation is sufficiently rigid
and repeatable for navigation, and separate that question from LiDAR/IMU
internal calibration, map quality, and Nav2 controller tuning.

The result of this audit decides whether the next action is:

1. keep the current mount and only correct a static chassis extrinsic;
2. repair or constrain the mechanical damping mount;
3. tune MID360/Batch-LIO vibration and timing parameters; or
4. close this issue and move to the map-quality audit.

MPPI, global-planner changes, and map-generation algorithm replacement are out
of scope for this branch.

## Current sensor boundary

V3 navigation uses the MID360 LiDAR and its built-in IMU as the primary motion
sensor pair. External INS/GNSS is not required by this audit and must not be
introduced as a navigation dependency.

Keep the two calibration layers separate:

```text
MID360 internal:
  IMU <-> LiDAR
  Batch-LIO extrinsic_T / extrinsic_R

Vehicle mounting:
  Batch-LIO body / lidar mount <-> base_link
  body_to_base_* and physical chassis TF
```

The current Batch-LIO factory-consistency baseline remains:

```yaml
extrinsic_est_en: false
extrinsic_T: [0.011, 0.02329, -0.04412]
extrinsic_R: [1,0,0, 0,1,0, 0,0,1]
time_diff_lidar_to_imu: 0.0
```

Do not change `extrinsic_R` merely because chassis yaw is suspected. Change the
LiDAR/IMU internal extrinsic only after repeatable LI-Init evidence indicates an
internal calibration problem.

## P0 implementation status

P0 calibration-source consolidation is implemented and **software-validated by offline replay on 2026-09-15**. Mechanical mount rigidity remains a vehicle-side P1/field validation item.

The runtime no longer treats copied `body_to_base_*` constants as the normal
source of truth. Instead:

```text
Batch-LIO runtime YAML
  mapping.extrinsic_R/T
        |
        v
     T_body_lidar

tracked_chassis_description / robot_state_publisher
        |
        v
     T_base_lidar

T_body_base = T_body_lidar * T_lidar_base
```

This derived `T_body_base` is consumed by both:

- `agt_batch_lio_adapter`, for `camera_init/body -> odom/base_link`;
- `agt_global_relocalization`, for the single
  `T_map_body -> T_map_base` publication boundary and for the candidate BBS
  `--base-from-body-*` arguments.

The canonical internal extrinsic is read from the exact Batch-LIO configuration
passed by the launch chain. The physical chassis relation is read from TF
published by `tracked_chassis_description`.

`offline_relocalization_demo.launch.py` no longer publishes its own
hard-coded base/lidar transforms. It starts the same chassis description
calibration used by the field sensor session.

The previous numeric `body_to_base_*` values are retained only as no active
runtime configuration; the adapter exposes an explicit legacy override mode
whose values must be supplied intentionally. This P0 change therefore removes
configuration divergence without silently inventing a new chassis calibration.

The mapping-era `mapping_body_livox_*` query transform remains frozen. It is
not changed by this mount-yaw cleanup because global relocalization is currently
operational and that transform belongs to the map/query frame contract rather
than the vehicle-mount authority.

## Hypotheses to distinguish

### H1 - Mechanical relative yaw

The damping mount has torsional compliance, backlash, or resonance, so the
actual sensor-to-chassis relation varies during motion.

Expected evidence:

- repeat-return tests do not reproduce the same sensor/chassis alignment;
- constraining the yaw freedom improves repeatability;
- base-frame obstacle clouds show mount-correlated rotation relative to the
  chassis model.

### H2 - Static chassis extrinsic is wrong

The mount is rigid but the stored `body -> base_link` or
`base_link -> lidar_link` calibration is biased.

Expected evidence:

- repeatability is good;
- the same fixed angular/translation bias is observed across runs;
- one static correction improves both odometry/base semantics and
  relocalization consistency.

### H3 - LIO vibration/timing problem

The physical mount relation is acceptable, but vibration, IMU saturation,
timestamp alignment, or deskew degrades the LIO estimate.

Expected evidence:

- degradation correlates with acceleration, rough terrain, or rotation rate;
- the MID360 IMU preflight/vibration statistics are abnormal;
- changing only the chassis static extrinsic does not remove the distortion.

### H4 - Downstream TF/self-filter problem

The LIO is stable, but local obstacle processing uses the wrong transform,
stale TF, or an inconsistent mount definition.

Expected evidence:

- raw/LIO point clouds are repeatable;
- `/agt/navigation/points_obstacles` is misaligned or intermittently missing;
- TF lookup failures or self-filter ratios correlate with navigation defects.

## Fast diagnostic workflow

### Gate A - Configuration snapshot

Before every experiment save:

```bash
ros2 param dump /agt_batch_lio_adapter
ros2 param dump /agt_obstacle_cloud_preprocessor
ros2 run tf2_ros tf2_echo base_link livox_frame
```

The TF command verifies the physical mount configuration published by
`tracked_chassis_description` only. A static TF remaining constant does not
prove the physical damping mount is rigid.

`body` is an internal Batch-LIO state frame and is not required to exist as a
robot-description TF frame. The resolved `T_body_base` must instead be checked
from the startup log of `agt_batch_lio_adapter` / `agt_global_relocalization`,
which derive it from the Batch-LIO config plus `livox_frame <- base_link`.

### Gate B - MID360 IMU preflight

Run the existing preflight with the robot stationary:

```bash
ros2 run agt_mapping_bringup mid360_imu_preflight.py --ros-args \
  -p duration_sec:=10.0
```

Record the recommended `acc_norm`, sample rate, acceleration norm stability,
and any clipping/unit warning before interpreting an LIO result.

### Gate C - Minimal runtime

The audit runtime should contain only the sensor/LIO chain required to observe
the problem:

```text
MID360 driver
  -> Batch-LIO
  -> agt_batch_lio_adapter
  -> Livox PointCloud2 bridge
  -> obstacle cloud preprocessor
  -> audit/diagnostic outputs
```

Do not start Nav2, global relocalization, Map Tracker, RTK manager, mission
runtime, or the camera for this test unless a later gate explicitly requires
them.

A dedicated `lidar_mount_audit.launch.py` should become the repeatable entry
point for this minimal chain.

### Gate D - One standardized rosbag

Use one run with labeled phases rather than many unrelated bags:

1. static, motors disabled;
2. static, motors powered;
3. left 90 deg and return, repeated;
4. right 90 deg and return, repeated;
5. straight 5-10 m and return;
6. short rough-surface pass and return;
7. static after motion.

Record at least:

```bash
ros2 bag record -o lidar_mount_audit \
  /livox/lidar \
  /livox/imu \
  /aft_mapped_to_init \
  /agt/odometry/local \
  /agt/livox/points \
  /agt/navigation/points_obstacles \
  /tf \
  /tf_static \
  /cmd_vel \
  /cmd_vel_smoothed
```

If a wheel/chassis odometry topic is available, record it as a diagnostic
reference only. It is not required to be fused into navigation.

## Metrics

The audit script/report should calculate at least:

### IMU

- sample rate and timestamp gaps;
- mean/std/peak angular velocity per axis;
- acceleration norm mean/std;
- clipping or saturation evidence where available.

### LIO / adapted odometry

- output rate and timestamp age;
- static XY/Z drift;
- static yaw drift;
- start-to-end position difference for return tests;
- start-to-end yaw difference for left/right return tests;
- symmetry/repeatability across repeated rotations.

### Point cloud / TF

- cloud input/output rate;
- TF lookup success/failure counts;
- self-filter and rear-filter removal ratios;
- optional cloud transformed to `base_link` for RViz inspection.

The first clean bag establishes the baseline. Avoid inventing a pass/fail
threshold before the baseline distribution is known; freeze numeric thresholds
only after at least one repeatable field dataset exists.

## Mechanical A/B

If safe and mechanically feasible, run the same test twice:

- A: current damping mount;
- B: temporary safe constraint that reduces yaw compliance without changing
  software parameters.

Compare the same metrics and the same fixed scene. A repeatable improvement in
return yaw, point-cloud overlap, or map sharpness in B is strong evidence for a
mechanical mount problem.

## Remaining implementation work

### P0 - Completed and replay-validated

- one Batch-LIO internal-extrinsic source is threaded through odometry and relocalization;
- physical mount geometry comes from `tracked_chassis_description` TF;
- candidate BBS base/body arguments are generated from the resolved transform;
- offline relocalization uses the same chassis description instead of hard-coded TF;
- `agt_batch_lio_adapter` and `agt_global_relocalization` resolved exactly the same
  `T_body_base` during replay:
  `t=[-0.221022, 0.023290, -0.844290]`,
  `q=[0.000000000, -0.113203214, 0.000000000, 0.993571856]`;
- automatic relocalization completed with
  `score=0.942599`, `fitness=0.116435`, `overlap=0.995376`;
- Localization Manager accepted the global correction and the
  `map -> odom -> base_link` chain became available;
- the initial stationary-gate rejection at `linear=0.070 m/s` is expected
  behavior and was followed by a successful stationary retry.

### P1 - Dedicated audit launch

Add `lidar_mount_audit.launch.py` with the minimal chain described above.

### P1 - Automated audit report

Add an analyzer that produces a versioned YAML/JSON report containing the
metrics in this document.

### P1 - Obstacle/TF observability

Extend obstacle-cloud diagnostics with TF lookup success/failure counters and,
when explicitly enabled, a `base_link` debug cloud.

## Exit criteria

This branch may merge to `main` when:

- online and offline paths use one consistent mount convention;
- the standardized audit can be replayed without Nav2 or global localization;
- one baseline bag and report are archived;
- the mechanical A/B result is recorded if the mount remains suspect;
- any accepted mount correction is applied consistently to odometry and
  relocalization consumers;
- the MID360 internal LiDAR/IMU extrinsic remains unchanged unless separate
  calibration evidence justifies changing it.

After this gate:

- if mount/LIO is stable, open a short-lived `fix/map-quality` branch;
- only after map/costmap behavior is understood should a
  `feature/nav2-mppi` branch be opened.
