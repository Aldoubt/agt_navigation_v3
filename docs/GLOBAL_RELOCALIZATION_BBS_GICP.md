# Global Relocalization V1: Polar Context + candidate 3D-BBS + small_gicp

## Current decision

V1 is LiDAR-map based and does **not** use RTK/INS as an automatic initial pose
or correction source. The robot must be stationary for startup/global recovery.

```text
localization missing / invalid
        |
        v
stop robot and wait for stable local LIO
        |
        v
build base_link query
        |
        v
AGT Polar Context place retrieval
        |
        v
Top-K mapping keyframes
        |
        v
descriptor yaw + keyframe pose seed
        |
        v
candidate-local CPU 3D-BBS
        |
        v
small_gicp 6-DoF refinement
        |
        v
score / fitness / overlap gates
        |
        v
/agt/relocalization/pose
        |
        v
agt_localization_manager
        |
        v
map -> odom
```

`agt_localization_manager` remains the only owner of `map -> odom`.

## Why the architecture changed

The first implementation sent every query directly into CPU 3D-BBS over the
entire ~100 m outdoor map, including large translation and angular search
ranges. That worked in tiny windows but repeatedly timed out on the full map.

The map contains trees, ground and repeated outdoor geometry. Those structures
generate many plausible coarse BnB branches, so CPU whole-map BBS is a poor V1
place-recognition layer.

The mapping pipeline already saves:

```text
poses.txt
patches/0.pcd
patches/1.pcd
...
```

Those keyframes are now reused as the global candidate database. This restores
the intended architecture: **place retrieval first, geometric registration
second**.

## Candidate retrieval

`build_relocalization_candidates` generates an AGT-owned Polar Context database
from the mapping keyframe patches. It does not vendor the canonical Scan Context
implementation.

The database stores:

- keyframe patch name;
- optimized map pose;
- ring key for place prefiltering;
- sector key for yaw alignment.

At runtime the query is ranked against the database. The best descriptor
candidates provide both a map region and a dominant yaw estimate.

## Frame convention

Mapping patches are stored in FAST-LIO `body` coordinates, while runtime
relocalization queries are built in robot `base_link` coordinates. The backend
therefore converts each saved `T_map_body` candidate into `T_map_base` using the
same calibrated MID360/body/base extrinsic used by the Batch-LIO adapter.

Current `T_base_body`:

```text
translation  [ 0.25960014, -0.02326770,  0.45244230 ]
quaternion   [-0.00047700,  0.100267018, 0.00159200, 0.994959177]  # xyzw
```

Inverse `T_body_base`:

```text
translation  [-0.16403417,  0.02439982, -0.49511119 ]
quaternion   [ 0.00047700, -0.100267018,-0.00159200, 0.994959177]  # xyzw
```

These values come from the measured `base_link -> lidar_link` mounting pose
composed with the pinned MID360 LiDAR/IMU extrinsic. If either physical mounting
or LiDAR/IMU calibration changes, update both adapter and relocalization config.

## Candidate-local 3D-BBS

3D-BBS remains the robust geometric coarse-registration stage, but it no longer
performs whole-map 6-DoF global search.

Current V1 policy:

```text
BBS assets             0.5 m minimum level, 5 levels
descriptor prefilter   40
candidate Top-K        2
candidate XY radius    +/- 4 m
candidate Z radius     +/- 2 m
roll/pitch residual    0 deg
yaw residual           0 deg
per-candidate timeout  8 s
BBS threshold          0.05
total backend timeout  18 s
CPU threads            8
```

The low BBS threshold is intentional. In this design BBS only needs to return a
useful coarse transform quickly. Final acceptance is performed after GICP.

When descriptor similarity is strong and the BBS result is already
geometrically useful, weaker candidates are skipped early.

## small_gicp refinement and final gates

small_gicp performs the final 6-DoF registration against a local crop of the
global map. It absorbs descriptor quantization and residual
roll/pitch/translation errors.

The final result exposes:

- refined pose;
- convergence;
- fitness;
- overlap;
- combined score;
- descriptor candidate metadata;
- BBS timing.

`agt_global_relocalization` then applies ROS-side quality gates and maps quality
into pose covariance before publishing `/agt/relocalization/pose`.

## Stationary-only V1 behavior

Global relocalization is not intended to run while the tracked robot is moving.

```text
BOOT / LOST / invalid global correction
        -> stop
        -> wait for local odometry
        -> verify stationary from pose delta
        -> build query
        -> relocalize
        -> validate
        -> LOCALIZED
        -> navigation may resume
```

Batch-LIO currently publishes zero twist on `/aft_mapped_to_init`, therefore the
stationary gate must not trust twist alone.

## Map assets

For every optimized map package generate:

```text
relocalization/
  global_map_downsampled.pcd
  voxelmaps_coords/
  relocalization_assets.yaml
  polar_context.db
  polar_context.yaml
```

Do not reuse candidate/BBS assets after the optimized map or keyframe set has
changed.

## Offline acceptance result

The first full replay passed on 2026-09-05 using:

```text
map package  agt_data/maps/bag_mapping_current
rosbag       bunker_mid360_mapping_20260901_211105
```

Observed sequence:

```text
QUERY_READY
-> BBS_SEARCHING
-> BBS_COARSE_FOUND
-> GICP_REFINING
-> SUCCEEDED
-> global_pose_accepted
-> LOCALIZED
```

Representative result:

```text
score           0.874344
fitness         0.128999
overlap         0.973186
position std    0.231676 m
yaw std         4.507872 deg
candidate       35.pcd
```

`LocalizationStatus` reported fresh local odometry and a valid global
correction. `map -> odom` was observed continuously after acceptance.

This is an offline functional gate only. Real-vehicle multi-location acceptance
is still required before field freeze.

## RTK/INS policy

RTK/INS remains useful for survey metadata, map-to-ENU work, inspection record
association and diagnostics. It does not seed V1 relocalization and does not own
`map -> odom`.

## Further reading

See `docs/RELOCALIZATION_DEBUGGING_RETROSPECTIVE_2026-09-05.md` for the full
failure-isolation and debugging lessons from the first successful offline gate.
