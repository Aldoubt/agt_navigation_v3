# Mapping and navigation LIO policy

## Fixed selections for the current demo phase

### Mapping mode

Use `robotics-laboratory/fast-lio2` on ROS 2 Humble:

- `fastlio2` front-end
- `pgo` loop closure + GTSAM pose graph optimization
- `/pgo/save_maps` for map export
- save patches when HBA refinement is planned
- `hba` for large-scene consistency refinement

The mapping stack owns global map consistency only. Its loop-closure-corrected pose must not be used as the navigation `odom -> base_link` stream.

### Navigation local odometry

As of 2026-09-24, use `fastlio2 -> agt_fastlio_adapter` by default for navigation local odometry. It consumes the original Livox/IMU stream and adapts `/fastlio2/lio_odom` to `/agt/odometry/local`; the localization manager remains the sole `map -> odom` owner. Use the FAST-specific `fastlio2_mid360_navigation.yaml` calibration.

`Functionhx/Batch-LIO` is retained only as an explicit `--lio-backend batch_lio` alternative, not the default. It is a ROS 2 Humble batch-wise Point-LIO variant with ~1 ms grouping and in-batch deskew. The external repository currently publishes odometry on `/aft_mapped_to_init` and uses `camera_init -> body` frame names, so an explicit AGT frame adapter is required before exposing `/agt/odometry/local`.

Do not silently relabel `camera_init/body` to `odom/base_link`. The adapter must use the measured static body-to-base transform and preserve the local-origin semantics explicitly.

## 3D map to 2D Nav2 map

The current production handoff is mapping-owned traversability PGM/YAML export and an accepted versioned Map Package under `<workspace>/maps`. Navigation resolves `registry.yaml` with `auto -> latest_validated` and consumes the ready-made map, localization PCD and relocalization assets. See [current defaults](mcp-navigation-defaults-20260924.md).

The converter flow below is a legacy/explicit alternative, not the default handoff:

```text
FAST-LIO2 + PGO
  -> save global PCD + patches
  -> optional HBA refinement
  -> choose final global_map.pcd
  -> agt_map_converter
  -> map.yaml + map.pgm + elevation/slope/obstacle layers
  -> validate_nav_map
```

Baseline conversion:

```bash
ros2 run agt_map_converter pcd_to_nav_map \
  /path/to/final/global_map.pcd \
  --output /path/to/maps/site_A/navigation \
  --resolution 0.10 \
  --max-step 0.22 \
  --max-slope-deg 20.0

ros2 run agt_map_converter validate_nav_map \
  /path/to/maps/site_A/navigation
```

The 2D occupancy is therefore a derived navigation asset, not the SLAM source of truth.

## RTK/INS role

`Aldoubt/agt_ins_driver` is the RTK/INS source. It is not the 3D LiDAR map-localization backend.

Use it for:

- startup map/geographic alignment support,
- quality-gated coarse prior if useful,
- inspection metadata,
- localization health comparison,
- future bounded global constraints.

The LiDAR global relocalizer still produces the authoritative `T_map_base` measurement used by `agt_localization_manager` to establish `map -> odom`.
