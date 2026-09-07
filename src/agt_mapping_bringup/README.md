# OctoMap navigation-map baseline

`octomap_navigation_baseline.launch.py` is the default V1 navigation-map
generator. It keeps the mapping and localization assets separate:

```text
MID360 CustomMsg + IMU → FAST-LIO2 → global_map.pcd (localization, untouched)
                                      ↓
                             /fastlio2/body_cloud
                                      ↓
                 rear dynamic filter (mapping-only branch)
                                      ↓
                     OctoMap O1-H2 → Nav2 map.pgm/map.yaml
```

The rear filter is evaluated only after FAST-LIO2 has transformed the MID360
cloud into `body`. It never changes the raw LiDAR/IMU inputs, calibration,
time synchronization, FAST-LIO2 state, Batch-LIO, or the localization PCD.

## Frozen default parameters

- FAST-LIO2: `scan_resolution=0.5`, `map_resolution=0.5`, `cube_len=200`,
  `det_range=300`; all calibration/IMU parameters are preserved.
- OctoMap O1-H2: resolution `0.1 m`; `0.0 < z < 2.0 m` for both input and
  2D occupancy projection.
- Rear dynamic filter: enabled; bearing `180°`, width `60°`, planar range
  `0.8–5.0 m`; voxel and self filtering are disabled to isolate this one
  mitigation.

The upstream FAST-LIO2 baseline still has `lidar_max_range=30 m`; `det_range`
does not expand that separate incoming-point limit.

## Mapping run

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch agt_mapping_bringup octomap_navigation_baseline.launch.py
```

When the mapping run ends, stop the launch cleanly. The filter writes cumulative
point accounting to `~/.ros/agt_octomap/rear_filter_statistics.yaml`. Use a
different absolute path through `filter_statistics:=...` for each run.

## Save and package

The localization PCD supplied below must be generated from the same mapping
coordinate frame as the OctoMap run. The script saves `/projected_map`, copies
the frozen generator configuration and filter statistics into `navigation/`,
then atomically creates a versioned Map Package.

```bash
ros2 run agt_mapping_bringup export_octomap_navigation_map.sh \
  /data/maps/run_001/navigation \
  /data/maps/run_001/global_map.pcd \
  run_001 octomap_o1h2_rear_v1 \
  ~/.ros/agt_octomap/rear_filter_statistics.yaml
```

The default Map Package root is `/home/yangxuan/ros2_ws/agt_data/maps`; set
`AGT_MAP_ROOT` only when an explicit alternate storage volume is required.

Load `navigation/map.yaml` through `agt_nav2_bringup`. Never derive the
localization PCD from the rear-filtered cloud.

## Boundary and next upgrades

The rear sector prevents a close following person from first entering the
static map. It can also hide a real static feature observed **only** in that
zone. Do not enlarge it without an RViz A/B check, especially for mapping runs
that reverse. Long-lived dynamic suppression, persistence scoring, vegetation
handling, and terrain/traversability layers remain later, explicit upgrades.
