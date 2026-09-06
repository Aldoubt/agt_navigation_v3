# agt_terrain_map_generator

Terrain-aware 2.5D navigation-map generation for `agt_navigation_v3`.

## Boundary

Input is a completed 3D localization/mapping asset such as `global_map.pcd`. This package must not sit in the FAST-LIO2 time-preserving sensor path and must not modify the localization PCD.

The pipeline is intentionally split into stable interfaces:

1. `GroundSegmenter`: ground vs. non-ground separation. First target adapter: Patchwork++.
2. `ElevationBuilder`: robust local ground elevation, baseline target is median + confidence.
3. `SlopeBuilder`: slope derived from the ground/elevation surface.
4. `ObstacleBuilder`: non-ground obstacle evidence relative to local ground height.
5. `MapExporter`: Nav2 occupancy and terrain-layer export into an AGT Map Package.

## Migration policy

`agt_map_converter` remains the regression/fallback converter until the terrain-aware path passes rosbag/PCD acceptance tests. Do not delete or silently redirect the old converter during v0.x.

## Map-package outputs

Expected final outputs are:

- `localization/global_map.pcd` (copied/reference only; never terrain-filtered)
- `navigation/map.pgm`
- `navigation/map.yaml`
- `terrain/elevation.*`
- `terrain/slope.*`
- `terrain/obstacle.*`
- `metadata.yaml`

## Current v0.1 status

The ROS2 package, common data types, algorithm interfaces, configuration and launch entry are present. `pipeline.enabled` defaults to `false` until concrete algorithm adapters/builders are wired and tested.
