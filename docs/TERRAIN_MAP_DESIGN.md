# Terrain Aware Map Generation Design v2.0

## Purpose

Define the architecture for converting AGT 3D mapping assets into navigation maps for an outdoor tracked robot operating on slopes, under trees and under significant chassis vibration.

This is not a simple `global_map.pcd -> PGM` converter. The navigation product is a 2.5D terrain representation derived from mapping assets while the original 3D localization map remains untouched.

## Authoritative mapping assets

The FAST-LIO2/PGO mapping pipeline already produces two useful representations:

```text
map.pcd / global_map.pcd       accumulated map-frame cloud
poses.txt                      optimized T_map_body for keyframes
patches/*.pcd                  body-frame keyframe clouds
```

The two representations have different uses:

- `global_map.pcd` is the localization/reference map and the fallback source for global-grid processing.
- `patches/*.pcd + poses.txt` are the preferred source for sensor/body-local algorithms such as Patchwork++.

Patchwork++ must **not** be run once on the entire accumulated global map. Its concentric-zone model, range limits and sensor-height assumptions are local to a sensor/body-centered cloud. Each keyframe patch is segmented locally and the resulting ground/non-ground points are transformed with the optimized `T_map_body` before global terrain aggregation.

## Preferred terrain pipeline

```text
FAST-LIO2 / PGO
      |
      +--------------------------+
      |                          |
      v                          v
localization/global_map.pcd   patches/*.pcd + poses.txt
      |                          |
      |                    PatchSetSource
      |                          |
      |                   GroundSegmenter
      |                    (Patchwork++)
      |                    /            \
      |                ground        non-ground
      |                    \            /
      |                     T_map_body
      |                          |
      +--------------------------+
                                 v
                         map-frame aggregation
                                 |
                   +-------------+-------------+
                   |             |             |
                   v             v             v
             ElevationBuilder  SlopeBuilder  ObstacleBuilder
                   |             |             |
                   +-------------+-------------+
                                 |
                                 v
                         Traversability fusion
                                 |
                                 v
                       Nav2 trinary occupancy
                                 |
                                 v
                         Navigation Map Package
                                 |
                                 v
                          agt_map_manager
```

## Fallback pipeline

`agt_map_converter` remains available during v0.x as an A/B regression and emergency fallback:

```text
global_map.pcd
  -> XY grid min/max-z
  -> slope / vertical-span obstacle
  -> optional trajectory free-carving
  -> Nav2 map
```

The fallback must not be removed until the terrain-aware pipeline passes the same map, localization and navigation acceptance datasets.

## Module boundaries

### PatchSetSource

Responsible for:
- reading `poses.txt`;
- pairing each pose row with `patches/<name>.pcd`;
- returning each patch in its recorded body frame plus `T_map_body`;
- validating missing/duplicate/inconsistent assets.

It does not classify ground or create grids.

### GroundSegmenter

Interface:

```text
body-frame PointCloud -> ground_cloud + non_ground_cloud
```

First adapter: Patchwork++.

Third-party headers/types stay inside the adapter. Core terrain types do not expose Eigen/PCL/Patchwork++ types.

For XYZ-only mapping patches, Patchwork++ reflected-noise removal must not assume meaningful intensity. The adapter must either provide a valid intensity source or disable intensity-dependent RNR explicitly.

### ElevationBuilder

Consumes map-frame ground points and estimates a robust ground surface. Baseline cell statistics:

- median ground height;
- point count;
- variance/robust spread;
- confidence.

The median/confidence representation replaces the old `min_z` assumption and is intended to reduce sensitivity to leaf returns, residual motion distortion and vibration-related outliers.

### SlopeBuilder

Computes slope from the valid elevation surface. Unknown/low-confidence cells remain unknown instead of being forced free.

### ObstacleBuilder

Consumes map-frame non-ground points relative to the estimated local ground surface. Obstacle evidence is based on height-above-ground and confidence rather than absolute Z.

### MapExporter

Exports the final terrain products only. It does not own map versions or active-map state.

## Navigation and localization separation

Localization uses the unmodified 3D mapping asset:

```text
localization/global_map.pcd
```

Navigation uses generated terrain assets:

```text
navigation/map.pgm
navigation/map.yaml
terrain/elevation.*
terrain/slope.*
terrain/obstacle.*
```

The navigation map must never replace or rewrite the localization map.

## Map package

```text
maps/
  field_x/
    v001/
      metadata.yaml
      localization/
        global_map.pcd
      mapping_assets/
        poses.txt
        patches/
      navigation/
        map.pgm
        map.yaml
      terrain/
        elevation.*
        slope.*
        obstacle.*
        confidence.*
      relocalization/
        assets/
```

`mapping_assets/` may be retained by policy or archived separately, but the map metadata must record whether patch-based terrain regeneration is possible.

## Patchwork++ dependency policy

- official source: `url-kaist/patchwork-plusplus`;
- first pinned baseline: v1.4.1 commit `3e6903a1d5537a4cc2ace897b0bbb98a92d6014c`;
- installed as an external native dependency, not vendored into public AGT interfaces;
- main `agt_terrain_map_generator` stays on the repository C++17 baseline;
- the Patchwork++ adapter target may use C++20 as required by the upstream library;
- if Patchwork++ is absent, the core package still builds and the adapter is unavailable explicitly rather than silently falling back.

## Initial field parameters

These are starting points, not acceptance constants:

```yaml
grid:
  resolution_m: 0.05

elevation:
  method: median

obstacle:
  min_height_m: 0.25
  max_height_m: 3.0

slope:
  max_angle_deg: 20.0
```

Patchwork++ `sensor_height`, range limits and seed thresholds must be calibrated against the actual body-frame patch origin and the measured Bunker/MID360 geometry before field acceptance.

## Large-map rule

Do not assume the entire outdoor map can always be materialized as a dense 5 cm grid in memory. Builders must be compatible with tile/chunk processing. The implementation should add a configurable tile size before enabling very large-map production use.

## Design principles

1. Preserve raw 3D information for localization.
2. Use patch-local ground segmentation when an algorithm assumes a sensor-centered cloud.
3. Aggregate terrain evidence only after applying optimized mapping poses.
4. Keep third-party algorithm types behind adapters.
5. Treat low-confidence terrain as unknown, not automatically free.
6. Keep `agt_map_converter` as a tested fallback during migration.
7. Keep map generation independent from map lifecycle management.
8. Keep navigation filtering out of the FAST-LIO2 input path.
