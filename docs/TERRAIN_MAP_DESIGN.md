# Terrain Aware Map Generation Design v1.0

## Purpose

Define the boundary and architecture for converting 3D LiDAR maps into navigation assets for outdoor tracked robots.

The module is not a simple PCD to PGM converter. It is a terrain-aware map generation pipeline.

## Pipeline

```
FAST-LIO2 mapping
        |
        v
global_map.pcd
        |
        v
Terrain Map Generator
        |
        +----------------+
        |                |
 Ground Segmentation  Terrain Analysis
        |                |
 Patchwork++       Elevation/Slope
        |
        v
Obstacle Analysis
        |
        v
Navigation Map Package
        |
        v
agt_map_manager
        |
        +----------------+
        |                |
      Nav2       Localization
```

## Responsibilities

### Terrain Map Generator

Responsible for:
- PCD processing
- ground extraction
- elevation generation
- slope calculation
- obstacle generation
- navigation map export

Not responsible for:
- map version management
- active map selection
- localization

### Ground Segmentation Plugin

Interface:

```
PointCloud -> ground_cloud + non_ground_cloud
```

Initial backend:

```
Patchwork++
```

Future backends may include other terrain or semantic methods.

### Navigation and Localization Separation

Localization uses:

```
localization/global_map.pcd
```

Navigation uses:

```
navigation/map.pgm
navigation/map.yaml
```

The navigation map must never replace the localization map.

## Map Package

```
maps/
  field_x/
    v001/
      metadata.yaml
      localization/
        global_map.pcd
      navigation/
        map.pgm
        map.yaml
      terrain/
        elevation
        slope
        obstacle
      relocalization/
        assets/
```

## Design Principles

1. Preserve raw 3D information for localization.
2. Use terrain-aware processing for navigation.
3. Avoid navigation filtering affecting FAST-LIO input.
4. Support plugin based scene adaptation.
5. Keep map generation independent from map lifecycle management.
