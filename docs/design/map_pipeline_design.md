# Map Generation Pipeline Design v1

## 1. Overview

The map generation process is treated as a reproducible pipeline instead of manual conversion.

```
FAST-LIO2 / SLAM output
        |
        v
Raw Point Cloud
        |
        v
Preprocess
        |
        +--> Localization Map
        |
        +--> Navigation Map
        |
        +--> Terrain Map
        |
        v
Map Package
```

## 2. Localization Map

Purpose:

- global relocalization
- point cloud matching

Typical assets:

- PCD global map
- octomap

## 3. Navigation Map

Purpose:

- Nav2 planning
- costmap initialization

Generation should consider:

- ground filtering
- obstacle height
- slope
- robot footprint
- resolution

## 4. Pipeline Configuration

Every generated map should record:

- input source
- filter parameters
- voxel size
- ground segmentation method
- projection parameters
- occupancy thresholds

Example:

```
generation/pipeline.yaml
```

## 5. Terrain Extension

Reserved outputs:

- elevation map
- slope map
- traversability map

## 6. Validation

Before activation:

- file integrity check
- coordinate consistency check
- localization/navigation asset consistency check
- robot footprint compatibility check
