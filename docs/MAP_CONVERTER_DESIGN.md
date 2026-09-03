# 3D Map To Navigation Map Converter Design

## Goal

Convert the FAST-LIO2 generated 3D point cloud map into Nav2 compatible navigation layers while preserving outdoor terrain information.

The converter must not perform simple point projection only.

## Pipeline

```text
global_map.pcd
      |
      v
voxelization
      |
      v
ground estimation
      |
      +----------------+
      |                |
      v                v
height analysis    slope analysis
      |                |
      +----------------+
               |
               v
      traversability decision
               |
               v
      Nav2 occupancy export
```

## Cell Information

Each grid cell should internally maintain:

- minimum height
- maximum height
- average height
- height variance
- point density
- slope estimate
- traversability state

## Terrain Rules

The converter must distinguish:

### Traversable slope

Continuous height change within tracked vehicle capability.

### Non-traversable obstacle

Examples:

- sudden height discontinuity
- vertical structures
- chassis-height blocking objects

## Vegetation Handling

Outdoor vegetation must not be treated as a simple occupied projection.

The converter should consider:

- object height above ground
- vertical continuity
- density distribution
- ground contact

This avoids converting tree canopies into full blocking walls while retaining trunks and true obstacles.

## V1 Implementation

Start with:

- PCD input
- voxel grid
- local ground estimation
- height threshold
- slope threshold
- occupancy export

Advanced terrain semantics can be added later.

## Outputs

```text
navigation/
├── map.yaml
├── map.pgm
├── height.pgm
├── slope.pgm
└── obstacle.pgm
```
