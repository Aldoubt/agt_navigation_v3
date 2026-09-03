# Advanced Design V2 Decisions

## 1. Terrain-aware navigation

The first implementation includes elevation awareness. Navigation map generation is not only PCD projection.

Pipeline:

```
3D PCD
  |
voxelization
  |
ground estimation
  |
+----------------+
| elevation map  |
| slope map      |
| obstacle map   |
+----------------+
  |
Nav2 occupancy/cost representation
```

The map converter must keep the possibility of terrain classification. Each cell should be able to store:

- elevation
- slope
- roughness
- obstacle confidence
- traversability score

The output can generate Nav2-compatible maps while preserving richer terrain layers.

## 2. Interactive map editing

The HMI should not edit PGM directly. The editable object is a Map Package.

A Map Package contains:

- localization source map
- navigation derived map
- terrain layers
- metadata
- version information
- preview information

The HMI should provide visualization and editing of terrain layers, forbidden areas, and map metadata.

## 3. Map Manager enters V1

A dedicated map manager is required in the first implementation.

Responsibilities:

- map package discovery
- version selection
- loading localization database
- loading Nav2 map
- synchronizing localization and navigation assets
- providing APIs for HMI

Never allow Nav2 and localization to load unrelated maps.

## 4. Controller requirement

The controller layer must support at least 50Hz command output.

The architecture must keep the controller replaceable:

```
Nav2 Controller Server
        |
        v
controller plugin
        |
        v
cmd_vel 50Hz+
        |
Bunker CAN driver
```

Controller selection is evaluated based on tracked vehicle dynamics, not only standard differential drive examples.

## 5. RTK/INS role

RTK/INS is an auxiliary global reference, not the primary localization source.

Usage:

Startup:

```
RTK/INS initial geographic reference
          |
          v
map coordinate alignment
          |
          v
LiDAR global localization validation
```

Runtime:

```
RTK/INS
  |
low-frequency global correction / health monitoring

LiDAR localization
  |
primary pose source
```

Tree occlusion means RTK availability cannot be assumed.

## 6. External library policy

Prefer integrating mature libraries through interfaces rather than copying large external source trees.

Candidates:

- elevation mapping style libraries for terrain layers
- Nav2 plugin interfaces
- map IO abstractions

Source copying is only considered when licensing, maintenance, and required modifications justify it.

## 7. Fixed-point cloud removal

Fixed robot/self point removal should be implemented as a preprocessing layer.

The architecture allows future migration of ideas from systems such as GLIM, but the first implementation should keep a clean interface:

```
raw point cloud
      |
robot/environment filtering
      |
SLAM/localization
```

Avoid coupling the entire navigation stack to one SLAM implementation.

## 8. GUI decision

Do not rewrite the HMI.

The existing Qt HMI remains the product interface. The navigation stack exposes stable ROS APIs/services so the HMI can evolve independently.

Alternative visualization/debug tools can be used during development (RViz2 and specialized debug views), but not as the final operator interface.
