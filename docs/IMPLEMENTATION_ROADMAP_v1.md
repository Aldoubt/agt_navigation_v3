# AGT Navigation v3 Implementation Roadmap v1

## Goal

Evolve the current ROS 2 navigation stack into a product-oriented capability architecture while preserving the existing localization and navigation invariants.

## Frozen Architecture Rules

- `agt_localization_manager` remains the only owner of `map -> odom`.
- Batch-LIO remains the source of continuous local odometry.
- Global localization provides initialization/correction only.
- RTK/INS provides quality information and must not directly publish `map -> odom`.
- Raw LiDAR timing path must not be modified by generic filtering.

## Phase 1: Capability Stabilization

### Map Capability

Upgrade `agt_map_manager`:

- Add map save workflow.
- Add lifecycle states.
- Add map package metadata.
- Support loading the latest active map after startup.

### PointCloud Capability

Upgrade `agt_pointcloud_preprocessor`:

- Separate mapping, navigation, and localization processing modes.
- Keep self-filtering in all modes.
- Enable navigation-only rear box filtering.
- Provide dedicated localization point cloud output.

### Localization Recovery

Add recovery state handling:

- BOOT
- WAIT_ODOM
- WAIT_GLOBAL_POSE
- TRACKING
- LOST
- RECOVERING

The recovery manager should trigger global relocalization and coordinate navigation pause/resume.

## Phase 2: Map Package Standardization

Target structure:

```
maps/<scene>/<version>/
├── metadata.yaml
├── navigation/
│   ├── map.yaml
│   └── map.pgm
├── localization/
│   └── global_map.pcd
├── terrain/
│   ├── elevation.pgm
│   └── slope.pgm
└── relocalization/
    └── assets/
```

## Phase 3: Regression Testing

Create repeatable tests for:

- Mapping quality.
- Global relocalization success rate.
- Navigation goal accuracy.
- Cold-start recovery.
- CPU/GPU/resource usage.

## Acceptance Targets

- Startup without RViz initial pose.
- Automatic map loading.
- Automatic global relocalization.
- Stable Nav2 operation after localization.
- Recovery after localization loss.
