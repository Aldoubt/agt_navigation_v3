# AGT Navigation Architecture Baseline v0.1

## 1. Purpose

This document defines the architecture baseline for `agt_navigation_v3` before capability refactoring and productization.

Goals:

- Keep the validated LIO + global localization + Nav2 architecture stable.
- Separate algorithm pipeline from product capability layer.
- Provide a stable context baseline for future AI-assisted development.

---

## 2. Core Architecture

```
                 HMI
                  |
          Capability Layer
                  |
 ------------------------------------------------
 |              |              |                 |
Map Manager  Perception   Localization     Navigation
 |              |              |                 |
Map Package PointCloud    map->odom          Nav2
consumer     Pipeline      Owner
                  |
               Sensors
                  |
                MID360
```

---

## 3. Localization Ownership

TF ownership rules:

```
map
 |
 |  owned by localization manager
 |
odom
 |
 |  owned by local odometry
 |
base_link
```

Rules:

- Only localization manager publishes `map -> odom`.
- RTK, ICP, semantic localization and other methods provide pose hypotheses only.
- Local odometry publishes continuous motion.

---

## 4. Point Cloud Pipeline

Raw LIO path must remain unchanged.

```
MID360 / Livox driver
 |
 +---------------------------+
 |                           |
Batch-LIO path          PointCloud2 path
 |                           |
local odometry         preprocessing
                             |
              +--------------+--------------+
              |                             |
        obstacle cloud              localization cloud
              |                             |
            Nav2                    Global localization
```

Filtering rules:

- Self filtering is allowed.
- Vehicle frame filtering uses `base_link`.
- Rear masking is navigation-specific and must not damage localization information.

---

## 5. Map Package Standard

Current Map Package consumed by navigation:

```
maps/<map_id>/<map_version>/

metadata.yaml
localization/global_map.pcd
localization/relocalization/
navigation/map.yaml
navigation/map.pgm
navigation/elevation.pgm       # optional
navigation/slope.pgm           # optional
navigation/obstacle.pgm        # optional
```

Navigation-side Map Manager responsibilities:

- Discover and validate immutable map packages.
- Activate one map ID/version through `active_map.yaml`.
- Publish the selected package to runtime consumers.
- Keep localization and navigation map IDs synchronized.

FAST-LIO2, PGO, map saving, and relocalization asset generation are owned by
the separate mapping producer and are not started by this repository.

---

## 6. Capability Layer

Recommended interfaces:

### Mapping boundary

Mapping control (`start_mapping`, `stop_mapping`, `save_map`) belongs to the
separate mapping producer. V3 exposes no mapping runtime and only consumes a
validated Map Package.

### Localization

```
initialize()
relocalize()
reset()
status()
```

### Navigation

```
start_navigation()
stop_navigation()
go_to_pose()
status()
```

### Map

```
list_maps()
load_map()
save_map()
select_scene()
```

---

## 7. Test Baseline

Every change should provide:

- Build result.
- ROS graph.
- TF tree.
- Parameter snapshot.
- Runtime log.
- Test report.

Required regression tests:

1. Mapping replay test.
2. Global localization test.
3. Nav2 navigation test.
4. Hardware smoke test.

---

## 8. Current Priority

P0:

- Upgrade map lifecycle management.
- Complete point cloud preprocessing modes.
- Standardize map package.

P1:

- Capability API for HMI.
- Localization recovery state machine.

P2:

- Scene plugins.
- Automatic benchmark pipeline.
