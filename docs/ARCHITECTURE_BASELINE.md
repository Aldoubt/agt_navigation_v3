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
PCD/PGM     PointCloud    map->odom          Nav2
Storage     Pipeline      Owner
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
MID360
 |
 +---------------------------+
 |                           |
LIO path                PointCloud2 path
 |                           |
FAST-LIO/Batch-LIO     preprocessing
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

Future map output:

```
maps/latest/

map.pcd
map.pgm
map.yaml
elevation.yaml
metadata.json
relocalization_assets/
```

Map manager responsibilities:

- Save map.
- Load active map.
- Maintain latest map.
- Support scene profiles.

---

## 6. Capability Layer

Recommended interfaces:

### Mapping

```
start_mapping()
stop_mapping()
save_map()
status()
```

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
