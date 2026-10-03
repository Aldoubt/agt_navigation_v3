# Map Manager Architecture Baseline v1

## 1. Purpose

This document freezes the architecture boundary of the navigation system map management layer.

The goal is to solve:

- localization map and navigation map version mismatch
- manual map file management problems
- HMI editing safety
- multi-map deployment and restart recovery

## 2. Architecture Position

Map Manager manages map package lifecycle between the Mapping producer, HMI, localization and Nav2. The Mapping producer owns map generation; Map Manager validates and publishes its completed assets.

```
HMI
 |
Mission Runtime
 |
Map Manager
 |
+----------------+
|                |
Localization     Navigation
|                |
Global           Nav2
Localization
```

## 3. Map Package Concept

A map is not a single file. It is an immutable package.

Recommended structure:

```
maps/
  warehouse_A/
    v001/
      metadata.yaml
      localization/
        global_map.pcd
        octomap.bt
      navigation/
        map.yaml
        map.pgm
      terrain/
        elevation.yaml
      generation/
        pipeline.yaml
      quality/
        report.yaml
```

## 4. Metadata Contract

metadata.yaml defines:

- map identity
- version
- coordinate frame
- localization assets
- navigation assets
- generation information
- quality information

## 5. Map Lifecycle

```
Mapping producer output
  |
candidate package
  |
validate and promote
  |
select / active
  |
archive
```

Released maps are immutable.

HMI editing must use an edit session and publish a new map version.

## 6. ROS2 Interfaces

Topics:

- /agt/map/status

Services:

- /agt/map/list
- /agt/map/load
- /agt/map/edit/start
- /agt/map/edit/publish
- /agt/map/edit/cancel

Map generation is not a Map Manager action. The producer supplies a completed candidate package; Map Manager validates and promotes it.

## 7. Runtime Binding

Map Manager selects a single map generation.

The same generation must be consumed by:

- Global Localization backend
- Nav2 map server
- costmap layers
- mission runtime

## 8. Future Extensions

Reserved interfaces:

- terrain map
- elevation map
- traversability map
- AI map quality analysis
