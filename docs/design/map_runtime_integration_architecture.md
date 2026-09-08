# Map Runtime Integration Architecture v1

## Purpose

This document defines the runtime application boundary after Map Manager.

Map Manager is responsible for map asset lifecycle and version selection. Runtime Integration is responsible for applying one validated map generation to navigation and localization backends.

## Architecture

```
HMI
 |
Mission Runtime
 |
Map Manager
 |
Map Runtime Bridge
 |
+-----------------------+
|                       |
Nav2 Runtime        Localization Runtime
|                       |
map_server          Global Localization
costmap             PCD / map backend
```

## Design Rules

1. A map package is immutable after release.
2. Runtime consumers never search maps independently.
3. Every backend consumes the same map generation ID.
4. Map switch requires coordinated reload.

## Runtime Flow

```
Map Manager

/agt/map/status

{
  map_id,
  version,
  generation
}

        |
        v

Map Runtime Bridge

        |
        +--> Nav2 map server reload
        |
        +--> Localization backend reload
        |
        +--> Mission state update
```

## Pending Implementation

- agt_map_runtime_bridge package
- Nav2 lifecycle reload adapter
- Localization reload adapter
- runtime acceptance test

## Acceptance Target

After implementation:

- switching maps produces deterministic runtime state
- restart can restore active map generation
- localization and navigation always use matching assets
