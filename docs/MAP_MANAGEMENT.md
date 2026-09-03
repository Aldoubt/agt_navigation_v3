# Map Management Design

## Purpose

`agt_navigation_v3` does not treat `map.yaml/map.pgm` as the primary map asset. The primary asset is a versioned Map Package. Navigation maps and localization maps are derived products from the same source.

## Map Package

Recommended structure:

```text
maps/
└── <map_id>/
    ├── metadata.yaml
    ├── localization/
    │   ├── global_map.pcd
    │   ├── submaps/
    │   └── scan_context.db
    │
    ├── navigation/
    │   ├── map.yaml
    │   ├── map.pgm
    │   ├── height.pgm
    │   ├── slope.pgm
    │   └── obstacle.pgm
    │
    └── preview.png
```

## Source of Truth

The 3D point cloud map is the source of truth:

```text
3D map
 |
 +--> global localization database
 |
 +--> Nav2 occupancy map
 |
 +--> terrain analysis
 |
 +--> visualization
```

The system must not edit a PGM and attempt to recover a 3D map.

## Map Manager

A dedicated `agt_map_manager` node will sit between HMI, Nav2 and localization.

Responsibilities:

- list available map packages
- validate package consistency
- load/unload localization database
- load/unload Nav2 map
- keep localization and navigation map IDs synchronized

A map switch is an atomic operation:

```text
select map package
        |
        +--> localization assets
        |
        +--> navigation assets
        |
        +--> runtime validation
```

## HMI Policy

The HMI displays map packages, not raw files. Operators select a named area/version instead of manually selecting YAML files.

Example:

```text
Forest Area North
Version: v3
Localization: READY
Navigation: READY
```

## Future Extensions

- map annotations
- keep-out zones
- preferred routes
- terrain restrictions
- multi-map transition areas
