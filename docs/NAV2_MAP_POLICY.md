# Nav2 Map Policy

## Principle

Nav2 consumes a derived navigation representation. It is not the owner of the robot's complete map knowledge.

## Runtime Architecture

```text
                 agt_map_manager
                       |
          +------------+------------+
          |                         |
          v                         v
       Nav2                      Localization

   map_server                 global_map.pcd
   costmaps                   scan_context
                              registration
```

## Costmap Design

### Global costmap

Uses:

- Nav2 static layer
- generated occupancy map

### Local costmap

Uses:

- voxel layer
- live MID360 point cloud
- obstacle updates

The robot should not rely only on the static PGM because outdoor environments change.

## Multi-map Handling

A map change must update both:

1. Nav2 navigation assets
2. 3D localization assets

Invalid combinations must be rejected.

Example invalid state:

```text
Nav2:
forest_A.pgm

Localization:
forest_B.pcd
```

## HMI Integration

The HMI should call map manager services rather than directly controlling Nav2 map_server.

Suggested operations:

- list maps
- load map package
- unload map
- validate package
- show localization readiness

## Future

Potential future layers:

- elevation layer
- terrain cost layer
- slope cost layer
- keep-out zones
- semantic regions
