# Runtime Integration Phase 2.12 Progress

## Completed

- Map Runtime Bridge ROS2 node boundary
- Runtime apply flow entry
- Runtime state publishing interface preparation
- Map generation consistency concept

## Current architecture

```
Map Manager
    |
    | ApplyMapRuntime
    v
Runtime Bridge Node
    |
    +----------------+
    |                |
  Nav2          Localization
 Adapter          Adapter
```

## Next tasks

1. Wire agt_robot_interfaces/msg/MapRuntimeState
2. Wire ApplyMapRuntime service server
3. Add launch integration
4. Connect Nav2 lifecycle adapter
5. Connect localization backend adapter

## Merge condition

Do not merge to main until:

- ROS2 build passes
- service/topic communication verified
- Nav2 adapter tested
- localization adapter tested
