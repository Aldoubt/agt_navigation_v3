# Runtime Integration Phase 2.11 Progress

## Completed

- Map Package lifecycle design
- Runtime State Machine
- Runtime Apply Controller design
- Backend adapter contracts
- Runtime Bridge ROS2 node skeleton
- Runtime state publishing placeholder

## Current Architecture

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

## Next Implementation

1. Wire agt_robot_interfaces service and message types
2. Implement `/agt/map/runtime/apply`
3. Publish `/agt/map/runtime/state`
4. Inspect current v3 Nav2 launch structure
5. Implement real Nav2 lifecycle adapter

## Merge Policy

Do not merge into main until:

- ROS2 build passes
- service communication tested
- Nav2 backend integrated
- localization backend integrated
