# Runtime Integration Phase 2.13 Progress

## Current goal

Connect Map Runtime Bridge with ROS2 interfaces while keeping Nav2 and localization backends isolated behind adapters.

## Completed

- Map Package lifecycle design
- Runtime Apply controller design
- Runtime state machine
- Backend adapter contracts
- Runtime Bridge node skeleton
- Runtime state publishing boundary

## Current implementation step

```
Map Manager
    |
    | ApplyMapRuntime
    v
Runtime Bridge Node
    |
    +--> Nav2 Adapter
    |
    +--> Localization Adapter
    |
    +--> Runtime State Publisher
```

## Next tasks

1. Bind agt_robot_interfaces messages/services.
2. Add ROS2 service server for map runtime apply.
3. Add runtime state publisher implementation.
4. Audit current v3 Nav2 launch and lifecycle usage.
5. Implement Nav2 lifecycle adapter against the real stack.

## Merge policy

Do not merge into main before:

- ROS2 build validation
- Runtime service test
- Nav2 adapter integration test
- Localization adapter integration test
