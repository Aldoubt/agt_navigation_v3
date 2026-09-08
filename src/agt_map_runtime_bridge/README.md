# agt_map_runtime_bridge

## Purpose

Runtime integration layer between `agt_map_manager` and navigation/localization backends.

## Architecture boundary

Map Manager owns map assets and versions.

Runtime Bridge owns applying one validated map generation to running components.

```
Map Manager
    |
    | /agt/map/status
    v
Runtime Bridge
    |
    +--> Nav2 adapter
    |
    +--> Localization adapter
```

## Current stage

Implemented:

- map runtime state contract
- map generation tracking
- runtime state machine skeleton
- typed ROS2 interface preparation

## Next steps

1. Implement ROS2 node wrapper.
2. Add Nav2 lifecycle adapter.
3. Add Global Localization reload adapter.
4. Add runtime consistency tests.
