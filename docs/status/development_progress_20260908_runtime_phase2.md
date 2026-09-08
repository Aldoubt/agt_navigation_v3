# Development Progress 2026-09-08 Runtime Phase 2

## Completed

- Runtime Bridge architecture
- typed runtime state contract
- launch/config skeleton
- apply runtime orchestration skeleton
- adapter boundary definition

## Current architecture

```
Map Manager
    |
    v
Runtime Bridge
    |
    +--> Nav2 Adapter
    |
    +--> Localization Adapter
```

## Next implementation

1. Implement ROS2 service server for ApplyMapRuntime.
2. Add Nav2 lifecycle adapter.
3. Add Global Localization adapter.
4. Add end-to-end map generation consistency test.

## Acceptance criteria

A map selected from HMI must result in:

- same map_id
- same version
- same generation

being used by navigation and localization backends.
