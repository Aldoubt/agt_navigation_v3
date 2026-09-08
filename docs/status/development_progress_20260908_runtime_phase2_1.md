# Runtime Integration Progress Phase 2.1

## Completed

- Runtime apply orchestration skeleton
- Nav2 adapter boundary
- Localization adapter boundary
- Apply service node skeleton
- Map generation consistency checker skeleton

## Architecture

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

## Consistency rule

Navigation and localization assets must share:

- map_id
- version
- generation

## Next steps

1. Bind real ROS2 service interfaces.
2. Implement Nav2 lifecycle operations.
3. Implement localization backend reload.
4. Add integration test scenarios.
