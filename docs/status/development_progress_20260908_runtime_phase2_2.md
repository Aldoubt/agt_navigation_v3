# Map Runtime Integration Progress Phase 2.2

## Completed

- Runtime Apply controller skeleton
- Runtime consistency checker
- Nav2 lifecycle adapter boundary
- Localization runtime adapter boundary

## Architecture rule

Map Manager does not directly control Nav2 or localization backend.
All runtime changes go through Runtime Bridge.

```
Map Manager
    |
    v
Runtime Bridge
    |
    +-- Nav2 Adapter
    |
    +-- Localization Adapter
```

## Next steps

1. Bind ROS2 service `/agt/map/runtime/apply`
2. Publish `/agt/map/runtime/state`
3. Implement real Nav2 lifecycle transitions
4. Implement Global Localization map reload
5. Add integration tests
