# Map Runtime Integration Phase 2.3 Progress

## Goal

Complete ROS2 communication binding between Map Manager and Runtime Bridge.

## Completed

- Runtime Apply controller
- Map consistency checker
- Nav2 adapter boundary
- Localization adapter boundary
- Runtime state machine

## Current implementation target

```
Map Manager
    |
    | ApplyMapRuntime
    v
Runtime Bridge
    |
    +-- Nav2 Lifecycle Adapter
    |
    +-- Localization Runtime Adapter
```

## Next steps

1. Implement ROS2 service server.
2. Publish runtime state topic.
3. Connect Nav2 lifecycle services.
4. Connect localization backend loading.
5. Run map version consistency tests.

## Merge policy

Do not merge into main until:

- ROS2 build passes
- Runtime integration test passes
- Nav2 and localization adapters have real backend bindings.
