# Runtime Integration Phase 2.5 Progress

## Goal

Close the ROS 2 communication loop between Map Manager and Runtime Bridge.

## Completed

- Map Package lifecycle design
- Runtime Apply controller design
- Runtime backend adapter contracts
- Runtime state machine definition
- Consistency checking rules

## Current implementation target

```
Map Manager
    |
    | ApplyMapRuntime
    v
Runtime Bridge
    |
    +--> Nav2 Adapter
    |
    +--> Localization Adapter
    |
    v
Runtime State Publisher
```

## Acceptance criteria before merge to main

- ROS 2 interfaces compile
- Runtime state can be published
- Apply request reaches controller
- Backend adapters can report READY/ERROR
- Map generation mismatch blocks activation
