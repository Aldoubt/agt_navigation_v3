# Map Runtime ROS2 Interface Design

## Goal

Define a stable communication contract between map management and runtime navigation.

## Data flow

```
Map Manager
    |
    | MapRuntimeState
    v
Map Runtime Bridge
    |
    +----------------+
    |                |
   Nav2       Global Localization
```

## Responsibilities

### Map Manager

- store map packages
- manage versions
- validate metadata
- publish available runtime candidates

### Runtime Bridge

- accept validated map generation
- coordinate component reload
- publish runtime state
- verify consistency

### Backend adapters

Nav2 and localization implementations are isolated behind adapters.

## Consistency rule

All runtime components must use the same:

- map_id
- version
- generation

A mismatch must enter ERROR state instead of silently continuing.

## Future interfaces

Topics:

- `/agt/map/status`
- `/agt/map/runtime/state`

Service:

- `/agt/map/runtime/apply`

Later additions:

- action interface for long reload operations
- recovery workflow after restart
