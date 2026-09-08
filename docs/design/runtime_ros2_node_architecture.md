# Runtime ROS2 Node Architecture

## Runtime Bridge Node

Responsibilities:

- Provide ApplyMapRuntime service
- Publish MapRuntimeState
- Coordinate runtime state transitions
- Call backend adapters

It does not implement:

- SLAM
- localization algorithms
- navigation algorithms

## Data Flow

```
Map Manager
    |
    | ApplyMapRuntime
    v
Runtime Bridge Node
    |
    +----------------+
    |                |
    v                v
Nav2 Adapter   Localization Adapter
    |                |
    v                v
Navigation     Global Localization
```

## Runtime State Topic

```
/agt/map/runtime/state
```

States:

- IDLE
- VALIDATING
- APPLYING
- WAITING_BACKENDS
- READY
- ERROR

## Design Rule

Map version consistency must be checked before navigation starts.
