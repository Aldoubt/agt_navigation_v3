# agt_map_runtime_bridge Phase 2.5

Current focus:

- ROS2 service integration
- Runtime state publisher
- Adapter readiness reporting

The bridge remains backend-agnostic.

It coordinates:

```
Map Manager
    |
Runtime Bridge
    |
+-----------+
|           |
Nav2    Localization
```

It does not implement navigation or localization algorithms.
