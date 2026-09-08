# agt_map_runtime_bridge

## Purpose

Runtime integration layer between `agt_map_manager` and navigation/localization backends.

## Current stage

Implemented:

- subscribe to validated map status
- publish runtime application state
- keep Map Manager independent from Nav2/localization implementations

## Next steps

1. Add typed ROS2 interfaces instead of String transport.
2. Add Nav2 lifecycle adapter.
3. Add Global Localization reload adapter.
4. Add runtime consistency checks using map generation IDs.
