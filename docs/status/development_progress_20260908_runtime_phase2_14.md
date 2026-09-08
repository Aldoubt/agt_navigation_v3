# Runtime Integration Phase 2.14 Progress

## Completed

- Map Runtime Bridge node boundary
- Runtime state handling entry
- Apply runtime workflow entry
- Adapter isolation design

## Current

ROS2 interface binding preparation.

Target interfaces:

- `/agt/map/runtime/apply`
- `/agt/map/runtime/state`

The node layer should depend on `agt_robot_interfaces` and keep backend adapters isolated.

## Next

1. Add ROS2 service server binding
2. Add RuntimeState publisher
3. Connect Nav2 lifecycle adapter
4. Connect localization backend adapter

## Merge policy

Do not merge into main until ROS2 build and runtime integration tests pass.
