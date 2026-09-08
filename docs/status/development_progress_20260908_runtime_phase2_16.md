# Runtime Integration Phase 2.16 Progress

## Completed

- Runtime Bridge ROS2 interface binding started
- ApplyMapRuntime service callback added
- MapRuntimeState publisher binding added
- agt_robot_interfaces dependency boundary established

## Current flow

Map Manager

-> ApplyMapRuntime service

-> Runtime Bridge Node

-> Runtime Apply Controller

-> Nav2 Adapter / Localization Adapter

-> MapRuntimeState

## Next

- Add launch entry
- Verify ROS2 build
- Implement Nav2 lifecycle adapter
- Implement localization backend adapter
