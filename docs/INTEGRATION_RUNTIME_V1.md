# Integration Runtime V1

This landing connects the existing AGT repositories without moving their business logic into the HMI or device drivers.

## Frozen boundaries

- HMI remains a presentation/task-editing client. Existing `/agt/task/*` placeholder endpoints are kept as a compatibility bridge.
- Nav2 owns point-to-point navigation through `/navigate_to_pose`.
- Autolabor C1 remains a capability provider. Runtime calls only `/camera_gimbal/acquire_view` and consumes its result timestamps/actual angles.
- `agt_ins_driver` remains the INS/RTK source. V1 records `/ins/navsatfix`; RTK does not own `odom` or directly drive Nav2.
- Localization remains responsible for `map -> odom`; FAST-LIO2/local odometry remains below it.

## Inspection sequence

```text
mission.yaml
   -> NavigateToPose
   -> base settle
   -> AcquireView
   -> image_stamp
   -> lookup map->base_link at image_stamp
   -> attach time-gated nearest NavSatFix + actual gimbal angles
   -> captures.csv + captures.jsonl
```

The image timestamp is the synchronization anchor. RTK association is accepted only when its timestamp is within `rtk_max_age_sec`; stale GNSS data is never silently attached to a capture. Operator cancel is forwarded to an active Nav2 or camera-gimbal action.

## Packages added

- `agt_robot_interfaces`: stable cross-repository messages/action.
- `agt_navigation_runtime`: mission orchestration, HMI placeholder bridge and V1 capture recorder.

## V1 intentionally not implemented yet

- RTK/map geodetic alignment and quality-gated GPSFactor injection.
- resume/checkpoint persistence after power loss.
- Nav2 terrain cost plugin and tracked-base controller tuning.
- automatic launch dependency supervision / lifecycle coordinator.
- HMI migration from JSON/String placeholders to `agt_robot_interfaces`.
