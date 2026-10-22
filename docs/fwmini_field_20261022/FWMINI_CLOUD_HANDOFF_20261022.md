# FW-mini V3 field integration handoff | 2026-10-22

Branch: Aldoubt/agt_navigation_v3 feature/fwmini-field-navigation-20261022
Base: experiment/greenhouse-task-continuity (4dc92547e9448f675e0da110f6c33d8e0512cc19), NOT main.
This field-tested V3 runtime includes Navigation Capability and Supervisor that the newer S7 main branch did not contain; do not merge into S7 main without a dedicated interface compatibility review.

## Tested software chain
MID360 -> FAST-LIO2 odometry -> 3D-BBS/GICP -> Localization Manager (map->odom) -> Nav2 FollowPath -> FW-mini navigation_guard -> chassis_adapter -> YHS CAN.
RViz DrawRoute/EditWaypoints/RoutePanel -> agt_rviz_patrol RouteWorkbench -> /navigation/follow_route -> agt_navigation_capability -> Nav2 /follow_path.

## Fixes collected
- Explicit string map_version for global relocalization and localization manager to prevent integer coercion of 20261022_144218.
- Explicit identity on relocalizer (map_id, map_version); matching scores alone do not make ambiguous candidates valid.
- FW-mini Supervisor reads PointCloud2 and checks Nav2 lifecycle manager + FollowPath with freshness limits, reducing false SENSOR_UNAVAILABLE/NAV2_UNAVAILABLE.
- Humble ClockType import compatibility in capability research code.
- Strict start_connection_policy=drawn_only: no hidden robot-to-route connector; start <=0.35m, lateral <=0.12m, yaw <=25deg, backward <=0.05m. All other V3 profiles retain auto_connector.
- RViz panel shows actually submitted preview length. Qt FW-mini profile tools provided as field operator console additions.

## Field assets (not committed)
map_id=fwmini_mid360; map_version=20261022_144218.
Map path: ~/ros2_ws/maps/fwmini_mid360/20261022_144218.
Mapping run: ~/ros2_ws/experiments/fwmini_live_20261022_144218.
No map PCD/PGM, rosbag, test artifacts or credentials are uploaded.

## Validation
V3 workbench pytest 45 PASS; Supervisor pytest 8 PASS.
Nav2/UI isolated ROS launch confirms FollowRoute and preview API, no motors.
FW-mini ABI isolated test: broken legacy command 16 CDR bytes; correct vendor command 20 CDR bytes, 0.2 rad/s -> 11.459 deg/s.
map->odom and global_pose_accepted observed in real runtime; final accurate route tracking and robust relocalization NOT VERIFIED.

## Remaining problems
Live controller tracking error not yet independently verified.
Local costmap warns sensor origin z < origin_z (negative LiDAR z).
MID360 ROS time domain and GNSS simulator removal still need full audit.
3D-BBS row ambiguity and false corrections still need fail-closed field validation.
Do not use automatic deployment to main.

## Layout
Field scripts: scripts/fwmini_*.sh
Field runbooks: docs/fwmini_field_20261022/
These scripts assume a historical Humble host layout under ~/ros2_ws and ~/fwmini_ws, and may need path/IP adjustments on a new machine.

## Independent build reproducibility (no live install replaced)
Humble: 4 V3 packages agt_rviz_route_tools, agt_rviz_patrol, agt_navigation_supervisor and agt_navigation_capability built successfully from this publish worktree with isolated /tmp build/install roots.
Qt plugin build PASS. The field precision run used LIVE software from pre-existing workspaces and is not to be conflated with this independent build.
