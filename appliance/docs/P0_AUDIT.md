# P0 source audit — 2026-10-07

Workspace started empty. The connected Windows workspace connector returned HTTP 404. GitHub installed-repository discovery returned Aldoubt/agt_navigation_v3 with push permission; clone remote, main history and code confirmed V3. No URL was inferred from a name. Fresh clones were clean; no existing dirty work was modified.

| REPOSITORY | REMOTE | CURRENT_BRANCH at audit | BASE_HEAD | DIRTY_STATE |
|---|---|---|---|---|
| Navigation | https://github.com/Aldoubt/agt_navigation_v3.git | main | e27abeb3fd63280ee8e48a47966e06588f8eebd8 | clean |
| Mapping | https://github.com/Aldoubt/agt-lio-pgo-mapping.git | main | 6c40ff8bedab472b5cb0b914088fec1ecd566630 | clean |
| Qt | https://github.com/chengyangkj/Ros_Qt5_Gui_App.git | main | b0825e3cba3e7186cba8a6b83ff230be37c8b1fb | clean |

Git remote -v, branch, status --short and log were inspected. Navigation uses feature/yhs-field-appliance-v1. Mapping needs no core changes. Qt uses a local feature branch, remote renamed upstream; no writable fork was found. Its changes are distributed as a patch pinned to upstream, never pushed to upstream.

## Mapping source evidence

`.repos` pins Aldoubt/fast-lio2 7f664a66ae7b3b683dddf7f5322c1ea55a3c8141. Its `pgo/src/pgo_node.cpp` SaveMaps (lines 245–325) writes **body_cloud unchanged** to patches/i.pcd and r_global/t_global to both poses files; final map transforms these same clouds using those poses. `pgo/include/pgo/pgo.hpp` / addKeyPose uses configured key_pose_delta_deg/key_pose_delta_trans. Optimization updates global poses, not the body cloud. Thus patch coordinates are body; poses are post-PGO T_map_body, quaternion wxyz. No spatial slicing is necessary or permitted.

`backends/agt_pgo_backend/src/pgo_backend_node.cpp` calls /pgo/save_maps. `artifacts/agt_mapping_artifacts/.../artifact_writer.py` wraps optimized PGO output, while `map_package_exporter.py` publishes a distinct mapping_source contract. Artifact checksums cover manifest; published package manifest embeds checksums excluding itself. These formats MUST NOT be confused.

Formal live entry: scripts/run_mid360_live_mapping.sh -> agt_mapping_bringup.cli -> mapping_live_yhs_mid360.launch.py. Session finish: /mapping/session/finish Trigger. Existing session_runtime.py waits for pipeline readiness, draining/export, full checksums and nonempty PCD; failures never attest verification. Existing process supervision handles finish and teardown. Wheel odometry is not an LIO/PGO input; its absence is a mapping warning.

2D: exporters/agt_pcd2grid_exporter (C++), body patches + timed optimized poses supported. Review: apps/agt_map_studio/src/main.cpp --review-package/--review-map/--review-output; lightweight review changes raster/keepout only. MainWindow.cpp confirm_mapping_review writes status: confirmed and source_mapping_package. General MapStudio 3D editing is not used for appliance review. Existing map-release workflow embeds workstation path ~/ros2_ws/experiments and Bunker defaults; appliance composes the existing executables with external YHS config instead of inheriting these defaults.

## Navigation evidence

See CURRENT_LOCALIZATION_ASSET_CONTRACT.md. AGENTS.md contains old Bunker hardware constants and V1 scope. This user's explicit YHS task supersedes that old scope. No Bunker hardware values are used as YHS defaults. Sole map->odom owner remains agt_localization_manager. bringup/agt_base_control/.../cmd_vel_guard.py requires fresh LocalizationStatus and guards both Nav2 and HMI velocity, output /mux/cmd_vel. Mapping/manual driving uses physical remote until that existing localization gate can be satisfied; it is not disabled for HMI convenience.

## Qt evidence (actual source, not README)

- src/app/widgets/nav_goal_widget.*: pose/name editor, theta UI in degrees, stored radians.
- src/app/widgets/nav_goal_table_view.*: topology names, status/delete/run; StartTaskChain uses QtConcurrent distance polling (not Nav2 action result). Appliance execution must be delegated to ROS2 executor.
- src/basic/map/topology_map.h: PointInfo x/y/theta/name/type, routes adjacency and RouteInfo; nlohmann JSON serialization.
- src/common/config/task_chain.h: TaskChain vector<PointInfo>, JSON route persistence, originally no map bundle identity or dwell.
- No task_processor source exists at this revision; equivalent queue logic is StartTaskChain.
- src/app/display/manager/display_manager.* / view_manager.*: QGraphicsView maps, topology editing, path and robot pose.
- src/channel/ros2/rclcomm.*: OccupancyGrid, TF, goal PoseStamped /goal_pose, initialpose fallback, speed topic default /cmd_vel (appliance must change to /agt/hmi/cmd_vel).
- src/common/config/config_manager.* and src/app/widgets/display_config_widget.*: configuration.
- src/app/mainwindow.cpp: map load/edit and legacy multi-nav wiring.

LICENSE file is GNU GPL Version 2, June 1991 text; record facts and distribution review in THIRD_PARTY_NOTICES.md. No legal compatibility conclusion is claimed.

## ROS1 YHS

No YHS driver exists in these cloned trees or the initial workspace. Do not reuse Bunker ROS2 driver. Driver model, topic/status type, CAN, bitrate, watchdog and ROS master remain CONFIG_REQUIRED. Explicit gateway uses typed standard-message contracts, status adapter parameterization and no TF publishing. Physical driver remains ROS1.
