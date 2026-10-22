# V3 relative-distance ROS 2 service (P1a)

Package: \`agt_rviz_patrol\` workbench, ROS2 Humble.
Interface: \`agt_navigation_interfaces/srv/PrepareDistanceRoute\`.
Service: \`/agt/field/prepare_distance\`.

Example **only after loading the selected map and getting NAV_READY**:

\`\`\`bash
ros2 service call /agt/field/prepare_distance \
  agt_navigation_interfaces/srv/PrepareDistanceRoute \
  "{distance_m: 7.0, expected_map_id: fwmini_mid360, expected_map_version: '20261022_144218'}"
\`\`\`

Responses distinguish **PREVIEW REQUESTED** from **VALIDATED**. The service
creates a two-vertex straight line in the current \`map\` frame. Map TF must
be fresh. The selected map version must match the request AND NavigationHealth,
and the workbench must use \`start_connection_policy=drawn_only\`.

The existing \`/is_path_valid\` asynchronous service must subsequently approve
the actual sampled path; the workbench state must display VALIDATED and
can_execute=true. Only an independent **operator confirmation of Execute**
may call \`/agt/path_tool/start\`. The distance service NEVER sends
\`FollowRoute\` or \`cmd_vel\`.

This is a **field test service API**, not a safety-validated 1 m/s controller.
The obstacle stop chain, collision monitor, operator UI distance buttons,
measured error reports, and cloud Humble build are still pending.

Tests: \`navigation/nav2/agt_rviz_patrol/test/test_workbench.py\`.
