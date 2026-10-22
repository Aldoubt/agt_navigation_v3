# FW-mini 2026-10-22: RViz route-workbench connection

Map: fwmini_mid360 / 20261022_144218

## Software chain
RViz DrawRoute / EditWaypoints / Workbench
 -> /agt/path_tool/edit, /agt/path_tool/preview, /agt/path_tool/start
 -> agt_rviz_path_tool (RouteWorkbench)
 -> /navigation/follow_route (V3 agt_navigation_capability)
 -> Nav2 /follow_path
 -> existing /cmd_vel_nav -> /cmd_vel -> fwmini_navigation_guard -> /agt/base/cmd_vel
 -> fwmini_chassis_adapter -> /ctrl_cmd -> YHS CAN driver.

One read-only V3 agt_navigation_supervisor owns /navigation/health.
For this FW-mini field trial its platform TF is base_link -> livox_frame;
sensor topics are /livox/lidar, /livox/imu and /agt/odometry/local;
wheel diagnostic topic is /odom; global localization is /agt/localization/status.
It checks Nav2 lifecycle and map-to-base TF and will not claim READY
without the expected map ID/version and fresh sensor/localization data.

## Start
1. Bring up MID360 and YHS CAN with the previously verified scripts.
2. Start primary map/navigation:
     bash ~/ros2_ws/scripts/fwmini_start_nav.sh 20261022_144218 --trial-motion
3. In another desktop terminal with the SAME ROS_DOMAIN_ID:
     bash ~/ros2_ws/scripts/fwmini_start_route_workbench.sh 20261022_144218

The addon does not launch duplicated LiDAR, CAN, localization, map TF, Nav2,
nor change motor motion enable. Do not quit the main Nav2 launch just to
change RViz; it can run beside Nav2's default RViz.

## Usage
- New RViz window has the Chinese panel "路线工作台" with drawing/editing tools.
- Fixed frame: map.
- Draw a short route near the robot (connector limit 1 m).
- Click Preview/Validate first, check that the path is clear and correct.
- Execute only unlocks if /navigation/health is READY, /is_path_valid passes,
  the preview and map identity are unchanged, and the user confirms the popup.
- Pause / Resume / Cancel use the established action cancellation path.
- Observe desired route /agt/path_tool/route and tracked /agt/path_tool/lio_path,
  alongside wheel trail /agt/path_tool/wheel_path (vendor odometry /odom).
- For precision claims compare with independent survey / reference measurements.

## Diagnostics
  ros2 node list | grep -E 'agt_navigation_supervisor|agt_navigation_capability|agt_rviz_path_tool'
  ros2 action list -t | grep -E '/navigation/follow_route|/follow_path'
  ros2 service list -t | grep -E '/is_path_valid|/agt/path_tool/'
  ros2 topic echo --once /navigation/health
  ros2 topic echo --once /agt/path_tool/editor_state

When execution is disabled check /navigation/health last_error_code first;
Nav2 /is_path_valid is provided by planner_server in Humble. Never forge
navigation health or use fake map -> odom transforms.

## Verification as of this round
- Rebuilt in Humble: agt_navigation_interfaces, agt_robot_interfaces,
  agt_navigation_supervisor, fwmini_multi_nav_bringup.
- Patched rclpy ClockType import to the Humble-supported API.
- Domain 229, localhost only, without motors: 3 nodes started,
  FollowRoute and S2 workbench services appeared; health stayed NOT_READY.
- Domain 230, localhost only: map version mismatch rejected during launch,
  invalid FollowRoute path aborted with INVALID_ROUTE, no motor driver.
- Source trees retain previous unrelated dirty/untracked files; no reset,
  no git commit / merge, no real vehicle route execution done.
