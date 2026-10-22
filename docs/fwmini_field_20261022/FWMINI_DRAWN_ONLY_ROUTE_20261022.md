# FW-mini drawn-only RouteWorkbench (2026-10-22)

Purpose: RouteWorkbench previously inserted robot position as an extra first vertex in build_preview(), silently adding a turning connector to an operator-drawn straight line.

## New FW-mini policy
- start_connection_policy=drawn_only: path is exactly the drawn points (with 0.05m interpolation), never the robot-to-first-point connector.
- Preview requires robot-to-start <=0.35m, cross-track error <=0.12m, yaw mismatch <=25 degrees, first point not more than 0.05m behind the forward line direction.
- Preview rejects with measured reason if not aligned, and it never sends motion when rejected.
- Existing map identity, fresh TF, Nav2 IsPathValid, health READY, and execution confirmation remain unchanged.
- RViz distance caption now reads "preview route length (actually submitted)".
- Other V3 platforms keep default auto_connector.
- FW-mini DWB vy=0 stays unchanged; lateral strafe control not enabled or tuned this round.
- No motor enable actions were taken.

## Captured failure used for regression
- Drawn points (-7.04446,-1.75431) to (-9.15506,-1.57217), length 2.118m, direction 175.1 degrees.
- Robot (-7.324,-1.810), heading 95.5 degrees.
- Old path had a hidden connector heading ~63 degrees, then 175 degrees, about a 112 degree corner.
- New policy fails preview because yaw differs 79.6 degrees and first point is 0.27m behind intended travel heading.

## Use in field
Keep primary fwmini_start_nav.sh (MID360, CAN, localization, Nav2) running.
Stop ONLY the route-workbench launch terminal with Ctrl+C, then restart:

    bash ~/ros2_ws/scripts/fwmini_start_route_workbench.sh 20261022_144218

Verify after starting:

    ros2 param get /agt_rviz_path_tool start_connection_policy
    ros2 topic echo --once /navigation/health
    ros2 topic echo --once /agt/path_tool/editor_state

The first command MUST return drawn_only. Redraw a short 0.5m - 1m forward line after NAV_READY, Preview/Validate, then confirm Execute manually. Current draft in process memory may not survive workbench restart.

## Test evidence
- agt_rviz_patrol pytest: 45 passed, including exact path, invalid heading/cross-track/behind start rejection, captured route regression, preview-to-execute path identity.
- agt_rviz_patrol + agt_rviz_route_tools Humble colcon build: both PASS.
- Isolated ROS_DOMAIN_ID=211 no CAN: launch policy drawn_only, parameters correct, FollowRoute and preview services present, health NOT_READY without sensors.
- Current real Nav2/CAN and motion enable unchanged; full vehicle route follow and precision validation still pending.
