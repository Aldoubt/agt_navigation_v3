# AGT Qt5 Field Console — single user entry point

This is the first **actual Qt5 desktop operator page** implemented in V3.
Works for **Bunker** and **FW-mini** through the same high-level ROS contract.
It is a GUI preview and task operator, not a second navigation stack.

## Open

After installing ROS2 Humble and building \`agt_operator_console\`:

\`\`\`bash
source /opt/ros/humble/setup.bash
source ~/agt_field_ws/install/navigation/setup.bash
ros2 run agt_operator_console agt_field_console --maps ~/agt_field_ws/maps
\`\`\`

To review the GUI without ROS, CAN or vehicle:

\`\`\`bash
QT_QPA_PLATFORM=offscreen ros2 run agt_operator_console agt_field_console \
  --offline-demo --maps ~/agt_field_ws/maps
\`\`\`

On a desktop, omit \`QT_QPA_PLATFORM=offscreen\` and start \`--offline-demo\`
to see the complete UI. **OFFLINE DEMO** never opens a ROS client,
sends a motion goal, or changes motor enable.

## Operator path

1. Choose \`Bunker 履带\` or \`FW-mini 舵轮\`.
2. Refresh available **published** Map Packages under the map root. Each
   selected map must have \`<id>/<version>/metadata.yaml\` and
   \`navigation/map.yaml\` with an image. This is a read-only visual
   selection; **it does not activate or hot-swap the live Nav2 map**.
3. GUI displays the PGM map in the center; route topics are drawn on top.
   The mapping workspace retains ownership of the map editor.
4. Wait until the real NavigationHealth is READY, matches the robot profile
   and selected map identity, and the RouteWorkbench reports the same map.
   FW-mini additionally needs a fresh, clear navigation guard status.
5. Choose 5 / 7 / 10 m (or 0.1–30 m custom), click
   "生成并校验路线" -> \`/agt/field/prepare_distance\`. Wait until the
   route back-end's independent \`/is_path_valid\` verdict is VALIDATED.
6. **Only then** "确认执行路线" becomes enabled. A second warning dialog
   requests human confirmation. The GUI calls \`/agt/path_tool/start\`, not
   \`/cmd_vel\`.
7. Use pause or cancel task buttons. Hardware E-stop and on-site observer
   remain required; GUI cancel is not a safety-rated E-stop.
8. Open an independently surveyed JSON acceptance result using "查看验收报告".

## Engineering boundaries

- Qt5 is the sole new GUI layer; ROS middleware remains in V3 backend.
- \`field_console_core.py\` is ROS/Qt independent. GUI uses
  \`field_console_bridge.py\` to read Health, RouteWorkbench JSON,
  FW-mini software guard state and the route; it calls typed ROS services.
- All ROS calls are asynchronous; the desktop event loop is not blocked.
- Before real use the two chassis must both expose the same V3
  \`PrepareDistanceRoute\` / RouteWorkbench services. The current
  Bunker setup does NOT automatically get that service just because
  the GUI includes Bunker as a choice. The mapping and Nav2 launch
  must be integrated and cloud/Humble tested per robot.
- The GUI does not activate a new map in Nav2; a mismatch with the active
  map is intentionally shown as blocked.
- The "一键建图" button remains DISABLED until FAST-LIO2 no-loop
  mapping -> PCD/PGM/3D-BBS assets and launch ownership are audited.
  "打开 Map Studio" opens only the mapping package's \`map_viewer\`;
  it is blocked while active navigation is observed.
- Ground/GPS factors and greenhouse row extraction are not silently
  described as completed features.
- FW-mini LaserScan guard and RPP YAML require independent ROS Humble
  simulation and physical braking validation; 1 m/s has NOT been released.
- The UI merely displays local maps; real map hash validation is still
  the backend's responsibility.

## Test plan

Offline:
\`\`\`bash
sudo apt install python3-pyqt5 python3-yaml
QT_QPA_PLATFORM=offscreen /usr/bin/python3 -m unittest discover \
  -s bringup/agt_operator_console/test -p 'test_field_console*.py' -v
\`\`\`

ROS integration (NO CAN, motion disabled):
- Valid Bunker/FW-mini map and matching NAV_READY -> prepare allowed.
- Lost/mismatched map, stale health/editor, invalid guard -> execute disabled.
- Prepare distance -> draft Path → async IsPathValid -> can_execute
  only after VALIDATED.
- Cancel/pause still available with appropriate ROS services;
  blocked/unknown map cannot send a new goal.
- Verify no GUI code publishes Twist/CtrlCmd or enables motion.
- Verify map rendering with real PGM/map.yaml, including nonzero origin.
- Run Qt GUI human mouse usability check.

Field acceptance (independent):
- 30m measured average speed >=1m/s in both trials when hardware proven.
- 5/7/10m terminal mean errors <=0.12/0.15/0.20m, five repeats.
- Stop for person/unknown front obstacle, validate blind-zone,
  physical stopping distance, hardware E-stop and human supervision.
- Both Bunker and FW-mini must pass separately; no software
  "PASS" can replace actual field evidence.
