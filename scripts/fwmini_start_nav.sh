#!/usr/bin/env bash
set -eo pipefail

ROS_WS="$HOME/ros2_ws"
FWMINI_WS="$HOME/fwmini_ws/worktrees/fwmini-nav2-agt-cmd-bridge"
MAP_ID="fwmini_mid360"

usage() {
  echo "Usage: $0 MAP_VERSION [--trial-motion]"
  echo "Example: $0 20261022_120000"
}
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then usage; exit 0; fi
if (( $# < 1 || $# > 2 )); then usage >&2; exit 2; fi
TRIAL_MOTION=false
if [[ "${2:-}" == "--trial-motion" ]]; then TRIAL_MOTION=true;
elif (( $# == 2 )); then echo "Unknown option: $2" >&2; exit 2; fi
MAP_VERSION="$1"
MAP_ROOT="$ROS_WS/maps/$MAP_ID/$MAP_VERSION"

[[ -s "$MAP_ROOT/metadata.yaml" ]] || { echo "Map package not found: $MAP_ROOT" >&2; exit 2; }
[[ -s "$MAP_ROOT/navigation/map.yaml" ]] || { echo "Missing Nav2 map YAML: $MAP_ROOT/navigation/map.yaml" >&2; exit 2; }
[[ -s "$MAP_ROOT/localization/global_map.pcd" ]] || { echo "Missing localization PCD" >&2; exit 2; }
[[ -s "$MAP_ROOT/localization/relocalization/polar_context.db" ]] || { echo "Missing Polar Context database" >&2; exit 2; }
[[ -f "$ROS_WS/install/setup.bash" && -f "$FWMINI_WS/install/nav2_trial/setup.bash" ]] || {
  echo "Missing ROS 2 or FW-mini navigation overlay." >&2
  exit 2
}

if ! pgrep -af "[l]ivox_ros_driver2_node" >/dev/null; then
  echo "MID360 driver is not running. Start it in another terminal first." >&2
  echo "See $ROS_WS/docs/FWMINI_MAPPING_LOCALIZATION_NAV_WORKFLOW_20261022.md" >&2
  exit 2
fi
if ! pgrep -af "[y]hs_can_control_node" >/dev/null; then
  echo "YHS CAN driver is not running. Start it in another terminal first." >&2
  exit 2
fi
CAN_INFO="$(ip -details link show can0 2>/dev/null || true)"
if [[ "$CAN_INFO" != *"state UP"* || "$CAN_INFO" != *"bitrate 500000"* ]]; then
  echo "can0 must be UP at 500000 bit/s before starting the navigation stack." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
source "$FWMINI_WS/install/nav2_trial/setup.bash"

echo "Map: $MAP_ID / $MAP_VERSION"
echo "Starting FAST-LIO2 + automatic 3D-BBS/GICP relocalization + Nav2."
echo "Chassis motion enabled for field trial: $TRIAL_MOTION"
echo "Nav2 global commands still need valid localization. Manual local search uses /fwmini/search_cmd_vel."
echo "The existing 0.2s watchdog, speed limits and stop service remain active."
exec ros2 launch fwmini_multi_nav_bringup fwmini_mid360_navigation.launch.py \
  map_yaml:="$MAP_ROOT/navigation/map.yaml" \
  global_map_pcd:="$MAP_ROOT/localization/global_map.pcd" \
  relocalization_assets:="$MAP_ROOT/localization/relocalization" \
  map_id:="$MAP_ID" \
  map_version:="$MAP_VERSION" \
  auto_relocalize:=true \
  start_chassis_adapter:=true \
  initial_motion_enabled:="$TRIAL_MOTION" \
  allow_local_search:=true \
  start_rviz:=true \
  use_sim_time:=true
