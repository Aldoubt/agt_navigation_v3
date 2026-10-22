#!/usr/bin/env bash
# Start drawing/preview/execution UI beside an ALREADY running FW-mini nav stack.
# This script does not enable/disable motion and sends no goals.
# ROS 2 Humble setup scripts read variables that may not yet be defined.
# Keep fail-fast/pipefail, but do not enable nounset while sourcing ROS.
set -eo pipefail
if [[ $# -ne 1 || "$1" == -* ]]; then
  echo "Usage: $0 MAP_VERSION (e.g. 20261022_144218)" >&2
  exit 2
fi
version="$1"
map_root="$HOME/ros2_ws/maps/fwmini_mid360/$version"
test -s "$map_root/metadata.yaml" || { echo "No map: $map_root" >&2; exit 2; }
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HOME/fwmini_ws/worktrees/fwmini-nav2-agt-cmd-bridge/install/nav2_trial/setup.bash"
exec ros2 launch fwmini_multi_nav_bringup fwmini_route_workbench.launch.py   map_package:="$map_root" map_id:=fwmini_mid360 map_version:="$version"   start_rviz:=true use_sim_time:=true
