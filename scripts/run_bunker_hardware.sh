#!/usr/bin/env bash
# The one lifecycle owner for Bunker, MID360/IMU, robot model and optional C1.
# Keep this terminal alive; navigation/inspection never launch these drivers.
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS_ROOT=$(cd -- "$SCRIPT_DIR/../../.." && pwd)
ROBOT_CONFIG=bunker_inspection
CAMERA=false
RTK=false
DRY_RUN=false
usage() {
  cat <<'EOF'
Usage: scripts/run_bunker_hardware.sh [--robot-config ID|PATH] [--camera] [--rtk] [--dry-run]
Starts the ONLY physical hardware owner; keep this terminal running. --camera
is required for inspection; navigation can attach with or without C1. --rtk is
metadata only and does not seed localization. Ctrl+C stops hardware separately
from navigation. --dry-run validates configuration without starting any drivers.
EOF
}
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robot-config) ROBOT_CONFIG=${2:?--robot-config requires a config}; shift 2 ;;
    --camera) CAMERA=true; shift ;;
    --rtk) RTK=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown hardware argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

# Resolve the SAME installed whole-robot configuration as the launch file.
source /opt/ros/humble/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u
CONFIG_TOOL="$WS_ROOT/install/agt_robot_bringup/share/agt_robot_bringup/tools/robot_config.py"
[[ -f "$CONFIG_TOOL" ]] || { echo "Missing built robot configuration resolver" >&2; exit 2; }
RESOLVED=$(python3 "$CONFIG_TOOL" resolve --robot-config "$ROBOT_CONFIG" --robot bunker_v1 \
  --set "enable_camera_gimbal=$CAMERA" --set "enable_rtk=$RTK") || exit 2
[[ "$RESOLVED" == *'base_adapter=bunker'* && "$RESOLVED" == *'enable_mid360=true'* \
  && "$RESOLVED" == *'enable_bunker_can=true'* ]] || {
  echo 'Refusing to start an incomplete Bunker hardware set' >&2; exit 2;
}
printf '%s\n' "$RESOLVED"
LAUNCH=(ros2 launch agt_robot_bringup robot_hardware.launch.py
  "robot_config:=$ROBOT_CONFIG" robot:=bunker_v1
  "enable_camera_gimbal:=$CAMERA" "enable_rtk:=$RTK")
if [[ "$DRY_RUN" == true ]]; then
  printf 'DRY_RUN: '; printf '%q ' "${LAUNCH[@]}"; printf '\n'
  exit 0
fi

# An existing physical or model owner is a hard error: never launch duplicates.
NODES=$(timeout 8 ros2 node list --no-daemon) || {
  echo 'ROS graph unavailable; no hardware started' >&2; exit 2;
}
for node in /robot_state_publisher /bunker /livox_lidar_publisher \
            /camera_gimbal/capability /cv_camera0/camera /pantilt_camera_serial0/driver; do
  if grep -Fxq "$node" <<<"$NODES"; then
    echo "Hardware/model owner already present ($node); refusing duplicate startup" >&2
    exit 2
  fi
done
echo 'Starting one external Bunker hardware owner; stop it in THIS terminal only.'
exec "${LAUNCH[@]}"
