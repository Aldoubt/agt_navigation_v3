#!/usr/bin/env bash
set -eo pipefail

ROS_WS="$HOME/ros2_ws"
FAST_LIVO_WS="$ROS_WS/worktrees/agt-navigation-v2-fast-livo2-pinned-p02"
MAPPING_OVERLAY="$ROS_WS/install_mapping_framework"
LIVOX_CONFIG="$ROS_WS/experiments/fwmini_live_preflight_20261021/session_20261021T082953Z/config/mid360_actual_145.json"

usage() {
  echo "Usage: $0 [--check] [NEW_OUTPUT_DIR]"
  echo "Default output: $ROS_WS/experiments/fwmini_live_YYYYMMDD_HHMMSS"
}

CHECK_ONLY=0
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then usage; exit 0; fi
if [[ "${1:-}" == "--check" ]]; then CHECK_ONLY=1; shift; fi
if (( $# > 1 )); then usage >&2; exit 2; fi

[[ -f "$FAST_LIVO_WS/install/setup.bash" ]] || { echo "Missing FAST-LIVO2 overlay: $FAST_LIVO_WS/install/setup.bash" >&2; exit 2; }
[[ -f "$ROS_WS/install/setup.bash" ]] || { echo "Missing ROS workspace overlay" >&2; exit 2; }
[[ -f "$MAPPING_OVERLAY/setup.bash" ]] || { echo "Missing mapping overlay: $MAPPING_OVERLAY/setup.bash" >&2; exit 2; }
[[ -f "$LIVOX_CONFIG" ]] || { echo "Missing validated MID360 config: $LIVOX_CONFIG" >&2; exit 2; }

source /opt/ros/humble/setup.bash
source "$FAST_LIVO_WS/install/setup.bash"
source "$ROS_WS/install/setup.bash"
source "$MAPPING_OVERLAY/setup.bash"

[[ "$ROS_DISTRO" == "humble" ]] || { echo "Expected ROS 2 Humble, got: ${ROS_DISTRO:-unset}" >&2; exit 2; }
ros2 pkg prefix fast_livo >/dev/null
ros2 pkg prefix agt_mapping_bringup >/dev/null

if ! ip -4 -o addr show | grep -Fq "192.168.1.5/"; then
  echo "MID360 config expects host IP 192.168.1.5, but this address is not assigned on this host." >&2
  echo "Update the host IP in $LIVOX_CONFIG only after checking the physical sensor subnet." >&2
  exit 2
fi
if pgrep -af "[l]ivox_ros_driver2_node" >/dev/null; then
  echo "A Livox driver is already running. Stop it first; live mapping owns its own driver." >&2
  exit 2
fi

RUN="${1:-$ROS_WS/experiments/fwmini_live_$(date +%Y%m%d_%H%M%S)}"
RUN="$(realpath -m "$RUN")"
if [[ -e "$RUN" ]]; then
  echo "Refusing to reuse existing output path: $RUN" >&2
  exit 2
fi

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "MID360 config: $LIVOX_CONFIG"
echo "New live mapping output: $RUN"
echo "This launch starts MID360 + IMU + FAST-LIVO2 LIO-only, and automatically exports map_package/."
echo "It also records raw_bag/ as a backup; the map is built live, not by replaying that bag."
if (( CHECK_ONLY )); then
  echo "PREFLIGHT_ONLY=PASS (no sensors or mapping started)"
  exit 0
fi

printf "%s\n" "$RUN" > "$ROS_WS/experiments/fwmini_last_live_run.txt"
exec ros2 launch agt_mapping_bringup mapping_live_yhs_mid360.launch.py \
  livox_config:="$LIVOX_CONFIG" \
  output_dir:="$RUN" \
  mapping_backend:=fast_livo2_lio \
  lidar_topic:=/livox/lidar \
  imu_topic:=/livox/imu \
  publish_freq:=10.0 \
  frame_id:=livox_frame \
  duration_seconds:=0.0 \
  start_rviz:=true \
  keep_open:=false
