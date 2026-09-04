#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${REPO_ROOT}/config/field_hardware.env"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: ROS 2 Humble not found" >&2
  exit 2
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

WS_ROOT="$(realpath -m "${REPO_ROOT}/../..")"
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${WS_ROOT}/install/setup.bash"
fi

if ! ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; then
  echo "ERROR: livox_ros_driver2 not built/sourced. Run bootstrap_humble.sh first." >&2
  exit 3
fi

CONFIG_PATH="${AGT_MID360_CONFIG:-${HOME}/.ros/agt_mid360/MID360_config.json}"
python3 "${REPO_ROOT}/scripts/generate_mid360_config.py" \
  --lidar-ip "${AGT_MID360_IP}" \
  --host-ip "${AGT_LIDAR_HOST_IP:-auto}" \
  --output "${CONFIG_PATH}"

echo "Starting MID360 CustomMsg mode: lidar=${AGT_MID360_IP}, config=${CONFIG_PATH}"
exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
  -p xfer_format:=1 \
  -p multi_topic:=0 \
  -p data_src:=0 \
  -p publish_freq:=10.0 \
  -p output_data_type:=0 \
  -p frame_id:=livox_frame \
  -p lvx_file_path:='' \
  -p user_config_path:="${CONFIG_PATH}" \
  -p cmdline_input_bd_code:=''
