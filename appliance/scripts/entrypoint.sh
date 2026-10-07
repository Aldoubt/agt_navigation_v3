#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/nav_ws/install/setup.bash
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
if [[ "${1:-}" == "hmi" ]]; then
  export AGT_FIELD_SOCKET=/data/run/appliance.sock
  export AGT_HMI_CONFIG=/data/profiles/yhs/hmi_config.json
  export LD_LIBRARY_PATH="/opt/hmi_build/lib:/opt/hmi_build:${LD_LIBRARY_PATH:-}"
  mkdir -p /data/logs/hmi
  cd /data/logs/hmi
  exec /opt/hmi_build/ros_qt5_gui_app
elif [[ "${1:-}" == "mock" ]]; then
  exec python3 -m agt_field.server --data-root /data --profile /data/profiles/mock_yhs --mock
elif [[ "${1:-}" == "runtime" || $# == 0 ]]; then
  exec ros2 run agt_mission_executor mission_executor --data-root /data --profile /data/profiles/yhs
else
  exec "$@"
fi
