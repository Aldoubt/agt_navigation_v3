#!/usr/bin/env bash
# Unified UI entry. Never starts CAN/drivers/motion; live mode attaches to ROS.
set -eo pipefail
HERE="$(cd -- "$(dirname -- "$BASH_SOURCE")" && pwd)"
if [[ -n "$AGT_FIELD_WS" ]]; then
  WS="$AGT_FIELD_WS"
else
  WS="$(cd "$HERE/../../../.." && pwd)"
fi
ROBOT="demo"
if [[ $# -gt 0 ]]; then ROBOT="$1"; shift; fi
if [[ "$ROBOT" != "demo" && "$ROBOT" != "fwmini" && "$ROBOT" != "bunker" ]]; then
  echo "Usage: $0 [demo|fwmini|bunker] [Qt console options]" >&2
  exit 2
fi
if ! python3 -c 'import PyQt5.QtWidgets, yaml' >/dev/null 2>&1; then
  echo "Missing Qt5/YAML. On Ubuntu 22.04: sudo apt install python3-pyqt5 python3-yaml" >&2
  exit 2
fi
if [[ "$ROBOT" == "demo" ]]; then
  # Cloud review: source checkout enough; no ROS/CAN libraries imported.
  export PYTHONPATH="$HERE/../../bringup/agt_operator_console:$PYTHONPATH"
  exec python3 -m agt_operator_console.field_console \
    --offline-demo --maps "$WS/maps" "$@"
fi
ROS_SETUP="/opt/ros/humble/setup.bash"
OVERLAY="$WS/install/navigation_$ROBOT/setup.bash"
for setup in "$ROS_SETUP" "$OVERLAY"; do
  if [[ ! -f "$setup" ]]; then
    echo "Missing ROS install: $setup. Build selected platform overlay first." >&2
    exit 2
  fi
  source "$setup"
done
# Opens ONLY operator UI; separate approved launch owns drivers/nav.
exec ros2 run agt_operator_console agt_field_console \
  --robot "$ROBOT" --maps "$WS/maps" "$@"
