#!/usr/bin/env bash
set -eo pipefail
AGT_GATEWAY_SOURCE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/noetic/setup.bash
export PYTHONPATH="$AGT_GATEWAY_SOURCE${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m agt_field.ros1_gateway "$@"
