#!/usr/bin/env bash
set -eo pipefail
# A child mapping process must not retain the navigation FAST-LIO/driver overlay.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
source "$1"
shift
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
exec "$@"
