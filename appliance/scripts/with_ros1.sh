#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
exec /usr/bin/python3 -m agt_field.ros1_gateway "$@"
