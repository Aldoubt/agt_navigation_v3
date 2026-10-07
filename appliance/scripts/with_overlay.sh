#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source "$1"
shift
exec "$@"
