#!/usr/bin/env bash
# Record the evidence needed to compare the legacy localization manager with
# agt_localization_ros shadow mode.  This script only records existing topics.
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <output_bag_directory>" >&2
  exit 2
fi

ros2 bag record --output "$1" \
  /agt/odometry/local \
  /agt/relocalization/pose \
  /agt/map_tracking/pose \
  /agt/map_tracking/status \
  /agt/localization/status \
  /agt/localization/metrics \
  /agt/localization/v1/shadow/state \
  /agt/localization/v1/shadow/map_odom \
  /agt/localization/v1/shadow/diagnostics \
  /tf \
  /tf_static \
  /clock
