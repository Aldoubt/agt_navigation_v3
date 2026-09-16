#!/usr/bin/env bash
set -euo pipefail
output_dir=${1:?usage: record_controller_baseline.sh OUTPUT_DIR}
mkdir -p "$output_dir"
ros2 topic info -v /cmd_vel
ros2 topic info -v /cmd_vel_smoothed
ros2 topic info -v /mux/cmd_vel
ros2 bag record -o "$output_dir" \
  /tf /tf_static /agt/odometry/local /wheel/odom /plan /local_plan \
  /navigate_to_pose/_action/feedback /cmd_vel /cmd_vel_smoothed /mux/cmd_vel \
  /agt/navigation/points_obstacles /local_costmap/costmap \
  /local_costmap/costmap_updates /local_costmap/costmap_raw \
  /local_costmap/published_footprint /global_costmap/costmap \
  /agt/localization/status /bunker_status
