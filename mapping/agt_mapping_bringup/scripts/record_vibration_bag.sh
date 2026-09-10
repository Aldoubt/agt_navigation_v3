#!/usr/bin/env bash
set -euo pipefail

OUT="${1:-$HOME/agt_bags/vibration_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$OUT")"

echo "Recording vibration diagnostic bag to: $OUT"
echo "Suggested sequence: 20s stationary -> 20s idle tracks -> straight slow -> straight medium -> in-place left/right -> rough terrain."

ros2 bag record -o "$OUT" \
  /agt/sensors/lidar/custom \
  /agt/sensors/imu/data \
  /wheel/odom \
  /agt/odometry/local \
  /ins/navsatfix \
  /ins/pose \
  /ins/velocity \
  /ins/odom \
  /ins/status \
  /tf \
  /tf_static
