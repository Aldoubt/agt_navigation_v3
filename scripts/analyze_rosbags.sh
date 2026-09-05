#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG_ROOT="$(cd "${REPO_ROOT}/../rosbag" && pwd)"
OUT="${REPO_ROOT}/docs/ROSBAG_ANALYSIS.md"
TMP="$(mktemp)"; trap 'rm -f "$TMP"' EXIT
for bag in "${BAG_ROOT}"/*; do [[ -d "$bag" ]] || continue; echo "### $(basename "$bag")" >> "$TMP"; ros2 bag info "$bag" >> "$TMP"; echo >> "$TMP"; done
cat > "$OUT" <<'EOF'
# ROS bag analysis

Generated with `ros2 bag info`. The bags use `/agt/sensors/...` names rather than live `/livox/lidar` and `/livox/imu` names.

| bag | LiDAR format | Fast-LIO2 / Batch-LIO | BBS/GICP |
|---|---|---|---|
| bunker_mid360_mapping_20260901_205036 | CustomMsg (`/agt/sensors/lidar/custom`) | YES | YES after CustomMsg→PointCloud2 |
| bunker_mid360_mapping_20260901_211105 | CustomMsg plus PointCloud2 (`/agt/commissioning/mapping/registered_points`) | YES, use CustomMsg | YES, use recorded PointCloud2 |

Both bags contain `sensor_msgs/msg/Imu` on `/agt/sensors/imu/data`. Neither contains exact `/livox/lidar` or `/livox/imu`; remap/configure the consumer. Reverse conversion is prohibited for LIO unless `offset_time` or per-point `timestamp` is present; otherwise it is geometry-only.

## Raw output

EOF
cat "$TMP" >> "$OUT"
echo "Wrote $OUT"
