#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE="${SCRIPT_DIR}/../src/agt_livox_tools/scripts/convert_livox_bag_format.sh"
INPUT=""; OUTPUT=""; MODE=""; SOURCE_TOPIC=""; TARGET_TOPIC=""; EXTRA=()
usage() { echo "Usage: $0 --input BAG --output BAG --mode custom_to_pc2|pc2_to_custom [options]"; }
while [[ $# -gt 0 ]]; do
  case "$1" in
    --input) INPUT="${2:?missing value}"; shift 2;;
    --output) OUTPUT="${2:?missing value}"; shift 2;;
    --mode) MODE="${2:?missing value}"; shift 2;;
    --source-topic) SOURCE_TOPIC="${2:?missing value}"; shift 2;;
    --target-topic) TARGET_TOPIC="${2:?missing value}"; shift 2;;
    *) EXTRA+=("$1"); shift;;
  esac
done
[[ -n "$INPUT" && -n "$OUTPUT" && -n "$MODE" ]] || { usage >&2; exit 2; }
case "$MODE" in
  custom_to_pc2) REAL_MODE=custom_to_pointcloud2; DEFAULT_TARGET=/livox/lidar_pc2; TYPE='livox_ros_driver2/msg/CustomMsg';;
  pc2_to_custom) REAL_MODE=pointcloud2_to_custom; DEFAULT_TARGET=/livox/lidar_custom; TYPE='sensor_msgs/msg/PointCloud2';;
  *) echo "ERROR: unsupported mode: $MODE" >&2; exit 2;;
esac
if [[ -z "$SOURCE_TOPIC" ]]; then
  INFO="$(ros2 bag info "$INPUT")"
  SOURCE_TOPIC="$(awk -v type="$TYPE" '$1 == "Topic:" && $4 == type {print $2}' <<<"$INFO" | tail -1)"
  [[ "$SOURCE_TOPIC" == /* ]] || SOURCE_TOPIC=/livox/lidar
fi
[[ -n "$TARGET_TOPIC" ]] || TARGET_TOPIC="$DEFAULT_TARGET"
exec "$BRIDGE" --input "$INPUT" --output "$OUTPUT" --mode "$REAL_MODE" --source-topic "$SOURCE_TOPIC" --target-topic "$TARGET_TOPIC" "${EXTRA[@]}"
