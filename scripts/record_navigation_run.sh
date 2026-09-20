#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
PROFILE=${HARDWARE_ACCEPTANCE_PROFILE:-"$REPO_ROOT/tests/acceptance/hardware_acceptance_profile.yaml"}
PROFILE_DURATION_SEC=$(awk '
  /^recording:/ {in_recording=1; next}
  in_recording && /^[^[:space:]]/ {in_recording=0}
  in_recording && $1 == "duration_sec:" {print $2; exit}
' "$PROFILE")
PROFILE_RECORD_RAW=$(awk '
  /^recording:/ {in_recording=1; next}
  in_recording && /^[^[:space:]]/ {in_recording=0}
  in_recording && $1 == "include_raw_sensors:" {print $2; exit}
' "$PROFILE")
STAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR=${1:-"$PWD/acceptance_$STAMP"}
DURATION_SEC=${ACCEPTANCE_DURATION_SEC:-${PROFILE_DURATION_SEC:-180}}
SPLIT_DURATION_SEC=${ACCEPTANCE_BAG_SPLIT_SEC:-60}
SPLIT_SIZE_BYTES=${ACCEPTANCE_BAG_SPLIT_BYTES:-536870912}
RECORD_RAW_DEFAULT=1
if [[ "$PROFILE_RECORD_RAW" == "false" ]]; then
  RECORD_RAW_DEFAULT=0
fi
BAG_DIR="$OUTPUT_DIR/navigation_bag"

if ! [[ "$DURATION_SEC" =~ ^[1-9][0-9]*$ ]]; then
  printf 'ACCEPTANCE_DURATION_SEC must be a positive integer, got: %s\n' "$DURATION_SEC" >&2
  exit 2
fi
if ! [[ "$SPLIT_DURATION_SEC" =~ ^[1-9][0-9]*$ ]]; then
  printf 'ACCEPTANCE_BAG_SPLIT_SEC must be a positive integer, got: %s\n' \
    "$SPLIT_DURATION_SEC" >&2
  exit 2
fi
if ! [[ "$SPLIT_SIZE_BYTES" =~ ^[1-9][0-9]*$ ]]; then
  printf 'ACCEPTANCE_BAG_SPLIT_BYTES must be a positive integer, got: %s\n' \
    "$SPLIT_SIZE_BYTES" >&2
  exit 2
fi
if [[ -e "$BAG_DIR" ]]; then
  printf 'Refusing to overwrite existing bag: %s\n' "$BAG_DIR" >&2
  exit 2
fi
mkdir -p -- "$OUTPUT_DIR"

{
  printf 'captured_at=%s\n' "$(date --iso-8601=seconds)"
  printf 'hostname=%s\n' "$(hostname)"
  printf 'duration_sec=%s\n' "$DURATION_SEC"
  printf 'bag_split_duration_sec=%s\n' "$SPLIT_DURATION_SEC"
  printf 'bag_split_size_bytes=%s\n' "$SPLIT_SIZE_BYTES"
  printf 'ros_distro=%s\n' "${ROS_DISTRO:-unknown}"
  printf 'rmw_implementation=%s\n' "${RMW_IMPLEMENTATION:-default}"
  printf 'robot_id=%s\n' "${ACCEPTANCE_ROBOT_ID:-not-set}"
  printf 'mid360_serial=%s\n' "${ACCEPTANCE_MID360_SERIAL:-not-set}"
  printf 'map_id=%s\n' "${ACCEPTANCE_MAP_ID:-not-set}"
  printf 'operator=%s\n' "${ACCEPTANCE_OPERATOR:-not-set}"
  printf 'git_commit=%s\n' "$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || printf 'unknown')"
  printf 'acceptance_profile=%s\n' "$PROFILE"
  printf 'acceptance_profile_sha256=%s\n' "$(sha256sum "$PROFILE" | awk '{print $1}')"
} >"$OUTPUT_DIR/run_context.txt"
df -Pk "$OUTPUT_DIR" >"$OUTPUT_DIR/disk_before.txt"
ros2 node list >"$OUTPUT_DIR/nodes.txt" 2>&1 || true
ros2 topic list -t >"$OUTPUT_DIR/topics.txt" 2>&1 || true
ros2 topic info /tf --verbose >"$OUTPUT_DIR/tf_topic_info.txt" 2>&1 || true
ros2 topic info /tf_static --verbose >"$OUTPUT_DIR/tf_static_topic_info.txt" 2>&1 || true

TOPICS=(
  /tf
  /tf_static
  /bunker_status
  /wheel/odom
  /agt/livox/points
  /agt/odometry/local
  /agt/odometry/adapter_status
  /agt/localization/status
  /agt/localization/metrics
  /agt/global_relocalization/status
  /diagnostics
  /agt/navigation/points_obstacles
  /map
  /plan
  /local_plan
  /global_costmap/costmap
  /local_costmap/costmap
  /goal_pose
  /cmd_vel_nav
  /cmd_vel
  /cmd_vel_smoothed
  /mux/cmd_vel
  /navigate_to_pose/_action/feedback
  /navigate_to_pose/_action/status
  /navigate_to_pose/_action/result
)
if [[ "${ACCEPTANCE_RECORD_RAW:-$RECORD_RAW_DEFAULT}" == "1" ]]; then
  TOPICS+=(/livox/lidar /livox/imu)
fi
if [[ "${ACCEPTANCE_RECORD_GNSS:-1}" == "1" ]]; then
  TOPICS+=(/ins/navsatfix)
fi

printf 'Recording %s topics for %s seconds to %s\n' "${#TOPICS[@]}" "$DURATION_SEC" "$BAG_DIR"
set +e
timeout --signal=INT --kill-after=60s "${DURATION_SEC}s" \
  ros2 bag record --include-hidden-topics \
  --max-cache-size 134217728 \
  --max-bag-duration "$SPLIT_DURATION_SEC" \
  --max-bag-size "$SPLIT_SIZE_BYTES" \
  -o "$BAG_DIR" "${TOPICS[@]}" \
  >"$OUTPUT_DIR/rosbag_record.log" 2>&1
bag_status=$?
set -e
if [[ "$bag_status" -ne 0 && "$bag_status" -ne 124 ]]; then
  printf 'ros2 bag record failed with status %s; see %s\n' \
    "$bag_status" "$OUTPUT_DIR/rosbag_record.log" >&2
  exit "$bag_status"
fi
if [[ ! -f "$BAG_DIR/metadata.yaml" ]]; then
  printf 'rosbag metadata was not created; see %s\n' "$OUTPUT_DIR/rosbag_record.log" >&2
  exit 1
fi
ros2 bag info "$BAG_DIR" >"$OUTPUT_DIR/bag_info.txt" 2>&1
df -Pk "$OUTPUT_DIR" >"$OUTPUT_DIR/disk_after.txt"
printf 'Recorded navigation evidence: %s\n' "$BAG_DIR"
