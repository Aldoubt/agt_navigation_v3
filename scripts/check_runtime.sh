#!/usr/bin/env bash
set -euo pipefail

EXPECTED_NODES=(
  /agt_localization_manager
  /robot_state_publisher
  /agt_pointcloud_preprocessor
)

mapfile -t NODES < <(ros2 node list 2>/dev/null | sed '/^[[:space:]]*$/d')
failed=0

for expected in "${EXPECTED_NODES[@]}"; do
  count=$(printf '%s\n' "${NODES[@]}" | awk -v name="$expected" '$0 == name {count++} END {print count+0}')
  if [[ "$count" -eq 1 ]]; then
    printf 'PASS node %-34s count=1\n' "$expected"
  else
    printf 'FAIL node %-34s count=%s (expected 1)\n' "$expected" "$count" >&2
    failed=1
  fi
done

for retired in /agt_obstacle_cloud_preprocessor /obstacle_cloud; do
  count=$(printf '%s\n' "${NODES[@]}" | awk -v name="$retired" '$0 == name {count++} END {print count+0}')
  if [[ "$count" -ne 0 ]]; then
    printf 'FAIL retired node still running: %s count=%s\n' "$retired" "$count" >&2
    failed=1
  fi
done

if [[ "$failed" -ne 0 ]]; then
  printf '\nCurrent ROS graph:\n' >&2
  printf '%s\n' "${NODES[@]}" >&2
  exit 1
fi

printf 'PASS runtime owner uniqueness\n'
