#!/usr/bin/env bash
set -euo pipefail

LIO_BACKEND=""
if [[ $# -gt 0 ]]; then
  if [[ $# -ne 2 || "$1" != --lio-backend ]]; then
    printf 'Usage: check_runtime.sh [--lio-backend batch_lio|fastlio2]\n' >&2
    exit 2
  fi
  LIO_BACKEND=$2
  case "$LIO_BACKEND" in
    batch_lio|fastlio2) ;;
    *) printf 'Invalid LIO backend: %s\n' "$LIO_BACKEND" >&2; exit 2 ;;
  esac
fi

EXPECTED_NODES=(
  /agt_localization_manager
  /robot_state_publisher
  /agt_pointcloud_preprocessor
)

FORBIDDEN_NODES=(/agt_obstacle_cloud_preprocessor /obstacle_cloud)
if [[ "$LIO_BACKEND" == batch_lio ]]; then
  EXPECTED_NODES+=(/agt_batch_lio_adapter /laserMapping)
  FORBIDDEN_NODES+=(/agt_fastlio_adapter /fastlio2/lio_node /batch_lio)
elif [[ "$LIO_BACKEND" == fastlio2 ]]; then
  EXPECTED_NODES+=(/agt_fastlio_adapter /fastlio2/lio_node)
  FORBIDDEN_NODES+=(/agt_batch_lio_adapter /laserMapping /batch_lio)
fi

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

for retired in "${FORBIDDEN_NODES[@]}"; do
  count=$(printf '%s\n' "${NODES[@]}" | awk -v name="$retired" '$0 == name {count++} END {print count+0}')
  if [[ "$count" -ne 0 ]]; then
    printf 'FAIL retired or unselected backend node still running: %s count=%s\n' "$retired" "$count" >&2
    failed=1
  fi
done

if [[ "$failed" -ne 0 ]]; then
  printf '\nCurrent ROS graph:\n' >&2
  printf '%s\n' "${NODES[@]}" >&2
  exit 1
fi

printf 'PASS runtime owner uniqueness\n'
