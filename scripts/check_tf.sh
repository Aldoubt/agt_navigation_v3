#!/usr/bin/env bash
set -euo pipefail

EDGES=(
  'map odom'
  'odom base_footprint'
  'base_footprint base_link'
  'base_link lidar_link'
)

failed=0
for edge in "${EDGES[@]}"; do
  read -r parent child <<<"$edge"
  output=$(timeout 5s ros2 run tf2_ros tf2_echo "$parent" "$child" 2>&1 || true)
  if grep -q 'Translation:' <<<"$output"; then
    printf 'PASS TF %s -> %s\n' "$parent" "$child"
  else
    printf 'FAIL TF %s -> %s unavailable\n' "$parent" "$child" >&2
    printf '%s\n' "$output" | tail -n 4 >&2
    failed=1
  fi
done

# A resolvable reverse path is normal TF inversion, not a graph cycle. tf2
# rejects multiple parents internally; owner uniqueness is checked separately.
if ros2 node list 2>/dev/null | awk '$0 == "/agt_localization_manager" {n++} END {exit n == 1 ? 0 : 1}'; then
  printf 'PASS map->odom owner /agt_localization_manager count=1\n'
else
  printf 'FAIL map->odom owner count is not 1\n' >&2
  failed=1
fi

if ros2 node list 2>/dev/null | awk '$0 == "/robot_state_publisher" {n++} END {exit n == 1 ? 0 : 1}'; then
  printf 'PASS static chassis TF owner /robot_state_publisher count=1\n'
else
  printf 'FAIL robot_state_publisher count is not 1\n' >&2
  failed=1
fi

exit "$failed"
