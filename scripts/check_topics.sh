#!/usr/bin/env bash
set -euo pipefail

topic=/agt/navigation/points_obstacles
if ! ros2 topic list | awk -v topic="$topic" '$0 == topic {found=1} END {exit !found}'; then
  printf 'FAIL missing topic %s\n' "$topic" >&2
  exit 1
fi

info=$(ros2 topic info "$topic" --verbose)
publisher_count=$(sed -n 's/^Publisher count: //p' <<<"$info" | tr -d '[:space:]')
if [[ "$publisher_count" != '1' ]]; then
  printf 'FAIL %s publisher count=%s (expected 1)\n' "$topic" "${publisher_count:-unknown}" >&2
  printf '%s\n' "$info" >&2
  exit 1
fi

if ! timeout 6s ros2 topic hz "$topic" 2>&1 | grep -q 'average rate:'; then
  printf 'FAIL %s exists but no messages arrived within 6 seconds\n' "$topic" >&2
  exit 1
fi

printf 'PASS topic %s has one live publisher\n' "$topic"
