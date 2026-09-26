#!/usr/bin/env bash
set -euo pipefail

topic=/agt/navigation/points_obstacles
if ! ros2 topic list --no-daemon --spin-time 2 \
    | awk -v topic="$topic" '$0 == topic {found=1} END {exit !found}'; then
  printf 'FAIL missing topic %s\n' "$topic" >&2
  exit 1
fi

info=$(ros2 topic info "$topic" --no-daemon --spin-time 2 --verbose)
publisher_count=$(sed -n 's/^Publisher count: //p' <<<"$info" | tr -d '[:space:]')
if [[ "$publisher_count" != '1' ]]; then
  printf 'FAIL %s publisher count=%s (expected 1)\n' "$topic" "${publisher_count:-unknown}" >&2
  printf '%s\n' "$info" >&2
  exit 1
fi

# `ros2 topic hz` writes through a pipe here, so its Python stdout can remain
# block-buffered until after `timeout` kills it.  That made a healthy stream
# fail this gate even while the preprocessor was publishing every scan.
if ! timeout 10s ros2 topic echo "$topic" --qos-reliability best_effort --once >/dev/null 2>&1; then
  printf 'FAIL %s exists but no message arrived within 10 seconds\n' "$topic" >&2
  exit 1
fi

printf 'PASS topic %s has one live publisher\n' "$topic"
