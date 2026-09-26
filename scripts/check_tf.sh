#!/usr/bin/env bash
set -euo pipefail

failed=0
if ! python3 - <<'PY'
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

edges = (
    ('map', 'odom'),
    ('odom', 'base_footprint'),
    ('base_footprint', 'base_link'),
    ('base_link', 'lidar_link'),
)
rclpy.init()
node = Node('agt_tf_preflight_check')
buffer = Buffer()
listener = TransformListener(buffer, node)
pending = set(edges)
deadline = time.monotonic() + 5.0
while pending and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    pending = {
        edge for edge in pending
        if not buffer.can_transform(edge[0], edge[1], Time())
    }

for parent, child in edges:
    if (parent, child) in pending:
        print(f'FAIL TF {parent} -> {child} unavailable', file=sys.stderr)
    else:
        print(f'PASS TF {parent} -> {child}')

node.destroy_node()
rclpy.shutdown()
raise SystemExit(1 if pending else 0)
PY
then
  failed=1
fi

# A resolvable reverse path is normal TF inversion, not a graph cycle. tf2
# rejects multiple parents internally; owner uniqueness is checked separately.
nodes=$(ros2 node list --no-daemon --spin-time 2 2>/dev/null) || nodes=""
if printf '%s\n' "$nodes" | awk '$0 == "/agt_localization_manager" {n++} END {exit n == 1 ? 0 : 1}'; then
  printf 'PASS map->odom owner /agt_localization_manager count=1\n'
else
  printf 'FAIL map->odom owner count is not 1\n' >&2
  failed=1
fi

if printf '%s\n' "$nodes" | awk '$0 == "/robot_state_publisher" {n++} END {exit n == 1 ? 0 : 1}'; then
  printf 'PASS static chassis TF owner /robot_state_publisher count=1\n'
else
  printf 'FAIL robot_state_publisher count is not 1\n' >&2
  failed=1
fi

exit "$failed"
