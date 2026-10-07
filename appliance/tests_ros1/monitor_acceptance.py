"""Noetic monitor-only gate against an isolated ROS1 driver fixture, never CAN."""

import json
from pathlib import Path
import socket
import sys
import time
from agt_field.gateway import Framer, encode

root, trace_root = map(Path, sys.argv[1:])
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.settimeout(0.01)
end = time.monotonic() + 10
while True:
    try:
        sock.connect(str(root / "run/gateway.sock"))
        break
    except (FileNotFoundError, ConnectionRefusedError):
        assert time.monotonic() < end
        time.sleep(0.05)
sock.sendall(encode(dict(type="hello", session="monitor_fixture")))
started = time.time()
frame = Framer()
rows = []
for sequence in range(1, 31):
    sock.sendall(
        encode(
            dict(
                type="cmd_vel",
                session="monitor_fixture",
                sequence=sequence,
                linear=0.1,
                angular=0.0,
            )
        )
    )
    try:
        rows.extend(frame.feed(sock.recv(65536)))
    except socket.timeout:
        pass
    time.sleep(0.02)
sock.close()
assert any(row["type"] == "odom" for row in rows)
assert any(row["type"] == "gateway_status" and row["motion_enabled"] is False for row in rows)
assert all(row["value"] for row in rows if row["type"] == "estop")
trace = [json.loads(line) for line in (trace_root / "driver_twist.jsonl").read_text().splitlines()]
recent = [row for row in trace if row["time"] >= started]
assert recent and all(row["linear"] == 0 and row["angular"] == 0 for row in recent)
print(
    json.dumps(
        dict(
            result="PASS_MONITOR_ONLY",
            missing_calibration_blocks_motion=True,
            odom_monitoring=True,
            physical_hardware="PENDING",
        )
    )
)
