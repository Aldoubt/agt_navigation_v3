"""Exercise the real Noetic endpoint against the isolated standard-message fixture."""

import json
import socket
import sys
import time
from pathlib import Path
from agt_field.gateway import Framer, encode

root = Path(sys.argv[1])
sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.settimeout(0.005)
deadline = time.monotonic() + 10
while True:
    try:
        sock.connect(str(root / "run/gateway.sock"))
        break
    except (FileNotFoundError, ConnectionRefusedError):
        if time.monotonic() >= deadline:
            raise
        time.sleep(0.05)
sock.sendall(encode(dict(type="hello", session="noetic_protocol_test")))
frame = Framer()
samples = []
sequence = 0
started_real = time.time()
started = time.monotonic()
next_send = started
while time.monotonic() - started < 1.3:
    now = time.monotonic()
    if now < started + 0.65 and now >= next_send:
        sequence += 1
        sock.sendall(
            encode(
                dict(
                    type="cmd_vel",
                    session="noetic_protocol_test",
                    sequence=sequence,
                    linear=0.1,
                    angular=0.0,
                )
            )
        )
        next_send = now + 0.02
    try:
        for row in frame.feed(sock.recv(65536)):
            if row["type"] == "odom":
                samples.append((time.monotonic(), row["message"]))
    except socket.timeout:
        pass
sock.close()
assert len(samples) >= 40, len(samples)
frequency = (len(samples) - 1) / (samples[-1][0] - samples[0][0])
assert 35 < frequency < 65, frequency
msg = samples[-1][1]
assert msg["header"]["frame_id"] == "test_wheel_odom"
assert msg["child_frame_id"] == "test_base"
assert msg["pose"]["pose"]["position"]["x"] == 2.4
assert msg["pose"]["covariance"] == [0.3] * 36
assert msg["twist"]["covariance"] == [0.2] * 36
trace = [json.loads(line) for line in (root / "driver_twist.jsonl").read_text().splitlines()]
assert any(row["linear"] > 0 and row["time"] >= started_real for row in trace)
assert trace[-1]["linear"] == 0
print(
    json.dumps(
        dict(
            result="PASS_NOETIC_SOFTWARE_ONLY",
            odom_rate_hz=frequency,
            odom_frame_pose_covariance=True,
            cmd_vel_transfer=True,
            stale_command_zero=True,
            physical_hardware="PENDING",
        )
    )
)
