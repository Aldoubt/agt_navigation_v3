"""Gateway framing/watchdog used by actual ROS1 and ROS2 endpoints."""

import json
import math
import time

from .contracts import ContractError

MAX_MESSAGE = 65536


class Framer:
    def __init__(self):
        self.buffer = b""

    def feed(self, data):
        self.buffer += data
        if len(self.buffer) > MAX_MESSAGE:
            raise ContractError("gateway frame too large")
        records = []
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            records.append(json.loads(line))
        return records


def encode(data):
    payload = json.dumps(data, separators=(",", ":"), allow_nan=False).encode() + b"\n"
    if len(payload) > MAX_MESSAGE:
        raise ContractError("gateway message too large")
    return payload


class Watchdog:
    def __init__(self, timeout, now=time.monotonic):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ContractError("watchdog timeout must be configured and positive")
        self.timeout, self.now = timeout, now
        self.connected = False
        self.last_rx = None
        self.sequence = -1
        self.velocity = (0.0, 0.0)
        self.session = None
        self.estop = True

    def connect(self, session):
        self.disconnect()
        self.connected, self.session = True, session
        self.sequence = -1

    def disconnect(self):
        self.connected = False
        self.last_rx = None
        self.velocity = (0.0, 0.0)
        self.estop = True

    def command(self, session, sequence, linear, angular):
        if (
            not self.connected
            or session != self.session
            or type(sequence) is not int
            or sequence <= self.sequence
        ):
            raise ContractError("stale gateway session/sequence")
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (linear, angular)):
            raise ContractError("nonfinite gateway velocity")
        self.sequence = sequence
        self.last_rx = self.now()
        # Even while estopped, discard velocity intent rather than replay after recovery.
        self.velocity = (float(linear), float(angular)) if not self.estop else (0.0, 0.0)

    def output(self):
        if (
            self.estop
            or not self.connected
            or self.last_rx is None
            or self.now() - self.last_rx > self.timeout
        ):
            self.velocity = (0.0, 0.0)
        return self.velocity
