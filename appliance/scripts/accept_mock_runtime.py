"""Run against ./agt up --mock; exits nonzero on any acceptance failure."""

import argparse
import json
from pathlib import Path
import time
from agt_field.server import rpc

parser = argparse.ArgumentParser()
parser.add_argument("--data-root", required=True)
args = parser.parse_args()
root = Path(args.data_root)


def call(command, **values):
    return rpc(root, dict(command=command, **values))


def wait(check, timeout=20):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        state = call("STATUS")
        if check(state):
            return state
        time.sleep(0.02)
    raise AssertionError("Runtime acceptance deadline exceeded")


def start_ready():
    end = time.monotonic() + 5
    while True:
        try:
            return call("START")
        except ValueError as exc:
            if "mission interlock" not in str(exc) or time.monotonic() >= end:
                raise
            time.sleep(0.02)


status = call("STATUS")
assert status["mock"], "Never run mock acceptance against real hardware"
assert status["devices"]["LiDAR"]["state"] == "ONLINE"
assert status["devices"]["CAN"]["state"] == "ACTIVE"
call("STOP_ALL")
call("CAN_UP")
assert call("PREFLIGHT")["pass_"]
identity = dict(map_bundle_id="docker_mock_" + str(time.time_ns()), map_version="1")
call("START_MAPPING", **identity)
assert call("STATUS")["mapping"] == "RUNNING"
call("STOP_MAPPING")
wait(lambda s: s["mapping"] == "REVIEW_REQUIRED" and not s["busy"])
call("REVIEW_MAP")
call("CONFIRM_MAP")
assert call("STATUS")["mapping"] == "READY"
call("ACTIVATE_MAP", **identity)
call("START_NAVIGATION")
wait(lambda s: s["localization"] == "READY")
route = dict(
    schema_version=1,
    route_id=identity["map_bundle_id"],
    map=call("STATUS")["map"],
    waypoints=[
        dict(
            id="P" + str(i + 1),
            x=float(i),
            y=0.0,
            yaw=0.0,
            sequence=i,
            dwell_seconds=0.2 if i < 2 else 0.0,
            action="wait",
        )
        for i in range(3)
    ],
)
call("SAVE_ROUTE", route=route)
call("LOAD_ROUTE", route_id=route["route_id"])
start_ready()
dwells = set()


def complete(s):
    if s["mission"]["state"] == "DWELLING":
        dwells.add(s["mission"]["waypoint_index"])
    return s["mission"]["state"] == "COMPLETED"


wait(complete)
assert {0, 1} <= dwells, dwells
for fault in ["localization", "gateway", "nav"]:
    call("STOP_ALL")
    call("CAN_UP")
    call("ACTIVATE_MAP", **identity)
    call("START_NAVIGATION")
    call("LOAD_ROUTE", route_id=route["route_id"])
    start_ready()
    call("MOCK_FAULT", fault=fault)
    assert call("STATUS")["mission"]["state"] == "ERROR"
call("STOP_ALL")
call("CAN_UP")
call("ACTIVATE_MAP", **identity)
call("START_NAVIGATION")
call("LOAD_ROUTE", route_id=route["route_id"])
start_ready()
call("CANCEL")
assert call("STATUS")["mission"]["state"] == "CANCELLED"
call("START_RECORDING")
call("STOP_RECORDING")
call("STOP_ALL")
print(
    json.dumps(
        dict(
            result="PASS_DOCKER_MOCK_RUNTIME",
            map=identity,
            mapping_stages=call("STATUS")["mapping_history"],
            waypoint_dwell_indices=sorted(dwells),
            mission_completed=True,
            faults=["localization lost", "gateway disconnected", "Nav2 failure", "cancelled"],
            physical_hardware="PENDING",
        )
    )
)
