"""S7.1 regression entrypoint adapted to the stronger S7.2 map-view gate.

Legacy Asset READY alone can never authorize a Path. Replay with a real ROS
mock Map Server + validated synthetic Site and retain both restart/SHA tests.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import String


def execute(config, good, bad, output):
    rclpy.init()
    caller = Node("s7_readonly_viewer_test_client")
    observed = []
    paths = []
    caller.create_subscription(
        String, "/agt/path_tool/ready_route_view/state",
        lambda msg: observed.append(json.loads(msg.data)), 10)
    caller.create_subscription(
        RosPath, "/agt/path_tool/ready_route_view/path",
        lambda msg: paths.append(len(msg.poses)), 10)
    sender = caller.create_publisher(String, "/agt/path_tool/ready_route_view/request", 10)
    env = dict(os.environ, ROS_DOMAIN_ID="231", ROS_LOCALHOST_ONLY="1")
    # Launch the executable directly. 'ros2 run' spawns a detached child on
    # some Humble installations; terminating that CLI can leave orphan nodes.
    viewer_exe = str(Path(get_package_prefix("agt_rviz_patrol")) /
                     "lib/agt_rviz_patrol/ready_route_viewer")
    events = []
    try:
        for boot in (1, 2):
            with (output / f"viewer_boot_{boot}.log").open("w") as logfile:
                proc = subprocess.Popen([
                    viewer_exe, "--ros-args",
                    "-p", f"trusted_config:={config}", "-p", "offline_hard_lock:=true",
                ], stdout=logfile, stderr=subprocess.STDOUT, env=env,
                   start_new_session=True)
                try:
                    # Fresh process must publish STALE; no saved READY subscription.
                    def receive(predicate, timeout=15):
                        deadline = time.monotonic() + timeout
                        while time.monotonic() < deadline:
                            rclpy.spin_once(caller, timeout_sec=0.2)
                            if observed and predicate(observed[-1]):
                                return dict(observed[-1])
                        raise AssertionError(f"view state timeout; last={observed[-3:]}")

                    status = receive(lambda x: x["state"] == "STALE"
                                     and x["code"] == "NO_ROUTE_SELECTED")
                    assert paths and paths[-1] == 0, paths[-3:]
                    events.append({"boot": boot, "phase": "fresh_no_selection", "result": status})
                    for stage, path, want in (
                        ("ready", good, "READY"),
                        ("wrong_ref", bad, "INVALID"),
                        ("recover_ready", good, "READY"),
                    ):
                        observed.clear()
                        deadline = time.monotonic() + 12
                        while time.monotonic() < deadline:
                            sender.publish(String(data=str(path)))
                            rclpy.spin_once(caller, timeout_sec=0.4)
                            if observed and observed[-1]["state"] == want:
                                break
                        else:
                            raise AssertionError(f"{stage}: expected {want}, got {observed[-3:]}")
                        value = dict(observed[-1])
                        # Path is volatile and view-only. INVALID clears the
                        # previously verified geometry, not just the label.
                        deadline2 = time.monotonic() + 3
                        expected_count = 3 if want == "READY" else 0
                        while time.monotonic() < deadline2 and (
                            not paths or paths[-1] != expected_count
                        ):
                            rclpy.spin_once(caller, timeout_sec=0.2)
                        assert paths and paths[-1] == expected_count, paths[-3:]
                        assert value["motion_permission"] == "DENIED_OFFLINE_NO_MOTION"
                        events.append({"boot": boot, "phase": stage, "result": value})
                finally:
                    # ros2 run may spawn a child executable; terminate only
                    # this test-owned process group, not other ROS nodes.
                    os.killpg(proc.pid, signal.SIGTERM)
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
            observed.clear()
        (output / "s7_ros_roundtrip.json").write_text(
            json.dumps(events, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        assert len(events) == 8
        print("S7_ROS_TWO_PROCESS_RESTART_REVALIDATION_PASS 8/8; NO_MOTION")
    finally:
        caller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    from s7_2_ros_map_roundtrip import run
    run(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]), Path(sys.argv[4]))
