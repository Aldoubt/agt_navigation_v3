# ruff: noqa: E402
"""Software-only ROS2 action/gateway integration; no physical TF or motion."""

import json
from pathlib import Path
import socket
import threading
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from agt_robot_interfaces.msg import LocalizationStatus
from agt_field.runtime import RosRuntime
from agt_field.mission import Mission
from agt_field.profile import load_profile
from agt_field.gateway import Framer, encode


def until(check, timeout=10):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return
        time.sleep(0.02)
    raise AssertionError("ROS state deadline exceeded")


def test_real_ros2_action_results_dwell_pause_cancel_and_lost(tmp_path):
    profile = load_profile(Path("/opt/nav_ws/src/agt_navigation_v3/profiles/mock_yhs"))
    run = tmp_path / "run"
    run.mkdir()
    socket_path = run / "gateway.sock"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(socket_path))
    server.listen(1)
    server.settimeout(0.1)
    ending = threading.Event()
    received = []

    def gateway():
        conn = None
        while not ending.is_set():
            if conn is None:
                try:
                    conn, _ = server.accept()
                    conn.settimeout(0.005)
                    framer = Framer()
                except socket.timeout:
                    continue
            try:
                try:
                    data = conn.recv(65536)
                    if not data:
                        conn.close()
                        conn = None
                        continue
                    received.extend(framer.feed(data))
                except socket.timeout:
                    pass
                conn.sendall(
                    encode(dict(type="estop", value=False))
                    + encode(dict(type="chassis", message='{"mock":true}'))
                    + encode(
                        dict(
                            type="odom",
                            message={
                                "header": {
                                    "stamp": {"sec": int(time.time()), "nanosec": 0},
                                    "frame_id": "test_wheel_odom",
                                },
                                "child_frame_id": "test_base",
                                "pose": {
                                    "pose": {
                                        "position": {"x": 2.4, "y": 0.0, "z": 0.0},
                                        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                                    },
                                    "covariance": [0.3] * 36,
                                },
                                "twist": {
                                    "twist": {
                                        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                                        "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                                    },
                                    "covariance": [0.2] * 36,
                                },
                            },
                        )
                    )
                )
                time.sleep(0.02)
            except OSError:
                if conn:
                    conn.close()
                conn = None
        if conn:
            conn.close()

    gt = threading.Thread(target=gateway)
    gt.start()
    c = SimpleNamespace(
        data=tmp_path,
        profile=profile,
        lock=threading.RLock(),
        gateway_ready=False,
        chassis_rx=0,
        localization="STOPPED",
        check_processes=lambda: None,
    )
    c.mission = Mission(lambda p, e: c.runtime.send(p, e), lambda e: c.runtime.cancel(e))
    c.runtime = RosRuntime(c)
    node = Node("software_nav2_fixture")
    transferred_odom = []
    node.create_subscription(
        Odometry, c.profile["topics"]["wheel_odom"], lambda msg: transferred_odom.append(msg), 10
    )
    control = {"localized": True, "fail": False, "goal_count": 0}
    status_pub = node.create_publisher(LocalizationStatus, "/agt/localization/status", 10)
    odom_pub = node.create_publisher(Odometry, "/agt/odometry/local", 10)

    def publish():
        msg = LocalizationStatus()
        msg.state = (
            LocalizationStatus.STATE_LOCALIZED
            if control["localized"]
            else LocalizationStatus.STATE_LOST
        )
        msg.map_id = "ros_mock"
        msg.map_version = "1"
        msg.local_odom_fresh = True
        msg.global_correction_valid = control["localized"]
        status_pub.publish(msg)
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.header.stamp = node.get_clock().now().to_msg()
        odom_pub.publish(odom)

    node.create_timer(0.02, publish)

    def execute(handle):
        control["goal_count"] += 1
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            if handle.is_cancel_requested:
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.01)
        if control["fail"]:
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    action = ActionServer(
        node,
        NavigateToPose,
        "/navigate_to_pose",
        execute_callback=execute,
        cancel_callback=lambda _: CancelResponse.ACCEPT,
        callback_group=ReentrantCallbackGroup(),
    )
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    binding = dict(map_bundle_id="ros_mock", map_version="1", bundle_sha256="0" * 64)
    route = dict(
        schema_version=1,
        route_id="ros_route",
        map=binding,
        waypoints=[
            dict(id="P1", x=0.0, y=0.0, yaw=0.0, dwell_seconds=0.1),
            dict(id="P2", x=1.0, y=0.0, yaw=0.0, dwell_seconds=0.1),
        ],
    )
    try:
        until(lambda: c.mission.localization_ready and c.gateway_ready)
        with c.lock:
            c.mission.bind(binding)
            c.mission.load(route)
            c.mission.start()
        until(lambda: c.mission.state == "COMPLETED")
        assert control["goal_count"] == 2
        assert transferred_odom
        assert transferred_odom[-1].header.frame_id == "test_wheel_odom"
        assert transferred_odom[-1].child_frame_id == "test_base"
        assert transferred_odom[-1].pose.pose.position.x == 2.4
        assert transferred_odom[-1].pose.covariance[0] == 0.3
        assert transferred_odom[-1].twist.covariance[0] == 0.2
        with c.lock:
            c.mission.load(route)
            c.mission.start()
        until(lambda: bool(c.runtime.handles))
        with c.lock:
            c.mission.pause()
        until(lambda: not c.mission.cancel_pending)
        with c.lock:
            c.mission.resume()
        until(lambda: c.mission.state == "NAVIGATING")
        with c.lock:
            c.mission.stop()
        until(lambda: not c.mission.cancel_pending)
        assert c.mission.state == "CANCELLED"
        with c.lock:
            c.mission.load(route)
            c.mission.start()
        control["localized"] = False
        until(lambda: c.mission.state == "ERROR")
        until(lambda: not c.mission.cancel_pending)
        control["localized"] = True
        until(lambda: c.mission.localization_ready)
        control["fail"] = True
        with c.lock:
            c.mission.load(route)
            c.mission.start()
        until(lambda: c.mission.state == "ERROR")
        assert "Nav2 action failed" in c.mission.detail
        assert any(row.get("type") == "cmd_vel" for row in received)
        assert all(row.get("linear", 0) == 0 for row in received)
        print(
            json.dumps(
                dict(
                    result="PASS_ROS2_SOFTWARE_ONLY",
                    nav2_action_goals=control["goal_count"],
                    pause_resume_cancel=True,
                    localization_lost=True,
                    nav2_failure=True,
                    physical_hardware="PENDING",
                )
            )
        )
    finally:
        ending.set()
        gt.join(timeout=2)
        server.close()
        executor.shutdown()
        thread.join(timeout=2)
        action.destroy()
        node.destroy_node()
        c.runtime.shutdown()
