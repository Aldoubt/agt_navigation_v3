"""rclpy: navigation capability rejects goals without payload drive permission."""
import threading
import time

import rclpy
from agt_navigation_interfaces.action import NavigateTo
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from agt_navigation_capability.capability import NavigationCapability


class Gated(NavigationCapability):
    def declare_parameter(self, name, value=None, *a, **k):
        if name == 'require_payload_drive_permission':
            value = True
        return super().declare_parameter(name, value, *a, **k)


def _send(client):
    fut = client.send_goal_async(NavigateTo.Goal())
    end = time.monotonic() + 5
    while not fut.done() and time.monotonic() < end:
        time.sleep(0.02)
    res = fut.result().get_result_async()
    while not res.done() and time.monotonic() < end + 5:
        time.sleep(0.02)
    return res.result().result


def test_goal_rejected_without_and_with_false_permission():
    rclpy.init()
    cap = Gated()
    probe = Node('cap_payload_probe')
    pub = probe.create_publisher(Bool, '/agt/payload/drive_permission', 10)
    client = ActionClient(probe, NavigateTo, '/navigation/navigate_to')
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(cap)
    ex.add_node(probe)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        assert client.wait_for_server(timeout_sec=5)
        assert _send(client).error_code == 'ARM_NOT_DRIVE_SAFE'   # never received
        for _ in range(5):
            pub.publish(Bool(data=False))
            time.sleep(0.05)
        assert _send(client).error_code == 'ARM_NOT_DRIVE_SAFE'   # explicit False
        for _ in range(5):
            pub.publish(Bool(data=True))
            time.sleep(0.05)
        # Permission granted: the payload gate passes; the next gate (health) rejects.
        assert _send(client).error_code == 'NOT_READY'
    finally:
        ex.shutdown()
        cap.destroy_node()
        probe.destroy_node()
        rclpy.try_shutdown()
