"""Offline-only ROS view bridge; volatile state, no action or controller client."""

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String

from .ready_route_resolver import inspect_view, read_json_regular


class ReadyRouteViewer(Node):
    def __init__(self):
        super().__init__("agt_ready_route_viewer")
        self.declare_parameter("trusted_config", "")
        self.declare_parameter("offline_hard_lock", True)
        if not self.get_parameter("offline_hard_lock").value:
            raise RuntimeError("S7 read-only viewer requires offline_hard_lock=True")
        config_path = self.get_parameter("trusted_config").value
        if not config_path:
            raise RuntimeError("trusted_config required; no implicit Site/Route fallback")
        self.config = read_json_regular(config_path)
        self.state_pub = self.create_publisher(String, "/agt/path_tool/ready_route_view/state", 10)
        self.path_pub = self.create_publisher(Path, "/agt/path_tool/ready_route_view/path", 1)
        self.create_subscription(String, "/agt/path_tool/ready_route_view/request", self.select, 10)
        self.selection = None
        self.last_state = "INVALID"
        self.create_timer(1.0, self.refresh)
        self.publish({"state": "STALE", "code": "RESELECT_AFTER_START",
                      "message": "Viewer restarted: select RouteRef and revalidate",
                      "motion_permission": "DENIED_OFFLINE_NO_MOTION"})

    def publish(self, data):
        data = dict(data)
        poses = data.pop("_preview_poses", [])
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        if data["state"] == "READY":
            for x, y, yaw in poses:
                p = PoseStamped()
                p.header = path.header
                p.pose.position.x, p.pose.position.y = x, y
                p.pose.position.z = 0.06
                p.pose.orientation.z = math.sin(yaw / 2.0)
                p.pose.orientation.w = math.cos(yaw / 2.0)
                path.poses.append(p)
        # Empty Path clears the last geometry on INVALID/STALE or Site switch.
        self.path_pub.publish(path)
        data["checked_monotonic"] = time.monotonic()
        self.last_state = data["state"]
        self.state_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def select(self, msg):
        # A selected file is *only* a RouteRef+evidence request. It cannot
        # choose the trusted deployment roots, current Profile or active Site.
        self.selection = msg.data if msg.data and len(msg.data) <= 4096 else None
        self.refresh()

    def refresh(self):
        if self.selection is None:
            self.publish({"state": "STALE", "code": "NO_ROUTE_SELECTED",
                          "message": "Select RouteRef file; LOCAL DRAFT remains separate",
                          "motion_permission": "DENIED_OFFLINE_NO_MOTION"})
            return
        try:
            selected = read_json_regular(self.selection)
            result = inspect_view(self.config, selected, include_preview=True)
        except (OSError, ValueError, UnicodeError) as exc:
            result = {"state": "INVALID", "code": "SELECTION_FILE_INVALID",
                      "message": type(exc).__name__ + ": selected ref is not a valid regular JSON file",
                      "motion_permission": "DENIED_OFFLINE_NO_MOTION"}
        self.publish(result)


def main(args=None):
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    if (not domain.isdigit() or not 1 <= int(domain) <= 232
            or os.environ.get("ROS_LOCALHOST_ONLY") != "1"):
        raise RuntimeError("S7 viewer requires isolated offline ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1")
    rclpy.init(args=args)
    node = None
    try:
        node = ReadyRouteViewer()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
