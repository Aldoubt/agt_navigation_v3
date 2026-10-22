"""ROS bridge for field console. No direct cmd_vel or motor enable publisher."""
from __future__ import annotations

import json
import time


class FieldROSBridge:
    def __init__(self, event_callback):
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from agt_navigation_interfaces.msg import NavigationHealth
        from agt_navigation_interfaces.srv import PrepareDistanceRoute
        from std_msgs.msg import String
        from nav_msgs.msg import Path
        from std_srvs.srv import Trigger

        self.rclpy = rclpy
        self.PrepareDistanceRoute, self.Trigger = PrepareDistanceRoute, Trigger
        self.emit = event_callback
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("agt_field_console")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(NavigationHealth, "/navigation/health",
                                      self._health, latched)
        self.node.create_subscription(String, "/agt/path_tool/editor_state",
                                      self._editor, latched)
        self.node.create_subscription(Path, "/agt/path_tool/route",
                                      self._route, latched)
        self.node.create_subscription(String, "/fwmini/navigation_guard/status",
                                      self._guard, 10)
        self.services = {
            "prepare": self.node.create_client(
                PrepareDistanceRoute, "/agt/field/prepare_distance"),
            "execute": self.node.create_client(Trigger, "/agt/path_tool/start"),
            "pause": self.node.create_client(Trigger, "/agt/path_tool/pause"),
            "cancel": self.node.create_client(Trigger, "/agt/path_tool/cancel"),
        }
        self.pending = {}

    def navigation_nodes_present(self):
        """Conservatively detect any known Nav2/route lifecycle ownership."""
        names = set(self.node.get_node_names())
        return bool(names & {
            "controller_server", "planner_server", "bt_navigator",
            "lifecycle_manager_navigation", "agt_rviz_path_tool",
            "agt_navigation_capability", "agt_localization_manager",
        })

    def spin(self):
        self.rclpy.spin_once(self.node, timeout_sec=0.0)

    def _health(self, m):
        self.emit("health", {
            "at": time.monotonic(),
            "data": {
                "status": m.status, "state": m.state,
                "localized": m.localized, "base_alive": m.base_alive,
                "goal_active": m.goal_active,
                "nav2_active": m.nav2_active, "map_id": m.map_id,
                "map_version": m.map_version, "robot_profile": m.robot_profile,
                "last_error_code": m.last_error_code,
                "last_error_message": m.last_error_message,
            },
        })

    def _editor(self, m):
        try:
            state = json.loads(m.data)
            if not isinstance(state, dict):
                return
        except (ValueError, TypeError):
            return
        self.emit("editor", {"at": time.monotonic(), "data": state})

    def _guard(self, m):
        self.emit("guard", {
            "at": time.monotonic(), "data": {"state": m.data},
        })

    def _route(self, m):
        self.emit("route", {
            "points": [(p.pose.position.x, p.pose.position.y) for p in m.poses],
            "frame": m.header.frame_id,
        })

    def request(self, action, *, distance=None, map_id=None, version=None):
        client = self.services.get(action)
        if client is None:
            raise ValueError(f"Unknown action {action}")
        if action in self.pending:
            raise ValueError(f"{action} already pending")
        if not client.service_is_ready():
            raise ValueError(f"ROS 服务未连接：{client.srv_name}")
        if action == "prepare":
            req = self.PrepareDistanceRoute.Request()
            req.distance_m = float(distance)
            req.expected_map_id = str(map_id)
            req.expected_map_version = str(version)
        else:
            req = self.Trigger.Request()
        future = client.call_async(req)
        self.pending[action] = future
        future.add_done_callback(lambda f, name=action: self._finished(name, f))

    def _finished(self, action, future):
        self.pending.pop(action, None)
        try:
            result = future.result()
            self.emit("result", {
                "action": action, "success": bool(result.success),
                "message": str(result.message),
            })
        except Exception as exc:
            self.emit("result", {"action": action, "success": False,
                                 "message": f"ROS 服务失败：{exc}"})

    def close(self):
        for future in self.pending.values():
            future.cancel()
        self.pending.clear()
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()
