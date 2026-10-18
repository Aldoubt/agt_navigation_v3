"""Offline-only ROS view bridge; volatile state, no action or controller client."""

import json
import math
import os
import time
import hashlib
from pathlib import Path as FsPath

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from std_msgs.msg import String

from .ready_route_resolver import inspect_view, read_json_regular
from .map_view_binding import inspect_bound_view


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
        for key in ("view_map_spec","view_robot","view_registry"):
            self.declare_parameter(key, "")
            value = self.get_parameter(key).value
            if value:
                self.config[key] = value
        self.config.setdefault("view_mode", "registry")
        self.map_server = self.create_client(GetParameters, "/map_server/get_parameters")
        self.map_source = ""
        self.source_received = 0.0
        self.grid = None
        self.grid_received = 0.0
        self.grid_epoch = 0
        map_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_timer(0.75, self.poll_source)
        self.pending_source = None
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
        if data.get("state") == "MAP_BOUND_TO_VIEW" and data.get("map_bound_to_view") is True:
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

    def on_map(self, msg):
        yaw = math.atan2(
            2*(msg.info.origin.orientation.w*msg.info.origin.orientation.z +
               msg.info.origin.orientation.x*msg.info.origin.orientation.y),
            1-2*(msg.info.origin.orientation.y**2+msg.info.origin.orientation.z**2))
        # signed OccupancyGrid int8 -> exact raw uint8 representation.
        raw = bytes(int(x)&255 for x in msg.data)
        self.grid = {
            "width":int(msg.info.width), "height":int(msg.info.height),
            "resolution":float(msg.info.resolution),
            "origin":[float(msg.info.origin.position.x),
                      float(msg.info.origin.position.y), float(yaw)],
            "frame_id":msg.header.frame_id,
            "grid_sha256":hashlib.sha256(raw).hexdigest(),
        }
        self.grid_received=time.monotonic()
        self.grid_epoch+=1
        # Clear any previously bound Path synchronously with map changes.
        self.publish({"state":"STALE","code":"MAP_GRID_CHANGED_RECHECK",
                      "message":"Live map changed; authoritative source and Route binding recheck required",
                      "asset_ready_verified":False,"map_bound_to_view":False,
                      "map_binding_state":"STALE",
                      "motion_permission":"DENIED_OFFLINE_NO_MOTION"})

    def poll_source(self):
        if self.pending_source and not self.pending_source.done():
            return
        if not self.map_server.service_is_ready():
            self.get_logger().debug("Map Server parameter service unavailable")
            self.map_source=""
            return
        self.pending_source=self.map_server.call_async(
            GetParameters.Request(names=["yaml_filename"]))
        self.pending_source.add_done_callback(self.source_result)

    def source_result(self, future):
        try:
            values=future.result().values
            source=values[0].string_value if len(values)==1 else ""
            if not source or not FsPath(source).is_absolute():
                raise ValueError("map source must be absolute")
            if source != self.map_source:
                self.grid_epoch+=1
                self.map_source=source
                self.publish({"state":"STALE","code":"MAP_SOURCE_CHANGED_RECHECK",
                              "message":"Map Server source updated; old display binding cleared",
                              "asset_ready_verified":False,"map_bound_to_view":False,
                              "map_binding_state":"STALE",
                              "motion_permission":"DENIED_OFFLINE_NO_MOTION"})
            self.source_received=time.monotonic()
        except Exception as exc:
            self.get_logger().debug(f"Map Server parameters unavailable: {type(exc).__name__}")
            self.map_source=""
            self.source_received=0.

    def current_view(self):
        now=time.monotonic()
        if (self.grid is None or not self.map_source or
            now-self.source_received>2.0):
            self.get_logger().debug("Map observation or source missing/stale")
            return None
        info=self.get_publishers_info_by_topic("/map")
        names=[(p.node_namespace.rstrip("/")+"/"+p.node_name).replace("//","/") for p in info]
        if len(names)!=1 or names[0]!="/map_server":
            self.get_logger().debug("Map publisher origin ambiguous")
            return None
        # In production, map identity comes only from an independent V3 registry;
        # the Site fixture shortcut is allowed only with an explicit lab guard.
        if self.config.get("view_mode") == "registry":
            from agt_map_manager.map_catalog import resolve_map
            try:
                selected=resolve_map(
                    str(self.config.get("view_map_spec","")),
                    str(self.config.get("view_robot","")),
                    self.config.get("view_registry",""))
                map_id,version=selected.map_id,selected.map_version
                if str(selected.navigation_map)!=self.map_source:
                    return None
            except Exception:
                return None
        else:
            map_id=self.config.get("site_id")
            version=self.config.get("site_revision")
        return dict(self.grid,map_id=map_id,map_version=version,
                    yaml_filename=self.map_source,publisher_count=len(names),
                    publisher_node=names[0],map_server_node="/map_server")

    def refresh(self):
        if self.selection is None:
            self.publish({"state": "STALE", "code": "NO_ROUTE_SELECTED",
                          "message": "Select RouteRef file; LOCAL DRAFT remains separate",
                          "motion_permission": "DENIED_OFFLINE_NO_MOTION"})
            return
        try:
            selected = read_json_regular(self.selection)
            epoch = self.grid_epoch
            result = inspect_bound_view(self.config, selected, self.current_view())
            if epoch != self.grid_epoch:
                result={"state":"STALE","code":"MAP_CHANGED_DURING_CHECK",
                        "message":"Map source or geometry changed during binding",
                        "asset_ready_verified":False,"map_bound_to_view":False,
                        "map_binding_state":"STALE",
                        "motion_permission":"DENIED_OFFLINE_NO_MOTION"}
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
        if rclpy.ok():
            rclpy.shutdown()
