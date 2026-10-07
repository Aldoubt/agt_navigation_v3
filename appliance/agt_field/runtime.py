"""ROS2 action/sensor adapter around the independently tested mission engine."""

from __future__ import annotations

import json
import math
import socket
import threading
import time
import uuid
from collections import deque

from .gateway import Framer, encode
from .contracts import ContractError


class RosRuntime:
    def __init__(self, controller):
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String, Bool
        from geometry_msgs.msg import Twist, PoseStamped
        from sensor_msgs.msg import Imu
        from agt_robot_interfaces.msg import LocalizationStatus
        from std_srvs.srv import Trigger

        self.c = controller
        self.rclpy = rclpy
        rclpy.init()
        self.node = Node("agt_mission_executor")
        self.client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self.trigger = self.node.create_client(Trigger, "/agt/localization/relocalize")
        self.handles = {}
        self.cancelled_epochs = set()
        self.pending_stop = None
        self.local_status = None
        self.localization_rx = 0
        self.telemetry = {}
        self.tf = {}
        self.goal_queue = deque()
        self.cancel_queue = deque()
        self.running = True
        self.gateway_rx = 0
        self.estop = True
        self.command = (0.0, 0.0, 0.0)
        self.state_pub = self.node.create_publisher(String, "/agt/mission/status", 10)
        self.wheel_pub = self.node.create_publisher(
            Odometry, self.c.profile["topics"]["wheel_odom"], 20
        )
        self.chassis_pub = self.node.create_publisher(
            String, self.c.profile["topics"]["chassis"], 10
        )
        self.estop_pub = self.node.create_publisher(Bool, self.c.profile["topics"]["estop"], 10)
        self.subscriptions = []
        self.subscriptions.append(
            self.node.create_subscription(
                LocalizationStatus, "/agt/localization/status", self.localization, 10
            )
        )
        self.subscriptions.append(
            self.node.create_subscription(
                Twist, self.c.profile["topics"]["guarded_cmd_vel"], self.cmd_vel, 1
            )
        )
        self.subscriptions.append(
            self.node.create_subscription(Odometry, "/agt/odometry/local", self.odom, 10)
        )
        self.subscriptions.append(
            self.node.create_subscription(PoseStamped, "/goal_pose", self.single_goal, 10)
        )
        self.subscriptions.append(
            self.node.create_subscription(
                Imu,
                self.c.profile["topics"]["imu"],
                lambda m: self.sample("IMU", m),
                qos_profile_sensor_data,
            )
        )
        from livox_ros_driver2.msg import CustomMsg

        self.subscriptions.append(
            self.node.create_subscription(
                CustomMsg,
                self.c.profile["topics"]["lidar"],
                lambda m: self.sample("LiDAR", m),
                qos_profile_sensor_data,
            )
        )
        from tf2_msgs.msg import TFMessage
        from rclpy.qos import QoSProfile, DurabilityPolicy

        def tf_cb(message):
            for t in message.transforms:
                pair = (t.header.frame_id, t.child_frame_id)
                self.tf.setdefault(t.child_frame_id, set()).add(t.header.frame_id)
                self.sample("TF:" + str(pair), t)

        self.subscriptions.append(self.node.create_subscription(TFMessage, "/tf", tf_cb, 100))
        self.subscriptions.append(
            self.node.create_subscription(
                TFMessage,
                "/tf_static",
                tf_cb,
                QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
        )
        self.node.create_timer(0.05, self.tick)
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()
        self.gateway_thread = threading.Thread(target=self.gateway, daemon=True)
        self.gateway_thread.start()

    def sample(self, name, message):
        now = time.monotonic()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        with self.c.lock:
            row = self.telemetry.setdefault(name, dict(times=deque(maxlen=100), stamp=0))
            row["times"].append(now)
            row["stamp"] = stamp

    def sensor_status(self, name):
        row = self.telemetry.get(name)
        if not row or not row["times"]:
            return dict(state="OFFLINE", frequency=0, last_timestamp=None)
        times = list(row["times"])
        age = time.monotonic() - times[-1]
        rate = (
            (len(times) - 1) / (times[-1] - times[0])
            if len(times) > 1 and times[-1] > times[0]
            else 0
        )
        ros_now = self.node.get_clock().now().nanoseconds / 1e9
        return dict(
            state="ONLINE" if age < self.c.profile["sensors"]["stale_sec"] else "OFFLINE",
            frequency=round(rate, 2),
            last_timestamp=row["stamp"],
            timestamp_age=ros_now - row["stamp"],
        )

    def localization(self, msg):
        with self.c.lock:
            self.local_status = msg
            self.localization_rx = time.monotonic()

    def odom(self, msg):
        self.sample("Local Odom", msg)
        with self.c.lock:
            self.measured = (
                abs(msg.twist.twist.linear.x),
                abs(msg.twist.twist.angular.z),
                time.monotonic(),
            )

    def cmd_vel(self, msg):
        with self.c.lock:
            self.command = (msg.linear.x, msg.angular.z, time.monotonic())

    def single_goal(self, msg):
        # Upstream single-point UI is converted into the same mission/action path.
        if msg.header.frame_id != "map":
            return
        q = msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        try:
            with self.c.lock:
                binding = self.c.mission.binding
                if not binding:
                    raise ContractError("no active map")
                self.c.validate_active()
                self.c.mission.load(
                    dict(
                        schema_version=1,
                        route_id="single_point",
                        map=binding,
                        waypoints=[
                            dict(
                                id="P1",
                                x=msg.pose.position.x,
                                y=msg.pose.position.y,
                                yaw=yaw,
                                dwell_seconds=0,
                                action="wait",
                            )
                        ],
                    )
                )
                self.c.mission.start()
        except Exception as exc:
            self.node.get_logger().error(str(exc))

    def send(self, point, epoch):
        self.goal_queue.append((point, epoch))

    def cancel(self, epoch):
        self.cancelled_epochs.add(epoch)
        self.cancel_queue.append(epoch)

    def dispatch(self, point, epoch):
        from nav2_msgs.action import NavigateToPose

        if not self.client.server_is_ready():
            self.c.mission.result(epoch, False, "Nav2 action server unavailable")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = point["x"]
        goal.pose.pose.position.y = point["y"]
        goal.pose.pose.orientation.z = math.sin(point["yaw"] / 2)
        goal.pose.pose.orientation.w = math.cos(point["yaw"] / 2)
        future = self.client.send_goal_async(goal)

        def accepted(f):
            with self.c.lock:
                try:
                    handle = f.result()
                except Exception as exc:
                    self.c.mission.result(epoch, False, str(exc))
                    self.c.mission.cancelled(epoch)
                    return
                if not handle.accepted:
                    self.c.mission.result(epoch, False, "Nav2 goal rejected")
                    self.c.mission.cancelled(epoch)
                    return
                self.handles[epoch] = handle

                def finished(result_future):
                    from action_msgs.msg import GoalStatus

                    with self.c.lock:
                        self.handles.pop(epoch, None)
                        result = result_future.result()
                        if epoch in self.cancelled_epochs:
                            self.c.mission.cancelled(epoch)
                            return
                        if result.status == GoalStatus.STATUS_SUCCEEDED:
                            # Existing measured-stop safety invariant is retained.
                            self.pending_stop = (epoch, time.monotonic())
                        else:
                            self.c.mission.result(
                                epoch, False, "Nav2 action failed: " + str(result.status)
                            )

                handle.get_result_async().add_done_callback(finished)
                if epoch in self.cancelled_epochs:
                    handle.cancel_goal_async()

        future.add_done_callback(accepted)

    def tick(self):
        from agt_robot_interfaces.msg import LocalizationStatus
        from std_msgs.msg import String

        with self.c.lock:
            now = time.monotonic()
            status = self.local_status
            ready = bool(
                status
                and now - self.localization_rx
                < self.c.profile["localization"]["status_timeout_sec"]
                and status.state == LocalizationStatus.STATE_LOCALIZED
                and status.local_odom_fresh
                and status.global_correction_valid
                and (
                    not self.c.mission.binding
                    or (
                        status.map_id == self.c.mission.binding["map_bundle_id"]
                        and status.map_version == self.c.mission.binding["map_version"]
                    )
                )
            )
            self.c.localization = "READY" if ready else ("LOST" if status else "STARTING")
            gateway = now - self.gateway_rx < 0.5 and not self.estop
            self.c.gateway_ready = gateway
            self.c.mission.readiness(ready, gateway)
            while self.goal_queue:
                p, e = self.goal_queue.popleft()
                if e in self.cancelled_epochs:
                    self.c.mission.cancelled(e)
                else:
                    self.dispatch(p, e)
            while self.cancel_queue:
                epoch = self.cancel_queue.popleft()
                if epoch in self.handles:
                    self.handles[epoch].cancel_goal_async()
            if self.pending_stop:
                epoch, started = self.pending_stop
                measured = getattr(self, "measured", (float("inf"), float("inf"), 0))
                base = self.c.profile["base"]
                if (
                    now - measured[2] < 0.5
                    and measured[0] <= base["stop_linear_mps"]
                    and measured[1] <= base["stop_angular_radps"]
                ):
                    self.c.mission.result(epoch, True)
                    self.pending_stop = None
                elif now - started > base["stop_timeout_sec"]:
                    self.c.mission.result(epoch, False, "measured stop timeout")
                    self.pending_stop = None
            self.c.mission.tick()
            self.c.check_processes()
            self.state_pub.publish(String(data=json.dumps(self.c.mission.status())))

    def gateway(self):
        from rosidl_runtime_py.set_message import set_message_fields
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String, Bool

        path = str(self.c.data / "run/gateway.sock")
        while self.running:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(0.1)
            try:
                sock.connect(path)
                session = uuid.uuid4().hex
                sequence = 0
                frame = Framer()
                sock.sendall(encode(dict(type="hello", session=session)))
                next_send = 0
                while self.running:
                    now = time.monotonic()
                    if now >= next_send:
                        with self.c.lock:
                            x, z, stamp = self.command
                            # A guard can continue publishing zeros; stale upstream guard intent never survives reconnect.
                            timeout = self.c.profile["base"].get("command_timeout_sec")
                            if (
                                not timeout
                                or now - stamp > timeout
                                or now - self.gateway_rx > 0.5
                                or self.estop
                            ):
                                x = z = 0.0
                        sequence += 1
                        sock.sendall(
                            encode(
                                dict(
                                    type="cmd_vel",
                                    session=session,
                                    sequence=sequence,
                                    linear=x,
                                    angular=z,
                                )
                            )
                        )
                        next_send = now + 0.02
                    try:
                        raw = sock.recv(16384)
                    except socket.timeout:
                        continue
                    if not raw:
                        raise ConnectionError("gateway EOF")
                    for row in frame.feed(raw):
                        with self.c.lock:
                            self.gateway_rx = time.monotonic()
                            if row["type"] == "odom":
                                msg = Odometry()
                                set_message_fields(msg, row["message"])
                                self.wheel_pub.publish(msg)
                                self.sample("Wheel Odom", msg)
                            elif row["type"] == "chassis":
                                self.chassis_pub.publish(String(data=row["message"]))
                                self.c.chassis_rx = time.monotonic()
                            elif row["type"] == "estop":
                                self.estop = bool(row["value"])
                                self.estop_pub.publish(Bool(data=self.estop))
            except (OSError, ValueError, KeyError, ContractError):
                with self.c.lock:
                    self.gateway_rx = 0
                    self.estop = True
                    self.command = (0, 0, 0)
                time.sleep(0.2)
            finally:
                sock.close()

    def relocalize(self):
        from std_srvs.srv import Trigger

        if not self.trigger.service_is_ready():
            raise ContractError("relocalization service unavailable")
        self.trigger.call_async(Trigger.Request())

    def spin(self):
        try:
            self.rclpy.spin(self.node)
        finally:
            self.running = False

    def shutdown(self):
        self.running = False
        self.rclpy.shutdown()
        self.thread.join(timeout=3)
        self.gateway_thread.join(timeout=3)
