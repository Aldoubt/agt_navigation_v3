"""Read-only owner of /navigation/health; never starts or restarts drivers."""

from collections import deque
import time

import rclpy
from agt_navigation_interfaces.msg import NavigationHealth
from agt_robot_interfaces.msg import LocalizationStatus
from lifecycle_msgs.srv import GetState
from livox_ros_driver2.msg import CustomMsg
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from .health_model import Inputs, decide, matches_selected_map


class Stream:
    def __init__(self):
        self.received = deque(maxlen=32)
        self.last_stamp = None
        self.good_stamps = 0

    def observe(self, stamp):
        now = time.monotonic()
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if self.last_stamp is None or stamp_ns > self.last_stamp:
            self.good_stamps = min(self.good_stamps + 1, 3)
        else:
            self.good_stamps = 0
        self.last_stamp = stamp_ns
        self.received.append(now)

    def healthy(self, max_age, min_rate):
        if not self.received or self.good_stamps < 2:
            return False
        if time.monotonic() - self.received[-1] > max_age:
            return False
        if min_rate <= 0 or len(self.received) < 3:
            return True
        duration = self.received[-1] - self.received[0]
        return duration > 0 and (len(self.received) - 1) / duration >= min_rate


class NavigationSupervisor(Node):
    def __init__(self):
        super().__init__('agt_navigation_supervisor')
        for name, default in (
            ('robot_profile', 'bunker_v1'), ('map_id', ''), ('map_version', ''),
            ('map_valid', True), ('sensor_max_age_sec', 1.5),
            ('odom_max_age_sec', 1.0), ('localization_max_age_sec', 2.0),
            ('min_lidar_hz', 2.0), ('min_imu_hz', 20.0),
            ('min_base_hz', 5.0), ('min_odom_hz', 5.0),
            # Input topics follow agt_robot_bringup/config/robot_topics.yaml;
            # defaults match the MID360 + Bunker reference robot.
            ('lidar_topic', '/livox/lidar'), ('imu_topic', '/livox/imu'),
            ('wheel_odom_topic', '/wheel/odom'),
            ('local_odom_topic', '/agt/odometry/local'),
            ('localization_status_topic', '/agt/localization/status'),
        ):
            self.declare_parameter(name, default)
        self.lidar, self.imu, self.base, self.odom = (Stream() for _ in range(4))
        self.localization = None
        self.localization_received = 0.0
        self.goal_active = False
        self.ever_ready = False
        self._last_health_fault = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.lifecycle = {}
        for name in ('planner_server', 'controller_server', 'bt_navigator'):
            client = self.create_client(GetState, f'/{name}/get_state')
            self.lifecycle[name] = {'client': client, 'active': False, 'pending': None}
        topic = lambda name: str(self.get_parameter(name).value)  # noqa: E731
        self.create_subscription(CustomMsg, topic('lidar_topic'),
                                 lambda m: self.lidar.observe(m.header.stamp), qos_profile_sensor_data)
        self.create_subscription(Imu, topic('imu_topic'),
                                 lambda m: self.imu.observe(m.header.stamp), qos_profile_sensor_data)
        self.create_subscription(Odometry, topic('wheel_odom_topic'),
                                 lambda m: self.base.observe(m.header.stamp), qos_profile_sensor_data)
        self.create_subscription(Odometry, topic('local_odom_topic'),
                                 lambda m: self.odom.observe(m.header.stamp), qos_profile_sensor_data)
        self.create_subscription(LocalizationStatus, topic('localization_status_topic'),
                                 self._on_localization, 10)
        self.create_subscription(Bool, '/navigation/goal_active',
                                 lambda m: setattr(self, 'goal_active', bool(m.data)), 10)
        self.publisher = self.create_publisher(
            NavigationHealth, '/navigation/health', QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.5, self._publish)
        self.create_timer(2.0, self._check_lifecycle)

    def _on_localization(self, msg):
        self.localization = msg
        self.localization_received = time.monotonic()

    def _check_lifecycle(self):
        for entry in self.lifecycle.values():
            client = entry['client']
            pending = entry['pending']
            if pending is not None and not pending.done():
                continue
            if not client.service_is_ready():
                entry['active'] = False
                entry['pending'] = None
                continue
            future = client.call_async(GetState.Request())
            entry['pending'] = future
            future.add_done_callback(lambda done, target=entry: self._state_result(done, target))

    @staticmethod
    def _state_result(future, entry):
        try:
            entry['active'] = future.result().current_state.label == 'active'
        except Exception:
            entry['active'] = False

    def _publish(self):
        value = lambda name: self.get_parameter(name).value
        sensor_age = float(value('sensor_max_age_sec'))
        odom_age = float(value('odom_max_age_sec'))
        lidar_ok = self.lidar.healthy(sensor_age, float(value('min_lidar_hz')))
        imu_ok = self.imu.healthy(sensor_age, float(value('min_imu_hz')))
        base_ok = self.base.healthy(sensor_age, float(value('min_base_hz')))
        odom_ok = self.odom.healthy(odom_age, float(value('min_odom_hz')))
        loc = self.localization
        loc_fresh = loc is not None and time.monotonic() - self.localization_received < float(
            value('localization_max_age_sec'))
        map_match = (loc is not None and matches_selected_map(
            loc.map_id, loc.map_version, str(value('map_id')),
            str(value('map_version'))))
        localized = bool(loc_fresh and map_match and loc.state == LocalizationStatus.STATE_LOCALIZED
                         and loc.local_odom_fresh and loc.global_correction_valid)
        static_tf = self.tf_buffer.can_transform('base_link', 'lidar_link', Time())
        global_tf = self.tf_buffer.can_transform('map', 'base_link', Time())
        nav2_ok = self.nav_client.server_is_ready() and all(
            item['active'] for item in self.lifecycle.values())
        snapshot = Inputs(
            map_valid=bool(value('map_valid')), platform_ready=bool(static_tf),
            lidar_alive=lidar_ok, imu_alive=imu_ok, base_alive=base_ok,
            odom_alive=odom_ok, localized=localized, tf_ready=bool(global_tf),
            nav2_active=nav2_ok, goal_active=self.goal_active)
        result = decide(snapshot, self.ever_ready)
        if result.status in ('DEGRADED', 'ERROR'):
            fault = (result.state, result.error_code, snapshot)
            if fault != self._last_health_fault:
                self.get_logger().warning(
                    f'navigation health {result.status}: {result.error_code}; '
                    f'platform={snapshot.platform_ready} lidar={snapshot.lidar_alive} '
                    f'imu={snapshot.imu_alive} base={snapshot.base_alive} '
                    f'odom={snapshot.odom_alive} localized={snapshot.localized} '
                    f'tf={snapshot.tf_ready} nav2={snapshot.nav2_active}')
            self._last_health_fault = fault
        elif self._last_health_fault is not None:
            self.get_logger().info(f'navigation health recovered: {result.status}')
            self._last_health_fault = None
        if result.status in ('READY', 'BUSY'):
            self.ever_ready = True
        msg = NavigationHealth()
        msg.stamp = self.get_clock().now().to_msg()
        msg.status = getattr(NavigationHealth, result.status)
        msg.state = result.state
        msg.robot_profile = str(value('robot_profile'))
        msg.map_id = str(value('map_id'))
        msg.map_version = str(value('map_version'))
        msg.platform_ready = snapshot.platform_ready
        msg.lidar_alive = lidar_ok
        msg.imu_alive = imu_ok
        # Expose wheel reception for diagnostics; it is not a readiness gate.
        msg.base_alive = base_ok
        msg.odom_alive = odom_ok
        msg.localized = localized
        # Binary confidence until the localization backend exposes a calibrated score.
        msg.localization_confidence = 1.0 if localized else 0.0
        msg.nav2_active = nav2_ok
        msg.goal_active = self.goal_active
        msg.last_error_code = result.error_code
        msg.last_error_message = result.message
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = NavigationSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
