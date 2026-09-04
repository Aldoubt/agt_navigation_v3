from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import NavSatFix, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from camera_gimbal_interfaces.action import AcquireView


class DemoPreflight(Node):
    def __init__(self):
        super().__init__('demo_preflight')
        params = {
            'timeout_sec': 5.0,
            'global_frame': 'map',
            'base_frame': 'base_link',
            'local_odom_topic': '/agt/odometry/local',
            'obstacle_cloud_topic': '/agt/navigation/points_obstacles',
            'navsat_topic': '/ins/navsatfix',
            'require_rtk': False,
        }
        for name, value in params.items():
            self.declare_parameter(name, value)

        self.odom = None
        self.cloud = None
        self.rtk = None
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.camera_client = ActionClient(self, AcquireView, '/camera_gimbal/acquire_view')
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, self.get_parameter('local_odom_topic').value, self._odom_cb, 10)
        self.create_subscription(PointCloud2, self.get_parameter('obstacle_cloud_topic').value, self._cloud_cb, 10)
        self.create_subscription(NavSatFix, self.get_parameter('navsat_topic').value, self._rtk_cb, 10)

    def _odom_cb(self, msg):
        self.odom = msg

    def _cloud_cb(self, msg):
        self.cloud = msg

    def _rtk_cb(self, msg):
        self.rtk = msg

    def run(self) -> bool:
        timeout = float(self.get_parameter('timeout_sec').value)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.odom is not None and self.cloud is not None:
                break

        checks: list[tuple[str, bool, str]] = []
        nav_ok = self.nav_client.wait_for_server(timeout_sec=0.5)
        checks.append(('Nav2 /navigate_to_pose', nav_ok, 'action server'))
        camera_ok = self.camera_client.wait_for_server(timeout_sec=0.5)
        checks.append(('C1 /camera_gimbal/acquire_view', camera_ok, 'action server'))
        checks.append(('local odometry', self.odom is not None, self.get_parameter('local_odom_topic').value))
        checks.append(('obstacle cloud', self.cloud is not None, self.get_parameter('obstacle_cloud_topic').value))

        tf_ok = False
        tf_detail = f"{self.get_parameter('global_frame').value}->{self.get_parameter('base_frame').value}"
        try:
            self.tf_buffer.lookup_transform(
                self.get_parameter('global_frame').value,
                self.get_parameter('base_frame').value,
                Time(), timeout=Duration(seconds=0.5))
            tf_ok = True
        except TransformException as exc:
            tf_detail = f'{tf_detail}: {exc}'
        checks.append(('global robot TF', tf_ok, tf_detail))

        if bool(self.get_parameter('require_rtk').value):
            rtk_ok = self.rtk is not None and int(self.rtk.status.status) >= 0
            detail = self.get_parameter('navsat_topic').value
            if self.rtk is not None:
                detail += f' status={int(self.rtk.status.status)}'
            checks.append(('RTK', rtk_ok, detail))

        all_ok = True
        for name, ok, detail in checks:
            print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
            all_ok = all_ok and ok
        print('DEMO PREFLIGHT PASS' if all_ok else 'DEMO PREFLIGHT FAILED')
        return all_ok


def main(args=None):
    rclpy.init(args=args)
    node = DemoPreflight()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not ok:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
