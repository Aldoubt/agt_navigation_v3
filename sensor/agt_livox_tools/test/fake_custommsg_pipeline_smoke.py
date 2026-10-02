#!/usr/bin/env python3
"""Send one synthetic Livox CustomMsg through a running format bridge.

Start only ``ros2 run agt_livox_tools livox_format_bridge`` in the same isolated
ROS_DOMAIN_ID first. This probe never opens a sensor or hardware device.
"""

import math
import struct
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


class FakeLivoxProbe(Node):
    def __init__(self):
        super().__init__('p6_fake_livox_probe')
        self.publisher = self.create_publisher(
            CustomMsg, '/livox/lidar', qos_profile_sensor_data)
        self.clouds = []
        self.subscription = self.create_subscription(
            PointCloud2, '/agt/livox/points', self.clouds.append,
            qos_profile_sensor_data)


def make_sample():
    msg = CustomMsg()
    msg.header.frame_id = 'livox_frame'
    msg.header.stamp.sec = 1800000000
    msg.header.stamp.nanosec = 123456000
    msg.timebase = 1800000000123456000
    msg.point_num = 1
    msg.lidar_id = 0
    point = CustomPoint()
    point.x, point.y, point.z = 1.25, -2.5, 0.75
    point.reflectivity, point.tag, point.line = 42, 16, 7
    point.offset_time = 123456789
    msg.points = [point]
    return msg


def main():
    rclpy.init()
    node = FakeLivoxProbe()
    sample = make_sample()
    try:
        deadline = time.monotonic() + 10.0
        while not node.clouds and time.monotonic() < deadline:
            node.publisher.publish(sample)
            rclpy.spin_once(node, timeout_sec=0.05)
        assert node.clouds, 'no secondary PointCloud2 arrived within 10 seconds'
        cloud = node.clouds[0]
        fields = {field.name: field for field in cloud.fields}
        required = {'x', 'y', 'z', 'intensity', 'tag', 'line', 'offset_time', 'timestamp'}
        assert required.issubset(fields), f'missing fields: {required - set(fields)}'
        assert cloud.width == 1 and cloud.height == 1
        assert cloud.header.frame_id == 'livox_frame'
        assert (cloud.header.stamp.sec, cloud.header.stamp.nanosec) == (
            sample.header.stamp.sec, sample.header.stamp.nanosec)

        formats = {
            PointField.FLOAT32: 'f', PointField.UINT8: 'B',
            PointField.UINT32: 'I', PointField.FLOAT64: 'd',
        }

        def value(name):
            field = fields[name]
            return struct.unpack_from(
                '<' + formats[field.datatype], bytes(cloud.data), field.offset)[0]

        assert math.isclose(value('x'), 1.25, abs_tol=1e-6)
        assert math.isclose(value('y'), -2.5, abs_tol=1e-6)
        assert math.isclose(value('z'), 0.75, abs_tol=1e-6)
        assert int(value('intensity')) == 42
        assert int(value('tag')) == 16 and int(value('line')) == 7
        assert int(value('offset_time')) == 123456789
        expected_time = (sample.timebase + sample.points[0].offset_time) * 1e-9
        assert math.isclose(value('timestamp'), expected_time, abs_tol=2e-7)
        print('PASS fake CustomMsg -> /agt/livox/points; geometry and point timing preserved')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
