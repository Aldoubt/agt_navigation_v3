from __future__ import annotations

import csv
import math
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


class CaptureCases(Node):
    """Capture de-skewed body-frame mapping clouds with a map-frame reference pose."""

    def __init__(self):
        super().__init__('agt_relocalization_capture_cases')
        p = self.declare_parameter
        p('cloud_topic', '/fastlio2/body_cloud')
        p('reference_frame', 'map')
        p('query_frame', 'body')
        p('output_dir', '~/.ros/agt_relocalization_benchmark/cases')
        p('sample_interval_sec', 5.0)
        p('min_points', 500)
        p('max_points', 120000)
        p('tf_timeout_sec', 0.20)

        self.output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.output_dir / 'cases.csv'
        self.last_stamp_ns = None
        self.case_index = 0
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._ensure_header()
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value, self.on_cloud, 10)
        self.get_logger().info(f'Capturing benchmark cases into {self.output_dir}')

    def _ensure_header(self):
        if self.csv_path.exists() and self.csv_path.stat().st_size > 0:
            with self.csv_path.open('r', encoding='utf-8') as f:
                rows = list(csv.DictReader(f))
                self.case_index = len(rows)
            return
        with self.csv_path.open('w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                'case_id', 'stamp_sec', 'pcd', 'points',
                'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
            ])

    @staticmethod
    def _stamp_ns(msg):
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    @staticmethod
    def _write_pcd(path: Path, xyz):
        with path.open('w', encoding='utf-8') as f:
            f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n')
            f.write('TYPE F F F\nCOUNT 1 1 1\n')
            f.write(f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(xyz)}\nDATA ascii\n')
            for x, y, z in xyz:
                f.write(f'{x:.6f} {y:.6f} {z:.6f}\n')

    def on_cloud(self, msg: PointCloud2):
        stamp_ns = self._stamp_ns(msg)
        interval_ns = int(float(self.get_parameter('sample_interval_sec').value) * 1e9)
        if self.last_stamp_ns is not None and stamp_ns - self.last_stamp_ns < interval_ns:
            return

        query_frame = str(self.get_parameter('query_frame').value)
        if msg.header.frame_id != query_frame:
            self.get_logger().warning(
                f'skip cloud frame={msg.header.frame_id!r}; expected de-skewed query_frame={query_frame!r}')
            return
        reference_frame = str(self.get_parameter('reference_frame').value)
        timeout = Duration(seconds=float(self.get_parameter('tf_timeout_sec').value))
        try:
            tf = self.tf_buffer.lookup_transform(
                reference_frame, query_frame, Time.from_msg(msg.header.stamp), timeout=timeout)
        except TransformException as exc:
            self.get_logger().warning(f'skip case: TF {reference_frame} <- {query_frame} unavailable: {exc}')
            return

        max_points = int(self.get_parameter('max_points').value)
        xyz = []
        for pt in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                xyz.append((x, y, z))
            if len(xyz) >= max_points:
                break
        min_points = int(self.get_parameter('min_points').value)
        if len(xyz) < min_points:
            self.get_logger().warning(f'skip sparse case: {len(xyz)} < {min_points}')
            return

        case_id = f'C{self.case_index:04d}'
        pcd_name = f'{case_id}.pcd'
        self._write_pcd(self.output_dir / pcd_name, xyz)
        tr = tf.transform.translation
        q = tf.transform.rotation
        stamp_sec = stamp_ns / 1e9
        with self.csv_path.open('a', newline='', encoding='utf-8') as f:
            csv.writer(f).writerow([
                case_id, f'{stamp_sec:.9f}', pcd_name, len(xyz),
                tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w,
            ])
        self.case_index += 1
        self.last_stamp_ns = stamp_ns
        self.get_logger().info(
            f'captured {case_id}: points={len(xyz)} ref=({tr.x:.2f},{tr.y:.2f},{tr.z:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = CaptureCases()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
