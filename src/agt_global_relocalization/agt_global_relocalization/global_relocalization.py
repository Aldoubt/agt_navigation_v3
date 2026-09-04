from __future__ import annotations

import json
import math
import os
import shlex
import subprocess
import tempfile
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Empty, String


class GlobalRelocalization(Node):
    def __init__(self):
        super().__init__('agt_global_relocalization')
        p = self.declare_parameter
        p('scan_topic', '/agt/livox/points')
        p('request_topic', '/agt/relocalization/request')
        p('output_pose_topic', '/agt/relocalization/pose')
        p('status_topic', '/agt/global_relocalization/status')
        p('map_frame', 'map')
        p('sdk_command', '')
        p('global_map', '')
        p('work_dir', '~/.ros/agt_global_relocalization')
        p('sdk_timeout_sec', 10.0)
        p('accumulate_clouds', 5)
        p('min_points', 2000)
        p('max_points', 250000)
        p('min_score', 0.50)
        p('max_fitness', 1.00)
        p('min_overlap', 0.20)
        p('best_position_std_m', 0.15)
        p('worst_position_std_m', 0.80)
        p('best_yaw_std_deg', 3.0)
        p('worst_yaw_std_deg', 15.0)

        self.clouds = deque(maxlen=max(1, int(self.get_parameter('accumulate_clouds').value)))
        self.busy = False
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.get_parameter('output_pose_topic').value, 10)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, 10)
        self.create_subscription(PointCloud2, self.get_parameter('scan_topic').value, self.on_cloud, 10)
        self.create_subscription(Empty, self.get_parameter('request_topic').value, self.on_request, 10)

    def on_cloud(self, msg):
        self.clouds.append(msg)

    def status(self, state, detail='', **extra):
        m = String()
        payload = {'state': state, 'detail': detail}
        payload.update(extra)
        m.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(m)
        self.get_logger().info(m.data)

    def on_request(self, _msg):
        if self.busy:
            self.status('BUSY', 'relocalization already running')
            return
        if not self.clouds:
            self.status('FAILED', 'no PointCloud2 scan available')
            return
        self.busy = True
        try:
            self.run_once()
        except Exception as exc:
            self.status('FAILED', str(exc))
        finally:
            self.busy = False

    def merged_points(self):
        rows = []
        max_points = int(self.get_parameter('max_points').value)
        for cloud in list(self.clouds):
            names = [f.name for f in cloud.fields]
            requested = ('x', 'y', 'z', 'intensity') if 'intensity' in names else ('x', 'y', 'z')
            for pt in point_cloud2.read_points(cloud, field_names=requested, skip_nans=True):
                x, y, z = float(pt[0]), float(pt[1]), float(pt[2])
                intensity = float(pt[3]) if len(pt) > 3 else 0.0
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    rows.append((x, y, z, intensity))
                if len(rows) >= max_points:
                    return rows
        return rows

    @staticmethod
    def write_ascii_pcd(path: Path, rows):
        with path.open('w', encoding='utf-8') as f:
            f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n')
            f.write('TYPE F F F F\nCOUNT 1 1 1 1\n')
            f.write(f'WIDTH {len(rows)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {len(rows)}\nDATA ascii\n')
            for x, y, z, intensity in rows:
                f.write(f'{x:.6f} {y:.6f} {z:.6f} {intensity:.3f}\n')

    def run_once(self):
        rows = self.merged_points()
        min_points = int(self.get_parameter('min_points').value)
        if len(rows) < min_points:
            raise RuntimeError(f'not enough scan points: {len(rows)} < {min_points}')

        command_template = str(self.get_parameter('sdk_command').value).strip()
        global_map = os.path.expanduser(str(self.get_parameter('global_map').value))
        if not command_template:
            raise RuntimeError('sdk_command is empty; configure the existing SDK wrapper executable')
        if not global_map or not Path(global_map).is_file():
            raise RuntimeError(f'global_map not found: {global_map!r}')

        work = Path(os.path.expanduser(str(self.get_parameter('work_dir').value)))
        work.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix='query_', dir=work) as td:
            scan_pcd = Path(td) / 'query_scan.pcd'
            self.write_ascii_pcd(scan_pcd, rows)
            timeout = float(self.get_parameter('sdk_timeout_sec').value)
            cmd = command_template.format(scan_pcd=str(scan_pcd), global_map=global_map, timeout_sec=timeout)
            self.status('RUNNING', 'calling SDK backend', points=len(rows))
            proc = subprocess.run(shlex.split(cmd), capture_output=True, text=True, timeout=timeout, check=False)
            if proc.returncode != 0:
                raise RuntimeError(f'SDK returned {proc.returncode}: {proc.stderr.strip()}')
            try:
                result = json.loads(proc.stdout.strip().splitlines()[-1])
            except Exception as exc:
                raise RuntimeError(f'SDK stdout must end with JSON result: {exc}') from exc

        if not bool(result.get('success', False)):
            raise RuntimeError(str(result.get('message', 'SDK reported failure')))
        score = float(result.get('score', 0.0))
        fitness = float(result.get('fitness', math.inf))
        overlap = float(result.get('overlap', 0.0))
        if score < float(self.get_parameter('min_score').value):
            raise RuntimeError(f'score gate failed: {score:.3f}')
        if fitness > float(self.get_parameter('max_fitness').value):
            raise RuntimeError(f'fitness gate failed: {fitness:.3f}')
        if overlap < float(self.get_parameter('min_overlap').value):
            raise RuntimeError(f'overlap gate failed: {overlap:.3f}')

        q = [float(result[k]) for k in ('qx', 'qy', 'qz', 'qw')]
        qn = math.sqrt(sum(v*v for v in q))
        if qn < 1e-9:
            raise RuntimeError('SDK returned invalid quaternion')
        q = [v / qn for v in q]

        score01 = min(1.0, max(0.0, score))
        pos_std = self._lerp('worst_position_std_m', 'best_position_std_m', score01)
        yaw_std = math.radians(self._lerp('worst_yaw_std_deg', 'best_yaw_std_deg', score01))

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.clouds[-1].header.stamp
        msg.header.frame_id = str(self.get_parameter('map_frame').value)
        msg.pose.pose.position.x = float(result['x'])
        msg.pose.pose.position.y = float(result['y'])
        msg.pose.pose.position.z = float(result.get('z', 0.0))
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = pos_std * pos_std
        cov[21] = cov[28] = math.radians(10.0) ** 2
        cov[35] = yaw_std * yaw_std
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)
        self.status('SUCCEEDED', 'global pose published', score=score, fitness=fitness, overlap=overlap,
                    position_std_m=pos_std, yaw_std_deg=math.degrees(yaw_std))

    def _lerp(self, low_name, high_name, t):
        low = float(self.get_parameter(low_name).value)
        high = float(self.get_parameter(high_name).value)
        return low + (high - low) * t


def main(args=None):
    rclpy.init(args=args)
    node = GlobalRelocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
