"""Manual-seed local GICP relocalization for field acceptance.

This node reuses the production query preparation and map snapshot logic from
GlobalRelocalization, but replaces the global BBS stage with an operator-provided
map-frame seed (normally RViz 2D Pose Estimate on /initialpose) followed by the
existing native map_gicp_tracker refinement.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import tempfile
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

from .global_relocalization import GlobalRelocalization


class ManualSeedRelocalization(GlobalRelocalization):
    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        p('manual_seed_topic', '/initialpose')
        p('manual_seed_radius_xy', 12.0)
        p('manual_seed_half_height', 5.0)
        p('manual_seed_max_corr', 1.5)
        p('manual_seed_threads', 4)
        p('manual_seed_timeout_sec', 8.0)
        p('manual_seed_max_fitness', 0.60)
        p('manual_seed_min_overlap', 0.20)
        p('manual_seed_position_std_m', 0.10)
        p('manual_seed_yaw_std_deg', 5.0)

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('manual_seed_topic').value,
            self.on_manual_seed,
            10,
        )
        self.get_logger().info(
            'Manual-seed GICP mode enabled: publish a map-frame seed on '
            f"{self.get_parameter('manual_seed_topic').value!r}.")

    def on_manual_seed(self, msg: PoseWithCovarianceStamped) -> None:
        map_frame = str(self.get_parameter('map_frame').value)
        if msg.header.frame_id and msg.header.frame_id != map_frame:
            self.status(
                'MANUAL_SEED_REJECTED',
                f'expected frame {map_frame!r}, got {msg.header.frame_id!r}',
            )
            return
        if self.busy:
            self.status('BUSY', 'relocalization already running')
            return
        if not self.clouds:
            self.status('MANUAL_SEED_REJECTED', 'no stationary PointCloud2 scan available yet')
            return

        stationary, detail = self.stationary_gate()
        if not stationary:
            self.status('MANUAL_SEED_REJECTED', detail)
            return

        self.busy = True
        try:
            self.run_seeded_once(msg)
        except Exception as exc:
            self.status('MANUAL_GICP_REJECTED', str(exc))
        finally:
            self.busy = False

    @staticmethod
    def _last_json(stdout: str):
        for line in reversed(stdout.splitlines()):
            line = line.strip()
            if not line:
                continue
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                continue
        raise RuntimeError('manual GICP backend returned no JSON result')

    def run_seeded_once(self, seed: PoseWithCovarianceStamped) -> None:
        rows = self.merged_points()
        min_points = int(self.get_parameter('min_points').value)
        if len(rows) < min_points:
            raise RuntimeError(f'not enough scan points: {len(rows)} < {min_points}')

        stamp = self.clouds[-1].header.stamp
        query_frame = str(self.get_parameter('query_frame').value)
        self.query_pub.publish(self.cloud_message(rows, stamp, query_frame))

        global_map, _assets_dir, map_id, map_version, generation = self.resolve_map_inputs()
        if not global_map or not Path(global_map).is_file():
            raise RuntimeError(f'global_map not found: {global_map!r}')

        pose = seed.pose.pose
        q = pose.orientation
        qn = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if qn <= 1e-12:
            raise RuntimeError('manual seed has zero quaternion')
        qx, qy, qz, qw = q.x/qn, q.y/qn, q.z/qn, q.w/qn

        work = Path(os.path.expanduser(str(self.get_parameter('work_dir').value)))
        work.mkdir(parents=True, exist_ok=True)
        timeout = float(self.get_parameter('manual_seed_timeout_sec').value)

        with tempfile.TemporaryDirectory(prefix='manual_seed_', dir=work) as td:
            scan_pcd = Path(td) / 'query_scan.pcd'
            self.write_ascii_pcd(scan_pcd, rows)
            command = [
                'ros2', 'run', 'agt_global_relocalization_native', 'map_gicp_tracker',
                '--map', global_map,
                '--scan', str(scan_pcd),
                '--x', str(float(pose.position.x)),
                '--y', str(float(pose.position.y)),
                '--z', str(float(pose.position.z)),
                '--qx', str(qx), '--qy', str(qy), '--qz', str(qz), '--qw', str(qw),
                '--radius', str(float(self.get_parameter('manual_seed_radius_xy').value)),
                '--half-height', str(float(self.get_parameter('manual_seed_half_height').value)),
                '--max-corr', str(float(self.get_parameter('manual_seed_max_corr').value)),
                '--threads', str(int(self.get_parameter('manual_seed_threads').value)),
            ]
            self.status(
                'MANUAL_GICP_REFINING',
                'refining operator seed against local map',
                seed_x=float(pose.position.x),
                seed_y=float(pose.position.y),
                seed_z=float(pose.position.z),
                points=len(rows),
            )
            completed = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout,
                check=False,
            )
            result = self._last_json(completed.stdout)
            if not result.get('success', False):
                message = result.get('message') or completed.stderr.strip() or 'manual GICP failed'
                raise RuntimeError(message)
            if completed.returncode != 0:
                raise RuntimeError(
                    completed.stderr.strip() or
                    f'manual GICP exited with code {completed.returncode}')

        self.verify_map_snapshot(map_id, map_version, generation)

        fitness = float(result.get('fitness', math.inf))
        overlap = float(result.get('overlap', 0.0))
        if fitness > float(self.get_parameter('manual_seed_max_fitness').value):
            raise RuntimeError(
                f'manual GICP fitness rejected: {fitness:.4f} > '
                f"{float(self.get_parameter('manual_seed_max_fitness').value):.4f}")
        if overlap < float(self.get_parameter('manual_seed_min_overlap').value):
            raise RuntimeError(
                f'manual GICP overlap rejected: {overlap:.3f} < '
                f"{float(self.get_parameter('manual_seed_min_overlap').value):.3f}")

        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = str(self.get_parameter('map_frame').value)
        out.pose.pose.position.x = float(result['x'])
        out.pose.pose.position.y = float(result['y'])
        out.pose.pose.position.z = float(result['z'])
        out.pose.pose.orientation.x = float(result['qx'])
        out.pose.pose.orientation.y = float(result['qy'])
        out.pose.pose.orientation.z = float(result['qz'])
        out.pose.pose.orientation.w = float(result['qw'])

        pos_std = float(self.get_parameter('manual_seed_position_std_m').value)
        yaw_std = math.radians(float(self.get_parameter('manual_seed_yaw_std_deg').value))
        covariance = [0.0] * 36
        covariance[0] = pos_std * pos_std
        covariance[7] = pos_std * pos_std
        covariance[14] = pos_std * pos_std
        covariance[21] = yaw_std * yaw_std
        covariance[28] = yaw_std * yaw_std
        covariance[35] = yaw_std * yaw_std
        out.pose.covariance = covariance

        xyz = (out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z)
        quat = (
            out.pose.pose.orientation.x,
            out.pose.pose.orientation.y,
            out.pose.pose.orientation.z,
            out.pose.pose.orientation.w,
        )
        self.aligned_cloud_pub.publish(
            self.cloud_message(self.apply_pose(rows, xyz, quat), stamp, out.header.frame_id))
        self.pose_pub.publish(out)
        self.status(
            'MANUAL_GICP_ACCEPTED',
            'operator seed refined and published',
            x=xyz[0],
            y=xyz[1],
            z=xyz[2],
            fitness=fitness,
            overlap=overlap,
            map_id=map_id,
            map_version=map_version,
            map_points=int(result.get('map_points', 0)),
            query_points=int(result.get('query_points', 0)),
        )


def main(args=None):
    rclpy.init(args=args)
    node = ManualSeedRelocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
