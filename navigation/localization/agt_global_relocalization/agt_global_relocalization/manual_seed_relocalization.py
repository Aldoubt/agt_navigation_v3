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
        # Opt-in P2.13/P2.14 experiments.  The production-compatible default remains
        # the existing unconstrained native small_gicp path.
        p('gicp_constraint_mode', 'full_se3')
        p('max_roll_delta_deg', 10.0)
        p('max_pitch_delta_deg', 10.0)

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('manual_seed_topic').value,
            self.on_manual_seed,
            10,
        )
        self.get_logger().info(
            'Manual-seed GICP mode enabled: publish a map-frame seed on '
            f"{self.get_parameter('manual_seed_topic').value!r}.")

    def _capture_cloud_contract(self, initial_audit: dict, rows, stamp):
        """Write P2.15 evidence only when the parent capture directory is set."""
        capture_root = str(self.get_parameter('cloud_contract_capture_dir').value).strip()
        contract = self.last_query_cloud_contract
        if not capture_root or not contract:
            return None
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        root = Path(os.path.expanduser(capture_root)) / f'attempt_{stamp_ns}'
        root.mkdir(parents=True, exist_ok=True)
        query_mode, _query_frame, _fixed_transform = self.query_frame_contract()
        query_label = 'body' if query_mode == 'mapping_body' else 'base'
        self.write_ascii_pcd(root / 'query_raw_sensor.pcd', contract['raw_rows'])
        self.write_ascii_pcd(root / f'query_{query_label}_prevoxel.pcd', contract['transformed_rows'])
        self.write_ascii_pcd(root / f'query_{query_label}_gicp.pcd', rows)
        payload = {
            'initial_pose_T_map_base': initial_audit,
            'relocalization_query_frame_mode': str(
                self.get_parameter('relocalization_query_frame_mode').value),
            'query_frame': contract['query_frame'],
            'query_cloud_label': query_label,
            'source_to_query_transforms': contract['transforms'],
            'raw_sensor_point_count': len(contract['raw_rows']),
            'base_prevoxel_point_count': len(contract['transformed_rows']),
            'gicp_query_point_count': len(rows),
        }
        (root / 'query_transform_contract.json').write_text(
            json.dumps(payload, indent=2, sort_keys=True), encoding='utf-8')
        return root

    def _seed_base_to_query_pose(self, base_pose: dict, mode: str) -> dict:
        """Return T_map_query; body mode uses T_map_body=T_map_base*T_base_body."""
        if mode == 'base_link':
            return dict(base_pose)
        return self.compose_pose(base_pose, self.inverse_pose(self.body_to_base_pose()))

    def _query_pose_to_base_pose(self, query_pose: dict, mode: str) -> dict:
        """Return T_map_base; body mode composes T_map_body*T_body_base exactly once."""
        if mode == 'base_link':
            return dict(query_pose)
        return self.compose_pose(query_pose, self.body_to_base_pose())

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

    @staticmethod
    def _rpy_from_xyzw(qx: float, qy: float, qz: float, qw: float) -> dict:
        """Return diagnostic RPY only; registration inputs and outputs are untouched."""
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return {'roll': roll, 'pitch': pitch, 'yaw': yaw}

    @staticmethod
    def _pose_audit(x: float, y: float, z: float,
                    qx: float, qy: float, qz: float, qw: float) -> dict:
        return {
            'x': float(x), 'y': float(y), 'z': float(z),
            'qx': float(qx), 'qy': float(qy), 'qz': float(qz), 'qw': float(qw),
            'rpy_rad': ManualSeedRelocalization._rpy_from_xyzw(qx, qy, qz, qw),
        }

    @staticmethod
    def _relative_pose_delta(initial: dict, refined: dict) -> dict:
        """Diagnostic initial^-1 * refined delta, expressed in initial coordinates."""
        iqx, iqy, iqz, iqw = (initial[k] for k in ('qx', 'qy', 'qz', 'qw'))
        rqx, rqy, rqz, rqw = (refined[k] for k in ('qx', 'qy', 'qz', 'qw'))
        # q_initial^-1 * q_refined; both quaternions were normalized at the API boundary.
        dx = iqw * rqx - iqx * rqw - iqy * rqz + iqz * rqy
        dy = iqw * rqy + iqx * rqz - iqy * rqw - iqz * rqx
        dz = iqw * rqz - iqx * rqy + iqy * rqx - iqz * rqw
        dw = iqw * rqw + iqx * rqx + iqy * rqy + iqz * rqz
        norm = math.sqrt(dx*dx + dy*dy + dz*dz + dw*dw)
        dx, dy, dz, dw = dx/norm, dy/norm, dz/norm, dw/norm
        return {
            'translation_map': {
                'x': refined['x'] - initial['x'],
                'y': refined['y'] - initial['y'],
                'z': refined['z'] - initial['z'],
            },
            'rotation_initial_frame_rpy_rad':
                ManualSeedRelocalization._rpy_from_xyzw(dx, dy, dz, dw),
        }

    def run_seeded_once(self, seed: PoseWithCovarianceStamped) -> None:
        rows = self.merged_points()
        min_points = int(self.get_parameter('min_points').value)
        if len(rows) < min_points:
            raise RuntimeError(f'not enough scan points: {len(rows)} < {min_points}')

        stamp = self.clouds[-1].header.stamp
        query_mode, query_frame, _fixed_query_transform = self.query_frame_contract()
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
        initial_base_audit = self._pose_audit(
            pose.position.x, pose.position.y, pose.position.z, qx, qy, qz, qw)
        initial_query = self._seed_base_to_query_pose(initial_base_audit, query_mode)
        initial_audit = self._pose_audit(
            initial_query['x'], initial_query['y'], initial_query['z'],
            initial_query['qx'], initial_query['qy'], initial_query['qz'], initial_query['qw'])
        contract_dir = self._capture_cloud_contract(initial_base_audit, rows, stamp)

        work = Path(os.path.expanduser(str(self.get_parameter('work_dir').value)))
        work.mkdir(parents=True, exist_ok=True)
        timeout = float(self.get_parameter('manual_seed_timeout_sec').value)
        constraint_mode = str(self.get_parameter('gicp_constraint_mode').value)
        max_roll_delta_deg = float(self.get_parameter('max_roll_delta_deg').value)
        max_pitch_delta_deg = float(self.get_parameter('max_pitch_delta_deg').value)
        if constraint_mode not in ('full_se3', 'gravity_constrained', 'gravity_prior'):
            raise RuntimeError(
                'gicp_constraint_mode must be full_se3, gravity_constrained, or gravity_prior')
        if max_roll_delta_deg < 0.0 or max_pitch_delta_deg < 0.0:
            raise RuntimeError('max_roll_delta_deg and max_pitch_delta_deg must be non-negative')

        with tempfile.TemporaryDirectory(prefix='manual_seed_', dir=work) as td:
            scan_pcd = Path(td) / 'query_scan.pcd'
            self.write_ascii_pcd(scan_pcd, rows)
            command = [
                'ros2', 'run', 'agt_global_relocalization_native', 'map_gicp_tracker',
                '--map', global_map,
                '--scan', str(scan_pcd),
                '--x', str(initial_query['x']),
                '--y', str(initial_query['y']),
                '--z', str(initial_query['z']),
                '--qx', str(initial_query['qx']), '--qy', str(initial_query['qy']),
                '--qz', str(initial_query['qz']), '--qw', str(initial_query['qw']),
                '--radius', str(float(self.get_parameter('manual_seed_radius_xy').value)),
                '--half-height', str(float(self.get_parameter('manual_seed_half_height').value)),
                '--max-corr', str(float(self.get_parameter('manual_seed_max_corr').value)),
                '--threads', str(int(self.get_parameter('manual_seed_threads').value)),
                '--constraint-mode', constraint_mode,
                '--max-roll-delta-deg', str(max_roll_delta_deg),
                '--max-pitch-delta-deg', str(max_pitch_delta_deg),
            ]
            if contract_dir is not None:
                command.extend(['--debug-crop', str(contract_dir / 'local_map_crop.pcd')])
            self.status(
                'MANUAL_GICP_REFINING',
                'refining operator seed against local map',
                seed_x=float(pose.position.x),
                seed_y=float(pose.position.y),
                seed_z=float(pose.position.z),
                points=len(rows),
                relocalization_query_frame_mode=query_mode,
                query_frame=query_frame,
                initial_pose_T_map_base=initial_base_audit,
                initial_pose_T_map_query=initial_audit,
                gicp_constraint_mode=constraint_mode,
                max_roll_delta_deg=max_roll_delta_deg,
                max_pitch_delta_deg=max_pitch_delta_deg,
                cloud_contract_dir=str(contract_dir) if contract_dir else '',
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
                if all(key in result for key in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')):
                    failed_refined = self._pose_audit(
                        result['x'], result['y'], result['z'],
                        result['qx'], result['qy'], result['qz'], result['qw'])
                    failed_refined_base = self._query_pose_to_base_pose(
                        failed_refined, query_mode)
                    self.status(
                        'MANUAL_GICP_RESULT',
                        'native refinement did not converge; acceptance gates not evaluated',
                        gicp_constraint_mode=str(result.get('constraint_mode', constraint_mode)),
                        relocalization_query_frame_mode=query_mode,
                        initial_pose_T_map_base=initial_base_audit,
                        initial_pose_T_map_query=initial_audit,
                        refined_pose_T_map_query=failed_refined,
                        refined_pose_T_map_base=failed_refined_base,
                        initial_to_refined_query=self._relative_pose_delta(initial_audit, failed_refined),
                        initial_to_refined_base=self._relative_pose_delta(
                            initial_base_audit, failed_refined_base),
                        fitness=result.get('fitness'), overlap=result.get('overlap'),
                        map_points=int(result.get('map_points', 0)),
                        query_points=int(result.get('query_points', 0)),
                        prior_violation=bool(result.get('gravity_prior_clipped', False)),
                        roll_delta_deg=result.get('roll_delta_deg'),
                        pitch_delta_deg=result.get('pitch_delta_deg'),
                        converged=False,
                    )
                message = result.get('message') or completed.stderr.strip() or 'manual GICP failed'
                raise RuntimeError(message)
            if completed.returncode != 0:
                raise RuntimeError(
                    completed.stderr.strip() or
                    f'manual GICP exited with code {completed.returncode}')

        self.verify_map_snapshot(map_id, map_version, generation)

        fitness = float(result.get('fitness', math.inf))
        overlap = float(result.get('overlap', 0.0))
        backend_refined_audit = self._pose_audit(
            result['x'], result['y'], result['z'],
            result['qx'], result['qy'], result['qz'], result['qw'])
        backend_refined_base = self._query_pose_to_base_pose(
            backend_refined_audit, query_mode)
        backend_refinement_delta = self._relative_pose_delta(initial_audit, backend_refined_audit)
        backend_base_refinement_delta = self._relative_pose_delta(
            initial_base_audit, backend_refined_base)
        # Preserve the native result before acceptance gates.  This is
        # diagnostic-only and is especially important for rejected experiments.
        self.status(
            'MANUAL_GICP_RESULT',
            'native refinement completed; acceptance gates pending',
            gicp_constraint_mode=str(result.get('constraint_mode', constraint_mode)),
            relocalization_query_frame_mode=query_mode,
            initial_pose_T_map_base=initial_base_audit,
            initial_pose_T_map_query=initial_audit,
            refined_pose_T_map_query=backend_refined_audit,
            refined_pose_T_map_base=backend_refined_base,
            initial_to_refined_query=backend_refinement_delta,
            initial_to_refined_base=backend_base_refinement_delta,
            fitness=fitness,
            overlap=overlap,
            map_points=int(result.get('map_points', 0)),
            query_points=int(result.get('query_points', 0)),
            prior_violation=bool(result.get('gravity_prior_clipped', False)),
            roll_delta_deg=result.get('roll_delta_deg'),
            pitch_delta_deg=result.get('pitch_delta_deg'),
        )
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
        out.pose.pose.position.x = backend_refined_base['x']
        out.pose.pose.position.y = backend_refined_base['y']
        out.pose.pose.position.z = backend_refined_base['z']
        out.pose.pose.orientation.x = backend_refined_base['qx']
        out.pose.pose.orientation.y = backend_refined_base['qy']
        out.pose.pose.orientation.z = backend_refined_base['qz']
        out.pose.pose.orientation.w = backend_refined_base['qw']

        refined_audit = self._pose_audit(
            out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z,
            out.pose.pose.orientation.x, out.pose.pose.orientation.y,
            out.pose.pose.orientation.z, out.pose.pose.orientation.w)
        refinement_delta = self._relative_pose_delta(initial_base_audit, refined_audit)

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
        quat = (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
                out.pose.pose.orientation.z, out.pose.pose.orientation.w)
        # rows are in query_frame.  In mapping_body that is body, so the map
        # visualization must use native T_map_body, while the published pose
        # remains the converted T_map_base required by LocalizationManager.
        query_xyz = (backend_refined_audit['x'], backend_refined_audit['y'],
                     backend_refined_audit['z'])
        query_quat = (backend_refined_audit['qx'], backend_refined_audit['qy'],
                      backend_refined_audit['qz'], backend_refined_audit['qw'])
        self.aligned_cloud_pub.publish(
            self.cloud_message(self.apply_pose(rows, query_xyz, query_quat), stamp, out.header.frame_id))
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
            gicp_constraint_mode=str(result.get('constraint_mode', constraint_mode)),
            relocalization_query_frame_mode=query_mode,
            query_frame=query_frame,
            initial_pose_T_map_base=initial_base_audit,
            initial_pose_T_map_query=initial_audit,
            refined_pose_T_map_query=backend_refined_audit,
            refined_pose_T_map_base=refined_audit,
            initial_to_refined_query=backend_refinement_delta,
            initial_to_refined_base=refinement_delta,
            published_pose=refined_audit,
            refined_to_published_translation_m=0.0,
            refined_to_published_rotation_rad=0.0,
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
