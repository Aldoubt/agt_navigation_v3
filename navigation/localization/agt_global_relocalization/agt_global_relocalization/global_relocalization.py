import copy
import json
import math
import os
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

from agt_common.bag_utils import resolve_bag
from agt_global_relocalization.map_tracking import (
    MapTracker, inertial_odom_message, trajectory_message,
)
from agt_global_relocalization.runtime_preflight import (
    Check, run_preflight, write_preflight,
)
from .registration_backend import SmallGICPBackend


class GlobalRelocalization(Node):
    def __init__(self):
        super().__init__('agt_global_relocalization')
        p = self.declare_parameter
        p('cloud_topic', '/agt/scan_body')
        p('odom_topic', '/agt/odometry/local')
        p('stationary_odom_topic', '/wheel/odom')
        p('candidate_topic', '/agt/global_pose_candidate')
        p('debug_state_topic', '/agt/relocalization/status')
        p('stationary_filter_window_samples', 5)
        p('stationary_linear_threshold_mps', .04)
        p('stationary_angular_threshold_rps', .03)
        p('stationary_hard_linear_threshold_mps', .15)
        p('stationary_hard_angular_threshold_rps', .20)
        p('global_backend', 'bbs')
        p('global_map', '')
        p('global_assets_dir', '')
        p('candidate_assets_dir', '')
        p('bbs_sdk_command', '')
        p('candidate_sdk_command', '')
        p('kiss_sdk_command', '')
        p('relocalization_query_frame_mode', 'mapping_body')
        p('bbs_query_frame_mode', 'mapping_body')
        p('gicp_timeout_sec', 90.0)
        p('map_frame', 'map')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('accumulate_clouds', 5)
        p('accumulate_max_clouds', 20)
        p('accumulate_min_duration_sec', 1.0)
        p('accumulate_min_points', 20000)
        p('accumulate_max_gap_sec', 0.35)
        p('voxel_leaf', 0.15)
        p('use_sim_time', False)
        p('relocalization_assets', '')
        p('auto_relocalize', True)
        p('auto_interval_sec', 60.0)
        p('min_score', 0.0)
        p('max_fitness', 0.5)
        p('min_overlap', 0.0)
        p('max_position_std_m', 1.0)
        p('max_yaw_std_deg', 20.0)
        p('pose_topic', '/agt/global_pose')
        p('tracking_enabled', False)
        p('tracking_process_topic', '/agt/tracking/odom')
        p('tracking_map_frame', 'agt_map_odom')
        p('tracking_map_frame', '')
        p('tracking_min_overlap', 0.0)
        p('tracking_max_fitness', 1.0)
        p('tracking_voxel_leaf', 0.15)
        p('tracking_local_map_radius_xy', 35.0)
        p('tracking_local_map_half_height', 8.0)
        p('tracking_min_local_map_points', 800)
        p('tracking_min_consecutive_good', 2)
        p('tracking_max_consecutive_bad', 3)
        p('tracking_max_position_jump_m', 1.0)
        p('tracking_max_yaw_jump_deg', 30.0)
        p('tracking_max_skew_sec', 0.15)
        p('tracking_min_odom_hz', 2.0)
        p('tracking_odom_topic', '/agt/odometry/local')
        p('body_to_base_calibration_file', '')
        p('batch_lio_config_file', '')
        p('mount_lidar_frame', 'livox_frame')
        p('mount_base_frame', 'base_link')
        p('tf_timeout_sec', 0.2)
        p('mapping_body_livox_translation', [-0.011, -0.02329, 0.04412])
        p('mapping_body_livox_rotation', [1., 0., 0., 0., 1., 0., 0., 0., 1.])
        self.mode, self.alias = self.normalize_query_frame_mode(
            self.get_parameter('relocalization_query_frame_mode').value)
        self.bbs_mode, self.bbs_alias = self.normalize_query_frame_mode(
            self.get_parameter('bbs_query_frame_mode').value)
        self.clouds = []
        self.query_scan_stamp_ns = None
        self.latest_cloud_stamp_ns = None
        self.odom = deque(maxlen=2000)
        self.motion_samples = deque(maxlen=20)
        self.latest_motion = None
        self.pending_request = False
        self.auto_started = time.monotonic()
        self.last_auto_request = -math.inf
        self.last_candidate = None
        self._body_to_base_cache = None
        self._run_dir = Path('/tmp/agt_global_relocalization')
        self._run_dir.mkdir(parents=True, exist_ok=True)
        self._gicp = SmallGICPBackend(self, self._run_dir)
        self._tracker = MapTracker(self, self._run_dir)
        self._gicp_lock = threading.Lock()
        self._relocalization_thread = None
        self._tracking_thread = None
        self._latest_map_odom = None
        self._shutdown = False
        self.create_subscription(PointCloud2, self.get_parameter('cloud_topic').value,
                                 self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._on_odom, 50)
        stationary_topic = self.get_parameter('stationary_odom_topic').value
        if stationary_topic == self.get_parameter('odom_topic').value:
            self.get_logger().warn(
                'stationary_odom_topic must be independent wheel odometry, not the '
                'laser-inertial local odometry; falling back to configured wheel topic')
            stationary_topic = '/wheel/odom'
        self.create_subscription(Odometry, stationary_topic,
                                 self._on_stationary_odom, 50)
        self.create_service(Trigger, '/agt/relocalize', self._on_relocalize)
        self.status_pub = self.create_publisher(String,
                                                self.get_parameter('debug_state_topic').value, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                               self.get_parameter('pose_topic').value, 10)
        self.candidate_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter('candidate_topic').value, 10)
        self.inertial_pub = self.create_publisher(Odometry, '/agt/tracking/inertial_odom', 20)
        self.trajectory_pub = self.create_publisher(String, '/agt/tracking/trajectory', 10)
        self.timer = self.create_timer(1.0, self._tick)
        self._tf_buffer = None
        self._tf_listener = None
        if self.mode == 'base_link' or self.bbs_mode == 'base_link':
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        self.status('READY', 'global_relocalization_ready',
                    backend=self.get_parameter('global_backend').value,
                    query_frame_mode=self.mode, query_frame_alias=self.alias,
                    bbs_query_frame_mode=self.bbs_mode,
                    bbs_query_frame_alias=self.bbs_alias,
                    map_tracking=self.get_parameter('tracking_enabled').value)

    @staticmethod
    def normalize_query_frame_mode(value):
        mode = str(value).strip().lower()
        if mode == 'body_aligned':
            return 'mapping_body', 'body_aligned'
        if mode not in ('mapping_body', 'base_link'):
            raise RuntimeError('relocalization query frame must be mapping_body or base_link')
        return mode, ''

    @staticmethod
    def has_complete_query(cloud_count, required_clouds):
        return cloud_count >= max(1, int(required_clouds))

    @staticmethod
    def query_window_ready(cloud_count, duration_sec, point_count,
                           required_clouds, min_duration_sec, min_points):
        return (GlobalRelocalization.has_complete_query(cloud_count, required_clouds)
                and float(duration_sec) >= float(min_duration_sec)
                and int(point_count) >= int(min_points))

    @staticmethod
    def robust_motion(samples, window):
        values = list(samples)[-max(1, int(window)):]
        if len(values) < max(1, int(window)):
            return None
        linear = float(np.median([abs(float(x[0])) for x in values]))
        angular = float(np.median([abs(float(x[1])) for x in values]))
        return linear, angular

    @staticmethod
    def observation_timestamp_error(previous_ns, current_ns, kind='local_odom'):
        if int(current_ns) <= 0:
            return kind + '_zero_timestamp'
        if previous_ns is not None and int(current_ns) <= int(previous_ns):
            return kind + '_timestamp_regression'
        return None

    @staticmethod
    def message_stamp_ns(msg):
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _reject_observation(self, reason, previous_ns, current_ns):
        self.clouds.clear()
        self.pending_request = False
        if reason.startswith('local_odom_'):
            self.odom.clear()
            self.motion_samples.clear()
            self.latest_motion = None
        self.status('REJECTED', reason, observation_error=reason,
                    previous_stamp_ns=previous_ns, current_stamp_ns=current_ns)
        self.get_logger().warn(
            'Rejected observation %s: previous=%s current=%s; query window and '
            'pending request cleared' % (reason, previous_ns, current_ns))

    @staticmethod
    def load_lio_body_to_lidar(path):
        import yaml
        values = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        t = values['t_il']
        r = values['r_il']
        if len(t) != 3 or len(r) != 9:
            raise RuntimeError('LIO calibration must contain t_il[3] and r_il[9]')
        # FAST-LIO/Batch-LIO estimates extrinsic LiDAR-to-IMU: p_imu = R_il*p_lidar+t_il.
        # Return the inverse because query and published base use the opposite direction.
        matrix = np.asarray(r, dtype=float).reshape(3, 3)
        if not np.all(np.isfinite(matrix)) or abs(np.linalg.det(matrix) - 1.) > 1.e-3:
            raise RuntimeError('LIO calibration r_il is not a proper rotation')
        rotation = matrix.T
        translation = -rotation @ np.asarray(t, dtype=float)
        trace = np.trace(rotation)
        cosine = np.clip((trace - 1.) * .5, -1., 1.)
        angle = math.acos(cosine)
        if angle < 1.e-10:
            q = np.array([0., 0., 0., 1.])
        elif abs(math.pi - angle) < 1.e-5:
            axis = np.sqrt(np.maximum(np.diag(rotation) + 1., 0.) / 2.)
            axis[np.argmax(axis)] = max(axis[np.argmax(axis)], 1.e-12)
            axis = rotation @ axis
            axis /= np.linalg.norm(axis)
            q = np.r_[axis * math.sin(angle / 2.), math.cos(angle / 2.)]
        else:
            axis = np.array([rotation[2, 1] - rotation[1, 2],
                             rotation[0, 2] - rotation[2, 0],
                             rotation[1, 0] - rotation[0, 1]]) / (2. * math.sin(angle))
            q = np.r_[axis * math.sin(angle / 2.), math.cos(angle / 2.)]
        return {'x': float(translation[0]), 'y': float(translation[1]), 'z': float(translation[2]),
                'qx': float(q[0]), 'qy': float(q[1]), 'qz': float(q[2]), 'qw': float(q[3])}

    def body_to_base_pose(self):
        if self._body_to_base_cache is not None:
            return dict(self._body_to_base_cache)
        calibration = self.get_parameter('body_to_base_calibration_file').value
        legacy = self.get_parameter('batch_lio_config_file').value
        calibration = calibration or legacy
        if calibration:
            try:
                pose = self.load_lio_body_to_lidar(calibration)
                self._body_to_base_cache = pose
                self.get_logger().info('Using explicit frozen LIO body-to-base calibration')
                return dict(pose)
            except Exception as exc:
                raise RuntimeError('Invalid explicit LIO body-to-base calibration: %s' % exc)
        if self._tf_buffer is not None:
            try:
                from rclpy.time import Time
                transform = self._tf_buffer.lookup_transform(
                    self.get_parameter('mount_base_frame').value,
                    self.get_parameter('mount_lidar_frame').value, Time(),
                    timeout=rclpy.duration.Duration(
                        seconds=self.get_parameter('tf_timeout_sec').value))
                t = transform.transform.translation
                q = transform.transform.rotation
                pose = {'x': t.x, 'y': t.y, 'z': t.z,
                        'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}
                self._body_to_base_cache = pose
                return dict(pose)
            except Exception as exc:
                raise RuntimeError('Unable to obtain body-to-base mount TF: %s' % exc)
        raise RuntimeError('base_link compatibility requires explicit calibration or mount TF')

    @staticmethod
    def _rotate_xyz(x, y, z, q):
        qx, qy, qz, qw = q
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm < 1.e-12:
            raise ValueError('zero quaternion')
        qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
        # v + 2*q_w*(q_vec x v) + 2*q_vec x (q_vec x v)
        tx = 2.*(qy*z - qz*y)
        ty = 2.*(qz*x - qx*z)
        tz = 2.*(qx*y - qy*x)
        return (x + qw*tx + qy*tz - qz*ty,
                y + qw*ty + qz*tx - qx*tz,
                z + qw*tz + qx*ty - qy*tx)

    @staticmethod
    def _normalized_xyzw(q, label='pose'):
        norm = math.sqrt(sum(float(v) * float(v) for v in q))
        if not math.isfinite(norm) or norm < 1.e-8:
            raise ValueError(label + ' has a zero or non-finite quaternion')
        return tuple(float(v) / norm for v in q)

    @staticmethod
    def compose_pose(a, b):
        """Compose transforms a then b, each in x/y/z/qx/qy/qz/qw form."""
        aq = GlobalRelocalization._normalized_xyzw(
            (a['qx'], a['qy'], a['qz'], a['qw']), 'left pose')
        bq = GlobalRelocalization._normalized_xyzw(
            (b['qx'], b['qy'], b['qz'], b['qw']), 'right pose')
        v = GlobalRelocalization._rotate_xyz(b['x'], b['y'], b['z'], aq)
        ax, ay, az, aw = aq
        bx, by, bz, bw = bq
        return {'x': a['x'] + v[0], 'y': a['y'] + v[1], 'z': a['z'] + v[2],
                'qx': aw*bx + ax*bw + ay*bz - az*by,
                'qy': aw*by - ax*bz + ay*bw + az*bx,
                'qz': aw*bz + ax*by - ay*bx + az*bw,
                'qw': aw*bw - ax*bx - ay*by - az*bz}

    @staticmethod
    def inverse_pose(p):
        q = GlobalRelocalization._normalized_xyzw(
            (p['qx'], p['qy'], p['qz'], p['qw']), 'pose')
        inv_q = (-q[0], -q[1], -q[2], q[3])
        t = GlobalRelocalization._rotate_xyz(-p['x'], -p['y'], -p['z'], inv_q)
        return {'x': t[0], 'y': t[1], 'z': t[2],
                'qx': inv_q[0], 'qy': inv_q[1], 'qz': inv_q[2], 'qw': inv_q[3]}

    def query_pose_to_base_pose(self, pose, mode):
        if mode == 'base_link':
            return dict(pose)
        # Native registration aligns query points expressed in the LIO body frame.
        # T_map_base = T_map_body * T_body_base, exactly once at this publication boundary.
        return self.compose_pose(pose, self.body_to_base_pose())

    def _on_odom(self, msg):
        if msg.header.frame_id != self.get_parameter('odom_frame').value or msg.child_frame_id != self.get_parameter('base_frame').value:
            self.get_logger().warn('Ignoring odometry with wrong frames')
            return
        stamp_ns = self.message_stamp_ns(msg)
        previous_ns = self.odom[-1][0] if self.odom else None
        error = self.observation_timestamp_error(previous_ns, stamp_ns)
        if error:
            self._reject_observation(error, previous_ns, stamp_ns)
            return
        self.odom.append((stamp_ns, msg))

    def _on_stationary_odom(self, msg):
        if msg.header.frame_id != self.get_parameter('odom_frame').value or msg.child_frame_id != self.get_parameter('base_frame').value:
            return
        stamp_ns = self.message_stamp_ns(msg)
        previous_ns = self.latest_motion[0] if self.latest_motion else None
        error = self.observation_timestamp_error(previous_ns, stamp_ns)
        if error:
            self._reject_observation(error, previous_ns, stamp_ns)
            return
        linear = float(msg.twist.twist.linear.x)
        angular = float(msg.twist.twist.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            return
        self.motion_samples.append((linear, angular))
        self.latest_motion = (stamp_ns, linear, angular)

    def stationary_now(self, stamp_ns):
        if self.latest_motion is None:
            return False, 'wheel_odom_missing'
        motion_stamp, _, _ = self.latest_motion
        if abs(stamp_ns - motion_stamp) > 350_000_000:
            return False, 'wheel_odom_not_time_aligned'
        motion = self.robust_motion(
            self.motion_samples, self.get_parameter('stationary_filter_window_samples').value)
        if motion is None:
            return False, 'wheel_odom_window_incomplete'
        linear, angular = motion
        hard_linear = float(self.get_parameter('stationary_hard_linear_threshold_mps').value)
        hard_angular = float(self.get_parameter('stationary_hard_angular_threshold_rps').value)
        if any(abs(float(v[0])) > hard_linear or abs(float(v[1])) > hard_angular
               for v in self.motion_samples):
            return False, 'wheel_odom_motion_spike'
        if linear > self.get_parameter('stationary_linear_threshold_mps').value or angular > self.get_parameter('stationary_angular_threshold_rps').value:
            return False, 'wheel_odom_not_stationary'
        return True, 'wheel_odom_stationary'

    def _on_cloud(self, msg):
        if self.mode == 'base_link' and msg.header.frame_id not in ('base_link', 'body'):
            self.get_logger().warn('Ignoring cloud with unexpected frame: ' + msg.header.frame_id)
            return
        stamp_ns = self.message_stamp_ns(msg)
        error = self.observation_timestamp_error(self.latest_cloud_stamp_ns, stamp_ns, 'point_cloud')
        if error:
            self._reject_observation(error, self.latest_cloud_stamp_ns, stamp_ns)
            return
        self.latest_cloud_stamp_ns = stamp_ns
        self.clouds.append(msg)
        maximum = max(self.get_parameter('accumulate_clouds').value,
                      self.get_parameter('accumulate_max_clouds').value)
        if len(self.clouds) > maximum:
            self.clouds = self.clouds[-maximum:]
        if self.pending_request or not self.get_parameter('auto_relocalize').value:
            return
        if time.monotonic() - self.last_auto_request < self.get_parameter('auto_interval_sec').value:
            return
        if not self._query_ready():
            return
        stationary, _ = self.stationary_now(stamp_ns)
        if not stationary:
            return
        self._request_relocalization(auto=True)

    def _query_ready(self):
        if not self.clouds:
            return False
        duration = (self.message_stamp_ns(self.clouds[-1]) - self.message_stamp_ns(self.clouds[0])) / 1.e9
        points = sum(cloud.width * cloud.height for cloud in self.clouds)
        return self.query_window_ready(
            len(self.clouds), duration, points,
            self.get_parameter('accumulate_clouds').value,
            self.get_parameter('accumulate_min_duration_sec').value,
            self.get_parameter('accumulate_min_points').value)

    def _request_relocalization(self, auto=False):
        if self.pending_request:
            return False, 'relocalization already pending'
        if not self._query_ready():
            return False, 'insufficient spatially registered query window'
        stamp_ns = self.message_stamp_ns(self.clouds[-1])
        stationary, reason = self.stationary_now(stamp_ns)
        if not stationary:
            return False, reason
        self.pending_request = True
        self.last_auto_request = time.monotonic() if auto else self.last_auto_request
        self._relocalization_thread = threading.Thread(target=self._run_relocalization, daemon=True)
        self._relocalization_thread.start()
        return True, 'relocalization started'

    def _run_relocalization(self):
        clouds = list(self.clouds)
        scan_stamp_ns = self.message_stamp_ns(clouds[-1])
        try:
            self.status('RUNNING', 'accumulated_query_registration_started')
            query_pcd, metadata = self._gicp.build_query(clouds, self.odom)
            result = self._gicp.register_global(query_pcd, scan_stamp_ns)
            if not result.get('success'):
                self.status('REJECTED', result.get('message', 'global registration failed'))
                return
            # Native result remains mapping-body until the common base-link conversion.
            base_result = self.query_pose_to_base_pose(result, self.mode)
            candidate = self._pose_message(base_result, scan_stamp_ns)
            self.candidate_pub.publish(candidate)
            self.pose_pub.publish(candidate)
            self.last_candidate = candidate
            self._latest_map_odom = self._map_odom_from_candidate(base_result, scan_stamp_ns)
            self.status('ACCEPTED', 'global_candidate_published',
                        score=result.get('score'), fitness=result.get('fitness'),
                        overlap=result.get('overlap'), query_frame_mode=self.mode)
        except Exception as exc:
            self.get_logger().error('relocalization failed: %s' % exc)
            self.status('REJECTED', str(exc))
        finally:
            self.pending_request = False
            self.clouds.clear()

    def _pose_message(self, pose, stamp_ns):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.get_parameter('map_frame').value
        msg.header.stamp.sec = stamp_ns // 1_000_000_000
        msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
        msg.pose.pose.position.x = float(pose['x'])
        msg.pose.pose.position.y = float(pose['y'])
        msg.pose.pose.position.z = float(pose['z'])
        msg.pose.pose.orientation.x = float(pose['qx'])
        msg.pose.pose.orientation.y = float(pose['qy'])
        msg.pose.pose.orientation.z = float(pose['qz'])
        msg.pose.pose.orientation.w = float(pose['qw'])
        position_var = float(self.get_parameter('max_position_std_m').value) ** 2
        yaw_var = math.radians(float(self.get_parameter('max_yaw_std_deg').value)) ** 2
        msg.pose.covariance[0] = position_var
        msg.pose.covariance[7] = position_var
        msg.pose.covariance[14] = position_var
        msg.pose.covariance[35] = yaw_var
        return msg

    def _map_odom_from_candidate(self, map_base, stamp_ns):
        odom = self._nearest_odom(stamp_ns)
        if odom is None:
            return None
        t = odom.pose.pose.position
        q = odom.pose.pose.orientation
        odom_base = {'x': t.x, 'y': t.y, 'z': t.z,
                     'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}
        return self.compose_pose(map_base, self.inverse_pose(odom_base))

    def _nearest_odom(self, stamp_ns):
        values = [value for value in self.odom if abs(value[0] - stamp_ns) <= 100_000_000]
        if not values:
            return None
        return min(values, key=lambda value: abs(value[0] - stamp_ns))[1]

    def _on_relocalize(self, request, response):
        success, detail = self._request_relocalization()
        response.success = success
        response.message = detail
        if not success:
            self.status('REJECTED', detail)
        return response

    def _tick(self):
        if self.get_parameter('tracking_enabled').value:
            self._tracker.tick()

    def status(self, state, detail, **extra):
        msg = String()
        payload = {'state': state, 'detail': detail, 'backend': self.get_parameter('global_backend').value,
                   'pending_request': self.pending_request, 'clouds': len(self.clouds),
                   'query_frame_mode': self.mode, 'timestamp': self.get_clock().now().nanoseconds}
        payload.update(extra)
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalRelocalization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
