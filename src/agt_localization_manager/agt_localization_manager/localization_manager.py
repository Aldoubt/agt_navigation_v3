from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from agt_robot_interfaces.msg import LocalizationStatus


@dataclass(frozen=True)
class _Pose3:
    p: Tuple[float, float, float]
    q: Tuple[float, float, float, float]  # x y z w


def _q_normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if n <= 1e-12:
        raise ValueError('zero quaternion')
    return tuple(v / n for v in q)


def _q_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _q_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _q_rotate(q, v):
    q = _q_normalize(q)
    vq = (v[0], v[1], v[2], 0.0)
    rq = _q_multiply(_q_multiply(q, vq), _q_conjugate(q))
    return rq[0], rq[1], rq[2]


def _inverse(t: _Pose3) -> _Pose3:
    q_inv = _q_conjugate(_q_normalize(t.q))
    p_inv = _q_rotate(q_inv, (-t.p[0], -t.p[1], -t.p[2]))
    return _Pose3(p_inv, q_inv)


def _compose(a: _Pose3, b: _Pose3) -> _Pose3:
    bp = _q_rotate(a.q, b.p)
    p = (a.p[0] + bp[0], a.p[1] + bp[1], a.p[2] + bp[2])
    q = _q_normalize(_q_multiply(a.q, b.q))
    return _Pose3(p, q)


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _odom_pose(msg: Odometry) -> _Pose3:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return _Pose3((p.x, p.y, p.z), _q_normalize((q.x, q.y, q.z, q.w)))


def _global_pose(msg: PoseWithCovarianceStamped) -> _Pose3:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return _Pose3((p.x, p.y, p.z), _q_normalize((q.x, q.y, q.z, q.w)))


class LocalizationManager(Node):
    """Own map->odom and turn a validated global base pose into a global correction.

    Global relocalization supplies T_map_base at time t. A time-near local
    odometry sample supplies T_odom_base at the same t. This node computes:

        T_map_odom = T_map_base * inverse(T_odom_base)

    It is intentionally the only node in this stack that publishes map->odom.
    """

    def __init__(self) -> None:
        super().__init__('agt_localization_manager')
        self.declare_parameter('local_odom_topic', '/agt/odometry/local')
        self.declare_parameter('global_pose_topic', '/agt/relocalization/pose')
        self.declare_parameter('status_topic', '/agt/localization/status')
        self.declare_parameter('relocalization_request_topic', '/agt/relocalization/request')
        self.declare_parameter('relocalization_service', '/agt/localization/relocalize')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('local_odom_timeout_sec', 0.30)
        self.declare_parameter('local_odom_lost_sec', 1.00)
        self.declare_parameter('global_match_max_skew_sec', 0.10)
        self.declare_parameter('odom_buffer_sec', 5.0)
        self.declare_parameter('max_global_position_std_m', 1.00)
        self.declare_parameter('max_global_yaw_std_deg', 20.0)
        self.declare_parameter('accept_zero_covariance', False)
        self.declare_parameter('tf_publish_rate_hz', 30.0)
        self.declare_parameter('map_id', '')
        self.declare_parameter('map_version', '')

        self._odom: Deque[Odometry] = deque()
        self._last_odom_rx_ns = 0
        self._correction: Optional[_Pose3] = None
        self._last_global_std = (math.inf, math.inf)
        self._state = LocalizationStatus.STATE_BOOT
        self._reason = 'boot'

        self._tf = TransformBroadcaster(self)
        self._status_pub = self.create_publisher(
            LocalizationStatus, self.get_parameter('status_topic').value, 10)
        self._request_pub = self.create_publisher(
            Empty, self.get_parameter('relocalization_request_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('local_odom_topic').value, self._on_odom, 100)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('global_pose_topic').value,
            self._on_global_pose,
            10,
        )
        self.create_service(
            Trigger,
            self.get_parameter('relocalization_service').value,
            self._on_relocalize,
        )

        tf_rate = max(float(self.get_parameter('tf_publish_rate_hz').value), 1.0)
        self.create_timer(1.0 / tf_rate, self._tick)
        self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            'Localization Manager started as the exclusive map->odom owner.')

    def _on_odom(self, msg: Odometry) -> None:
        if msg.header.frame_id != self.get_parameter('odom_frame').value:
            self.get_logger().warn(
                f'Ignoring local odom with frame {msg.header.frame_id!r}',
                throttle_duration_sec=2.0,
            )
            return
        if msg.child_frame_id != self.get_parameter('base_frame').value:
            self.get_logger().warn(
                f'Ignoring local odom child frame {msg.child_frame_id!r}',
                throttle_duration_sec=2.0,
            )
            return

        self._odom.append(msg)
        self._last_odom_rx_ns = self.get_clock().now().nanoseconds
        newest_ns = _stamp_ns(msg.header.stamp)
        keep_ns = int(float(self.get_parameter('odom_buffer_sec').value) * 1e9)
        while self._odom and newest_ns - _stamp_ns(self._odom[0].header.stamp) > keep_ns:
            self._odom.popleft()

    def _nearest_odom(self, target_ns: int) -> Optional[Odometry]:
        if not self._odom:
            return None
        sample = min(self._odom, key=lambda m: abs(_stamp_ns(m.header.stamp) - target_ns))
        skew = abs(_stamp_ns(sample.header.stamp) - target_ns) / 1e9
        if skew > float(self.get_parameter('global_match_max_skew_sec').value):
            return None
        return sample

    def _validate_global(self, msg: PoseWithCovarianceStamped):
        if msg.header.frame_id != self.get_parameter('map_frame').value:
            return False, math.inf, math.inf, 'global_pose_wrong_frame'
        target_ns = _stamp_ns(msg.header.stamp)
        if target_ns <= 0:
            return False, math.inf, math.inf, 'global_pose_zero_stamp'

        cov = msg.pose.covariance
        pos_var = max(float(cov[0]), float(cov[7]), float(cov[14]), 0.0)
        yaw_var = max(float(cov[35]), 0.0)
        all_zero = all(abs(float(v)) < 1e-12 for v in cov)
        if all_zero and not bool(self.get_parameter('accept_zero_covariance').value):
            return False, math.inf, math.inf, 'global_pose_missing_covariance'

        pos_std = math.sqrt(pos_var)
        yaw_std_deg = math.degrees(math.sqrt(yaw_var))
        if pos_std > float(self.get_parameter('max_global_position_std_m').value):
            return False, pos_std, yaw_std_deg, 'global_position_uncertainty_too_large'
        if yaw_std_deg > float(self.get_parameter('max_global_yaw_std_deg').value):
            return False, pos_std, yaw_std_deg, 'global_yaw_uncertainty_too_large'
        return True, pos_std, yaw_std_deg, 'accepted'

    def _on_global_pose(self, msg: PoseWithCovarianceStamped) -> None:
        ok, pos_std, yaw_std, reason = self._validate_global(msg)
        self._last_global_std = (pos_std, yaw_std)
        if not ok:
            self._reason = reason
            self.get_logger().warn(f'Rejected global pose: {reason}')
            return

        local = self._nearest_odom(_stamp_ns(msg.header.stamp))
        if local is None:
            self._reason = 'no_time_aligned_local_odom'
            self.get_logger().warn(
                'Rejected global pose: no local odometry sample within configured time skew.')
            return

        try:
            map_base = _global_pose(msg)
            odom_base = _odom_pose(local)
            self._correction = _compose(map_base, _inverse(odom_base))
        except ValueError as exc:
            self._reason = f'invalid_pose:{exc}'
            self.get_logger().error(self._reason)
            return

        self._state = LocalizationStatus.STATE_LOCALIZED
        self._reason = 'global_pose_accepted'
        self.get_logger().info(
            f'Global correction accepted: position_std={pos_std:.3f}m, '
            f'yaw_std={yaw_std:.2f}deg')

    def _on_relocalize(self, request, response):
        del request
        self._correction = None
        self._state = LocalizationStatus.STATE_RELOCALIZING
        self._reason = 'relocalization_requested'
        self._request_pub.publish(Empty())
        response.success = True
        response.message = 'global correction invalidated and relocalization requested'
        return response

    def _local_age(self) -> float:
        if self._last_odom_rx_ns <= 0:
            return math.inf
        return max(0.0, (self.get_clock().now().nanoseconds - self._last_odom_rx_ns) / 1e9)

    def _update_state(self) -> None:
        age = self._local_age()
        timeout = float(self.get_parameter('local_odom_timeout_sec').value)
        lost = float(self.get_parameter('local_odom_lost_sec').value)

        if self._last_odom_rx_ns <= 0:
            self._state = LocalizationStatus.STATE_WAIT_LOCAL_ODOM
            self._reason = 'waiting_local_odom'
            return
        if self._correction is None:
            if self._state != LocalizationStatus.STATE_RELOCALIZING:
                self._state = LocalizationStatus.STATE_WAIT_GLOBAL
                self._reason = 'waiting_global_pose'
            return
        if age > lost:
            self._state = LocalizationStatus.STATE_LOST
            self._reason = f'local_odom_lost:{age:.2f}s'
        elif age > timeout:
            self._state = LocalizationStatus.STATE_DEGRADED
            self._reason = f'local_odom_stale:{age:.2f}s'
        else:
            self._state = LocalizationStatus.STATE_LOCALIZED
            if self._reason.startswith('local_odom_'):
                self._reason = 'tracking'

    def _publish_tf(self) -> None:
        if self._correction is None:
            return
        if self._state not in (
            LocalizationStatus.STATE_LOCALIZED,
            LocalizationStatus.STATE_DEGRADED,
        ):
            # Stop refreshing a dynamic TF once localization is LOST. The TF
            # buffer will naturally expire instead of making a stale transform
            # look permanently valid to Nav2.
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('map_frame').value
        t.child_frame_id = self.get_parameter('odom_frame').value
        p = self._correction.p
        q = self._correction.q
        t.transform.translation.x = p[0]
        t.transform.translation.y = p[1]
        t.transform.translation.z = p[2]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self._tf.sendTransform(t)

    def _tick(self) -> None:
        self._update_state()
        self._publish_tf()

    def _publish_status(self) -> None:
        self._update_state()
        age = self._local_age()
        out = LocalizationStatus()
        out.stamp = self.get_clock().now().to_msg()
        out.state = self._state
        out.local_odom_fresh = math.isfinite(age) and age <= float(
            self.get_parameter('local_odom_timeout_sec').value)
        out.global_correction_valid = self._correction is not None
        out.local_odom_age_sec = float(age if math.isfinite(age) else 1.0e9)
        # A global correction is an anchor, not a periodic sensor reading. Its
        # usefulness does not expire merely because a new global pose is absent.
        out.global_correction_age_sec = 0.0 if self._correction is not None else 1.0e9
        out.global_position_std_m = float(
            self._last_global_std[0] if math.isfinite(self._last_global_std[0]) else 1.0e9)
        out.global_yaw_std_deg = float(
            self._last_global_std[1] if math.isfinite(self._last_global_std[1]) else 1.0e9)
        out.map_id = str(self.get_parameter('map_id').value)
        out.map_version = str(self.get_parameter('map_version').value)
        out.reason = self._reason
        self._status_pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
