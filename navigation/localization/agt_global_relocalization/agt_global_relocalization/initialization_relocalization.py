"""Single-owner automatic/manual initialization with explicit operator fallback.

Automatic attempt count belongs to the field orchestrator. Switching to manual
cancels pending automatic requests within this same serialized ROS executor;
there is never a second global-pose publisher racing the automatic backend.
"""
import json

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from agt_robot_interfaces.msg import LocalizationStatus

from .manual_seed_relocalization import ManualSeedRelocalization
from .initialization_policy import validate_mode, validate_seed, validate_refinement
from .start_hint import load_start_hint, StartHintError


class InitializationRelocalization(ManualSeedRelocalization):
    def __init__(self):
        super().__init__()
        self.declare_parameter('initialization_mode', 'auto_then_manual')
        self.declare_parameter('manual_seed_max_age_sec', 30.0)
        self.declare_parameter('manual_scan_max_age_sec', 0.50)
        self.declare_parameter('manual_seed_max_translation_m', 2.0)
        self.declare_parameter('manual_seed_max_yaw_deg', 30.0)
        self.declare_parameter('start_hint_file', '')
        self.declare_parameter('selected_map_id', '')
        self.declare_parameter('selected_map_version', '')
        self.mode = validate_mode(str(self.get_parameter('initialization_mode').value))
        # Only explicit requests from the orchestrator may start automatic work.
        self.set_parameters([Parameter('auto_request', value=False)])
        self.manual_enabled = self.mode == 'manual'
        self.initialized = False
        self.awaiting_ack = False
        self.ack_deadline_ns = 0
        self._near_hint_attempted = False
        self.phase = 'WAIT_MANUAL_INITIAL_POSE' if self.manual_enabled else 'WAIT_AUTO_REQUEST'
        self.init_pub = self.create_publisher(String, '/agt/localization/initialization_status', 10)
        self.create_service(Trigger, '/agt/relocalization/enter_manual', self.enter_manual)
        self.create_service(Trigger, '/agt/localization/relocalize_near_start',
                            self.relocalize_near_start)
        self.create_subscription(LocalizationStatus, '/agt/localization/status', self.on_localization, 10)
        self.create_timer(0.5, self.publish_phase)

    def publish_phase(self):
        if self.awaiting_ack and self.get_clock().now().nanoseconds > self.ack_deadline_ns:
            self.awaiting_ack = False
            self.phase = 'WAIT_MANUAL_INITIAL_POSE' if self.manual_enabled else 'WAIT_AUTO_REQUEST'
        msg = String()
        msg.data = json.dumps({'mode': self.mode, 'state': self.phase,
                               'manual_enabled': self.manual_enabled,
                               'initialized': self.initialized})
        self.init_pub.publish(msg)
        if self.phase == 'WAIT_MANUAL_INITIAL_POSE':
            self.status(self.phase, 'Nav2 is gated; keep stationary and use RViz 2D Pose Estimate')

    def _expect_manager_ack(self):
        self.awaiting_ack = True
        self.ack_deadline_ns = self.get_clock().now().nanoseconds + 3_000_000_000
        self.phase = 'VERIFY_MANUAL' if self.manual_enabled else 'VERIFY_AUTO'

    def on_localization(self, msg):
        if (self.awaiting_ack and msg.state == LocalizationStatus.STATE_LOCALIZED
                and msg.global_correction_valid and msg.local_odom_fresh):
            self.initialized = True
            self.awaiting_ack = False
            self.phase = 'INITIALIZED'

    def enter_manual(self, request, response):
        del request
        if self.mode == 'auto':
            response.success, response.message = False, 'auto mode does not permit manual fallback'
        elif self.busy or self.awaiting_ack or self.initialized:
            response.success, response.message = False, 'busy or already localized; do not replace an accepted anchor'
        else:
            self.pending_request = False
            if self.auto_timer is not None:
                self.auto_timer.cancel()
            self.manual_enabled = True
            self.clouds.clear()  # Collect a fresh stationary query for the operator.
            self.phase = 'WAIT_MANUAL_INITIAL_POSE'
            response.success, response.message = True, self.phase
            self.status(self.phase, 'automatic search disabled; waiting for /initialpose')
        return response

    def relocalize_near_start(self, request, response):
        """Try one bounded GICP seed; never claim localization before manager ACK.

        A service call is intentionally synchronous in the single-threaded
        executor: it cannot race a simultaneous full-map or manual attempt.
        The shell retries only QUERY_NOT_READY while a fresh stationary scan
        is collected, then proceeds to the existing global BBS if rejected.
        """
        del request
        hint_path = str(self.get_parameter('start_hint_file').value).strip()
        if not hint_path:
            response.success, response.message = False, 'SKIPPED: no approved start hint'
            return response
        if self.manual_enabled or self.mode != 'auto_then_manual':
            response.success, response.message = False, 'SKIPPED: non-automatic mode'
            return response
        if self.busy or self.pending_request or self.awaiting_ack or self.initialized:
            response.success, response.message = False, 'BUSY: another localization is active'
            return response
        if self._near_hint_attempted:
            response.success, response.message = False, 'NEAR_START_FAILED: already attempted'
            return response
        selected_id = str(self.get_parameter('selected_map_id').value).strip()
        selected_version = str(self.get_parameter('selected_map_version').value).strip()
        global_map, _assets, live_id, live_version, _generation = self.resolve_map_inputs()
        if (live_id or live_version) and (live_id, live_version) != (selected_id, selected_version):
            response.success, response.message = False, 'CONFIG_ERROR: active map changed'
            return response
        try:
            # Re-read SHA at the point of use: a map/hint changed since shell
            # preflight must not seed an unrelated map.
            pose = load_start_hint(hint_path, map_id=selected_id,
                                   map_version=selected_version, map_pcd=global_map)
        except (StartHintError, OSError) as exc:
            response.success, response.message = False, f'CONFIG_ERROR: {exc}'
            return response
        ready, _state, detail = self._request_readiness()
        if not ready:
            response.success, response.message = False, f'QUERY_NOT_READY: {detail}'
            return response
        stamp = self.clouds[-1].header.stamp
        now = self.get_clock().now().nanoseconds / 1e9
        scan_time = stamp.sec + stamp.nanosec / 1e9
        if not -0.10 <= now - scan_time <= float(self.get_parameter('manual_scan_max_age_sec').value):
            self.clouds.clear()
            response.success, response.message = False, 'QUERY_NOT_READY: stationary scan stale'
            return response

        # The reviewed source coordinate is T_map_body. The existing local
        # GICP API takes T_map_base; do not silently treat body as base_link.
        try:
            initial_base = self._query_pose_to_base_pose(pose, 'mapping_body')
            seed = PoseWithCovarianceStamped()
            seed.header.frame_id = str(self.get_parameter('map_frame').value)
            seed.header.stamp = stamp
            for key, axis in (('x', 'x'), ('y', 'y'), ('z', 'z')):
                setattr(seed.pose.pose.position, axis, initial_base[key])
            for key, axis in (('qx', 'x'), ('qy', 'y'), ('qz', 'z'), ('qw', 'w')):
                setattr(seed.pose.pose.orientation, axis, initial_base[key])
            self._near_hint_attempted = True
            self.busy = True
            self.phase = 'NEAR_START_GICP'
            self.status(self.phase, 'reviewed map-version/PCD-bound T_map_body seed')
            self.run_seeded_once(seed)
        except Exception as exc:
            self.awaiting_ack = False
            self.phase = 'WAIT_AUTO_REQUEST'
            self.status('NEAR_START_REJECTED', str(exc))
            self.clouds.clear()  # Force a new stationary query for full-map search.
            response.success, response.message = False, f'NEAR_START_FAILED: {exc}'
        else:
            # Proposed, NOT localized: only Localization Manager can ACK it.
            response.success, response.message = True, 'POSE_PROPOSED: awaiting manager LOCALIZED ACK'
        finally:
            self.busy = False
        return response

    def on_request(self, msg):
        if self.manual_enabled:
            # Manager invalidates the anchor before publishing this request.
            self.pending_request = False
            self.initialized = False
            self.awaiting_ack = False
            self.clouds.clear()
            self.phase = 'WAIT_MANUAL_INITIAL_POSE'
            self.status(self.phase, 'manual recovery requested; no global BBS search will run')
            return
        self.initialized = False
        self.awaiting_ack = False
        self.phase = 'AUTO_SEARCH'
        if self._near_hint_attempted:
            # Do not re-use an unsuccessfully accepted near-start query.
            self.clouds.clear()
        super().on_request(msg)

    def run_once(self):
        if self.manual_enabled:
            raise RuntimeError('automatic search is disabled in manual phase')
        super().run_once()
        self._expect_manager_ack()

    def on_manual_seed(self, msg):
        if not self.manual_enabled or self.initialized or self.awaiting_ack or self.busy:
            self.status('MANUAL_SEED_REJECTED', 'manual initialization is not armed')
            return
        try:
            validate_seed(msg.pose.pose, msg.header.frame_id, str(self.get_parameter('map_frame').value))
            now = self.get_clock().now().nanoseconds / 1e9
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if stamp <= 0 or not -0.10 <= now-stamp <= float(self.get_parameter('manual_seed_max_age_sec').value):
                raise ValueError('manual seed timestamp is stale, zero or in the future')
            ready, _, detail = self._request_readiness()
            if not ready:
                raise ValueError(detail)
            stamp = self.clouds[-1].header.stamp
            scan_time = stamp.sec + stamp.nanosec / 1e9
            if not -0.10 <= now-scan_time <= float(self.get_parameter('manual_scan_max_age_sec').value):
                raise ValueError('latest stationary scan is stale or in the future')
        except ValueError as exc:
            self.status('MANUAL_SEED_REJECTED', str(exc))
            return
        self.phase = 'MANUAL_GICP_REFINING'
        super().on_manual_seed(msg)
        if not self.awaiting_ack:
            self.phase = 'WAIT_MANUAL_INITIAL_POSE'
            self.clouds.clear()

    def validate_refined_seed(self, initial, refined, fitness, overlap):
        validate_refinement(initial, refined, fitness, overlap,
                            float(self.get_parameter('manual_seed_max_translation_m').value),
                            float(self.get_parameter('manual_seed_max_yaw_deg').value))

    def run_seeded_once(self, seed):
        super().run_seeded_once(seed)
        # Publication alone isn't enough: the manager must accept timestamp/odom.
        self._expect_manager_ack()


def main(args=None):
    rclpy.init(args=args)
    node = InitializationRelocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
