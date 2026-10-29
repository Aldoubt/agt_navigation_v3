"""Exercise callback ordering without starting a ROS context or creating a node."""

from collections import deque
import json
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pytest
from std_msgs.msg import String

from agt_localization_manager.localization_manager import (
    LocalizationManager,
    LocalizationState,
    _Pose3,
    _RecoveryStateMachine,
    _TrackingInputBudget,
)


@pytest.fixture
def manager():
    # Bypass Node construction: these callbacks use only the in-memory fakes.
    node = object.__new__(LocalizationManager)
    clock = SimpleNamespace(nanoseconds=10_000_000_000, steady_nanoseconds=10_000_000_000)

    def now():
        sec, nanosec = divmod(clock.nanoseconds, 1_000_000_000)
        return SimpleNamespace(
            nanoseconds=clock.nanoseconds,
            to_msg=lambda: Time(sec=sec, nanosec=nanosec),
        )

    params = {
        'odom_frame': 'odom', 'base_frame': 'base_link', 'map_frame': 'map',
        'odom_buffer_sec': 30.0, 'local_odom_timeout_sec': 0.30,
        'local_odom_lost_sec': 1.00, 'tracking_recovery_auto_request': True,
        'debug_identity_map_odom': False,
        'global_match_max_skew_sec': 0.10,
        'accept_zero_covariance': False,
        'max_global_position_std_m': 1.0, 'max_global_yaw_std_deg': 20.0,
        'max_tracking_translation_innovation_m': 0.50,
        'max_tracking_yaw_innovation_deg': 5.0,
        'tracking_consecutive_accepts': 2,
        'tracking_consistency_translation_m': 0.20,
        'tracking_consistency_yaw_deg': 2.0,
        'tracking_input_hold_max_sec': 60.0,
        'tracking_input_hold_max_distance_m': 70.0,
        'tracking_input_hold_max_odom_step_m': 0.50,
        'tracking_input_hold_max_odom_speed_mps': 3.0,
        'tracking_input_hold_max_odom_yaw_step_deg': 30.0,
    }
    node.get_clock = lambda: SimpleNamespace(now=now)
    node.get_parameter = lambda name: SimpleNamespace(value=params[name])
    node.get_logger = lambda: SimpleNamespace(
        warn=lambda *args, **kwargs: None, info=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None)
    node._odom = deque()
    node._last_odom_rx_ns = 0
    node._last_odom_steady_ns = 0
    node._state = LocalizationState.TRACKING
    node._reason = 'map_tracking_ok'
    node._tracking_health = 'TRACKING_OK'
    node._tracking_innovation_bad = False
    node._tracking_recovery_requested = False
    node._input_hold_active = False
    node._input_hold_budget = _TrackingInputBudget()
    node._steady_now_ns = lambda: clock.steady_nanoseconds
    node._last_tracking_pose_stamp_ns = 0
    node._tracking_failure_stamp_ns = 0
    node._backend_debug_state = None
    node._recovery_pending = False
    node._recovery = _RecoveryStateMachine(cooldown_sec=5.0)
    node._correction_current = _Pose3((2.0, -1.0, 0.3), (0.0, 0.0, 0.0, 1.0))
    node._correction_target = node._correction_current
    transforms, requests = [], []
    node._tf = SimpleNamespace(sendTransform=transforms.append)
    node._request_pub = SimpleNamespace(publish=requests.append)
    return SimpleNamespace(
        node=node, clock=clock, params=params, transforms=transforms, requests=requests)


def receive_odom(manager):
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    msg.header.stamp = manager.node.get_clock().now().to_msg()
    msg.pose.pose.orientation.w = 1.0
    manager.node._on_odom(msg)


def tracking_callback(node, kind):
    if kind == 'pose':
        node._on_tracking_pose(PoseWithCovarianceStamped())
    else:
        node._on_tracking_status(String(data=json.dumps({'state': kind})))


@pytest.mark.parametrize('callbacks', [
    ('pose',),
    ('HOLD',),
    ('pose', 'TRACKING_OK', 'HOLD', 'WAIT_LOCALIZED'),
])
def test_late_tracking_callbacks_do_not_latch_transient_odom_loss(manager, callbacks):
    node = manager.node
    anchor = node._correction_current
    receive_odom(manager)
    manager.clock.nanoseconds += 4_400_000_000
    node._update_state()
    assert node._state == LocalizationState.LOST
    assert node._reason == 'local_odom_lost:4.40s'

    for kind in callbacks:
        tracking_callback(node, kind)
    node._publish_tf()
    assert not manager.transforms

    # The same global anchor becomes usable again once local odometry returns.
    manager.clock.nanoseconds += 20_000_000
    receive_odom(manager)
    node._update_state()
    assert node._state in {LocalizationState.LOCALIZED, LocalizationState.TRACKING}
    assert node._reason == 'tracking'
    assert node._correction_current is anchor
    assert node._correction_target is anchor
    assert not manager.requests
    assert node._recovery.request_count == 0
    node._publish_tf()
    assert len(manager.transforms) == 1
    assert manager.transforms[0].transform.translation.x == anchor.p[0]


@pytest.mark.parametrize('auto_request', [False, True])
def test_actual_tracking_loss_stays_closed_after_late_callbacks(manager, auto_request):
    node = manager.node
    manager.params['tracking_recovery_auto_request'] = auto_request
    receive_odom(manager)
    tracking_callback(node, 'RECOVERY_REQUIRED')
    expected_state = (
        LocalizationState.RELOCALIZING if auto_request else LocalizationState.LOST)
    expected_reason = node._reason
    for kind in ('pose', 'HOLD', 'TRACKING_OK', 'WAIT_LOCALIZED'):
        tracking_callback(node, kind)
    manager.clock.nanoseconds += 20_000_000
    receive_odom(manager)
    node._update_state()
    node._publish_tf()
    assert node._state == expected_state
    assert node._reason == expected_reason
    assert node._correction_current is None
    assert node._correction_target is None
    assert not manager.transforms
    assert len(manager.requests) == int(auto_request)
    assert node._recovery.request_count == int(auto_request)


def test_manual_global_recovery_stays_closed_until_global_pose(manager):
    node = manager.node
    receive_odom(manager)
    node._on_relocalize(None, SimpleNamespace())
    for kind in ('pose', 'HOLD', 'TRACKING_OK'):
        tracking_callback(node, kind)
    receive_odom(manager)
    node._update_state()
    node._publish_tf()
    assert node._state == LocalizationState.RELOCALIZING
    assert node._reason == 'relocalization_requested'
    assert node._correction_current is None
    assert not manager.transforms
    assert len(manager.requests) == 1


def test_fresh_odom_and_tracking_cannot_supply_missing_global_anchor(manager):
    node = manager.node
    node._state = LocalizationState.WAIT_GLOBAL
    node._correction_current = node._correction_target = None
    receive_odom(manager)
    for kind in ('pose', 'HOLD', 'TRACKING_OK'):
        tracking_callback(node, kind)
    node._update_state()
    node._publish_tf()
    assert node._state == LocalizationState.WAIT_GLOBAL
    assert node._reason == 'waiting_global_pose'
    assert not manager.transforms
    assert not manager.requests
