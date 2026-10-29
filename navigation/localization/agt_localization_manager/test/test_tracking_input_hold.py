"""Exercise bounded dead-reckoning callbacks without any ROS context or node."""

import json
import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pytest
from std_msgs.msg import String

from agt_localization_manager.localization_manager import (
    LocalizationState, _compose, _odom_pose,
)
from test_local_odom_recovery import manager  # noqa: F401


def status(m, state='INPUT_INSUFFICIENT', decision='NOT_ATTEMPTED'):
    m.node._on_tracking_status(String(data=json.dumps({
        'state': state, 'decision': decision, 'reason': 'min_query_points_after_map_crop'})))
    m.node._update_state()


def odom(m, dt=0.1, x=0.0, stamp_offset=0.0):
    m.clock.nanoseconds += round(dt * 1e9)
    m.clock.steady_nanoseconds += round(dt * 1e9)
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_link'
    sec, ns = divmod(m.clock.nanoseconds + round(stamp_offset * 1e9), 1_000_000_000)
    msg.header.stamp.sec, msg.header.stamp.nanosec = sec, ns
    msg.pose.pose.position.x = float(x)
    msg.pose.pose.orientation.w = 1.0
    m.node._on_odom(msg)
    m.node._update_state()
    return msg


def pose(m):
    local = m.node._odom[-1]
    value = _compose(m.node._correction_current, _odom_pose(local))
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = local.header.stamp
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = value.p
    (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
     msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = value.q
    msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[14] = 0.01
    msg.pose.covariance[35] = 0.01
    return msg


def anchor(m):
    odom(m, dt=0.0)
    m.node._on_global_pose(pose(m))
    return m.node._correction_current


def advance(m, seconds, x=0.0):
    for _ in range(round(seconds * 10)):
        odom(m, x=x)


def test_input_shortage_keeps_anchor_tf_without_recovery(manager):
    m = manager
    saved = anchor(m)
    status(m)
    assert m.node._state == LocalizationState.LOCALIZED
    assert m.node._reason == 'map_tracking_input_hold'
    m.node._publish_tf()
    assert len(m.transforms) == 1
    assert m.node._correction_current is saved
    assert not m.requests


def test_budget_starts_at_last_accepted_anchor_not_first_shortage(manager):
    m = manager
    saved = anchor(m)
    advance(m, 58.0)
    status(m)
    advance(m, 1.9)
    status(m)
    assert m.node._state == LocalizationState.LOCALIZED
    odom(m)
    assert m.node._state == LocalizationState.DEGRADED
    assert m.node._reason == 'map_tracking_input_hold_expired:time_limit'
    for state in ('INPUT_INSUFFICIENT', 'WAIT_QUERY', 'TRACKING_OK'):
        status(m, state)
        assert m.node._state == LocalizationState.DEGRADED
    assert m.node._correction_current is saved
    m.node._publish_tf()
    assert m.transforms
    assert not m.requests


def test_cumulative_out_and_back_distance_expires_with_zero_net_displacement(manager):
    m = manager
    anchor(m)
    status(m)
    for step in range(1, 176):
        odom(m, x=step * 0.2)
    for step in range(174, 0, -1):
        odom(m, x=step * 0.2)
    assert m.node._state == LocalizationState.LOCALIZED
    odom(m, x=0.0)
    assert m.node._state == LocalizationState.DEGRADED
    assert m.node._reason == 'map_tracking_input_hold_expired:distance_limit'
    assert not m.requests


def test_only_actual_consistent_tracking_acceptance_resets_budget(manager):
    m = manager
    anchor(m)
    advance(m, 60.0)
    status(m)
    assert m.node._state == LocalizationState.DEGRADED
    m.node._on_tracking_pose(pose(m))
    status(m, 'TRACKING_OK', 'ACCEPT')
    assert m.node._state == LocalizationState.DEGRADED  # first pose awaits consistency
    odom(m)
    m.node._on_tracking_pose(pose(m))
    m.node._update_state()
    assert m.node._state == LocalizationState.TRACKING
    status(m)
    advance(m, 59.9)
    assert m.node._state == LocalizationState.LOCALIZED
    odom(m)
    assert m.node._state == LocalizationState.DEGRADED


@pytest.mark.parametrize('gap', [0.4, 1.2])
def test_odom_outage_does_not_regain_hold_on_fresh_odom(manager, gap):
    m = manager
    saved = anchor(m)
    status(m)
    m.clock.nanoseconds += round(gap * 1e9)
    m.clock.steady_nanoseconds += round(gap * 1e9)
    m.node._update_state()
    assert m.node._state in {LocalizationState.DEGRADED, LocalizationState.LOST}
    status(m)
    odom(m)
    for state in ('INPUT_INSUFFICIENT', 'WAIT_QUERY', 'TRACKING_OK'):
        status(m, state)
        assert m.node._state == LocalizationState.DEGRADED
    assert m.node._correction_current is saved
    assert not m.requests


@pytest.mark.parametrize('bad_odom', ['jump', 'reordered', 'nonfinite'])
def test_discontinuous_odom_cannot_extend_hold(manager, bad_odom):
    m = manager
    anchor(m)
    status(m)
    odom(m, x=2.0 if bad_odom == 'jump' else math.nan if bad_odom == 'nonfinite' else 0.0,
         stamp_offset=-0.2 if bad_odom == 'reordered' else 0.0)
    assert m.node._state == LocalizationState.DEGRADED
    odom(m)
    status(m)
    assert m.node._state == LocalizationState.DEGRADED
    assert not m.requests


@pytest.mark.parametrize('state', [LocalizationState.DEGRADED, LocalizationState.LOST,
                                  LocalizationState.RELOCALIZING, LocalizationState.WAIT_GLOBAL])
def test_input_shortage_cannot_open_existing_fault(manager, state):
    m = manager
    anchor(m)
    m.node._state = state
    m.node._reason = 'existing_fault'
    m.node._tracking_health = 'DEGRADED'
    if state == LocalizationState.WAIT_GLOBAL:
        m.node._correction_current = m.node._correction_target = None
    status(m)
    assert m.node._state == state
    assert not m.requests


@pytest.mark.parametrize('name', ['tracking_input_hold_max_sec',
    'tracking_input_hold_max_distance_m', 'tracking_input_hold_max_odom_step_m',
    'tracking_input_hold_max_odom_speed_mps', 'tracking_input_hold_max_odom_yaw_step_deg'])
@pytest.mark.parametrize('value', [0.0, -1.0, math.inf, math.nan])
def test_invalid_hold_thresholds_fail_closed(manager, name, value):
    m = manager
    anchor(m)
    m.params[name] = value
    status(m)
    assert m.node._state == LocalizationState.DEGRADED
    assert not m.requests


def test_real_recovery_required_keeps_original_invalidation_path(manager):
    m = manager
    anchor(m)
    status(m)
    status(m, 'RECOVERY_REQUIRED', 'REJECT')
    assert m.node._state == LocalizationState.RELOCALIZING
    assert m.node._correction_current is None
    assert len(m.requests) == 1


def test_quality_degraded_survives_hold_wait_and_input_statuses(manager):
    m = manager
    anchor(m)
    status(m)
    status(m, 'DEGRADED', 'REJECT')
    for state in ('HOLD', 'WAIT_QUERY', 'INPUT_INSUFFICIENT', 'TRACKING_OK'):
        status(m, state)
        assert m.node._state == LocalizationState.DEGRADED
        assert m.node._tracking_health == 'DEGRADED'


def test_duplicate_pose_cannot_satisfy_consistency_or_reset_budget(manager):
    m = manager
    anchor(m)
    advance(m, 60.0)
    status(m)
    candidate = pose(m)
    for _ in range(3):
        m.node._on_tracking_pose(candidate)
        status(m, 'TRACKING_OK', 'ACCEPT')
    assert m.node._tracking_consistent_count == 1
    assert m.node._input_hold_budget.anchor_ns == 10_000_000_000
    assert m.node._state == LocalizationState.DEGRADED


def test_status_before_rejected_pose_cannot_reset_input_budget(manager):
    m = manager
    anchor(m)
    status(m)
    odom(m)
    status(m, 'TRACKING_OK', 'ACCEPT')
    candidate = pose(m)
    candidate.pose.pose.position.x += 1.0  # manager innovation rejection
    m.node._on_tracking_pose(candidate)
    for state in ('HOLD', 'WAIT_QUERY', 'INPUT_INSUFFICIENT', 'TRACKING_OK'):
        status(m, state)
        assert m.node._state == LocalizationState.DEGRADED
    assert m.node._input_hold_budget.anchor_ns == 10_000_000_000


def test_old_pose_after_discontinuity_cannot_change_target_or_open_hold(manager):
    m = manager
    saved = anchor(m)
    status(m)
    odom(m)
    old = pose(m)
    m.node._on_tracking_pose(old)  # first consistency sample
    odom(m)
    pre_gap = pose(m)
    odom(m, dt=0.4)
    m.node._on_tracking_pose(pre_gap)  # second sample is from before the gap
    status(m, 'TRACKING_OK', 'ACCEPT')
    assert m.node._correction_target is saved
    assert m.node._state == LocalizationState.DEGRADED


def test_monotonic_deadline_with_slow_ros_clock_still_expires(manager):
    m = manager
    anchor(m)
    status(m)
    for _ in range(300):
        m.clock.steady_nanoseconds += 100_000_000
        odom(m)
    assert m.node._state == LocalizationState.DEGRADED
    assert m.node._reason == 'map_tracking_input_hold_expired:time_limit'


def test_ros_clock_rollback_invalidates_hold(manager):
    m = manager
    anchor(m)
    status(m)
    advance(m, 1.0)
    m.clock.nanoseconds -= 500_000_000
    m.node._update_state()
    assert m.node._state == LocalizationState.DEGRADED
    assert m.node._reason == 'map_tracking_input_hold_expired:clock_rollback'


def test_increasing_but_old_odom_stamps_do_not_count_as_fresh_hold(manager):
    m = manager
    anchor(m)
    status(m)
    for i in range(1, 5):
        odom(m, dt=0.2, stamp_offset=-0.1 * i)
    assert m.node._state == LocalizationState.DEGRADED
    assert m.node._reason == 'map_tracking_input_hold_expired:odom_stamp_age'


def test_nonfinite_global_pose_cannot_replace_anchor(manager):
    m = manager
    saved = anchor(m)
    candidate = pose(m)
    candidate.pose.pose.position.x = math.nan
    m.node._on_global_pose(candidate)
    assert m.node._correction_current is saved


def test_debug_budget_reports_actual_limits_without_mutating_budget(manager):
    m = manager
    anchor(m)
    status(m)
    before = vars(m.node._input_hold_budget).copy()
    data = json.loads(m.node._debug_state())
    assert data['input_hold_active'] is True
    assert data['input_hold_anchor_stamp_ns'] == 10_000_000_000
    assert data['input_hold_max_sec'] == 60.0
    assert data['input_hold_max_distance_m'] == 70.0
    assert data['input_hold_age_sec'] == 0.0
    assert vars(m.node._input_hold_budget) == before


def accept_tracking(m):
    odom(m)
    m.node._on_tracking_pose(pose(m))
    m.node._update_state()


@pytest.mark.parametrize('reference', ['valid', None, 0, -1, True, '123', 1.2, math.nan, []])
def test_late_accept_before_quality_failure_cannot_clear_newer_failure(manager, reference):
    m = manager
    anchor(m)
    accept_tracking(m)
    accept_tracking(m)
    status(m)
    odom(m)
    delayed = pose(m)
    odom(m)
    watermark = m.clock.nanoseconds
    payload = {'state': 'DEGRADED', 'decision': 'REJECT',
               'reference_cloud_stamp_ns': watermark if reference == 'valid' else reference}
    m.node._on_tracking_status(String(data=json.dumps(payload)))
    saved = m.node._correction_target
    assert m.node._tracking_consistent_count == 0
    m.node._on_tracking_pose(delayed)
    status(m, 'TRACKING_OK', 'ACCEPT')
    assert m.node._state == LocalizationState.DEGRADED
    assert m.node._correction_target is saved
    accept_tracking(m)
    assert m.node._state == LocalizationState.DEGRADED  # one new accept is insufficient
    accept_tracking(m)
    assert m.node._state == LocalizationState.TRACKING


def test_first_real_reject_breaks_accept_consistency_without_immediate_stop(manager):
    m = manager
    anchor(m)
    accept_tracking(m)
    assert m.node._tracking_consistent_count == 1
    odom(m)
    m.node._on_tracking_status(String(data=json.dumps({
        'state': 'HOLD', 'decision': 'REJECT',
        'reference_cloud_stamp_ns': m.clock.nanoseconds})))
    m.node._update_state()
    assert m.node._state == LocalizationState.LOCALIZED
    assert m.node._tracking_consistent_count == 0
    accept_tracking(m)
    assert m.node._tracking_consistent_count == 1
    assert m.node._input_hold_budget.anchor_ns == 10_000_000_000
    accept_tracking(m)
    assert m.node._state == LocalizationState.TRACKING


@pytest.mark.parametrize('old_failure', ['HOLD', 'DEGRADED', 'RECOVERY_REQUIRED'])
def test_late_old_reject_does_not_invalidate_a_newer_accepted_anchor(manager, old_failure):
    m = manager
    anchor(m)
    accept_tracking(m)
    failure_ref = m.clock.nanoseconds
    accept_tracking(m)
    saved = m.node._correction_current
    saved_budget = m.node._input_hold_budget.anchor_ns
    m.node._on_tracking_status(String(data=json.dumps({
        'state': old_failure, 'decision': 'REJECT',
        'reference_cloud_stamp_ns': failure_ref})))
    m.node._update_state()
    assert m.node._state == LocalizationState.TRACKING
    assert m.node._correction_current is saved
    assert m.node._input_hold_budget.anchor_ns == saved_budget
    assert not m.requests
