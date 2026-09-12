from agt_operator_console.replay_audit import DISABLED, FAIL, PASS, WARN, ReplayMetrics
from agt_operator_console import replay_audit


def test_formal_map_contract_requires_mapping_body_query_mode():
    package = {'relocalization_contract': {
        'formal_pose_semantics': 'T_map_body',
        'map_cloud_frame': 'body',
        'query_frame_mode': 'mapping_body',
        'query_frame': 'body',
    }}
    assert replay_audit._inspect_relocalization_contract(package, 'mapping_body') == ('MATCH', '')
    status, reason = replay_audit._inspect_relocalization_contract(package, 'base_link')
    assert status == 'MISMATCH'
    assert 'requires mapping_body' in reason


def test_immutable_legacy_map_contract_is_reported_not_rejected():
    assert replay_audit._inspect_relocalization_contract({}, 'mapping_body') == (
        'LEGACY_UNDECLARED', '')


def _lio(metrics, delays):
    for index, delay in enumerate(delays):
        now = float(index)
        metrics.observe_lio(now, now - delay)


def test_stable_lio_latency_passes():
    metrics = ReplayMetrics(min_lio_messages=5, max_lio_delay_growth_sec_per_sec=0.01)
    _lio(metrics, [0.02, 0.021, 0.019, 0.020, 0.021, 0.020])
    status, summary, _ = metrics._lio_summary()
    assert status == PASS
    assert summary['clock_to_header_delay_sec']['p95'] < 0.03


def test_monotonically_growing_lio_latency_fails():
    metrics = ReplayMetrics(min_lio_messages=5, max_lio_delay_growth_sec_per_sec=0.01,
                            min_lio_delay_growth_sec=0.05)
    _lio(metrics, [0.01, 0.04, 0.07, 0.10, 0.13, 0.16])
    status, _, reasons = metrics._lio_summary()
    assert status == FAIL
    assert 'backlog' in reasons[0]


def test_no_odom_fails():
    status, summary, _ = ReplayMetrics()._lio_summary()
    assert status == FAIL
    assert summary['message_count'] == 0


def test_localization_reaching_localized_passes():
    metrics = ReplayMetrics(min_localized_ratio=0.5)
    metrics.observe_lio(0.0, 0.0)
    metrics.observe_localization(0.0, 'LOCALIZED', True)
    metrics.advance_clock(10.0)
    status, summary, _ = metrics._localization_summary()
    assert status == PASS
    assert summary['time_to_global_anchor_sec'] == 0.0
    assert summary['localized_duration_ratio'] == 1.0


def test_localization_lost_fails():
    metrics = ReplayMetrics()
    metrics.observe_lio(0.0, 0.0)
    metrics.observe_localization(0.0, 'LOCALIZED', True)
    metrics.observe_localization(1.0, 'LOST', True)
    status, summary, _ = metrics._localization_summary()
    assert status == FAIL
    assert summary['lost_count'] == 1


def test_tracker_occasional_hold_is_warning_but_recovery_is_fail():
    metrics = ReplayMetrics(tracker_expected=True)
    metrics.observe_tracker({'state': 'TRACKING_OK', 'fitness': 0.1})
    metrics.observe_tracker({'state': 'HOLD', 'fitness': 0.2})
    assert metrics._tracker_summary()[0] == WARN
    metrics.observe_tracker({'state': 'RECOVERY_REQUIRED'})
    assert metrics._tracker_summary()[0] == FAIL


def test_disabled_tracker_is_not_applicable_not_warning():
    assert ReplayMetrics(tracker_expected=False)._tracker_summary()[0] == DISABLED


def test_manual_seed_diagnostics_and_runtime_tf_are_deterministic():
    metrics = ReplayMetrics()
    metrics.observe_manual_status({'state': 'MANUAL_SEED_REJECTED', 'detail': 'no scan'})
    assert metrics.manual['manual_seed_received'] is True
    metrics.observe_manual_status({'state': 'MANUAL_GICP_REFINING', 'detail': 'refining'})
    metrics.observe_manual_status({'state': 'MANUAL_GICP_ACCEPTED', 'detail': 'accepted',
                                   'fitness': 0.12, 'overlap': 0.8})
    metrics.observe_relocalization_pose()
    metrics.advance_clock(10.0)
    metrics.observe_tf_check('tf_odom_base')
    metrics.observe_tf('tf_odom_base', 10.0)
    metrics.finish_runtime_observations()
    assert metrics.manual['manual_seed_received'] is True
    assert metrics.manual['manual_seed_rejected_count'] == 1
    assert metrics.manual['manual_gicp_attempts'] == 1
    assert metrics.manual['manual_gicp_accepted'] is True
    assert metrics.manual['last_fitness'] == 0.12
    assert metrics.manual['relocalization_pose_published'] is True
    assert metrics.nav['tf_odom_base_runtime']['ever_seen'] is True
    assert metrics.nav['tf_odom_base_runtime']['availability'] == 1.0


def test_final_summary_is_deterministic():
    def build():
        metrics = ReplayMetrics(min_lio_messages=2, tracker_expected=True)
        _lio(metrics, [0.01, 0.01])
        metrics.observe_localization(0.0, 'LOCALIZED', True)
        metrics.advance_clock(1.0)
        metrics.observe_tracker({'state': 'TRACKING_OK', 'overlap': 0.8})
        metrics.nav.update({
            'map_server_active': True, 'planner_server_active': True,
            'planner_action_available': True, 'tf_map_odom': True,
            'tf_odom_base': True, 'planner_test': 'NOT_REQUESTED',
        })
        return metrics.summary(PASS, {'map_id': 'fixed'}, [])
    assert build() == build()


def test_clock_subscription_uses_best_effort_clock_qos_profile():
    source = open(replay_audit.__file__, encoding='utf-8').read()
    assert 'reliability=ReliabilityPolicy.BEST_EFFORT' in source
    assert "self.create_subscription(ClockMessage, '/clock', self._on_clock, CLOCK_QOS)" in source
