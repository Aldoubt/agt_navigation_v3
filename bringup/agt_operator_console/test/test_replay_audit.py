from agt_operator_console.replay_audit import FAIL, PASS, WARN, ReplayMetrics


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
