from agt_nav_benchmark.localization_shadow import (
    LegacySample,
    ShadowSample,
    synchronize_samples,
    wrap_to_pi,
)


def _legacy(timestamp_ns=1_000_000_000):
    return LegacySample(timestamp_ns, 'LOCALIZED', 1.0, 2.0, 0.0, 3.13, 0.2, 0.1)


def _shadow(timestamp_ns=1_100_000_000, yaw=-3.13):
    return ShadowSample(timestamp_ns, 'LOCALIZED', 1.3, 2.4, 0.0, yaw, 0.3, 0.2, 'ACCEPTED', 'global_pose')


def test_wrap_to_pi_handles_boundary():
    assert abs(wrap_to_pi(-3.13 - 3.13)) < 0.05


def test_synchronize_computes_pose_error_and_acceptance():
    rows, unmatched = synchronize_samples([_legacy()], [_shadow()], max_sync_sec=0.2)
    assert unmatched == 0
    assert len(rows) == 1
    assert abs(rows[0].translation_error - 0.5) < 1e-12
    assert rows[0].yaw_error < 0.05
    assert rows[0].correction_accept == 'accepted'
    assert 'translation_m' in rows[0].innovation


def test_synchronize_rejects_outside_time_window():
    rows, unmatched = synchronize_samples([_legacy()], [_shadow(timestamp_ns=1_500_000_000)], max_sync_sec=0.2)
    assert rows == []
    assert unmatched == 1
