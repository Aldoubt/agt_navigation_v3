from agt_navigation_supervisor.health_model import Inputs, decide, matches_selected_map


def ready(**updates):
    data = dict(map_valid=True, platform_ready=True, lidar_alive=True,
                imu_alive=True, base_alive=True, odom_alive=True,
                localized=True, tf_ready=True, nav2_active=True)
    data.update(updates)
    return Inputs(**data)


def test_ready_busy_and_fail_closed_regression():
    assert decide(ready()).status == 'READY'
    assert decide(ready(goal_active=True)).status == 'BUSY'
    lost = decide(ready(localized=False, goal_active=True), ever_ready=True)
    assert lost.status == 'DEGRADED'
    assert lost.error_code == 'LOCALIZATION_UNAVAILABLE'


def test_invalid_map_is_error_before_nav2():
    invalid = decide(ready(map_valid=False), ever_ready=True)
    assert invalid.state == 'MAP_ERROR'
    assert invalid.status == 'ERROR'


def test_cold_start_reports_first_missing_stage():
    missing = decide(ready(lidar_alive=False, odom_alive=False))
    assert missing.status == 'NOT_READY'
    assert missing.state == 'WAIT_SENSORS'


def test_localization_must_match_selected_map_identity():
    assert matches_selected_map('site', 'v2', 'site', 'v2')
    assert not matches_selected_map('', '', 'site', 'v2')
    assert not matches_selected_map('site', 'v1', 'site', 'v2')
    assert not matches_selected_map('site', 'v2', '', '')
