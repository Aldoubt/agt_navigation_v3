from types import SimpleNamespace

from agt_navigation_capability.policy import health_allows_motion, navigation_succeeded


def healthy(**changes):
    fields = dict(READY=1, BUSY=2, status=1, robot_profile='bunker_v1',
                  map_id='orchard', map_version='v1', platform_ready=True,
                  lidar_alive=True, imu_alive=True, base_alive=True,
                  odom_alive=True, localized=True, nav2_active=True)
    fields.update(changes)
    return SimpleNamespace(**fields)


def test_success_requires_nav2_reached_and_fresh_health():
    status = healthy()
    good = health_allows_motion(status, 0.2, 'bunker_v1', 'orchard', 'v1')
    assert navigation_succeeded(4, good)
    assert not navigation_succeeded(2, good)
    assert not navigation_succeeded(4, health_allows_motion(
        status, 2.0, 'bunker_v1', 'orchard', 'v1'))
    assert not navigation_succeeded(4, health_allows_motion(
        healthy(localized=False), 0.2, 'bunker_v1', 'orchard', 'v1'))
    assert health_allows_motion(healthy(base_alive=False), 0.2, 'bunker_v1', 'orchard', 'v1')
    assert not health_allows_motion(healthy(odom_alive=False), 0.2, 'bunker_v1', 'orchard', 'v1')
