"""No device needed: C1 is not implicitly healthy on an action name alone."""
from types import SimpleNamespace

from agt_navigation_runtime.camera_health import camera_ready


def healthy(**overrides):
    data = dict(STATE_READY=1, state=1, last_error='', move_action_ready=True,
                gimbal_serial_connected=True, gimbal_feedback_alive=True, camera_alive=True,
                busy=False, camera_age=0.4)
    data.update(overrides)
    return SimpleNamespace(**data)


def test_recent_ready_health_passes():
    assert camera_ready(healthy(), 10.0, 10.7)[0]


def test_missing_stale_busy_and_unhealthy_are_rejected():
    cases = [
        (None, None, 10.0),
        (healthy(), 1.0, 10.0),
        (healthy(state=2, last_error='moving'), 9.5, 10.0),
        (healthy(move_action_ready=False), 9.5, 10.0),
        (healthy(gimbal_serial_connected=False), 9.5, 10.0),
        (healthy(gimbal_feedback_alive=False), 9.5, 10.0),
        (healthy(camera_alive=False), 9.5, 10.0),
        (healthy(camera_age=2.2), 9.5, 10.0),
    ]
    for health, received, now in cases:
        assert not camera_ready(health, received, now)[0]
