"""Fail-closed, fresh C1 readiness gate shared by preflight and each capture."""
from __future__ import annotations

import math


def camera_ready(health, received_monotonic: float | None, now_monotonic: float,
                 *, max_age_sec: float = 2.0) -> tuple[bool, str]:
    if health is None or received_monotonic is None:
        return False, 'no C1 capability health message'
    age = now_monotonic - received_monotonic
    if not math.isfinite(age) or age < 0.0 or age > max_age_sec:
        return False, f'C1 capability health stale: age={age:.2f}s'
    if health.state != health.STATE_READY:
        return False, f'C1 not ready: state={health.state} reason={health.last_error}'
    if bool(getattr(health, 'busy', False)):
        return False, 'C1 busy'
    for attr in ('move_action_ready', 'gimbal_serial_connected', 'gimbal_feedback_alive', 'camera_alive'):
        if not bool(getattr(health, attr, False)):
            return False, f'C1 {attr} is false'
    image_age = float(getattr(health, 'camera_age', -1.0))
    if not math.isfinite(image_age) or image_age < 0.0 or image_age > max_age_sec:
        return False, f'C1 camera frame stale: age={image_age:.2f}s'
    return True, 'C1 READY (fresh camera, serial, feedback and action)'
