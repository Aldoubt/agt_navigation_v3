"""Pure geometry for 5/7/10 m field-test lines.

No ROS publishers, actions, CAN or motion authority. Start is an explicitly
confirmed map-frame pose, not a stale or guessed current robot pose.
"""
from __future__ import annotations

import math


def generate_line(x: float, y: float, yaw_rad: float, distance_m: float,
                  spacing_m: float = 0.05) -> list[dict[str, float]]:
    numbers = (x, y, yaw_rad, distance_m, spacing_m)
    if not all(math.isfinite(float(v)) for v in numbers):
        raise ValueError("nonfinite position, yaw, distance or spacing")
    if not (0.0 < distance_m <= 30.0):
        raise ValueError("distance must be in (0, 30] metres")
    if not (0.01 <= spacing_m <= 0.5):
        raise ValueError("spacing must be in [0.01, 0.5] metres")
    segments = max(1, math.ceil(distance_m / spacing_m))
    dx, dy = math.cos(yaw_rad), math.sin(yaw_rad)
    return [
        {"x": x + distance_m * (i / segments) * dx,
         "y": y + distance_m * (i / segments) * dy,
         "yaw_rad": math.atan2(dy, dx)}
        for i in range(segments + 1)
    ]


def terminal_error_m(actual_x: float, actual_y: float,
                     target_x: float, target_y: float) -> float:
    if not all(math.isfinite(float(v)) for v in
               (actual_x, actual_y, target_x, target_y)):
        raise ValueError("nonfinite measured/target point")
    return math.hypot(actual_x - target_x, actual_y - target_y)
