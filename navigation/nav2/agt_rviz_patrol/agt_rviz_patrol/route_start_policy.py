"""Pure geometry contract for executing only the operator-drawn route.

No ROS dependencies: a validated path contains exactly the drawn vertices,
not a synthesized robot-to-first-vertex connector. Preconditions prevent the
controller from unexpectedly reversing or pivoting to reach a distant start.
"""
import math


def validate_drawn_route_start(points, robot, *, max_distance=0.35,
                               max_lateral=0.12, max_heading_deg=25.0,
                               max_behind=0.05):
    """Return first-segment geometry; raise ValueError with actionable evidence."""
    if len(points) < 2:
        raise ValueError("Draw at least two distinct route points")
    if len(robot) != 3 or any(not math.isfinite(float(v)) for v in robot):
        raise ValueError("Robot map pose invalid")
    if any(not math.isfinite(float(v)) for point in points for v in point):
        raise ValueError("Drawn route has nonfinite coordinates")
    if not (0.0 < max_distance <= 2.0 and 0.0 <= max_lateral <= max_distance
            and 0.0 < max_heading_deg <= 180.0 and 0.0 <= max_behind <= max_distance):
        raise ValueError("Invalid strict drawn-route thresholds")

    start = points[0]
    tangent = next((
        (float(p[0]) - float(start[0]), float(p[1]) - float(start[1]))
        for p in points[1:]
        if math.hypot(float(p[0]) - float(start[0]),
                      float(p[1]) - float(start[1])) >= 1.e-4
    ), None)
    if tangent is None:
        raise ValueError("Drawn route has no nonzero initial segment")
    ux, uy = (a / math.hypot(*tangent) for a in tangent)
    sx, sy = float(start[0]) - robot[0], float(start[1]) - robot[1]
    distance = math.hypot(sx, sy)
    along = ux*sx + uy*sy
    lateral = ux*sy - uy*sx
    path_yaw = math.atan2(uy, ux)
    diff_deg = abs(math.degrees(math.atan2(
        math.sin(robot[2]-path_yaw), math.cos(robot[2]-path_yaw))))

    errors = []
    if distance > max_distance + 1.e-9:
        errors.append(f"route start {distance:.2f}m from robot (limit {max_distance:.2f}m)")
    if abs(lateral) > max_lateral + 1.e-9:
        errors.append(f"cross-track offset {abs(lateral):.2f}m (limit {max_lateral:.2f}m)")
    if along < -max_behind - 1.e-9:
        errors.append(f"route start is behind path direction by {-along:.2f}m")
    if diff_deg > max_heading_deg + 1.e-9:
        errors.append(
            f"robot/path heading mismatch {diff_deg:.1f}deg (limit {max_heading_deg:.1f}deg; "
            f"robot {math.degrees(robot[2]):.1f}deg vs path {math.degrees(path_yaw):.1f}deg)"
        )
    if errors:
        raise ValueError("DRAWN_ROUTE_START_INVALID: " + "; ".join(errors)
                         + "; rotate/position robot or redraw start near its forward heading")
    return {
        "start_distance_m": distance, "cross_track_m": lateral,
        "along_track_m": along, "heading_error_deg": diff_deg,
        "first_heading_rad": path_yaw,
    }



def make_relative_straight_vertices(robot, distance_m):
    """Two drawn-only map-frame vertices; does not create an action or velocity.

    This operation snapshots map->base_link at the moment it is requested.
    Returns the fixed global line even if the robot subsequently moves.
    """
    if len(robot) != 3 or not all(math.isfinite(float(v)) for v in robot):
        raise ValueError("invalid map-frame robot pose")
    if not math.isfinite(float(distance_m)) or not 0.1 <= distance_m <= 30.0:
        raise ValueError("distance_m must be a finite number in [0.1, 30.0]")
    x, y, yaw = (float(v) for v in robot)
    distance_m = float(distance_m)
    return [(x, y), (x + distance_m * math.cos(yaw),
                      y + distance_m * math.sin(yaw))]
