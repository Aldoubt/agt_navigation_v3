"""Success requires every safety prerequisite at the moment Nav2 finishes."""

HEALTH_MAX_AGE_SEC = 1.5


def health_allows_motion(health, age_sec, robot_profile, map_id, map_version):
    return bool(
        health is not None and age_sec <= HEALTH_MAX_AGE_SEC
        and health.robot_profile == robot_profile
        and health.map_id == map_id and health.map_version == map_version
        and health.status in (health.READY, health.BUSY)
        and health.platform_ready and health.lidar_alive and health.imu_alive
        and health.odom_alive and health.localized
        and health.nav2_active)


def navigation_succeeded(nav2_status, health_ok):
    # action_msgs/GoalStatus.STATUS_SUCCEEDED is 4.
    return nav2_status == 4 and health_ok


def payload_permission_ok(required, value, received_monotonic, now_monotonic, timeout_sec):
    """Fail-closed payload interlock: only a fresh True counts when required."""
    if not required:
        return True
    return bool(value) and received_monotonic is not None and \
        0.0 <= now_monotonic - received_monotonic <= timeout_sec
