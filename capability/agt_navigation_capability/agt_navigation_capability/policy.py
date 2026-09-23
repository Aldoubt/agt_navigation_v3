"""Success requires every safety prerequisite at the moment Nav2 finishes."""


def health_allows_motion(health, age_sec, robot_profile, map_id, map_version):
    return bool(
        health is not None and age_sec <= 1.5
        and health.robot_profile == robot_profile
        and health.map_id == map_id and health.map_version == map_version
        and health.status in (health.READY, health.BUSY)
        and health.platform_ready and health.lidar_alive and health.imu_alive
        and health.base_alive and health.odom_alive and health.localized
        and health.nav2_active)


def navigation_succeeded(nav2_status, health_ok):
    # action_msgs/GoalStatus.STATUS_SUCCEEDED is 4.
    return nav2_status == 4 and health_ok
