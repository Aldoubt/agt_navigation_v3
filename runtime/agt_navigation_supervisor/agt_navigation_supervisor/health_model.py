"""Pure, fail-closed Navigation Health state transition model."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Inputs:
    map_valid: bool
    platform_ready: bool
    lidar_alive: bool
    imu_alive: bool
    base_alive: bool
    odom_alive: bool
    localized: bool
    tf_ready: bool
    nav2_active: bool
    goal_active: bool = False


@dataclass(frozen=True)
class Decision:
    status: str
    state: str
    error_code: str
    message: str


def matches_selected_map(localization_id: str, localization_version: str,
                         selected_id: str, selected_version: str) -> bool:
    """Require both producers to name the same exact validated map version."""
    return bool(selected_id and selected_version and
                localization_id == selected_id and
                localization_version == selected_version)


def decide(values: Inputs, ever_ready: bool = False) -> Decision:
    if not values.map_valid:
        return Decision('ERROR', 'MAP_ERROR', 'MAP_ERROR', 'map package or robot compatibility invalid')
    stages = (
        (values.platform_ready and values.base_alive, 'WAIT_PLATFORM', 'PLATFORM_UNAVAILABLE'),
        (values.lidar_alive and values.imu_alive, 'WAIT_SENSORS', 'SENSOR_UNAVAILABLE'),
        (values.odom_alive, 'WAIT_ODOMETRY', 'ODOM_UNAVAILABLE'),
        (values.localized and values.tf_ready, 'RELOCALIZING', 'LOCALIZATION_UNAVAILABLE'),
        (values.nav2_active, 'LOCALIZED', 'NAV2_UNAVAILABLE'),
    )
    for valid, state, code in stages:
        if not valid:
            status = 'DEGRADED' if ever_ready else 'NOT_READY'
            return Decision(status, state, code, state.lower())
    if values.goal_active:
        return Decision('BUSY', 'NAVIGATING', '', '')
    return Decision('READY', 'NAV_READY', '', '')
