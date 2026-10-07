"""Hardware parameters live in mounted profiles; null means CONFIG_REQUIRED."""

import ipaddress
import re
from pathlib import Path

from .contracts import ContractError, number, read_yaml

SECTIONS = ("robot", "topics", "sensors", "base", "mapping", "localization", "navigation")


def load_profile(root):
    root = Path(root).resolve()
    data = {name: read_yaml(root / (name + ".yaml")) for name in SECTIONS}
    data["root"] = root
    if data["robot"].get("profile") not in {"yhs", "mock_yhs"}:
        raise ContractError("unknown robot profile")
    base = data["base"]
    interface = base.get("can_interface")
    if interface is not None and (
        not isinstance(interface, str) or not re.fullmatch(r"[a-zA-Z0-9_]{1,15}", interface)
    ):
        raise ContractError("invalid CAN interface")
    bitrate = base.get("can_bitrate")
    if bitrate is not None and (type(bitrate) is not int or not 1000 <= bitrate <= 1000000):
        raise ContractError("invalid CAN bitrate")
    for name in ("command_timeout_sec", "driver_watchdog_sec"):
        if base.get(name) is not None and number(base[name], nonnegative=True) <= 0:
            raise ContractError("watchdog must be positive")
    for name in ("host_ip", "sensor_ip"):
        if data["sensors"].get(name) is not None:
            try:
                ipaddress.ip_address(data["sensors"][name])
            except ValueError as exc:
                raise ContractError(f"invalid {name}") from exc
    return data


def calibration_errors(profile):
    if profile["robot"]["profile"] == "mock_yhs":
        return []
    root = profile["root"]
    errors = []
    for name in ("lidar_extrinsics", "imu_extrinsics", "base_geometry", "calibration_version"):
        d = read_yaml(root / "calibration" / (name + ".yaml"))
        if d.get("status") != "VERIFIED" or not d.get("verified_by"):
            errors.append(f"CALIBRATION_REQUIRED: {name}")
    for name, section in (
        ("urdf", "robot"),
        ("livox_config", "sensors"),
        ("fastlio_config", "localization"),
        ("projection_config", "mapping"),
    ):
        value = profile[section].get(name)
        if not value or not (root / value).is_file():
            errors.append(f"CONFIG_REQUIRED: {name}")
    if (
        not isinstance(profile["navigation"].get("footprint"), list)
        or len(profile["navigation"]["footprint"]) < 3
    ):
        errors.append("CONFIG_REQUIRED: footprint")
    if profile["navigation"].get("footprint_padding") is None:
        errors.append("CONFIG_REQUIRED: footprint_padding")
    perception = profile["navigation"].get("perception", {})
    for key in ["self_center_xyz", "self_size_xyz", "sensor_ground_z_m"]:
        if perception.get(key) is None:
            errors.append("CONFIG_REQUIRED: perception." + key)
    for key in ["lidar_frame", "imu_frame"]:
        if not profile["robot"].get(key):
            errors.append("CONFIG_REQUIRED: " + key)
    for key in ["interface", "host_ip", "sensor_ip"]:
        if not profile["sensors"].get(key):
            errors.append("CONFIG_REQUIRED: " + key)
    return errors


def require_real(profile, *, motion=False):
    errors = calibration_errors(profile)
    if profile["robot"]["profile"] == "mock_yhs":
        return
    if motion:
        base = profile["base"]
        for stop_key in ("stop_linear_mps", "stop_angular_radps", "stop_timeout_sec"):
            if base.get(stop_key) is None:
                errors.append("CONFIG_REQUIRED: " + stop_key)
            else:
                number(base[stop_key], nonnegative=True)
        for k in ("command_timeout_sec", "driver_watchdog_sec", "can_interface", "can_bitrate"):
            if base.get(k) is None:
                errors.append("CONFIG_REQUIRED: " + k)
        if base.get("driver_watchdog_verified") is not True:
            errors.append("driver watchdog must be verified before enabling gateway motion")
        for k in ("ros1_cmd_vel", "ros1_odom", "ros1_chassis", "ros1_estop"):
            if not profile["topics"].get(k):
                errors.append("CONFIG_REQUIRED: " + k)
        for k, v in profile["navigation"].get("motion_limits", {}).items():
            if v is None or number(v, nonnegative=True) <= 0:
                errors.append("CONFIG_REQUIRED: motion limit " + k)
    if errors:
        raise ContractError("; ".join(errors))
