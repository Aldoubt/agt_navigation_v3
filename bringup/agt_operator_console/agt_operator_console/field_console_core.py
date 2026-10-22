"""Pure operator UI contracts; no ROS, Qt, CAN or actuator side effects."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import time

import yaml

PROFILES = {
    "fwmini": ("FW-mini 舵轮", "fwmini"),
    "bunker": ("Bunker 履带", "bunker"),
}
DISTANCES_M = (5.0, 7.0, 10.0)
HEALTH_TIMEOUT_S = 2.5
EDITOR_TIMEOUT_S = 2.5
GUARD_TIMEOUT_S = 2.0


@dataclass(frozen=True)
class MapPackage:
    map_id: str
    version: str
    metadata: Path
    yaml_file: Path
    image: Path
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float

    @property
    def label(self):
        return f"{self.map_id} / {self.version}"


def read_map_package(metadata: Path) -> MapPackage:
    """Inspect source assets only. Backend performs immutable hash validation."""
    metadata = Path(metadata).resolve()
    doc = yaml.safe_load(metadata.read_text(encoding="utf-8"))
    if not isinstance(doc, dict):
        raise ValueError("metadata must be YAML mapping")
    map_id, version = doc.get("map_id"), doc.get("map_version")
    if not isinstance(map_id, str) or not isinstance(version, str):
        raise ValueError("map_id/map_version must be strings")
    if (not map_id or not version or "/" in map_id or "/" in version
            or metadata.parent.name != version or metadata.parent.parent.name != map_id):
        raise ValueError("metadata identity does not match map directory")
    navigation = metadata.parent / "navigation" / "map.yaml"
    if not navigation.is_file():
        raise ValueError("navigation/map.yaml missing")
    cfg = yaml.safe_load(navigation.read_text(encoding="utf-8"))
    if not isinstance(cfg, dict) or not isinstance(cfg.get("image"), str):
        raise ValueError("invalid map.yaml")
    origin = cfg.get("origin", [])
    if not isinstance(origin, (list, tuple)) or len(origin) != 3:
        raise ValueError("map origin must have three components")
    resolution = float(cfg.get("resolution"))
    numbers = [resolution, *(float(v) for v in origin)]
    if not all(math.isfinite(v) for v in numbers) or resolution <= 0:
        raise ValueError("map geometry invalid")
    image = (navigation.parent / cfg["image"]).resolve()
    if not image.is_relative_to(navigation.parent.resolve()) or not image.is_file():
        raise ValueError("map image missing or outside navigation directory")
    return MapPackage(map_id, version, metadata, navigation, image,
                      resolution, float(origin[0]), float(origin[1]),
                      float(origin[2]))


def discover_map_packages(map_root):
    """Return (valid map packages, warnings), never fabricate a map identity."""
    root = Path(map_root).expanduser()
    packages, warnings = [], []
    if not root.is_dir():
        return packages, [f"地图目录不存在: {root}"]
    for meta in sorted(root.glob("*/*/metadata.yaml")):
        try:
            packages.append(read_map_package(meta))
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError) as exc:
            warnings.append(f"{meta}: {exc}")
    return packages, warnings


def health_gate(profile, selected_map, health, editor, guard, *, now=None):
    """Fail-closed operator UI gate; backend has independent actual guards.

    health/editor/guard are snapshots {data: dict, at: monotonic()}.
    """
    now = time.monotonic() if now is None else now
    if profile not in PROFILES:
        return "BLOCKED: 未知车型", False, False
    if selected_map is None:
        return "请先选择地图", False, False
    if health is None or now - health["at"] > HEALTH_TIMEOUT_S:
        return "定位状态未连接或过期", False, False
    h = health["data"]
    if int(h.get("status", -1)) != 1:
        return "定位未 READY：" + str(h.get("state", "UNKNOWN")), False, False
    if not h.get("localized") or not h.get("nav2_active") or not h.get("base_alive"):
        return "定位、Nav2 或底盘未就绪", False, False
    if (h.get("map_id"), str(h.get("map_version"))) != (
            selected_map.map_id, selected_map.version):
        return "机器人使用的地图与所选地图不同", False, False
    actual = str(h.get("robot_profile", "")).lower()
    if PROFILES[profile][1] not in actual:
        return "正在运行的机器人型号与选择不一致", False, False
    if editor is None or now - editor["at"] > EDITOR_TIMEOUT_S:
        return "路线工作台尚未连接", False, False
    state = editor["data"]
    if (state.get("map_id"), str(state.get("map_version"))) != (
            selected_map.map_id, selected_map.version):
        return "路线工作台地图不匹配", False, False
    if state.get("preview_only", True):
        return "路线工作台当前为仅预览模式", False, False
    if state.get("locked") or state.get("validating"):
        return "任务占用中或正在校验", False, False
    if profile == "fwmini":
        if guard is None or now - guard["at"] > GUARD_TIMEOUT_S:
            return "FW-mini 停车守卫状态缺失", False, False
        guard_state = str(guard["data"].get("state", ""))
        if guard_state not in ("WAIT_COMMAND", "NORMAL_ROUTE"):
            return "FW-mini 守卫阻止执行：" + guard_state, False, False
    return "可以生成路线", True, bool(state.get("can_execute", False))


def validate_distance(distance):
    try:
        distance = float(distance)
    except (ValueError, TypeError):
        raise ValueError("距离必须是数字") from None
    if not math.isfinite(distance) or not 0.1 <= distance <= 30.0:
        raise ValueError("允许的距离为 0.1～30 米")
    return distance
