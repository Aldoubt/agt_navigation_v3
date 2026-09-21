#!/usr/bin/env python3
"""Offline forensics for the Nav2 local costmap (no ROS runtime required).

Question answered: when the local costmap looks dirty, is the dark area
NO_INFORMATION (never observed), a LETHAL mark (a spurious obstacle), or
INFLATED cost (a real obstacle plus geometry)? The three cases have completely
different fixes, and only the recorded cell values can separate them.

Input : a directory produced by scripts/record_local_costmap_diagnostics.sh
        (one rosbag2 sqlite3 bag per scenario) plus its scenario.yaml.
Output: per-scenario CSV time series and a summary.md with the verdict.

Only stdlib is used so the tool runs on any host; the bag must use the sqlite3
storage plugin (the recorder script enforces that).

  python3 tools/local_costmap_forensics.py --root DIR [--statistics FILE]
  python3 tools/local_costmap_forensics.py --bag DIR --bag DIR
  python3 tools/local_costmap_forensics.py --selftest
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sqlite3
import statistics
import struct
import sys
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

COSTMAP_RAW = "/local_costmap/costmap_raw"
COSTMAP_MAPPED = "/local_costmap/costmap"
FOOTPRINT = "/local_costmap/published_footprint"

FREE = 0
INSCRIBED = 253
LETHAL = 254
NO_INFORMATION = 255


# --------------------------------------------------------------------------- CDR


class CdrReader:
    """Minimal CDR (little-endian) reader for the messages decoded here."""

    ENCAPSULATION = 4  # CDR alignment is measured from the end of the encapsulation header

    def __init__(self, blob: bytes) -> None:
        if len(blob) < 4:
            raise ValueError("CDR payload is shorter than its encapsulation header")
        self._data = blob
        self._offset = self.ENCAPSULATION

    def _align(self, size: int) -> None:
        remainder = (self._offset - self.ENCAPSULATION) % size
        if remainder:
            self._offset += size - remainder

    def i8_array(self, count: int) -> bytes:
        end = self._offset + count
        if end > len(self._data):
            raise ValueError("CDR payload ended inside an int8 array")
        chunk = self._data[self._offset:end]
        self._offset = end
        return chunk

    def u32(self) -> int:
        self._align(4)
        return struct.unpack_from("<I", self._data, self._advance(4))[0]

    def _advance(self, size: int) -> int:
        start = self._offset
        self._offset += size
        return start

    def i32(self) -> int:
        self._align(4)
        return struct.unpack_from("<i", self._data, self._advance(4))[0]

    def f32(self) -> float:
        self._align(4)
        return struct.unpack_from("<f", self._data, self._advance(4))[0]

    def f64(self) -> float:
        self._align(8)
        return struct.unpack_from("<d", self._data, self._advance(8))[0]

    def string(self) -> str:
        length = self.u32()
        if length == 0:
            return ""
        raw = self.i8_array(length)
        return raw.rstrip(b"\x00").decode("utf-8", errors="replace")

    def header(self) -> Tuple[float, str]:
        seconds = self.i32()
        nanoseconds = self.u32()
        frame_id = self.string()
        return seconds + nanoseconds * 1e-9, frame_id


@dataclass
class OccupancyGrid:
    stamp: float
    frame_id: str
    resolution: float
    width: int
    height: int
    origin: Tuple[float, float]
    values: bytes


def decode_occupancy_grid(blob: bytes) -> OccupancyGrid:
    reader = CdrReader(blob)
    stamp, frame_id = reader.header()
    reader.i32()  # map_load_time.sec
    reader.u32()  # map_load_time.nanosec
    resolution = reader.f32()
    width = reader.u32()
    height = reader.u32()
    origin_x = reader.f64()
    origin_y = reader.f64()
    reader.f64()  # origin.position.z
    for _ in range(4):  # origin.orientation quaternion
        reader.f64()
    count = reader.u32()
    values = reader.i8_array(count)
    return OccupancyGrid(stamp, frame_id, resolution, width, height, (origin_x, origin_y), values)


def decode_polygon_stamped(blob: bytes) -> Tuple[float, str, List[Tuple[float, float]]]:
    reader = CdrReader(blob)
    stamp, frame_id = reader.header()
    count = reader.u32()
    points: List[Tuple[float, float]] = []
    for _ in range(count):
        points.append((reader.f32(), reader.f32()))
        reader.f32()  # z
    return stamp, frame_id, points


# --------------------------------------------------------------------------- bags


@dataclass
class BagMessage:
    timestamp: float  # seconds
    data: bytes


@dataclass
class Bag:
    path: Path
    topics: Dict[str, str] = field(default_factory=dict)  # name -> type
    messages: Dict[str, List[BagMessage]] = field(default_factory=dict)

    def has(self, topic: str) -> bool:
        return topic in self.topics


def _storage_identifier(bag_dir: Path) -> Optional[str]:
    metadata = bag_dir / "metadata.yaml"
    if not metadata.is_file():
        return None
    text = metadata.read_text(encoding="utf-8", errors="replace")
    try:  # PyYAML is available in a ROS environment but must not be required
        import yaml  # type: ignore

        parsed = yaml.safe_load(text) or {}
        info = parsed.get("rosbag2_bagfile_information", {})
        return info.get("storage_identifier")
    except Exception:
        match = re.search(r"storage_identifier:\s*(\S+)", text)
        return match.group(1) if match else None


def load_bag(bag_dir: Path, wanted: Sequence[str], max_frames: int) -> Bag:
    bag = Bag(path=bag_dir)
    identifier = _storage_identifier(bag_dir)
    if identifier and identifier != "sqlite3":
        raise RuntimeError(
            f"{bag_dir}: storage '{identifier}' is not supported; re-record with --storage sqlite3")
    db_files = sorted(bag_dir.glob("*.db3"))
    if not db_files:
        raise RuntimeError(f"{bag_dir}: no .db3 file found (was the bag finalised?)")

    topic_ids: Dict[int, str] = {}
    rows: List[Tuple[str, float, bytes]] = []
    for db_file in db_files:
        connection = sqlite3.connect(f"file:{db_file}?mode=ro", uri=True)
        try:
            cursor = connection.cursor()
            for topic_id, name, type_name in cursor.execute("SELECT id, name, type FROM topics"):
                if name in wanted:
                    topic_ids[int(topic_id)] = name
                    bag.topics[name] = type_name
            if not topic_ids:
                continue
            placeholders = ",".join("?" for _ in topic_ids)
            query = (
                f"SELECT topic_id, timestamp, data FROM messages "
                f"WHERE topic_id IN ({placeholders}) ORDER BY timestamp"
            )
            for topic_id, timestamp, data in cursor.execute(query, list(topic_ids)):
                rows.append((topic_ids[int(topic_id)], float(timestamp) * 1e-9, bytes(data)))
        finally:
            connection.close()

    for topic in wanted:
        bag.messages.setdefault(topic, [])
    rows.sort(key=lambda item: item[1])
    stride = max(1, math.ceil(len(rows) / (max_frames * max(1, len(wanted)))))
    for index, (name, timestamp, data) in enumerate(rows):
        if index % stride:
            continue
        bag.messages[name].append(BagMessage(timestamp, data))
    return bag


# --------------------------------------------------------------------------- metrics


@dataclass
class FrameStats:
    stamp: float
    pct_free: float
    pct_inflated: float
    pct_lethal: float
    pct_unknown: float
    footprint_lethal_cells: int
    footprint_max_cost: int


def classify(values: bytes) -> Tuple[int, int, int, int]:
    """Classify raw costmap cells: free / inflated (incl. inscribed) / lethal / unknown."""
    free = inflated = lethal = unknown = 0
    for raw_byte in values:
        value = raw_byte & 0xFF
        if value == FREE:
            free += 1
        elif value >= NO_INFORMATION:
            unknown += 1
        elif value == LETHAL:
            lethal += 1
        else:  # 1..252 cost gradient, 253 INSCRIBED_INFLATED_OBSTACLE
            inflated += 1
    return free, inflated, lethal, unknown


def footprint_cells(
    grid: OccupancyGrid, polygon: Sequence[Tuple[float, float]]
) -> Tuple[int, int]:
    """Return (lethal cell count, highest normalised 0-99 cost) inside the footprint."""
    if not polygon:
        return 0, 0
    resolution = grid.resolution
    origin_x, origin_y = grid.origin
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)
    start_x = max(0, int(math.floor((min_x - origin_x) / resolution)))
    end_x = min(grid.width - 1, int(math.ceil((max_x - origin_x) / resolution)))
    start_y = max(0, int(math.floor((min_y - origin_y) / resolution)))
    end_y = min(grid.height - 1, int(math.ceil((max_y - origin_y) / resolution)))

    lethal_cells = 0
    max_cost = 0
    for cell_x in range(start_x, end_x + 1):
        world_x = origin_x + (cell_x + 0.5) * resolution
        for cell_y in range(start_y, end_y + 1):
            world_y = origin_y + (cell_y + 0.5) * resolution
            if not _inside_polygon(world_x, world_y, polygon):
                continue
            value = grid.values[cell_y * grid.width + cell_x] & 0xFF
            if value == LETHAL:
                lethal_cells += 1
            elif value == INSCRIBED:
                max_cost = max(max_cost, 99)
            elif value < NO_INFORMATION:
                max_cost = max(max_cost, min(99, value))
    return lethal_cells, max_cost


def _inside_polygon(x: float, y: float, polygon: Sequence[Tuple[float, float]]) -> bool:
    inside = False
    count = len(polygon)
    for index in range(count):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % count]
        if (y1 > y) != (y2 > y):
            x_cross = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < x_cross:
                inside = not inside
    return inside


def analyse_bag(bag: Bag) -> List[FrameStats]:
    raw_frames = bag.messages.get(COSTMAP_RAW, [])
    polygons = bag.messages.get(FOOTPRINT, [])
    polygon_index = 0
    stats: List[FrameStats] = []

    for message in raw_frames:
        try:
            grid = decode_occupancy_grid(message.data)
        except Exception:
            continue
        total = grid.width * grid.height
        if total == 0:
            continue
        free, inflated, lethal, unknown = classify(grid.values)

        # published_footprint is published in the costmap frame; consume the newest
        # polygon that is not newer than the grid stamp.
        polygon: List[Tuple[float, float]] = []
        while polygon_index < len(polygons) and polygons[polygon_index].timestamp <= grid.stamp + 1.0:
            try:
                _, frame_id, candidate = decode_polygon_stamped(polygons[polygon_index].data)
            except Exception:
                polygon_index += 1
                continue
            if frame_id == grid.frame_id:
                polygon = candidate
            polygon_index += 1

        lethal_cells, max_cost = footprint_cells(grid, polygon) if polygon else (0, 0)

        stats.append(FrameStats(
            stamp=grid.stamp,
            pct_free=100.0 * free / total,
            pct_inflated=100.0 * inflated / total,
            pct_lethal=100.0 * lethal / total,
            pct_unknown=100.0 * unknown / total,
            footprint_lethal_cells=lethal_cells,
            footprint_max_cost=max_cost,
        ))
    return stats


# --------------------------------------------------------------------------- reporting


def median_or_none(values: Iterable[float]) -> Optional[float]:
    collected = [value for value in values]
    if not collected:
        return None
    return statistics.median(collected)


def window(stats: Sequence[FrameStats], start: float, end: float) -> List[FrameStats]:
    return [item for item in stats if start <= item.stamp <= end]


def summarise(stats: Sequence[FrameStats]) -> Dict[str, Optional[float]]:
    return {
        "frames": float(len(stats)),
        "pct_free": median_or_none(item.pct_free for item in stats),
        "pct_inflated": median_or_none(item.pct_inflated for item in stats),
        "pct_lethal": median_or_none(item.pct_lethal for item in stats),
        "pct_unknown": median_or_none(item.pct_unknown for item in stats),
        "footprint_lethal_cells": median_or_none(float(item.footprint_lethal_cells) for item in stats),
        "footprint_max_cost": median_or_none(float(item.footprint_max_cost) for item in stats),
    }


def format_value(value: Optional[float]) -> str:
    return "-" if value is None else f"{value:.2f}"


def _parse_scenario_yaml(text: str) -> Optional[dict]:
    """Indentation-based fallback for scenario.yaml, used when PyYAML is absent.

    The recorder writes a deliberately flat document, so a small parser keeps the
    analyser usable on a host whose system python has no PyYAML.
    """
    scenarios: List[dict] = []
    current: Optional[dict] = None
    cue: Optional[dict] = None
    in_cues = False
    for raw_line in text.splitlines():
        line = raw_line.rstrip()
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        stripped = line.strip()
        if stripped == "scenarios:":
            continue
        if stripped.startswith("- name:") and line.startswith("  - "):
            current = {"name": stripped.split(":", 1)[1].strip().strip('"')}
            scenarios.append(current)
            cue = None
            in_cues = False
            continue
        if current is None:
            continue
        if stripped == "cues:":
            in_cues = True
            current.setdefault("cues", [])
            continue
        if stripped.startswith("- epoch:") and line.startswith("      - "):
            cue = {"epoch": float(stripped.split(":", 1)[1].strip())}
            current.setdefault("cues", []).append(cue)
            continue
        if cue is not None and stripped.startswith("note:"):
            cue["note"] = stripped.split(":", 1)[1].strip().strip('"')
            continue
        if in_cues and line.startswith("    ") and ":" in stripped:
            in_cues = False
        if ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        value = value.strip().strip('"')
        if key in {"title", "bag_dir", "action"}:
            current[key] = value
        elif key in {"started_epoch", "ended_epoch"} and value:
            current[key] = float(value)
        elif key == "metadata_complete":
            current[key] = value.lower() == "true"
    if not scenarios:
        return None
    return {"scenarios": scenarios}


def load_scenarios(root: Path) -> Dict[str, dict]:
    scenario_file = root / "scenario.yaml"
    if not scenario_file.is_file():
        return {}
    text = scenario_file.read_text(encoding="utf-8", errors="replace")
    try:
        import yaml  # type: ignore

        parsed = yaml.safe_load(text) or {}
    except Exception:
        parsed = _parse_scenario_yaml(text) or {}
    return {item["name"]: item for item in parsed.get("scenarios", []) if "name" in item}


def build_windows(
    stats: Sequence[FrameStats], cues: Sequence[dict]
) -> Dict[str, List[FrameStats]]:
    """Split a scenario into comparable observation windows.

    With the recorder's standard cues the first cue marks the start of the stimulus
    (vehicle stationary, person behind, ...) and the second cue marks the moment the
    stimulus is removed, so the decay of lethal/unknown cells can be measured.
    """
    if not stats:
        return {}
    start = stats[0].stamp
    end = stats[-1].stamp
    windows: Dict[str, List[FrameStats]] = {"whole": list(stats)}
    if not cues:
        return windows
    first = float(cues[0]["epoch"])
    if len(cues) >= 2:
        second = float(cues[1]["epoch"])
        windows["during"] = window(stats, first, second)
        windows["after_2_12s"] = window(stats, second + 2.0, second + 12.0)
        windows["after_12s_plus"] = window(stats, second + 12.0, end)
    else:
        windows["during"] = window(stats, first, min(first + 10.0, end))
        windows["after_2_12s"] = window(stats, first + 10.0, min(first + 20.0, end))
        windows["after_12s_plus"] = window(stats, first + 20.0, end)
    if not windows["during"]:
        windows["during"] = window(stats, start, end)
    return windows


def peak(values: Sequence[Optional[float]]) -> Optional[float]:
    present = [value for value in values if value is not None]
    return max(present) if present else None


def verdict_for(scenario: str, windows: Dict[str, List[FrameStats]]) -> str:
    if not windows:
        return f"{scenario}: 没有可用于判读的帧"
    whole = summarise(windows["whole"])
    during = summarise(windows["during"]) if windows.get("during") else whole
    after = summarise(windows["after_2_12s"]) if windows.get("after_2_12s") else None
    late = summarise(windows["after_12s_plus"]) if windows.get("after_12s_plus") else None

    unknown_peak = peak([
        (during or {}).get("pct_unknown"),
        (whole or {}).get("pct_unknown"),
    ]) or 0.0
    lethal_during = (during or {}).get("pct_lethal") or 0.0
    lethal_after = (after or {}).get("pct_lethal")
    lethal_late = (late or {}).get("pct_lethal")
    inflated_peak = peak([
        (during or {}).get("pct_inflated"),
        (whole or {}).get("pct_inflated"),
    ]) or 0.0
    footprint_peak = peak([
        (during or {}).get("footprint_lethal_cells"),
        (after or {}).get("footprint_lethal_cells"),
        (whole or {}).get("footprint_lethal_cells"),
    ]) or 0.0

    verdicts: List[str] = []
    if unknown_peak >= 20.0:
        verdicts.append(
            f"H2 未知空间着色：观测窗口内未观测 cell 峰值占 {unknown_peak:.1f}%，"
            "深色区域主要是 NO_INFORMATION（从未被观测），不是假障碍")
    if lethal_during >= 0.5 or footprint_peak >= 1.0:
        footprint_note = (
            f"，足迹内致命 cell 峰值 {footprint_peak:.0f} 个（若稳定不动，优先怀疑自车/挂载反射 H4）"
            if footprint_peak >= 1.0 else "")
        if lethal_after is None:
            verdicts.append(
                f"H3 致命标记：窗口内 lethal 占 {lethal_during:.2f}%{footprint_note}"
                "（该段没有第二个提示，无法判清除时延）")
        elif lethal_after <= 0.5 * max(lethal_during, 1e-9):
            verdicts.append(
                f"H3 清除正常：lethal 由 {lethal_during:.2f}% 降到 {lethal_after:.2f}%"
                f"（12 s 后 {format_value(lethal_late)}%）{footprint_note}")
        else:
            verdicts.append(
                f"H3 清除滞后：刺激移除后 lethal 仍有 {lethal_after:.2f}%"
                f"（移除前 {lethal_during:.2f}%，12 s 后 {format_value(lethal_late)}%）"
                f"{footprint_note} → 查 TF 失败帧比例、mark_threshold、raytrace 范围")
    if inflated_peak >= 20.0 and lethal_during < 0.5:
        verdicts.append(
            f"H1 膨胀几何：膨胀代价峰值占 {inflated_peak:.1f}% 而几乎无致命 cell，"
            "属膨胀半径/footprint/通道宽度效应，不是感知错误")
    if not verdicts:
        verdicts.append(
            "证据不足：三种成分都不突出；请确认录到了 costmap_raw，"
            "并附上 preprocessor 的 statistics_output 与 TF 失败计数")
    return f"{scenario}: " + "；".join(verdicts)


def analyse_bag_dir(bag_dir: Path, max_frames: int, out_dir: Path) -> List[FrameStats]:
    bag = load_bag(bag_dir, [COSTMAP_RAW, COSTMAP_MAPPED, FOOTPRINT], max_frames)
    stats = analyse_bag(bag)
    if stats:
        csv_path = out_dir / f"{bag_dir.name}_frames.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["stamp", "pct_free", "pct_inflated", "pct_lethal", "pct_unknown",
                             "footprint_lethal_cells", "footprint_max_cost"])
            for item in stats:
                writer.writerow([f"{item.stamp:.3f}", f"{item.pct_free:.2f}", f"{item.pct_inflated:.2f}",
                                 f"{item.pct_lethal:.2f}", f"{item.pct_unknown:.2f}",
                                 item.footprint_lethal_cells, item.footprint_max_cost])
    return stats


def statistics_table(path: Path) -> List[str]:
    if not path.is_file():
        return [f"- 未找到过滤统计文件：`{path}`（说明 preprocessor 未以 `statistics_output` 启动，或未正常退出）"]
    text = path.read_text(encoding="utf-8", errors="replace")
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    table = ["```text", *lines, "```"]
    return table


def run_analysis(root: Path, bag_dirs: Sequence[Path], statistics: Optional[Path],
                  out_dir: Path, max_frames: int) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    scenarios = load_scenarios(root) if root else {}
    report = ["# Local costmap forensics", "",
              f"- generated: {time.strftime('%Y-%m-%d %H:%M:%S')}",
              f"- bags: {len(bag_dirs)}", ""]

    all_ok = True
    for bag_dir in bag_dirs:
        name = bag_dir.name
        stats = analyse_bag_dir(bag_dir, max_frames, out_dir)
        report.append(f"## {name}")
        if not stats:
            report.append("- 没有解码到 `/local_costmap/costmap_raw` 帧；确认该话题已被录制")
            all_ok = False
            report.append("")
            continue

        summary = summarise(stats)
        report.append("| metric | median | peak |")
        report.append("| --- | ---: | ---: |")
        report.append(f"| frames | {format_value(summary.get('frames'))} | - |")
        for key in ("pct_free", "pct_inflated", "pct_lethal", "pct_unknown",
                    "footprint_lethal_cells", "footprint_max_cost"):
            peak_value = peak([getattr(item, key) for item in stats])
            report.append(
                f"| {key} | {format_value(summary.get(key))} | {format_value(peak_value)} |")

        scenario = scenarios.get(name.split("_")[0], {})
        windows = build_windows(stats, scenario.get("cues", []))
        if len(windows) > 1:
            report.append("")
            report.append("| window | pct_lethal | pct_unknown | pct_inflated | footprint lethal cells |")
            report.append("| --- | ---: | ---: | ---: | ---: |")
            labels = {
                "during": "刺激存在期间（cue1→cue2）",
                "after_2_12s": "刺激移除后 +2…+12 s",
                "after_12s_plus": "刺激移除后 +12 s 之后",
            }
            for key in ("during", "after_2_12s", "after_12s_plus"):
                item = summarise(windows.get(key, []))
                report.append(
                    f"| {labels[key]} | {format_value(item.get('pct_lethal'))} | "
                    f"{format_value(item.get('pct_unknown'))} | "
                    f"{format_value(item.get('pct_inflated'))} | "
                    f"{format_value(item.get('footprint_lethal_cells'))} |")

        report.append("")
        report.append(f"**判读**：{verdict_for(name, windows)}")
        report.append("")

    if statistics is not None:
        report.append("## preprocessor filter statistics")
        report.extend(statistics_table(statistics))
        report.append("")

    summary_path = out_dir / "summary.md"
    summary_path.write_text("\n".join(report) + "\n", encoding="utf-8")
    print("\n".join(report))
    print(f"\n[local_costmap_forensics] wrote {summary_path}")
    return 0 if all_ok else 1


# --------------------------------------------------------------------------- selftest


def _cdr_align(buffer: bytearray, size: int) -> None:
    remainder = (len(buffer) - CdrReader.ENCAPSULATION) % size
    if remainder:
        buffer.extend(b"\x00" * (size - remainder))


def _encode_string(buffer: bytearray, value: str) -> None:
    encoded = value.encode("utf-8") + b"\x00"
    buffer.extend(struct.pack("<I", len(encoded)))
    buffer.extend(encoded)


def _encode_header(buffer: bytearray, seconds: int, nanos: int, frame_id: str) -> None:
    _cdr_align(buffer, 4)
    buffer.extend(struct.pack("<iI", seconds, nanos))
    _encode_string(buffer, frame_id)


def encode_occupancy_grid(seconds: int, nanos: int, frame_id: str, width: int, height: int,
                          resolution: float, origin: Tuple[float, float], values: bytes) -> bytes:
    buffer = bytearray(b"\x00\x01\x00\x00")
    _encode_header(buffer, seconds, nanos, frame_id)
    _cdr_align(buffer, 4)
    buffer.extend(struct.pack("<iI", seconds, nanos))  # map_load_time
    _cdr_align(buffer, 4)
    buffer.extend(struct.pack("<f", resolution))
    _cdr_align(buffer, 4)
    buffer.extend(struct.pack("<II", width, height))
    _cdr_align(buffer, 8)
    buffer.extend(struct.pack("<ddd", origin[0], origin[1], 0.0))
    buffer.extend(struct.pack("<dddd", 0.0, 0.0, 0.0, 1.0))
    _cdr_align(buffer, 4)
    buffer.extend(struct.pack("<I", len(values)))
    buffer.extend(values)
    return bytes(buffer)


def encode_polygon_stamped(seconds: int, nanos: int, frame_id: str,
                           points: Sequence[Tuple[float, float]]) -> bytes:
    buffer = bytearray(b"\x00\x01\x00\x00")
    _encode_header(buffer, seconds, nanos, frame_id)
    _cdr_align(buffer, 4)
    buffer.extend(struct.pack("<I", len(points)))
    for point in points:
        _cdr_align(buffer, 4)
        buffer.extend(struct.pack("<fff", point[0], point[1], 0.0))
    return bytes(buffer)


def selftest() -> int:
    with tempfile.TemporaryDirectory() as tmp:
        bag_dir = Path(tmp) / "s1_起点狭窄处静止"
        bag_dir.mkdir()
        database = bag_dir / "s1_0.db3"
        connection = sqlite3.connect(database)
        connection.executescript(
            "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,"
            " serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);"
            "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL,"
            " timestamp INTEGER NOT NULL, data BLOB NOT NULL);"
        )
        topics = [(1, COSTMAP_RAW, "nav_msgs/msg/OccupancyGrid"),
                  (2, COSTMAP_MAPPED, "nav_msgs/msg/OccupancyGrid"),
                  (3, FOOTPRINT, "geometry_msgs/msg/PolygonStamped")]
        connection.executemany("INSERT INTO topics VALUES (?,?,?,'cdr','')", topics)

        width = height = 16  # 16 x 16 cells at 0.5 m -> 8 x 8 m window
        values = bytearray([FREE]) * (width * height)
        for index in range(4):  # a 4-cell lethal row keeps the expected fraction exact
            values[index] = LETHAL
        values[40] = NO_INFORMATION
        raw_blob = encode_occupancy_grid(100, 0, "odom", width, height, 0.5, (-4.0, -4.0), bytes(values))
        mapped = bytearray(values)
        mapped[40] = 255
        mapped_blob = encode_occupancy_grid(100, 0, "odom", width, height, 0.5, (-4.0, -4.0), bytes(mapped))
        footprint_blob = encode_polygon_stamped(
            100, 0, "odom", [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)])

        connection.executemany(
            "INSERT INTO messages (topic_id, timestamp, data) VALUES (?,?,?)",
            [(1, 100_000_000_000, raw_blob),
             (2, 100_000_000_000, mapped_blob),
             (3, 100_000_000_000, footprint_blob)])
        connection.commit()
        connection.close()
        (bag_dir / "metadata.yaml").write_text(
            "rosbag2_bagfile_information:\n  storage_identifier: sqlite3\n", encoding="utf-8")

        bag = load_bag(bag_dir, [COSTMAP_RAW, COSTMAP_MAPPED, FOOTPRINT], max_frames=100)
        stats = analyse_bag(bag)
        if len(stats) != 1:
            print(f"SELFTEST FAIL: expected 1 frame, decoded {len(stats)}")
            return 1
        frame = stats[0]
        total = width * height
        expected_lethal = 100.0 * 4 / total
        expected_unknown = 100.0 * 1 / total
        if abs(frame.pct_lethal - expected_lethal) > 1e-6:
            print(f"SELFTEST FAIL: lethal {frame.pct_lethal} != {expected_lethal}")
            return 1
        if abs(frame.pct_unknown - expected_unknown) > 1e-6:
            print(f"SELFTEST FAIL: unknown {frame.pct_unknown} != {expected_unknown}")
            return 1
        if frame.footprint_lethal_cells != 0:
            print("SELFTEST FAIL: footprint should not touch the lethal row")
            return 1
        # footprint inside the lethal row must be detected
        noisy = bytearray([FREE]) * (width * height)
        for cell_y in range(0, 2):
            for cell_x in range(0, width):
                noisy[cell_y * width + cell_x] = LETHAL
        noisy_blob = encode_occupancy_grid(101, 0, "odom", width, height, 0.5, (-4.0, -4.0), bytes(noisy))
        noisy_polygon = encode_polygon_stamped(
            101, 0, "odom", [(-3.6, -3.9), (-2.6, -3.9), (-2.6, -2.9), (-3.6, -2.9)])
        bag2 = Bag(path=bag_dir)
        bag2.topics = {COSTMAP_RAW: "nav_msgs/msg/OccupancyGrid", FOOTPRINT: "geometry_msgs/msg/PolygonStamped"}
        bag2.messages = {COSTMAP_RAW: [BagMessage(101.0, noisy_blob)],
                         FOOTPRINT: [BagMessage(101.0, noisy_polygon)]}
        stats2 = analyse_bag(bag2)
        if not stats2 or stats2[0].footprint_lethal_cells <= 0:
            print("SELFTEST FAIL: footprint lethal cells were not detected")
            return 1
        print("SELFTEST PASS: CDR decoding, costmap classification and footprint check are consistent")
    return 0


# --------------------------------------------------------------------------- cli


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", type=Path, help="diagnostics output directory containing s*/ bag folders")
    parser.add_argument("--bag", type=Path, action="append", default=[], help="bag directory (repeatable)")
    parser.add_argument("--statistics", type=Path, help="preprocessor statistics_output YAML")
    parser.add_argument("--out", type=Path, help="analysis output directory (default: <root>/analysis)")
    parser.add_argument("--max-frames", type=int, default=1500, help="frames decoded per topic (default 1500)")
    parser.add_argument("--selftest", action="store_true", help="validate the decoder on a synthetic bag")
    args = parser.parse_args()

    if args.selftest:
        return selftest()

    bag_dirs = list(args.bag)
    if args.root is not None:
        if not args.root.is_dir():
            parser.error(f"--root is not a directory: {args.root}")
        if not bag_dirs:
            bag_dirs = sorted(path for path in args.root.iterdir() if path.is_dir() and path.name.startswith("s"))
    if not bag_dirs:
        parser.error("provide --root or at least one --bag")

    out_dir = args.out or ((args.root / "analysis") if args.root else Path("analysis"))
    return run_analysis(args.root or Path("."), bag_dirs, args.statistics, out_dir, args.max_frames)


if __name__ == "__main__":
    sys.exit(main())
