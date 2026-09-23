"""Map quality metrics, including connected unknown regions."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Tuple

import numpy as np

try:
    from scipy import ndimage
except ImportError:  # pragma: no cover - fallback is for minimal installations
    ndimage = None


@dataclass
class MapQualityMetric:
    topic: str
    timestamp_ns: int
    width: int
    height: int
    resolution: float
    unknown_ratio: float
    free_ratio: float
    occupied_ratio: float
    unknown_connected_components: int
    largest_unknown_area: float
    quality: str
    risk: str


def _unknown_components(unknown: np.ndarray, connectivity: int = 8) -> Tuple[int, int]:
    """Return component count and largest component size in cells."""
    height, width = unknown.shape
    if ndimage is not None:
        if connectivity == 4:
            structure = np.asarray(((0, 1, 0), (1, 1, 1), (0, 1, 0)), dtype=np.uint8)
        else:
            structure = np.ones((3, 3), dtype=np.uint8)
        labels, components = ndimage.label(unknown, structure=structure)
        if components == 0:
            return 0, 0
        counts = np.bincount(labels.ravel())
        return int(components), int(np.max(counts[1:]))

    # Dependency-free fallback for installations without scipy.
    visited = np.zeros_like(unknown, dtype=bool)
    if connectivity == 4:
        neighbors = ((-1, 0), (1, 0), (0, -1), (0, 1))
    else:
        neighbors = tuple((dy, dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if (dy, dx) != (0, 0))
    components = 0
    largest = 0
    for y, x in zip(*np.nonzero(unknown & ~visited)):
        components += 1
        stack = [(int(y), int(x))]
        visited[y, x] = True
        size = 0
        while stack:
            cy, cx = stack.pop()
            size += 1
            for dy, dx in neighbors:
                ny, nx = cy + dy, cx + dx
                if 0 <= ny < height and 0 <= nx < width and unknown[ny, nx] and not visited[ny, nx]:
                    visited[ny, nx] = True
                    stack.append((ny, nx))
        largest = max(largest, size)
    return components, largest


def quality_label(unknown_ratio: float) -> str:
    if unknown_ratio < 0.10:
        return 'GOOD'
    if unknown_ratio <= 0.30:
        return 'WARNING'
    return 'BAD'


def risk_label(unknown_ratio: float) -> str:
    if unknown_ratio > 0.50:
        return 'HIGH'
    if unknown_ratio > 0.30:
        return 'MEDIUM'
    return 'LOW'


def analyze_map_message(message, topic: str, timestamp_ns: int, connectivity: int = 8) -> MapQualityMetric:
    width = int(message.info.width)
    height = int(message.info.height)
    values = np.asarray(message.data, dtype=np.int16)
    expected = width * height
    if values.size != expected:
        raise ValueError(f'{topic} has {values.size} cells, expected {expected}')
    grid = values.reshape((height, width))
    total = max(expected, 1)
    unknown = grid < 0
    components, largest_cells = _unknown_components(unknown, connectivity)
    unknown_ratio = float(np.count_nonzero(unknown) / total)
    return MapQualityMetric(
        topic=topic,
        timestamp_ns=timestamp_ns,
        width=width,
        height=height,
        resolution=float(message.info.resolution),
        unknown_ratio=unknown_ratio,
        free_ratio=float(np.count_nonzero(grid == 0) / total),
        occupied_ratio=float(np.count_nonzero(grid >= 100) / total),
        unknown_connected_components=components,
        largest_unknown_area=float(largest_cells * float(message.info.resolution) ** 2),
        quality=quality_label(unknown_ratio),
        risk=risk_label(unknown_ratio),
    )


def write_map_quality_csv(metrics: Iterable[MapQualityMetric], output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'topic', 'timestamp', 'width', 'height', 'resolution', 'unknown_ratio',
            'free_ratio', 'occupied_ratio', 'unknown_connected_components',
            'largest_unknown_area', 'quality', 'risk',
        ])
        for item in metrics:
            writer.writerow([
                item.topic, f'{item.timestamp_ns / 1e9:.9f}', item.width, item.height,
                f'{item.resolution:.6f}', f'{item.unknown_ratio:.6f}',
                f'{item.free_ratio:.6f}', f'{item.occupied_ratio:.6f}',
                item.unknown_connected_components, f'{item.largest_unknown_area:.6f}',
                item.quality, item.risk,
            ])
