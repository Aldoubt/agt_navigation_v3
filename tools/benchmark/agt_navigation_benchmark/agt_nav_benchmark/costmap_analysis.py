"""OccupancyGrid statistics for static and rolling costmaps."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass
class CostmapMetric:
    topic: str
    timestamp_ns: int
    map_size: str
    unknown_ratio: float
    occupied_ratio: float
    free_ratio: float


def analyze_costmap_message(message, topic: str, timestamp_ns: int) -> CostmapMetric:
    values = np.asarray(message.data, dtype=np.int16)
    total = max(int(values.size), 1)
    return CostmapMetric(
        topic=topic,
        timestamp_ns=timestamp_ns,
        map_size=f'{int(message.info.width)}x{int(message.info.height)}',
        unknown_ratio=float(np.count_nonzero(values < 0) / total),
        occupied_ratio=float(np.count_nonzero(values >= 100) / total),
        free_ratio=float(np.count_nonzero(values == 0) / total),
    )


def write_costmap_csv(metrics: Iterable[CostmapMetric], output_path: Path) -> None:
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'topic', 'timestamp', 'map_size', 'unknown_ratio', 'occupied_ratio', 'free_ratio',
        ])
        for item in metrics:
            writer.writerow([
                item.topic, f'{item.timestamp_ns / 1e9:.9f}', item.map_size,
                f'{item.unknown_ratio:.6f}', f'{item.occupied_ratio:.6f}', f'{item.free_ratio:.6f}',
            ])
