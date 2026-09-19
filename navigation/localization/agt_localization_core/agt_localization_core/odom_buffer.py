"""Timestamped odometry selection with no ROS dependency."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

from .pose_math import Pose3


@dataclass(frozen=True)
class OdomSample:
    stamp_ns: int
    pose: Pose3


class OdomBuffer:
    """Legacy-equivalent timestamp buffer; callers own frame validation."""

    def __init__(self, duration_sec: float) -> None:
        self._duration_ns = max(0, int(float(duration_sec) * 1.0e9))
        self._samples: Deque[OdomSample] = deque()

    def append(self, sample: OdomSample) -> None:
        self._samples.append(sample)
        newest = sample.stamp_ns
        while self._samples and newest - self._samples[0].stamp_ns > self._duration_ns:
            self._samples.popleft()

    def nearest(self, stamp_ns: int, max_skew_sec: float) -> Optional[OdomSample]:
        if not self._samples:
            return None
        sample = min(self._samples, key=lambda item: abs(item.stamp_ns - stamp_ns))
        if abs(sample.stamp_ns - stamp_ns) / 1.0e9 > float(max_skew_sec):
            return None
        return sample

    def __len__(self) -> int:
        return len(self._samples)
