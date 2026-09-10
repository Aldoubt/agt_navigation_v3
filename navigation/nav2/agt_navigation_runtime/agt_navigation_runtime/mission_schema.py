from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
import math
import yaml


@dataclass(frozen=True)
class ViewSpec:
    tag: str
    heading: float
    roll: float = 0.0
    pitch: float = 0.0
    tolerance: float = 0.0
    timeout: float = 0.0
    stable_samples: int = 0
    settle_time: float = 0.0
    image_timeout: float = 0.0
    save_image: bool = True
    required: bool = True


@dataclass(frozen=True)
class PointSpec:
    id: str
    x: float
    y: float
    yaw: float
    frame_id: str = 'map'
    settle_time: float = 1.0
    views: list[ViewSpec] = field(default_factory=list)


@dataclass(frozen=True)
class MissionSpec:
    mission_id: str
    map_id: str
    points: list[PointSpec]
    source_file: str


def _finite(value: Any, name: str) -> float:
    v = float(value)
    if not math.isfinite(v):
        raise ValueError(f'{name} must be finite')
    return v


def load_mission(path: str) -> MissionSpec:
    source = Path(path).expanduser().resolve()
    data = yaml.safe_load(source.read_text(encoding='utf-8')) or {}
    if int(data.get('version', 1)) != 1:
        raise ValueError('unsupported mission schema version')
    mission_id = str(data.get('mission_id', '')).strip()
    if not mission_id:
        raise ValueError('mission_id is required')
    map_id = str(data.get('map_id', '')).strip()
    points_raw = data.get('points') or []
    if not points_raw:
        raise ValueError('mission must contain at least one point')

    points: list[PointSpec] = []
    seen: set[str] = set()
    for p in points_raw:
        point_id = str(p.get('id', '')).strip()
        if not point_id or point_id in seen:
            raise ValueError(f'invalid or duplicate point id: {point_id!r}')
        seen.add(point_id)
        pose = p.get('pose') or {}
        views: list[ViewSpec] = []
        for i, v in enumerate(p.get('views') or []):
            tag = str(v.get('tag', f'{point_id}_view_{i:02d}')).strip()
            views.append(ViewSpec(
                tag=tag,
                heading=_finite(v.get('heading', 0.0), 'heading'),
                roll=_finite(v.get('roll', 0.0), 'roll'),
                pitch=_finite(v.get('pitch', 0.0), 'pitch'),
                tolerance=_finite(v.get('tolerance', 0.0), 'tolerance'),
                timeout=_finite(v.get('timeout', 0.0), 'timeout'),
                stable_samples=int(v.get('stable_samples', 0)),
                settle_time=_finite(v.get('settle_time', 0.0), 'settle_time'),
                image_timeout=_finite(v.get('image_timeout', 0.0), 'image_timeout'),
                save_image=bool(v.get('save_image', True)),
                required=bool(v.get('required', True)),
            ))
        points.append(PointSpec(
            id=point_id,
            x=_finite(pose.get('x'), 'pose.x'),
            y=_finite(pose.get('y'), 'pose.y'),
            yaw=_finite(pose.get('yaw', 0.0), 'pose.yaw'),
            frame_id=str(pose.get('frame_id', 'map')),
            settle_time=_finite(p.get('settle_time', 1.0), 'settle_time'),
            views=views,
        ))
    return MissionSpec(mission_id=mission_id, map_id=map_id, points=points, source_file=str(source))
