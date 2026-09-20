#!/usr/bin/env python3
"""Generate exact map-frame 5/7/10 m out-and-back Nav2 acceptance missions."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import yaml


def target_xy(start_x: float, start_y: float, heading_deg: float, distance_m: float):
    heading_rad = math.radians(heading_deg)
    return (
        start_x + distance_m * math.cos(heading_rad),
        start_y + distance_m * math.sin(heading_rad),
    )


def build_mission(
    *, map_id: str, distance_m: float, start_x: float, start_y: float,
    heading_deg: float, repeats: int, target_hold_sec: float, home_hold_sec: float,
) -> dict:
    target_x, target_y = target_xy(start_x, start_y, heading_deg, distance_m)
    yaw = math.radians(heading_deg)
    distance_tag = f'{distance_m:g}'.replace('.', 'p')
    points = []
    for index in range(1, repeats + 1):
        points.extend([
            {
                'id': f'D{distance_tag}M_T{index:02d}',
                'pose': {
                    'x': round(target_x, 6), 'y': round(target_y, 6),
                    'yaw': round(yaw, 9), 'frame_id': 'map',
                },
                'settle_time': target_hold_sec,
                'views': [],
            },
            {
                'id': f'HOME_T{index:02d}',
                'pose': {
                    'x': round(start_x, 6), 'y': round(start_y, 6),
                    'yaw': round(yaw, 9), 'frame_id': 'map',
                },
                'settle_time': home_hold_sec,
                'views': [],
            },
        ])
    return {
        'version': 1,
        'mission_id': f'distance_{distance_tag}m_acceptance',
        'map_id': map_id,
        'points': points,
    }


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=(
            'Generate metric map-frame missions. Heading 0 deg follows map +X; '
            '90 deg follows map +Y.'
        ))
    result.add_argument('--map-id', required=True)
    result.add_argument('--start-x', type=float, required=True)
    result.add_argument('--start-y', type=float, required=True)
    result.add_argument('--heading-deg', type=float, default=0.0)
    result.add_argument('--distances', type=float, nargs='+', default=[5.0, 7.0, 10.0])
    result.add_argument('--repeats', type=int, default=5)
    result.add_argument('--target-hold-sec', type=float, default=15.0)
    result.add_argument('--home-hold-sec', type=float, default=2.0)
    result.add_argument(
        '--output-dir', type=Path,
        default=Path('~/.ros/agt_distance_acceptance').expanduser())
    result.add_argument('--force', action='store_true')
    return result


def main() -> int:
    args = parser().parse_args()
    if args.repeats <= 0:
        raise SystemExit('--repeats must be > 0')
    if not args.map_id.strip():
        raise SystemExit('--map-id must not be empty')
    if any(not math.isfinite(value) for value in (
        args.start_x, args.start_y, args.heading_deg,
        args.target_hold_sec, args.home_hold_sec, *args.distances,
    )):
        raise SystemExit('all numeric values must be finite')
    if any(distance <= 0.0 for distance in args.distances):
        raise SystemExit('--distances must contain positive values')
    if args.target_hold_sec < 0.0 or args.home_hold_sec < 0.0:
        raise SystemExit('hold times must be >= 0')

    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = ['distance_m,start_x,start_y,target_x,target_y,heading_deg,yaw_rad']
    pending = []
    for distance in args.distances:
        tag = f'{distance:g}'.replace('.', 'p')
        path = output_dir / f'distance_{tag}m.yaml'
        if path.exists() and not args.force:
            raise SystemExit(f'refusing to overwrite {path}; pass --force to replace it')
        mission = build_mission(
            map_id=args.map_id, distance_m=distance,
            start_x=args.start_x, start_y=args.start_y,
            heading_deg=args.heading_deg, repeats=args.repeats,
            target_hold_sec=args.target_hold_sec, home_hold_sec=args.home_hold_sec,
        )
        pending.append((path, mission))
        target_x, target_y = target_xy(
            args.start_x, args.start_y, args.heading_deg, distance)
        rows.append(
            f'{distance:g},{args.start_x:.6f},{args.start_y:.6f},'
            f'{target_x:.6f},{target_y:.6f},{args.heading_deg:.6f},'
            f'{math.radians(args.heading_deg):.9f}')

    csv_path = output_dir / 'targets.csv'
    if csv_path.exists() and not args.force:
        raise SystemExit(f'refusing to overwrite {csv_path}; pass --force to replace it')
    for path, mission in pending:
        path.write_text(
            yaml.safe_dump(mission, sort_keys=False, allow_unicode=True),
            encoding='utf-8')
        print(path)
    csv_path.write_text('\n'.join(rows) + '\n', encoding='utf-8')
    print(csv_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
