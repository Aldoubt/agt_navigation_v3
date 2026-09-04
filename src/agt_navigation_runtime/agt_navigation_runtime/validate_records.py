from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys


def truthy(value: str) -> bool:
    return str(value).strip().lower() in {'1', 'true', 'yes'}


def validate(directory: Path, expected_points: int, views_per_point: int, require_rtk: bool) -> list[str]:
    errors: list[str] = []
    csv_path = directory / 'captures.csv'
    mission_path = directory / 'mission.yaml'
    manifest_path = directory / 'manifest.json'
    if not csv_path.is_file():
        return ['missing captures.csv']
    if not mission_path.is_file():
        errors.append('missing mission.yaml')
    if not manifest_path.is_file():
        errors.append('missing manifest.json')

    with csv_path.open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    expected_rows = expected_points * views_per_point
    if len(rows) != expected_rows:
        errors.append(f'expected {expected_rows} capture rows, got {len(rows)}')

    by_point: dict[str, list[dict]] = {}
    for row in rows:
        by_point.setdefault(row.get('point_id', ''), []).append(row)
        image_path = Path(row.get('image_path', '')).expanduser()
        if not row.get('image_path'):
            errors.append(f"{row.get('point_id')}/{row.get('view_tag')}: empty image_path")
        elif not image_path.is_file():
            errors.append(f"{row.get('point_id')}/{row.get('view_tag')}: image missing: {image_path}")
        if not truthy(row.get('pose_valid', '')):
            errors.append(f"{row.get('point_id')}/{row.get('view_tag')}: pose_valid=false")
        if require_rtk and not truthy(row.get('rtk_valid', '')):
            errors.append(f"{row.get('point_id')}/{row.get('view_tag')}: rtk_valid=false")
        if int(float(row.get('camera_error_code') or 0)) != 0:
            errors.append(
                f"{row.get('point_id')}/{row.get('view_tag')}: camera_error_code={row.get('camera_error_code')}")
        for key in ('gimbal_heading', 'gimbal_roll', 'gimbal_pitch'):
            if row.get(key, '') == '':
                errors.append(f"{row.get('point_id')}/{row.get('view_tag')}: missing {key}")

    if len(by_point) != expected_points:
        errors.append(f'expected records for {expected_points} inspection points, got {len(by_point)}')
    for point_id, point_rows in by_point.items():
        if len(point_rows) != views_per_point:
            errors.append(f'{point_id}: expected {views_per_point} views, got {len(point_rows)}')

    return errors


def main(argv=None):
    parser = argparse.ArgumentParser(description='Validate AGT inspection-demo capture records.')
    parser.add_argument('directory', help='Mission record directory containing captures.csv')
    parser.add_argument('--expected-points', type=int, required=True)
    parser.add_argument('--views-per-point', type=int, default=3)
    parser.add_argument('--require-rtk', action='store_true', help='Fail if any capture lacks valid RTK')
    args = parser.parse_args(argv)

    directory = Path(args.directory).expanduser().resolve()
    errors = validate(directory, args.expected_points, args.views_per_point, args.require_rtk)
    if errors:
        print('DEMO RECORD VALIDATION FAILED', file=sys.stderr)
        for error in errors:
            print(f' - {error}', file=sys.stderr)
        raise SystemExit(2)
    print(
        f'DEMO RECORD VALIDATION PASS: points={args.expected_points} '
        f'views_per_point={args.views_per_point}')


if __name__ == '__main__':
    main()
