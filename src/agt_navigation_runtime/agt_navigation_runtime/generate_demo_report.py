from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def _truthy(value) -> bool:
    return str(value).strip().lower() in {'1', 'true', 'yes'}


def build_report(directory: Path) -> str:
    manifest_path = directory / 'manifest.json'
    captures_path = directory / 'captures.csv'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8')) if manifest_path.exists() else {}
    with captures_path.open(newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))

    by_point = defaultdict(list)
    for row in rows:
        by_point[row.get('point_id', '')].append(row)

    images_ok = sum(1 for r in rows if r.get('image_path') and Path(r['image_path']).exists())
    pose_ok = sum(1 for r in rows if _truthy(r.get('pose_valid')))
    rtk_ok = sum(1 for r in rows if _truthy(r.get('rtk_valid')))
    camera_ok = sum(1 for r in rows if str(r.get('camera_error_code', '0')).strip() in {'', '0'})

    lines = [
        '# AGT RViz Patrol Demo Report', '',
        f"- mission_id: `{manifest.get('mission_id', '')}`",
        f"- map_id: `{manifest.get('map_id', '')}`",
        f"- capture records: **{len(rows)}**",
        f"- inspection points with captures: **{len(by_point)}**",
        f"- image files present: **{images_ok}/{len(rows)}**",
        f"- map pose valid: **{pose_ok}/{len(rows)}**",
        f"- RTK valid: **{rtk_ok}/{len(rows)}**",
        f"- camera result OK: **{camera_ok}/{len(rows)}**",
        '', '## Per point', '',
        '| Point | Views | Images | Pose | RTK | Camera OK |',
        '| --- | ---: | ---: | ---: | ---: | ---: |',
    ]

    for point_id in sorted(by_point):
        rs = by_point[point_id]
        lines.append(
            f"| {point_id} | {len(rs)} | "
            f"{sum(1 for r in rs if r.get('image_path') and Path(r['image_path']).exists())} | "
            f"{sum(1 for r in rs if _truthy(r.get('pose_valid')))} | "
            f"{sum(1 for r in rs if _truthy(r.get('rtk_valid')))} | "
            f"{sum(1 for r in rs if str(r.get('camera_error_code', '0')).strip() in {'', '0'})} |"
        )

    lines += ['', '## Capture details', '']
    for row in rows:
        lines += [
            f"### {row.get('point_id', '')} / {row.get('view_tag', '')}",
            f"- image: `{row.get('image_path', '')}`",
            f"- map pose valid: `{row.get('pose_valid', '')}`; x={row.get('x', '')}, y={row.get('y', '')}",
            f"- RTK valid: `{row.get('rtk_valid', '')}`; lat={row.get('latitude', '')}, lon={row.get('longitude', '')}",
            f"- gimbal actual: heading={row.get('gimbal_heading', '')}, roll={row.get('gimbal_roll', '')}, pitch={row.get('gimbal_pitch', '')}",
            f"- camera_error_code: `{row.get('camera_error_code', '')}`",
            '',
        ]
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Generate a Markdown summary for one AGT inspection run.')
    parser.add_argument('mission_directory')
    parser.add_argument('--output', default='demo_report.md')
    args = parser.parse_args(argv)
    directory = Path(args.mission_directory).expanduser().resolve()
    report = build_report(directory)
    output = directory / args.output
    output.write_text(report, encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
