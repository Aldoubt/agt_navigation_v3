from __future__ import annotations

import argparse
import csv
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Convert robotics-laboratory/fast-lio2 PGO patches + poses.txt into benchmark cases.csv.')
    parser.add_argument('map_dir', type=Path,
                        help='directory produced by /pgo/save_maps with save_patches=true')
    parser.add_argument('--stride', type=int, default=5,
                        help='keep every Nth PGO keyframe for a fast first sweep')
    parser.add_argument('--output', type=Path, default=None,
                        help='output cases.csv; default <map_dir>/cases.csv')
    args = parser.parse_args(argv)

    map_dir = args.map_dir.expanduser().resolve()
    poses = map_dir / 'poses.txt'
    patches = map_dir / 'patches'
    output = args.output.expanduser().resolve() if args.output else map_dir / 'cases.csv'
    if args.stride < 1:
        parser.error('--stride must be >= 1')
    if not poses.is_file():
        raise SystemExit(f'poses.txt not found: {poses}')
    if not patches.is_dir():
        raise SystemExit(f'patches directory not found: {patches}')

    rows = []
    for index, raw in enumerate(poses.read_text(encoding='utf-8').splitlines()):
        line = raw.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 8:
            raise SystemExit(f'invalid poses.txt line {index + 1}: expected 8 fields, got {len(parts)}')
        patch_name, tx, ty, tz, qw, qx, qy, qz = parts
        if index % args.stride != 0:
            continue
        patch = patches / patch_name
        if not patch.is_file():
            raise SystemExit(f'patch listed by poses.txt does not exist: {patch}')
        rows.append([
            f'K{index:05d}', '', f'patches/{patch_name}', '',
            float(tx), float(ty), float(tz),
            float(qx), float(qy), float(qz), float(qw),
        ])

    if not rows:
        raise SystemExit('no benchmark cases selected')
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            'case_id', 'stamp_sec', 'pcd', 'points',
            'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
        ])
        writer.writerows(rows)
    print(f'PGO CASE IMPORT PASS: {len(rows)} cases -> {output}')
    print('NOTE: these are closed-set cases because the same patches contributed to the map.')


if __name__ == '__main__':
    main()
