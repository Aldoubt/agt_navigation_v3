from __future__ import annotations

import argparse
from pathlib import Path
import sys
import yaml


def read_pgm_header(path: Path):
    with path.open('rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError(f'{path.name}: expected binary PGM P5, got {magic!r}')
        line = f.readline().strip()
        while line.startswith(b'#'):
            line = f.readline().strip()
        width, height = [int(v) for v in line.split()]
        maxval = int(f.readline().strip())
        if maxval != 255:
            raise ValueError(f'{path.name}: expected maxval 255, got {maxval}')
        payload = f.read()
    if len(payload) != width * height:
        raise ValueError(
            f'{path.name}: payload size {len(payload)} does not match {width}x{height}')
    return width, height, payload


def validate(directory: Path) -> list[str]:
    errors: list[str] = []
    required = ['map.yaml', 'map.pgm', 'elevation.pgm', 'slope.pgm', 'obstacle.pgm']
    for name in required:
        if not (directory / name).is_file():
            errors.append(f'missing {name}')
    if errors:
        return errors

    try:
        nav = yaml.safe_load((directory / 'map.yaml').read_text(encoding='utf-8')) or {}
        if nav.get('image') != 'map.pgm':
            errors.append('map.yaml image must be map.pgm')
        resolution = float(nav.get('resolution', 0.0))
        if resolution <= 0.0:
            errors.append('map.yaml resolution must be > 0')
        origin = nav.get('origin')
        if not isinstance(origin, list) or len(origin) != 3:
            errors.append('map.yaml origin must be [x, y, yaw]')
        free_thresh = float(nav.get('free_thresh', -1.0))
        occupied_thresh = float(nav.get('occupied_thresh', -1.0))
        if not (0.0 <= free_thresh < occupied_thresh <= 1.0):
            errors.append('map.yaml thresholds must satisfy 0 <= free < occupied <= 1')
    except Exception as exc:
        errors.append(f'map.yaml parse failed: {exc}')

    dims = None
    payloads = {}
    for name in required[1:]:
        try:
            w, h, payload = read_pgm_header(directory / name)
            if dims is None:
                dims = (w, h)
            elif dims != (w, h):
                errors.append(f'{name}: dimensions {(w, h)} do not match {dims}')
            payloads[name] = payload
        except Exception as exc:
            errors.append(str(exc))

    map_payload = payloads.get('map.pgm', b'')
    if map_payload:
        occupied = sum(1 for v in map_payload if v <= 64)
        free = sum(1 for v in map_payload if v >= 250)
        unknown = len(map_payload) - occupied - free
        if occupied == 0:
            errors.append('map.pgm contains no occupied cells')
        if free == 0:
            errors.append('map.pgm contains no free cells')
        total = len(map_payload)
        print(
            f'map cells={total} free={free} ({free/total:.1%}) '
            f'occupied={occupied} ({occupied/total:.1%}) unknown={unknown} ({unknown/total:.1%})')

    metadata = directory / 'converter_metadata.yaml'
    if metadata.exists():
        try:
            meta = yaml.safe_load(metadata.read_text(encoding='utf-8')) or {}
            print(f"source_pcd={meta.get('source_pcd', '')}")
        except Exception as exc:
            errors.append(f'converter_metadata.yaml parse failed: {exc}')
    else:
        print('warning: converter_metadata.yaml not present')

    return errors


def main(argv=None):
    parser = argparse.ArgumentParser(description='Validate AGT Nav2 map-converter output.')
    parser.add_argument('directory', help='Directory containing map.yaml/map.pgm terrain layers')
    args = parser.parse_args(argv)
    directory = Path(args.directory).expanduser().resolve()
    errors = validate(directory)
    if errors:
        print('MAP VALIDATION FAILED', file=sys.stderr)
        for error in errors:
            print(f' - {error}', file=sys.stderr)
        raise SystemExit(2)
    print('MAP VALIDATION PASS')


if __name__ == '__main__':
    main()
