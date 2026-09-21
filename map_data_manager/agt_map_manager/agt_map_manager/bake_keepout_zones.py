"""Bake Map Studio keep-out polygons into a Nav2 occupancy raster."""

from __future__ import annotations

import argparse
import math
import shutil
from pathlib import Path

import yaml


def _token(data: bytes, offset: int) -> tuple[bytes, int]:
    size = len(data)
    while offset < size:
        if data[offset] == ord('#'):
            end = data.find(b'\n', offset)
            offset = size if end < 0 else end + 1
        elif chr(data[offset]).isspace():
            offset += 1
        else:
            break
    start = offset
    while offset < size and not chr(data[offset]).isspace() and data[offset] != ord('#'):
        offset += 1
    if start == offset:
        raise ValueError('invalid PGM header')
    return data[start:offset], offset


def _read_pgm(path: Path) -> tuple[int, int, bytearray]:
    data = path.read_bytes()
    magic, offset = _token(data, 0)
    width_token, offset = _token(data, offset)
    height_token, offset = _token(data, offset)
    max_token, offset = _token(data, offset)
    if magic != b'P5' or max_token != b'255':
        raise ValueError('keep-out baking requires an 8-bit binary P5 PGM')
    width, height = int(width_token), int(height_token)
    if offset >= len(data) or not chr(data[offset]).isspace():
        raise ValueError('PGM header is not terminated')
    if data[offset:offset + 2] == b'\r\n':
        offset += 2
    else:
        offset += 1
    pixels = bytearray(data[offset:])
    if width <= 0 or height <= 0 or len(pixels) != width * height:
        raise ValueError('PGM dimensions do not match its raster')
    return width, height, pixels


def _on_segment(x: float, y: float, a: tuple[float, float],
                b: tuple[float, float], tolerance: float = 1e-9) -> bool:
    cross = (x - a[0]) * (b[1] - a[1]) - (y - a[1]) * (b[0] - a[0])
    if abs(cross) > tolerance:
        return False
    return (min(a[0], b[0]) - tolerance <= x <= max(a[0], b[0]) + tolerance
            and min(a[1], b[1]) - tolerance <= y <= max(a[1], b[1]) + tolerance)


def _inside(x: float, y: float, polygon: list[tuple[float, float]]) -> bool:
    contained = False
    previous = polygon[-1]
    for current in polygon:
        if _on_segment(x, y, previous, current):
            return True
        if ((current[1] > y) != (previous[1] > y)):
            intersection = ((previous[0] - current[0]) * (y - current[1])
                            / (previous[1] - current[1]) + current[0])
            if x < intersection:
                contained = not contained
        previous = current
    return contained


def bake_keepout_zones(map_yaml: Path, zones_yaml: Path, output_dir: Path,
                       inflate_cells: int = 1) -> dict:
    map_yaml = map_yaml.expanduser().resolve()
    zones_yaml = zones_yaml.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    if output_dir.exists():
        raise FileExistsError(f'output directory already exists: {output_dir}')
    if inflate_cells < 0:
        raise ValueError('inflate_cells must be non-negative')

    map_data = yaml.safe_load(map_yaml.read_text(encoding='utf-8')) or {}
    zones_data = yaml.safe_load(zones_yaml.read_text(encoding='utf-8')) or {}
    image = Path(str(map_data.get('image', '')))
    image_path = image if image.is_absolute() else map_yaml.parent / image
    resolution = float(map_data['resolution'])
    origin = map_data['origin']
    if resolution <= 0 or not isinstance(origin, list) or len(origin) < 2:
        raise ValueError('map YAML has invalid resolution/origin')

    width, height, pixels = _read_pgm(image_path.resolve())
    selected: set[tuple[int, int]] = set()
    zones = zones_data.get('zones') or []
    for zone in zones:
        if zone.get('type') != 'keepout':
            continue
        polygon = [tuple(map(float, point)) for point in zone.get('polygon_m') or []]
        if len(polygon) < 3 or any(len(point) != 2 for point in polygon):
            raise ValueError('each keepout zone requires at least three [x, y] points')
        min_x = max(0, math.floor((min(point[0] for point in polygon) - origin[0]) / resolution))
        max_x = min(
            width - 1,
            math.floor((max(point[0] for point in polygon) - origin[0]) / resolution))
        min_y = max(0, math.floor((min(point[1] for point in polygon) - origin[1]) / resolution))
        max_y = min(
            height - 1,
            math.floor((max(point[1] for point in polygon) - origin[1]) / resolution))
        for grid_y in range(min_y, max_y + 1):
            world_y = float(origin[1]) + (grid_y + 0.5) * resolution
            for grid_x in range(min_x, max_x + 1):
                world_x = float(origin[0]) + (grid_x + 0.5) * resolution
                if _inside(world_x, world_y, polygon):
                    selected.add((grid_x, grid_y))

    baked = set(selected)
    for grid_x, grid_y in selected:
        for dy in range(-inflate_cells, inflate_cells + 1):
            for dx in range(-inflate_cells, inflate_cells + 1):
                nx, ny = grid_x + dx, grid_y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    baked.add((nx, ny))

    changed = 0
    for grid_x, grid_y in baked:
        pgm_row = height - 1 - grid_y
        index = pgm_row * width + grid_x
        if pixels[index] != 0:
            pixels[index] = 0
            changed += 1

    output_dir.mkdir(parents=True)
    output_pgm = output_dir / 'map.pgm'
    output_pgm.write_bytes(f'P5\n{width} {height}\n255\n'.encode('ascii') + pixels)
    map_data['image'] = 'map.pgm'
    (output_dir / 'map.yaml').write_text(
        yaml.safe_dump(map_data, sort_keys=False), encoding='utf-8')
    shutil.copy2(zones_yaml, output_dir / 'keepout_zones.yaml')
    for name in ('map_refinement.yaml', 'review_status.yaml'):
        source = map_yaml.parent / name
        if source.is_file():
            shutil.copy2(source, output_dir / name)

    report = {
        'schema_version': 1,
        'source_map_yaml': str(map_yaml),
        'source_keepout_zones': str(zones_yaml),
        'zone_count': sum(1 for zone in zones if zone.get('type') == 'keepout'),
        'polygon_cells': len(selected),
        'baked_cells': len(baked),
        'changed_to_occupied': changed,
        'inflate_cells': inflate_cells,
    }
    (output_dir / 'keepout_bake.yaml').write_text(
        yaml.safe_dump(report, sort_keys=False), encoding='utf-8')
    return report


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='Bake Map Studio keep-out polygons into an occupied Nav2 PGM.')
    parser.add_argument('--map', required=True, help='Confirmed map.yaml')
    parser.add_argument('--zones', required=True, help='Map Studio keepout_zones.yaml')
    parser.add_argument('--output', required=True, help='New navigation-map directory')
    parser.add_argument('--inflate-cells', type=int, default=1)
    args = parser.parse_args(argv)
    try:
        report = bake_keepout_zones(
            Path(args.map), Path(args.zones), Path(args.output), args.inflate_cells)
    except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))
    print(
        f"Baked {report['zone_count']} keep-out zone(s): "
        f"{report['changed_to_occupied']} cells changed to occupied in {args.output}")


if __name__ == '__main__':
    main()
