"""Apply reproducible manual occupancy edits to a generated Nav2 map.

Patch coordinates are expressed in map-frame meters, so an operator can mark a
polygon in RViz and keep the edit as source-controlled YAML instead of painting
an opaque binary PGM by hand.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import numpy as np
import yaml

from .pcd_to_nav_map import write_pgm
from .validate_nav_map import read_pgm_header, validate


VALUES = {'free': 254, 'occupied': 0, 'unknown': 205}


def _read_pgm(path: Path) -> np.ndarray:
    width, height, payload = read_pgm_header(path)
    return np.frombuffer(payload, dtype=np.uint8).reshape((height, width)).copy()


def _inside_polygon(x: float, y: float, polygon) -> bool:
    inside = False
    j = len(polygon) - 1
    for i, (xi, yi) in enumerate(polygon):
        xj, yj = polygon[j]
        crosses = ((yi > y) != (yj > y))
        if crosses:
            x_at_y = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < x_at_y:
                inside = not inside
        j = i
    return inside


def _polygon_mask(shape, origin, resolution, polygon):
    height, width = shape
    min_x, min_y = origin[0], origin[1]
    xs = [float(p[0]) for p in polygon]
    ys = [float(p[1]) for p in polygon]
    gx0 = max(0, int(np.floor((min(xs) - min_x) / resolution)) - 1)
    gx1 = min(width - 1, int(np.ceil((max(xs) - min_x) / resolution)) + 1)
    gy0 = max(0, int(np.floor((min(ys) - min_y) / resolution)) - 1)
    gy1 = min(height - 1, int(np.ceil((max(ys) - min_y) / resolution)) + 1)

    mask = np.zeros(shape, dtype=bool)
    for gy in range(gy0, gy1 + 1):
        wy = min_y + (gy + 0.5) * resolution
        disk_row = height - 1 - gy
        for gx in range(gx0, gx1 + 1):
            wx = min_x + (gx + 0.5) * resolution
            if _inside_polygon(wx, wy, polygon):
                mask[disk_row, gx] = True
    return mask


def apply_patch(directory: Path, patch_file: Path, output: Path | None = None):
    directory = directory.expanduser().resolve()
    patch_file = patch_file.expanduser().resolve()
    if output is None:
        target = directory
    else:
        target = output.expanduser().resolve()
        if target.exists():
            raise FileExistsError(f'output already exists: {target}')
        shutil.copytree(directory, target)

    nav = yaml.safe_load((target / 'map.yaml').read_text(encoding='utf-8')) or {}
    resolution = float(nav['resolution'])
    origin = nav['origin']
    map_image = _read_pgm(target / nav.get('image', 'map.pgm'))
    obstacle_path = target / 'obstacle.pgm'
    obstacle_image = _read_pgm(obstacle_path) if obstacle_path.is_file() else None

    spec = yaml.safe_load(patch_file.read_text(encoding='utf-8')) or {}
    edits = spec.get('edits') or []
    if not edits:
        raise ValueError('patch contains no edits')

    history = []
    for index, edit in enumerate(edits):
        mode = str(edit.get('mode', '')).lower()
        if mode not in VALUES:
            raise ValueError(f'edit {index}: mode must be one of {sorted(VALUES)}')
        polygon = edit.get('polygon_m')
        if not isinstance(polygon, list) or len(polygon) < 3:
            raise ValueError(f'edit {index}: polygon_m requires at least three [x,y] points')
        polygon = [(float(p[0]), float(p[1])) for p in polygon]
        mask = _polygon_mask(map_image.shape, origin, resolution, polygon)
        changed = int(mask.sum())
        if changed == 0:
            raise ValueError(f'edit {index}: polygon does not cover any map cells')
        map_image[mask] = VALUES[mode]
        if obstacle_image is not None:
            obstacle_image[mask] = VALUES[mode]
        history.append({
            'mode': mode,
            'polygon_m': [[x, y] for x, y in polygon],
            'cells': changed,
            'note': str(edit.get('note', '')),
        })

    write_pgm(target / nav.get('image', 'map.pgm'), map_image)
    if obstacle_image is not None:
        write_pgm(obstacle_path, obstacle_image)

    metadata_path = target / 'converter_metadata.yaml'
    metadata = {}
    if metadata_path.is_file():
        metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
    metadata.setdefault('manual_patch_history', []).append({
        'patch_file': str(patch_file),
        'edits': history,
    })
    metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')

    errors = validate(target)
    if errors:
        raise RuntimeError('patched map failed validation: ' + '; '.join(errors))
    return target, history


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Patch AGT map.pgm using reproducible map-frame polygons.')
    parser.add_argument('map_directory')
    parser.add_argument('patch_yaml')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--output', '-o', help='write a new patched map directory')
    group.add_argument('--in-place', action='store_true', help='modify the supplied directory')
    args = parser.parse_args(argv)

    directory = Path(args.map_directory)
    patch_file = Path(args.patch_yaml)
    output = None if args.in_place else Path(args.output)
    try:
        target, history = apply_patch(directory, patch_file, output)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.error(str(exc))
    print(f'Patched Nav2 map: {target}')
    for item in history:
        print(f" - {item['mode']}: {item['cells']} cells {item['note']}")


if __name__ == '__main__':
    main()
