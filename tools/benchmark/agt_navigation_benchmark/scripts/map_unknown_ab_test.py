#!/usr/bin/env python3
"""Create original and unknown-as-free PGM/YAML assets from a recorded /map."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agt_nav_benchmark.bag_reader import RosbagReader


def _yaw_from_quaternion(q):
    return float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def _pgm_values(grid, unknown_as_free: bool):
    values = np.asarray(grid.data, dtype=np.int16).reshape((int(grid.info.height), int(grid.info.width)))
    pgm = np.full(values.shape, 205, dtype=np.uint8)
    pgm[values == 0] = 254
    pgm[values >= 100] = 0
    if unknown_as_free:
        pgm[values < 0] = 254
    return np.flipud(pgm)


def _write_yaml(path: Path, image_name: str, grid):
    origin = grid.info.origin
    content = (
        f'image: {image_name}\n'
        f'resolution: {float(grid.info.resolution):.9f}\n'
        f'origin: [{float(origin.position.x):.9f}, {float(origin.position.y):.9f}, '
        f'{_yaw_from_quaternion(origin.orientation):.9f}]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
    )
    path.write_text(content, encoding='utf-8')


def _ratios(grid, unknown_as_free=False):
    values = np.asarray(grid.data, dtype=np.int16)
    unknown = values < 0
    free = values == 0
    if unknown_as_free:
        free = free | unknown
    return {
        'unknown_ratio': float(np.mean(unknown)) if not unknown_as_free else 0.0,
        'free_ratio': float(np.mean(free)),
        'occupied_ratio': float(np.mean(values >= 100)),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description='Run an offline original-vs-unknown-free map A/B preparation.')
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--topic', default='/map')
    parser.add_argument('--output-dir', type=Path, default=Path('reports/map_ab_test'))
    args = parser.parse_args(argv)

    reader = RosbagReader(str(args.bag))
    records = list(reader.records([args.topic]))
    if not records:
        raise RuntimeError(f'No messages found on {args.topic}')
    record = records[-1]
    grid = record.message
    args.output_dir.mkdir(parents=True, exist_ok=True)
    original_pgm = args.output_dir / 'original_map.pgm'
    unknown_free_pgm = args.output_dir / 'unknown_free_map.pgm'
    Image.fromarray(_pgm_values(grid, False), mode='L').save(original_pgm)
    Image.fromarray(_pgm_values(grid, True), mode='L').save(unknown_free_pgm)
    _write_yaml(args.output_dir / 'original_map.yaml', original_pgm.name, grid)
    _write_yaml(args.output_dir / 'unknown_free_map.yaml', unknown_free_pgm.name, grid)
    original = _ratios(grid, False)
    unknown_free = _ratios(grid, True)
    (args.output_dir / 'comparison.md').write_text(
        '# Unknown-Free Map A/B Test\n\n'
        'This is an offline asset preparation step. It does not rerun Nav2 planner or modify any navigation configuration.\n\n'
        f'- source bag: `{args.bag}`\n- source topic: `{args.topic}`\n'
        f'- map size: `{int(grid.info.width)} x {int(grid.info.height)}`\n\n'
        '## Ratios\n\n'
        '| version | unknown ratio | free ratio | occupied ratio |\n'
        '|---|---:|---:|---:|\n'
        f"| original | {original['unknown_ratio'] * 100.0:.2f}% | {original['free_ratio'] * 100.0:.2f}% | {original['occupied_ratio'] * 100.0:.2f}% |\n"
        f"| unknown-free | {unknown_free['unknown_ratio'] * 100.0:.2f}% | {unknown_free['free_ratio'] * 100.0:.2f}% | {unknown_free['occupied_ratio'] * 100.0:.2f}% |\n\n"
        'Use these generated YAML/PGM pairs as controlled planner inputs in a separate experiment.\n',
        encoding='utf-8',
    )
    print(f'A/B assets written to {args.output_dir}')


if __name__ == '__main__':
    main()
