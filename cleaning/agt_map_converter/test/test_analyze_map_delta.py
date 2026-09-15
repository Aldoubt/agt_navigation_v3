from pathlib import Path

import numpy as np
import yaml

from agt_map_converter.analyze_map_delta import analyze_map_delta
from agt_map_converter.pcd_to_nav_map import write_pgm


def _write_ascii_pcd(path: Path, points):
    lines = [
        '# .PCD v0.7',
        'VERSION 0.7',
        'FIELDS x y z',
        'SIZE 4 4 4',
        'TYPE F F F',
        'COUNT 1 1 1',
        f'WIDTH {len(points)}',
        'HEIGHT 1',
        f'POINTS {len(points)}',
        'DATA ascii',
    ]
    lines.extend(f'{x} {y} {z}' for x, y, z in points)
    path.write_text('\n'.join(lines) + '\n', encoding='ascii')


def _make_map(directory: Path, grid: np.ndarray, metadata=None):
    directory.mkdir(parents=True)
    for name in ('map.pgm', 'obstacle.pgm'):
        write_pgm(directory / name, np.flipud(grid))
    debug = np.full(grid.shape, 254, dtype=np.uint8)
    write_pgm(directory / 'elevation.pgm', np.flipud(debug))
    write_pgm(directory / 'slope.pgm', np.flipud(debug))
    (directory / 'map.yaml').write_text(yaml.safe_dump({
        'image': 'map.pgm',
        'mode': 'trinary',
        'resolution': 1.0,
        'origin': [0.0, 0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }, sort_keys=False), encoding='utf-8')
    (directory / 'converter_metadata.yaml').write_text(
        yaml.safe_dump(metadata or {}, sort_keys=False), encoding='utf-8')


def test_analyze_map_delta_exports_cell_region_and_pcd_evidence(tmp_path: Path):
    candidate = np.full((6, 6), 254, dtype=np.uint8)
    candidate[0, 0] = 0  # keep a structural occupied cell
    candidate[1, 2] = 0  # selected candidate occupied -> reference free cell
    reference = candidate.copy()
    reference[1, 2] = 254

    metadata = {
        'min_points': 2,
        'max_step': 0.22,
        'max_slope_deg': 20.0,
        'trajectory_front_m': 0.4,
        'trajectory_rear_m': 0.4,
        'trajectory_half_width_m': 0.4,
    }
    candidate_dir = tmp_path / 'candidate'
    reference_dir = tmp_path / 'reference'
    _make_map(candidate_dir, candidate, metadata)
    _make_map(reference_dir, reference, metadata)

    pcd = tmp_path / 'source.pcd'
    _write_ascii_pcd(pcd, [
        (1.5, 1.5, 0.0),  # neighboring ground evidence
        (2.5, 1.5, 0.0),  # ground in selected cell
        (2.5, 1.5, 3.0),  # high return in selected cell
        (2.6, 1.5, 3.2),
    ])
    poses = tmp_path / 'poses.txt'
    poses.write_text(
        '0.pcd 1.5 1.5 0 1 0 0 0\n',
        encoding='utf-8')

    output = tmp_path / 'analysis'
    result = analyze_map_delta(
        reference_dir, candidate_dir, pcd, poses, output)

    assert result['selected_transition']['cell_count'] == 1
    assert result['source_pcd_evidence']['region_count'] == 1
    assert result['transition_counts'][
        'candidate_occupied_to_reference_free'] == 1
    assert (output / 'delta_cells.csv').is_file()
    assert (output / 'delta_cells.yaml').is_file()
    assert (output / 'delta_regions.csv').is_file()
    assert (output / 'delta_regions.yaml').is_file()
    assert (output / 'delta_mask.pgm').is_file()
    assert (output / 'delta_vs_trajectory.pgm').is_file()

    cells = yaml.safe_load(
        (output / 'delta_cells.yaml').read_text(encoding='utf-8'))['cells']
    assert len(cells) == 1
    assert cells[0]['point_count'] == 3
    assert cells[0]['height_above_ground_p95_m'] > 2.0
    assert cells[0]['near_ground_fraction'] > 0.0
    trigger = result['converter_trigger_attribution']
    assert trigger['selected_cells'] == 1
    assert trigger['neither_trigger_cells'] == 0
    assert trigger['attributed_cells'] == 1
    assert trigger['consistency_status'] == 'PASS'
    assert (output / 'delta_trigger_classes.pgm').is_file()


def test_analyze_map_delta_rejects_geometry_mismatch(tmp_path: Path):
    candidate = np.full((4, 4), 254, dtype=np.uint8)
    candidate[0, 0] = 0
    reference = candidate.copy()
    reference[1, 1] = 0
    candidate_dir = tmp_path / 'candidate'
    reference_dir = tmp_path / 'reference'
    _make_map(candidate_dir, candidate)
    _make_map(reference_dir, reference)

    nav = yaml.safe_load((reference_dir / 'map.yaml').read_text())
    nav['resolution'] = 0.5
    (reference_dir / 'map.yaml').write_text(
        yaml.safe_dump(nav, sort_keys=False), encoding='utf-8')

    pcd = tmp_path / 'source.pcd'
    _write_ascii_pcd(pcd, [(1.5, 1.5, 0.0)])
    poses = tmp_path / 'poses.txt'
    poses.write_text('0.pcd 1.5 1.5 0 1 0 0 0\n', encoding='utf-8')

    import pytest
    with pytest.raises(ValueError, match='resolution mismatch'):
        analyze_map_delta(
            reference_dir, candidate_dir, pcd, poses, tmp_path / 'analysis')


def test_analyze_map_delta_attributes_slope_only_trigger(tmp_path: Path):
    candidate = np.full((5, 5), 254, dtype=np.uint8)
    candidate[0, 0] = 0
    candidate[2, 2] = 0
    reference = candidate.copy()
    reference[2, 2] = 254

    metadata = {
        'min_points': 2,
        'max_step': 0.22,
        'max_slope_deg': 20.0,
        'trajectory_front_m': 0.1,
        'trajectory_rear_m': 0.1,
        'trajectory_half_width_m': 0.1,
    }
    candidate_dir = tmp_path / 'candidate'
    reference_dir = tmp_path / 'reference'
    _make_map(candidate_dir, candidate, metadata)
    _make_map(reference_dir, reference, metadata)

    # Every grid cell has a very small vertical span (0.05 m), while elevation
    # rises 0.5 m per x-cell. The current min-z gradient therefore produces a
    # slope-only obstacle trigger at the selected cell.
    points = []
    for gy in range(5):
        for gx in range(5):
            z0 = 0.5 * gx
            points.append((gx + 0.5, gy + 0.5, z0))
            points.append((gx + 0.5, gy + 0.5, z0 + 0.05))
    pcd = tmp_path / 'source.pcd'
    _write_ascii_pcd(pcd, points)

    poses = tmp_path / 'poses.txt'
    poses.write_text(
        '0.pcd 0.5 0.5 0 1 0 0 0\n',
        encoding='utf-8')

    output = tmp_path / 'analysis'
    result = analyze_map_delta(
        reference_dir, candidate_dir, pcd, poses, output)

    trigger = result['converter_trigger_attribution']
    assert trigger['span_only_cells'] == 0
    assert trigger['slope_only_cells'] == 1
    assert trigger['both_trigger_cells'] == 0
    assert trigger['neither_trigger_cells'] == 0

    cells = yaml.safe_load(
        (output / 'delta_cells.yaml').read_text(encoding='utf-8'))['cells']
    assert cells[0]['trigger_class'] == 'slope_only'
    assert cells[0]['converter_vertical_span_m'] < 0.22
    assert cells[0]['converter_slope_deg'] > 20.0
