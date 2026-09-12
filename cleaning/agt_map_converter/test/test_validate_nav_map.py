from pathlib import Path
import yaml

from agt_map_converter.validate_nav_map import acceptance_result, validate


def write_pgm(path: Path, width=2, height=2, payload=b'\xff\x00\xff\x00'):
    path.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + payload)


def write_acceptance_evidence(path: Path, *, status: str, conflicts: int):
    regions = [] if conflicts == 0 else [{
        'id': 1,
        'cell_count': conflicts,
        'grid_bbox': {'min_x': 0, 'min_y': 0, 'max_x': 0, 'max_y': 0},
        'map_bbox_m': {'min_x': 0.0, 'min_y': 0.0, 'max_x': 0.1, 'max_y': 0.1},
        'centroid_m': {'x': 0.05, 'y': 0.05},
        'approximate_area_m2': conflicts * 0.01,
        'cells': [],
    }]
    (path / 'converter_metadata.yaml').write_text(yaml.safe_dump({
        'source_pcd': '/frozen/map.pcd',
        'trajectory_poses': '/frozen/poses.txt',
        'trajectory_pose_count': 1,
        'trajectory_qa_status': status,
        'trajectory_conflict_cells_before_carve': conflicts,
        'trajectory_cleared_cells': conflicts,
        'trajectory_conflict_ratio_of_swept_cells': conflicts / 10.0,
        'trajectory_conflict_region_count': len(regions),
        'trajectory_conflict_evidence': 'trajectory_conflicts.yaml',
        'trajectory_conflict_debug_pgm': 'trajectory_conflicts.pgm',
    }), encoding='utf-8')
    (path / 'trajectory_conflicts.yaml').write_text(yaml.safe_dump({
        'trajectory_conflict_cells': conflicts,
        'trajectory_swept_cells': 10,
        'trajectory_conflict_ratio_of_swept_cells': conflicts / 10.0,
        'region_count': len(regions),
        'regions': regions,
    }), encoding='utf-8')
    write_pgm(path / 'trajectory_conflicts.pgm')


def test_validate_nav_map_passes_minimal_output(tmp_path: Path):
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump({
        'image': 'map.pgm',
        'resolution': 0.1,
        'origin': [0.0, 0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }), encoding='utf-8')
    for name in ('map.pgm', 'elevation.pgm', 'slope.pgm', 'obstacle.pgm'):
        write_pgm(tmp_path / name)
    assert validate(tmp_path) == []


def test_validate_nav_map_detects_dimension_mismatch(tmp_path: Path):
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump({
        'image': 'map.pgm',
        'resolution': 0.1,
        'origin': [0.0, 0.0, 0.0],
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }), encoding='utf-8')
    write_pgm(tmp_path / 'map.pgm')
    write_pgm(tmp_path / 'elevation.pgm')
    write_pgm(tmp_path / 'slope.pgm')
    write_pgm(tmp_path / 'obstacle.pgm', width=1, height=1, payload=b'\x00')
    errors = validate(tmp_path)
    assert any('dimensions' in error for error in errors)


def test_validate_rejects_unknown_cells_made_free_by_threshold(tmp_path: Path):
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump({
        'image': 'map.pgm',
        'resolution': 0.1,
        'origin': [0.0, 0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }), encoding='utf-8')
    write_pgm(tmp_path / 'map.pgm', width=4, height=1, payload=bytes((205, 0, 254, 205)))
    for name in ('elevation.pgm', 'slope.pgm', 'obstacle.pgm'):
        write_pgm(tmp_path / name, width=4, height=1, payload=b'\xff\x00\xff\x00')

    assert 'map.yaml free_thresh makes PGM value 205 free instead of unknown' in validate(tmp_path)


def test_acceptance_requires_trajectory_qa(tmp_path: Path):
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump({
        'image': 'map.pgm', 'resolution': 0.1, 'origin': [0.0, 0.0, 0.0],
        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }), encoding='utf-8')
    for name in ('map.pgm', 'elevation.pgm', 'slope.pgm', 'obstacle.pgm'):
        write_pgm(tmp_path / name)
    assert acceptance_result(tmp_path)[0] == 'FAIL'


def test_acceptance_reports_review_for_trajectory_conflicts(tmp_path: Path):
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump({
        'image': 'map.pgm', 'resolution': 0.1, 'origin': [0.0, 0.0, 0.0],
        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }), encoding='utf-8')
    for name in ('map.pgm', 'elevation.pgm', 'slope.pgm', 'obstacle.pgm'):
        write_pgm(tmp_path / name)
    write_acceptance_evidence(tmp_path, status='REVIEW', conflicts=1)
    assert acceptance_result(tmp_path) == ('REVIEW', [])


def test_acceptance_reports_pass_for_zero_conflicts(tmp_path: Path):
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump({
        'image': 'map.pgm', 'resolution': 0.1, 'origin': [0.0, 0.0, 0.0],
        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }), encoding='utf-8')
    for name in ('map.pgm', 'elevation.pgm', 'slope.pgm', 'obstacle.pgm'):
        write_pgm(tmp_path / name)
    write_acceptance_evidence(tmp_path, status='PASS', conflicts=0)
    assert acceptance_result(tmp_path) == ('PASS', [])
