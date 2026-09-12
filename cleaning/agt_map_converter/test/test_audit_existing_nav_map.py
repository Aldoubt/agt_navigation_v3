from pathlib import Path

import numpy as np
import pytest
import yaml

from agt_map_converter.audit_existing_nav_map import audit_existing_navigation_map
from agt_map_converter.pcd_to_nav_map import write_pgm


def _make_package(tmp_path: Path, *, occupied_in_corridor: bool, unknown_in_corridor: bool):
    package = tmp_path / 'package'
    navigation = package / 'navigation'
    localization = package / 'localization'
    navigation.mkdir(parents=True)
    localization.mkdir()
    source_pcd = tmp_path / 'formal.pcd'
    source_pcd.write_bytes(b'fixed source pcd')
    localization_pcd = localization / 'global_map.pcd'
    localization_pcd.write_bytes(b'fixed source pcd')
    import hashlib
    digest = hashlib.sha256(b'fixed source pcd').hexdigest()
    poses = tmp_path / 'formal_poses.txt'
    poses.write_text('0.pcd 0.25 0.25 0 1 0 0 0\n', encoding='utf-8')
    grid = np.full((5, 5), 254, dtype=np.uint8)
    grid[0, 0] = 0  # Structural occupied cell outside the swept corridor.
    if occupied_in_corridor:
        grid[2, 2] = 0
    if unknown_in_corridor:
        grid[2, 3] = 205
    for name in ('map.pgm', 'elevation.pgm', 'slope.pgm', 'obstacle.pgm'):
        write_pgm(navigation / name, np.flipud(grid))
    (navigation / 'map.yaml').write_text(yaml.safe_dump({
        'image': 'map.pgm', 'mode': 'trinary', 'resolution': 0.1,
        'origin': [0.0, 0.0, 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }, sort_keys=False), encoding='utf-8')
    (navigation / 'converter_metadata.yaml').write_text(yaml.safe_dump({
        'source_pcd': str(source_pcd), 'trajectory_poses': str(poses),
        'trajectory_pose_count': 1, 'trajectory_front_m': 0.2,
        'trajectory_rear_m': 0.2, 'trajectory_half_width_m': 0.15,
    }, sort_keys=False), encoding='utf-8')
    (package / 'metadata.yaml').write_text(yaml.safe_dump({
        'map_id': 'test', 'map_version': 'v001', 'frame_id': 'map',
        'assets': {'localization_map': {
            'path': 'localization/global_map.pcd', 'sha256': digest,
        }},
    }, sort_keys=False), encoding='utf-8')
    return navigation, poses


def test_existing_map_audit_reports_only_occupied_conflicts(tmp_path: Path):
    navigation, poses = _make_package(
        tmp_path, occupied_in_corridor=True, unknown_in_corridor=True)
    result = audit_existing_navigation_map(navigation, poses, tmp_path / 'audit')
    summary = result['summary']
    assert result['acceptance_status'] == 'REVIEW'
    assert summary['trajectory_occupied_cells'] == 1
    assert summary['trajectory_unknown_cells'] == 1
    assert summary['occupied_conflict_region_count'] == 1
    assert (tmp_path / 'audit' / 'trajectory_occupied_conflicts.pgm').is_file()
    assert (tmp_path / 'audit' / 'trajectory_unknown_corridor.pgm').is_file()
    assert (tmp_path / 'audit' / 'existing_map_audit.yaml').is_file()


def test_existing_map_audit_is_deterministic_and_unknown_is_diagnostic(tmp_path: Path):
    navigation, poses = _make_package(
        tmp_path, occupied_in_corridor=False, unknown_in_corridor=True)
    first = audit_existing_navigation_map(navigation, poses, tmp_path / 'audit_one')
    second = audit_existing_navigation_map(navigation, poses, tmp_path / 'audit_two')
    assert first['acceptance_status'] == 'PASS'
    assert first['summary'] == second['summary']
    assert first['occupied_conflict_regions'] == second['occupied_conflict_regions'] == []


def test_existing_map_audit_rejects_wrong_generation_trajectory(tmp_path: Path):
    navigation, poses = _make_package(
        tmp_path, occupied_in_corridor=False, unknown_in_corridor=False)
    wrong_poses = tmp_path / 'wrong_poses.txt'
    wrong_poses.write_text(poses.read_text(encoding='utf-8'), encoding='utf-8')
    with pytest.raises(ValueError, match='does not match converter metadata provenance'):
        audit_existing_navigation_map(navigation, wrong_poses, tmp_path / 'audit')
