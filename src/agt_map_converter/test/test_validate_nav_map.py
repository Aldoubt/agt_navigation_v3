from pathlib import Path
import yaml

from agt_map_converter.validate_nav_map import validate


def write_pgm(path: Path, width=2, height=2, payload=b'\xff\x00\xff\x00'):
    path.write_bytes(f'P5\n{width} {height}\n255\n'.encode() + payload)


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
