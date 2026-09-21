from pathlib import Path

import yaml

from agt_map_manager.bake_keepout_zones import bake_keepout_zones


def _write_map(root: Path) -> Path:
    root.mkdir()
    pixels = bytes([254] * 100)
    (root / 'map.pgm').write_bytes(b'P5\n10 10\n255\n' + pixels)
    map_yaml = root / 'map.yaml'
    map_yaml.write_text(yaml.safe_dump({
        'image': 'map.pgm',
        'resolution': 1.0,
        'origin': [0.0, 0.0, 0.0],
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'negate': 0,
        'mode': 'trinary',
    }), encoding='utf-8')
    return map_yaml


def test_bakes_keepout_polygon_and_preserves_source(tmp_path: Path):
    source = tmp_path / 'source'
    map_yaml = _write_map(source)
    zones = source / 'keepout_zones.yaml'
    zones.write_text(yaml.safe_dump({
        'version': 1,
        'zones': [{
            'id': 1,
            'type': 'keepout',
            'polygon_m': [[2.0, 2.0], [4.0, 2.0], [4.0, 4.0], [2.0, 4.0]],
        }],
    }), encoding='utf-8')
    source_before = (source / 'map.pgm').read_bytes()

    output = tmp_path / 'baked'
    report = bake_keepout_zones(map_yaml, zones, output, inflate_cells=0)

    assert report['zone_count'] == 1
    assert report['polygon_cells'] == 4
    assert report['changed_to_occupied'] == 4
    assert (source / 'map.pgm').read_bytes() == source_before
    raster = (output / 'map.pgm').read_bytes().split(b'\n', 3)[3]
    assert raster[(10 - 1 - 2) * 10 + 2] == 0
    assert raster[(10 - 1 - 3) * 10 + 3] == 0
    assert yaml.safe_load((output / 'map.yaml').read_text())['image'] == 'map.pgm'


def test_inflates_baked_zone_by_one_cell(tmp_path: Path):
    source = tmp_path / 'source'
    map_yaml = _write_map(source)
    zones = source / 'keepout_zones.yaml'
    zones.write_text(yaml.safe_dump({
        'zones': [{
            'type': 'keepout',
            'polygon_m': [[2.0, 2.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0]],
        }],
    }), encoding='utf-8')

    report = bake_keepout_zones(map_yaml, zones, tmp_path / 'baked', inflate_cells=1)

    assert report['polygon_cells'] == 1
    assert report['baked_cells'] == 9
    assert report['changed_to_occupied'] == 9
