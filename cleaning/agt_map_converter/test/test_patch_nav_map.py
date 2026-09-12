from pathlib import Path

import numpy as np
import yaml

from agt_map_converter.patch_nav_map import apply_patch
from agt_map_converter.pcd_to_nav_map import write_pgm
from agt_map_converter.validate_nav_map import read_pgm_header


def test_patch_nav_map_world_polygon(tmp_path: Path):
    source = tmp_path / 'source'
    source.mkdir()

    image = np.full((10, 10), 254, dtype=np.uint8)
    image[8, 1] = 0  # map cell gx=1, gy=1
    image[7, 2] = 0  # retain one occupied cell for validator
    write_pgm(source / 'map.pgm', image)
    write_pgm(source / 'obstacle.pgm', image)
    write_pgm(source / 'elevation.pgm', np.full((10, 10), 254, dtype=np.uint8))
    write_pgm(source / 'slope.pgm', np.full((10, 10), 254, dtype=np.uint8))

    (source / 'map.yaml').write_text(
        yaml.safe_dump({
            'image': 'map.pgm',
            'mode': 'trinary',
            'resolution': 1.0,
            'origin': [0.0, 0.0, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196,
        }, sort_keys=False),
        encoding='utf-8',
    )
    (source / 'converter_metadata.yaml').write_text('{}\n', encoding='utf-8')

    patch = tmp_path / 'patch.yaml'
    patch.write_text(
        yaml.safe_dump({
            'edits': [{
                'mode': 'free',
                'note': 'remove temporary obstacle',
                'polygon_m': [[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]],
            }],
        }, sort_keys=False),
        encoding='utf-8',
    )

    output = tmp_path / 'patched'
    target, history = apply_patch(source, patch, output)

    width, height, payload = read_pgm_header(target / 'map.pgm')
    result = np.frombuffer(payload, dtype=np.uint8).reshape((height, width))
    assert result[8, 1] == 254
    assert result[7, 2] == 0
    assert history[0]['cells'] == 1

    metadata = yaml.safe_load(
        (target / 'converter_metadata.yaml').read_text(encoding='utf-8'))
    assert metadata['manual_patch_history'][-1]['edits'][0]['mode'] == 'free'
