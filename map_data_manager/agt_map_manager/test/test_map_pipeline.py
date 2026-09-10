from pathlib import Path

import yaml

from agt_map_manager.map_package import validate_package
from agt_map_manager.map_pipeline import generate_map_package
from agt_map_manager.runtime_binding import resolve_active_map
from agt_map_manager.select_map_package import select


def _write_ascii_pcd(path: Path) -> None:
    points = [
        (0.05, 0.05, 0.00), (0.08, 0.08, 0.01),
        (0.25, 0.05, 0.00), (0.28, 0.08, 0.02),
        (0.45, 0.05, 0.00), (0.45, 0.05, 0.50),
    ]
    header = (
        '# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\n'
        'TYPE F F F\nCOUNT 1 1 1\nWIDTH 6\nHEIGHT 1\nPOINTS 6\nDATA ascii\n')
    path.write_text(header + ''.join(f'{x} {y} {z}\n' for x, y, z in points), encoding='ascii')


def _write_relocalization_assets(root: Path) -> Path:
    root.mkdir(parents=True)
    (root / 'relocalization_assets.yaml').write_text('schema_version: 1\n', encoding='utf-8')
    (root / 'global_map_downsampled.pcd').write_bytes(b'pcd')
    (root / 'polar_context.db').write_bytes(b'polar-context-db')
    (root / 'polar_context.yaml').write_text('schema_version: 1\n', encoding='utf-8')
    voxels = root / 'voxelmaps_coords'
    voxels.mkdir()
    (voxels / 'voxel_params.txt').write_text('min_level_res 0.5\n', encoding='utf-8')
    (voxels / '0.pcd').write_bytes(b'pcd')
    return root


def test_pipeline_publishes_hash_validated_package_with_provenance(tmp_path: Path):
    pcd = tmp_path / 'global_map.pcd'
    _write_ascii_pcd(pcd)
    config = tmp_path / 'pipeline.yaml'
    config.write_text(yaml.safe_dump({
        'schema_version': 1,
        'converter': {
            'resolution': 0.2,
            'margin': 0.1,
            'min_points': 2,
            'max_step': 0.22,
            'max_slope_deg': 45.0,
        },
    }), encoding='utf-8')

    package = generate_map_package(
        map_root=tmp_path / 'maps',
        map_id='field_a',
        map_version='v001',
        source_pcd=pcd,
        pipeline_config=config,
        relocalization_assets_dir=_write_relocalization_assets(tmp_path / 'relocalization'),
    )

    info = validate_package(package / 'metadata.yaml', verify_hashes=True)
    assert info.valid
    assert info.asset_path('generation_pipeline').endswith('generation/pipeline.yaml')
    assert info.asset_path('quality_report').endswith('quality/report.yaml')
    metadata = yaml.safe_load((package / 'metadata.yaml').read_text(encoding='utf-8'))
    assert metadata['generation']['pipeline_asset'] == 'generation_pipeline'
    assert metadata['quality']['status'] == 'pass'

    active_state = tmp_path / 'active_map.yaml'
    selected = select(tmp_path / 'maps', 'field_a', 'v001', active_state)
    assert selected == package
    binding = resolve_active_map(active_state)
    assert binding.package.key == ('field_a', 'v001')
    assert binding.generation == 1
