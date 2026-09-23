"""Read-only validation of PCD/PGM/YAML map assets.

This module deliberately does not reconstruct a point cloud or convert assets.
It reports what is present and leaves map generation to the mapping pipeline.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import yaml
from PIL import Image


def _pcd_header(path: Path):
    header = {}
    data_offset = 0
    with path.open('rb') as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD header has no DATA line')
            data_offset += len(line)
            text = line.decode('ascii', errors='replace').strip()
            if text and not text.startswith('#'):
                parts = text.split()
                header[parts[0].upper()] = parts[1:]
            if text.upper().startswith('DATA '):
                break
    return header, data_offset


def _pcd_xyz(path: Path, header, data_offset: int) -> np.ndarray:
    fields = header.get('FIELDS', [])
    if not {'x', 'y', 'z'}.issubset(fields):
        raise ValueError('PCD must contain x, y and z fields')
    data_kind = header.get('DATA', [''])[0].lower()
    counts = [int(value) for value in header.get('COUNT', ['1'] * len(fields))]
    expanded = []
    field_index = {}
    for name, count in zip(fields, counts):
        field_index[name] = len(expanded)
        expanded.extend([name] * count)
    point_count = int(header.get('POINTS', [int(header.get('WIDTH', ['0'])[0]) * int(header.get('HEIGHT', ['1'])[0])])[0])
    if data_kind == 'ascii':
        values = np.loadtxt(path, dtype=float, skiprows=sum(1 for _ in _header_lines(path)))
        values = np.atleast_2d(values)
        return values[:, [field_index['x'], field_index['y'], field_index['z']]]
    if data_kind != 'binary':
        if data_kind == 'binary_compressed':
            return _pcd_xyz_binary_compressed(path, header, data_offset, fields, counts, point_count)
        raise ValueError(f'PCD DATA {data_kind!r} is not supported')
    sizes = [int(value) for value in header.get('SIZE', [])]
    types = header.get('TYPE', [])
    if len(sizes) != len(fields) or len(types) != len(fields):
        raise ValueError('PCD binary header must include SIZE and TYPE for every field')
    numpy_types = {'F': {4: '<f4', 8: '<f8'}, 'I': {1: '<i1', 2: '<i2', 4: '<i4', 8: '<i8'}, 'U': {1: '<u1', 2: '<u2', 4: '<u4', 8: '<u8'}}
    dtype_fields = []
    for name, size, kind, count in zip(fields, sizes, types, counts):
        dtype_fields.append((name, numpy_types[kind][size], (count,)) if count > 1 else (name, numpy_types[kind][size]))
    dtype = np.dtype(dtype_fields)
    with path.open('rb') as stream:
        stream.seek(data_offset)
        records = np.fromfile(stream, dtype=dtype, count=point_count)
    return np.column_stack([records['x'].reshape(-1), records['y'].reshape(-1), records['z'].reshape(-1)]).astype(float)


def _lzf_decompress(payload: bytes, expected_size: int) -> bytes:
    """Decode the small LZF stream used by PCL binary_compressed PCD files."""
    output = bytearray()
    index = 0
    while index < len(payload) and len(output) < expected_size:
        control = payload[index]
        index += 1
        if control < 32:
            literal_count = control + 1
            output.extend(payload[index:index + literal_count])
            index += literal_count
            continue
        length = control >> 5
        reference = (control & 0x1F) << 8
        if length == 7:
            if index >= len(payload):
                raise ValueError('truncated LZF length')
            length += payload[index]
            index += 1
        if index >= len(payload):
            raise ValueError('truncated LZF reference')
        reference += payload[index]
        index += 1
        reference = len(output) - reference - 1
        if reference < 0:
            raise ValueError('invalid LZF back-reference')
        for _ in range(length + 2):
            if reference >= len(output):
                raise ValueError('invalid LZF copy range')
            output.append(output[reference])
            reference += 1
    if len(output) != expected_size:
        raise ValueError(f'LZF size mismatch: got {len(output)}, expected {expected_size}')
    return bytes(output)


def _pcd_xyz_binary_compressed(path, header, data_offset, fields, counts, point_count):
    # PCL stores binary_compressed fields in structure-of-arrays order, not
    # the interleaved layout used by DATA binary.
    sizes = [int(value) for value in header.get('SIZE', [])]
    types = header.get('TYPE', [])
    if len(sizes) != len(fields) or len(types) != len(fields):
        raise ValueError('PCD compressed header must include SIZE and TYPE for every field')
    numpy_types = {
        'F': {4: '<f4', 8: '<f8'},
        'I': {1: '<i1', 2: '<i2', 4: '<i4', 8: '<i8'},
        'U': {1: '<u1', 2: '<u2', 4: '<u4', 8: '<u8'},
    }
    uncompressed_size = sum(size * count * point_count for size, count in zip(sizes, counts))
    with path.open('rb') as stream:
        stream.seek(data_offset)
        size_bytes = stream.read(8)
        if len(size_bytes) != 8:
            raise ValueError('truncated PCD compressed size header')
        compressed_size = int.from_bytes(size_bytes[:4], 'little')
        declared_size = int.from_bytes(size_bytes[4:], 'little')
        compressed = stream.read(compressed_size)
    raw = _lzf_decompress(compressed, declared_size)
    if declared_size != uncompressed_size:
        raise ValueError(f'PCD compressed size mismatch: got {declared_size}, expected {uncompressed_size}')
    offset = 0
    columns = {}
    for name, size, kind, count in zip(fields, sizes, types, counts):
        dtype = np.dtype(numpy_types[kind][size])
        for component in range(count):
            values = np.frombuffer(raw, dtype=dtype, count=point_count, offset=offset)
            offset += size * point_count
            if component == 0:
                columns[name] = values
    return np.column_stack([columns['x'], columns['y'], columns['z']]).astype(float)


def _header_lines(path: Path):
    with path.open('rb') as stream:
        for line in stream:
            text = line.decode('ascii', errors='replace').strip()
            yield text
            if text.upper().startswith('DATA '):
                break


def validate_pcd(path: Path) -> Dict[str, Any]:
    header, data_offset = _pcd_header(path)
    xyz = _pcd_xyz(path, header, data_offset)
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    if len(xyz):
        minimum = xyz.min(axis=0)
        maximum = xyz.max(axis=0)
        bbox = maximum - minimum
        xy_area = max(float(bbox[0] * bbox[1]), 1e-12)
        z_range = float(bbox[2])
        density = float(len(xyz) / xy_area)
    else:
        minimum = maximum = np.zeros(3)
        z_range = density = 0.0
    return {
        'path': str(path),
        'point_count': int(len(xyz)),
        'bounding_box_min': minimum.tolist(),
        'bounding_box_max': maximum.tolist(),
        'z_range': z_range,
        'density_estimation': density,
        'status': 'OK' if len(xyz) else 'WARNING: no finite XYZ points',
    }


def _yaml_for_pgm(pgm_path: Path, yaml_path: Optional[Path]):
    if yaml_path is not None:
        with yaml_path.open('r', encoding='utf-8') as stream:
            return yaml.safe_load(stream) or {}
    return {}


def validate_pgm(path: Path, yaml_path: Optional[Path] = None) -> Dict[str, Any]:
    image = np.asarray(Image.open(path).convert('L'), dtype=np.uint8)
    metadata = _yaml_for_pgm(path, yaml_path)
    unknown = image == 205
    occupied_thresh = float(metadata.get('occupied_thresh', 0.65))
    # map_server's default convention maps dark pixels to occupied.
    occupied = image <= int((1.0 - occupied_thresh) * 255.0)
    free = image >= int(float(metadata.get('free_thresh', 0.25)) * 255.0)
    resolution = metadata.get('resolution')
    return {
        'path': str(path),
        'width': int(image.shape[1]),
        'height': int(image.shape[0]),
        'resolution': float(resolution) if resolution is not None else None,
        'unknown_ratio': float(np.mean(unknown)),
        'occupied_ratio': float(np.mean(occupied & ~unknown)),
        'free_ratio': float(np.mean(free & ~unknown & ~occupied)),
        'status': 'OK',
    }


def validate_yaml(path: Path) -> Dict[str, Any]:
    with path.open('r', encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream) or {}
    image = metadata.get('image')
    image_path = (path.parent / image).resolve() if image else None
    return {
        'path': str(path),
        'image': str(image_path) if image_path else None,
        'resolution': metadata.get('resolution'),
        'origin': metadata.get('origin'),
        'image_exists': bool(image_path and image_path.exists()),
        'status': 'OK' if image_path and image_path.exists() else 'WARNING: image is missing',
    }


def write_occupancy_grid_asset_report(metric, output_path: Path) -> None:
    """Write a report for the recorded OccupancyGrid when no asset path is supplied."""
    with output_path.open('w', encoding='utf-8') as stream:
        stream.write('# Map Asset Validation Report\n\n')
        stream.write('This report validates the recorded `/map` OccupancyGrid snapshot.\n')
        stream.write('No PCD/PGM reconstruction or conversion is performed.\n\n')
        stream.write('## Recorded OccupancyGrid\n\n')
        stream.write(f'- topic: `{metric.topic}`\n- timestamp: `{metric.timestamp_ns / 1e9:.9f}`\n')
        stream.write(f'- size: `{metric.width} x {metric.height}`\n- resolution: `{metric.resolution:.4f} m`\n')
        stream.write(f'- unknown ratio: `{metric.unknown_ratio * 100.0:.2f}%`\n')
        stream.write(f'- occupied ratio: `{metric.occupied_ratio * 100.0:.2f}%`\n')
        stream.write(f'- unknown connected components: `{metric.unknown_connected_components}`\n')
        stream.write(f'- largest unknown area: `{metric.largest_unknown_area:.3f} m^2`\n')
        stream.write(f'- quality: `{metric.quality}`; risk: `{metric.risk}`\n\n')
        stream.write('## External Asset Validation\n\n')
        stream.write('Run `python3 -m agt_nav_benchmark.map_asset_validator --pgm ... --yaml ...` or `--pcd ...` to validate exported mapping assets.\n')


def write_asset_report(pcd=None, pgm=None, yaml_path=None, output_path=Path('map_asset_report.md')) -> None:
    results = {}
    if pcd:
        results['PCD'] = validate_pcd(Path(pcd))
    if pgm:
        results['PGM'] = validate_pgm(Path(pgm), Path(yaml_path) if yaml_path else None)
    if yaml_path:
        results['YAML'] = validate_yaml(Path(yaml_path))
    with output_path.open('w', encoding='utf-8') as stream:
        stream.write('# Map Asset Validation Report\n\n')
        stream.write('Read-only validation; no point-cloud reconstruction or map conversion was performed.\n\n')
        for name, result in results.items():
            stream.write(f'## {name}\n\n')
            for key, value in result.items():
                stream.write(f'- {key}: `{value}`\n')
            stream.write('\n')
        if not results:
            stream.write('No asset path was supplied.\n')


def main(argv=None):
    parser = argparse.ArgumentParser(description='Validate PCD/PGM/YAML map assets without modifying them.')
    parser.add_argument('--pcd', type=Path)
    parser.add_argument('--pgm', type=Path)
    parser.add_argument('--yaml', dest='yaml_path', type=Path)
    parser.add_argument('--output', type=Path, default=Path('reports/map_asset_report.md'))
    args = parser.parse_args(argv)
    if not any((args.pcd, args.pgm, args.yaml_path)):
        parser.error('at least one of --pcd, --pgm or --yaml is required')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_asset_report(args.pcd, args.pgm, args.yaml_path, args.output)
    print(f'Asset validation complete: {args.output}')


if __name__ == '__main__':
    main()
