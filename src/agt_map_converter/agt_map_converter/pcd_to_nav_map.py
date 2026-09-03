from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np
import yaml


def _pcd_header(path: Path):
    meta = {}
    header_bytes = 0
    with path.open('rb') as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError('PCD DATA line not found')
            header_bytes += len(line)
            text = line.decode('ascii', errors='strict').strip()
            if not text or text.startswith('#'):
                continue
            parts = text.split()
            meta[parts[0].upper()] = parts[1:]
            if parts[0].upper() == 'DATA':
                break
    return meta, header_bytes


def _dtype_for(meta):
    fields = meta['FIELDS']
    sizes = [int(v) for v in meta['SIZE']]
    types = meta['TYPE']
    counts = [int(v) for v in meta.get('COUNT', ['1'] * len(fields))]
    dtype = []
    for name, size, typ, count in zip(fields, sizes, types, counts):
        if count != 1:
            raise ValueError(f'PCD field {name} COUNT={count} is not supported in demo V1')
        table = {
            ('F', 4): '<f4', ('F', 8): '<f8',
            ('I', 1): '<i1', ('I', 2): '<i2', ('I', 4): '<i4', ('I', 8): '<i8',
            ('U', 1): '<u1', ('U', 2): '<u2', ('U', 4): '<u4', ('U', 8): '<u8',
        }
        key = (typ.upper(), size)
        if key not in table:
            raise ValueError(f'unsupported PCD type/size {key} for field {name}')
        dtype.append((name, table[key]))
    return np.dtype(dtype)


def load_xyz(path: Path):
    meta, offset = _pcd_header(path)
    fields = meta.get('FIELDS', [])
    for required in ('x', 'y', 'z'):
        if required not in fields:
            raise ValueError(f'PCD is missing {required!r} field')
    data_mode = meta['DATA'][0].lower()
    points = int((meta.get('POINTS') or meta.get('WIDTH') or ['0'])[0])
    if data_mode == 'binary_compressed':
        raise ValueError('binary_compressed PCD is not supported; export ASCII or binary PCD')
    if data_mode == 'binary':
        dtype = _dtype_for(meta)
        with path.open('rb') as f:
            f.seek(offset)
            arr = np.fromfile(f, dtype=dtype, count=points if points > 0 else -1)
        xyz = np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float64, copy=False)
    elif data_mode == 'ascii':
        with path.open('r', encoding='ascii') as f:
            while True:
                line = f.readline()
                if line.strip().upper().startswith('DATA '):
                    break
            arr = np.loadtxt(f, dtype=np.float64)
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)
        idx = [fields.index('x'), fields.index('y'), fields.index('z')]
        xyz = arr[:, idx]
    else:
        raise ValueError(f'unsupported PCD DATA mode: {data_mode}')
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if xyz.size == 0:
        raise ValueError('PCD contains no finite XYZ points')
    return xyz


def write_pgm(path: Path, image: np.ndarray):
    image = np.asarray(image, dtype=np.uint8)
    with path.open('wb') as f:
        f.write(f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'.encode('ascii'))
        f.write(image.tobytes(order='C'))


def fill_nearest(grid):
    out = grid.copy()
    valid = np.isfinite(out)
    if not valid.any():
        return out
    # Small deterministic propagation for slope estimation. Unknown cells remain unknown if too far away.
    for _ in range(3):
        changed = False
        for axis in (0, 1):
            for shift in (-1, 1):
                shifted = np.roll(out, shift, axis=axis)
                can = ~np.isfinite(out) & np.isfinite(shifted)
                if axis == 0:
                    can[0 if shift > 0 else -1, :] = False
                else:
                    can[:, 0 if shift > 0 else -1] = False
                if can.any():
                    out[can] = shifted[can]
                    changed = True
        if not changed:
            break
    return out


def convert(xyz, resolution, margin, min_points, max_step, max_slope_deg):
    min_x = float(np.min(xyz[:, 0]) - margin)
    min_y = float(np.min(xyz[:, 1]) - margin)
    max_x = float(np.max(xyz[:, 0]) + margin)
    max_y = float(np.max(xyz[:, 1]) + margin)
    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))
    ix = np.clip(((xyz[:, 0] - min_x) / resolution).astype(np.int64), 0, width - 1)
    iy = np.clip(((xyz[:, 1] - min_y) / resolution).astype(np.int64), 0, height - 1)

    count = np.zeros((height, width), dtype=np.int32)
    min_z = np.full((height, width), np.inf, dtype=np.float64)
    max_z = np.full((height, width), -np.inf, dtype=np.float64)
    np.add.at(count, (iy, ix), 1)
    np.minimum.at(min_z, (iy, ix), xyz[:, 2])
    np.maximum.at(max_z, (iy, ix), xyz[:, 2])

    valid = count >= int(min_points)
    elevation = np.where(valid, min_z, np.nan)
    span = np.where(valid, max_z - min_z, np.nan)
    filled = fill_nearest(elevation)
    gy, gx = np.gradient(filled, resolution, resolution)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    slope[~np.isfinite(elevation)] = np.nan

    obstacle = valid & ((span > max_step) | (slope > max_slope_deg))
    free = valid & ~obstacle

    # Nav2 trinary map convention: black occupied, white free, gray unknown.
    pgm = np.full((height, width), 205, dtype=np.uint8)
    pgm[free] = 254
    pgm[obstacle] = 0

    def normalized_layer(values, invert=False):
        image = np.full(values.shape, 205, dtype=np.uint8)
        mask = np.isfinite(values)
        if not mask.any():
            return image
        lo, hi = float(np.nanpercentile(values, 2)), float(np.nanpercentile(values, 98))
        if hi <= lo:
            hi = lo + 1.0
        scaled = np.clip((values - lo) / (hi - lo), 0.0, 1.0)
        if invert:
            scaled = 1.0 - scaled
        image[mask] = (scaled[mask] * 254.0).astype(np.uint8)
        return image

    # PGM rows are top-to-bottom; map origin is bottom-left, so flip vertically on disk.
    return {
        'origin': [min_x, min_y, 0.0],
        'occupancy': np.flipud(pgm),
        'elevation': np.flipud(normalized_layer(elevation)),
        'slope': np.flipud(normalized_layer(slope, invert=True)),
        'obstacle': np.flipud(np.where(obstacle, 0, np.where(valid, 254, 205)).astype(np.uint8)),
        'shape': [height, width],
        'valid_cells': int(valid.sum()),
        'occupied_cells': int(obstacle.sum()),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description='Convert FAST-LIO2 PCD map to Nav2 + terrain PGM layers.')
    parser.add_argument('pcd')
    parser.add_argument('--output', '-o', required=True)
    parser.add_argument('--resolution', type=float, default=0.10)
    parser.add_argument('--margin', type=float, default=1.0)
    parser.add_argument('--min-points', type=int, default=2)
    parser.add_argument('--max-step', type=float, default=0.22,
                        help='max vertical span in a grid cell before occupied; field-tune on Bunker')
    parser.add_argument('--max-slope-deg', type=float, default=20.0,
                        help='max terrain slope before occupied; field-tune on Bunker')
    args = parser.parse_args(argv)
    if args.resolution <= 0 or args.margin < 0 or args.min_points < 1:
        parser.error('invalid grid parameters')

    pcd = Path(args.pcd).expanduser().resolve()
    out = Path(args.output).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    xyz = load_xyz(pcd)
    layers = convert(xyz, args.resolution, args.margin, args.min_points,
                     args.max_step, args.max_slope_deg)
    write_pgm(out / 'map.pgm', layers['occupancy'])
    write_pgm(out / 'elevation.pgm', layers['elevation'])
    write_pgm(out / 'slope.pgm', layers['slope'])
    write_pgm(out / 'obstacle.pgm', layers['obstacle'])
    map_yaml = {
        'image': 'map.pgm',
        'mode': 'trinary',
        'resolution': float(args.resolution),
        'origin': layers['origin'],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    (out / 'map.yaml').write_text(yaml.safe_dump(map_yaml, sort_keys=False), encoding='utf-8')
    metadata = {
        'source_pcd': str(pcd),
        'resolution': float(args.resolution),
        'max_step': float(args.max_step),
        'max_slope_deg': float(args.max_slope_deg),
        'grid_shape': layers['shape'],
        'valid_cells': layers['valid_cells'],
        'occupied_cells': layers['occupied_cells'],
        'warning': 'Demo V1 thresholds are not final; verify slope/edge behavior on the real Bunker.',
    }
    (out / 'converter_metadata.yaml').write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
    print(f'Wrote Nav2 map package to {out}')


if __name__ == '__main__':
    main()
