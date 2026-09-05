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


def load_trajectory_poses(path: Path):
    """Load mapping poses.txt as planar body-frame poses (x, y, yaw).

    FAST-LIO mapping writes rows as:
      patch.pcd tx ty tz qw qx qy qz

    The optional swept-footprint carve below is intentionally body-centered.
    Its asymmetric longitudinal bounds include the current body->base_link
    offset plus the configured Nav2 Bunker footprint/padding.
    """
    poses = []
    for lineno, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
        text = line.strip()
        if not text or text.startswith('#'):
            continue
        parts = text.split()
        if len(parts) < 8:
            raise ValueError(f'{path}:{lineno}: expected patch tx ty tz qw qx qy qz')
        tx, ty = float(parts[1]), float(parts[2])
        qw, qx, qy, qz = (float(parts[4]), float(parts[5]),
                          float(parts[6]), float(parts[7]))
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if norm <= 1e-12:
            raise ValueError(f'{path}:{lineno}: zero quaternion')
        qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm
        yaw = math.atan2(
            2.0 * (qw*qz + qx*qy),
            1.0 - 2.0 * (qy*qy + qz*qz),
        )
        poses.append((tx, ty, yaw))
    if not poses:
        raise ValueError(f'{path}: no valid trajectory poses')
    return poses


def carve_trajectory_free(pgm, origin, resolution, poses,
                          front_m, rear_m, half_width_m):
    """Mark the physically traversed body/base footprint as known free space.

    This is evidence-based clearing for mapping artifacts (tree canopy, sparse
    vertical returns, sensor self remnants). It is applied only when a caller
    explicitly supplies the mapping trajectory. Obstacles outside the swept
    footprint remain untouched.
    """
    if not poses:
        return 0
    min_x, min_y, _ = origin
    height, width = pgm.shape
    radius = math.hypot(max(front_m, rear_m), half_width_m)
    cells = int(math.ceil(radius / resolution)) + 1
    before = pgm.copy()

    for x, y, yaw in poses:
        cx = int((x - min_x) / resolution)
        cy = int((y - min_y) / resolution)
        c, s = math.cos(yaw), math.sin(yaw)
        x0, x1 = max(0, cx - cells), min(width - 1, cx + cells)
        y0, y1 = max(0, cy - cells), min(height - 1, cy + cells)
        for gy in range(y0, y1 + 1):
            wy = min_y + (gy + 0.5) * resolution
            for gx in range(x0, x1 + 1):
                wx = min_x + (gx + 0.5) * resolution
                dx, dy = wx - x, wy - y
                local_x = c * dx + s * dy
                local_y = -s * dx + c * dy
                if -rear_m <= local_x <= front_m and abs(local_y) <= half_width_m:
                    pgm[gy, gx] = 254

    return int(np.count_nonzero((before != 254) & (pgm == 254)))


def convert(xyz, resolution, margin, min_points, max_step, max_slope_deg,
            trajectory_poses=None, trajectory_front_m=0.40,
            trajectory_rear_m=0.72, trajectory_half_width_m=0.46):
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
    # numpy.gradient requires at least two samples along every axis. Small
    # commissioning maps can legitimately collapse to one row or column;
    # there is no measurable slope along that axis, so use zero there.
    if filled.shape[0] < 2:
        gy = np.zeros_like(filled)
    else:
        gy = np.gradient(filled, resolution, axis=0)
    if filled.shape[1] < 2:
        gx = np.zeros_like(filled)
    else:
        gx = np.gradient(filled, resolution, axis=1)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    slope[~np.isfinite(elevation)] = np.nan

    obstacle = valid & ((span > max_step) | (slope > max_slope_deg))
    free = valid & ~obstacle

    # Nav2 trinary map convention: black occupied, white free, gray unknown.
    pgm = np.full((height, width), 205, dtype=np.uint8)
    pgm[free] = 254
    pgm[obstacle] = 0

    origin = [min_x, min_y, 0.0]
    trajectory_cleared_cells = carve_trajectory_free(
        pgm, origin, resolution, trajectory_poses,
        trajectory_front_m, trajectory_rear_m, trajectory_half_width_m)

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

    # Keep obstacle/debug output consistent with any optional trajectory carve.
    obstacle_image = np.where(pgm == 0, 0, np.where(pgm == 254, 254, 205)).astype(np.uint8)
    known = pgm != 205
    final_obstacle = pgm == 0

    # PGM rows are top-to-bottom; map origin is bottom-left, so flip vertically on disk.
    return {
        'origin': origin,
        'occupancy': np.flipud(pgm),
        'elevation': np.flipud(normalized_layer(elevation)),
        'slope': np.flipud(normalized_layer(slope, invert=True)),
        'obstacle': np.flipud(obstacle_image),
        'shape': [height, width],
        'valid_cells': int(known.sum()),
        'occupied_cells': int(final_obstacle.sum()),
        'trajectory_cleared_cells': trajectory_cleared_cells,
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
    parser.add_argument('--trajectory-poses', default='',
                        help='optional FAST-LIO poses.txt used as traversed free-space evidence')
    parser.add_argument('--trajectory-front-m', type=float, default=0.40,
                        help='body-frame forward swept-footprint clear distance')
    parser.add_argument('--trajectory-rear-m', type=float, default=0.72,
                        help='body-frame rear swept-footprint clear distance')
    parser.add_argument('--trajectory-half-width-m', type=float, default=0.46,
                        help='body-frame swept-footprint half width')
    args = parser.parse_args(argv)
    if (args.resolution <= 0 or args.margin < 0 or args.min_points < 1
            or args.trajectory_front_m < 0 or args.trajectory_rear_m < 0
            or args.trajectory_half_width_m < 0):
        parser.error('invalid grid parameters')

    pcd = Path(args.pcd).expanduser().resolve()
    out = Path(args.output).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    xyz = load_xyz(pcd)
    trajectory_path = None
    trajectory_poses = None
    if args.trajectory_poses:
        trajectory_path = Path(args.trajectory_poses).expanduser().resolve()
        trajectory_poses = load_trajectory_poses(trajectory_path)
    layers = convert(xyz, args.resolution, args.margin, args.min_points,
                     args.max_step, args.max_slope_deg,
                     trajectory_poses=trajectory_poses,
                     trajectory_front_m=args.trajectory_front_m,
                     trajectory_rear_m=args.trajectory_rear_m,
                     trajectory_half_width_m=args.trajectory_half_width_m)
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
        'trajectory_poses': str(trajectory_path) if trajectory_path else '',
        'trajectory_pose_count': len(trajectory_poses) if trajectory_poses else 0,
        'trajectory_front_m': float(args.trajectory_front_m),
        'trajectory_rear_m': float(args.trajectory_rear_m),
        'trajectory_half_width_m': float(args.trajectory_half_width_m),
        'trajectory_cleared_cells': layers['trajectory_cleared_cells'],
        'warning': 'Demo V1 thresholds are not final; verify slope/edge behavior on the real Bunker.',
    }
    (out / 'converter_metadata.yaml').write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
    print(f'Wrote Nav2 map package to {out}')


if __name__ == '__main__':
    main()
