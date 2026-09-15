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


def neighborhood_min(grid, radius_cells):
    """Low-memory square-neighborhood minimum for finite elevation cells."""
    if radius_cells < 0:
        raise ValueError('radius_cells must be >= 0')
    base = np.where(np.isfinite(grid), grid, np.inf)
    out = np.full_like(base, np.inf)
    height, width = base.shape
    for dy in range(-radius_cells, radius_cells + 1):
        y_src0 = max(0, -dy)
        y_src1 = min(height, height - dy)
        y_dst0 = y_src0 + dy
        y_dst1 = y_src1 + dy
        for dx in range(-radius_cells, radius_cells + 1):
            x_src0 = max(0, -dx)
            x_src1 = min(width, width - dx)
            x_dst0 = x_src0 + dx
            x_dst1 = x_src1 + dx
            np.minimum(
                out[y_dst0:y_dst1, x_dst0:x_dst1],
                base[y_src0:y_src1, x_src0:x_src1],
                out=out[y_dst0:y_dst1, x_dst0:x_dst1])
    out[~np.isfinite(out)] = np.nan
    return out


def slope_from_elevation(elevation, resolution):
    """Return slope degrees while preserving unknown/NaN cells."""
    filled = fill_nearest(elevation)
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
    return slope, filled


def trusted_ground_elevation(
        elevation, valid, radius_cells, height_tolerance_m):
    """Select min-z cells that are locally consistent with the low surface.

    This is intentionally a conservative MQ2-A heuristic, not semantic ground
    segmentation. Cells whose min-z sits far above the local low surface are
    treated as low-confidence terrain rather than directly shaping the slope
    surface.
    """
    if height_tolerance_m < 0.0:
        raise ValueError('height_tolerance_m must be >= 0')
    local_low = neighborhood_min(elevation, radius_cells)
    confident = (
        valid & np.isfinite(elevation) & np.isfinite(local_low)
        & ((elevation - local_low) <= float(height_tolerance_m)))
    trusted = np.where(confident, elevation, np.nan)
    return trusted, confident, local_low


def anchored_ground_connectivity(
        candidate_mask, elevation, seed_mask, resolution,
        max_connect_slope_deg, connect_radius_cells=1):
    """Keep local-ground candidates connected to trajectory evidence.

    Connectivity is 8-neighbor and requires adjacent candidate elevations to be
    vertically continuous under a permissive slope bound. This rejects
    disconnected flat high surfaces without claiming they are obstacles.
    """
    if not (0.0 < max_connect_slope_deg < 90.0):
        raise ValueError('max_connect_slope_deg must be within (0, 90)')
    if connect_radius_cells < 1:
        raise ValueError('connect_radius_cells must be >= 1')
    if candidate_mask.shape != elevation.shape or seed_mask.shape != elevation.shape:
        raise ValueError('ground connectivity arrays must have identical shape')

    height, width = candidate_mask.shape
    connected = np.zeros_like(candidate_mask, dtype=bool)
    pending = []
    seed = candidate_mask & seed_mask & np.isfinite(elevation)
    for gy, gx in np.argwhere(seed):
        connected[gy, gx] = True
        pending.append((int(gy), int(gx)))

    tan_limit = math.tan(math.radians(float(max_connect_slope_deg)))
    radius = int(connect_radius_cells)
    while pending:
        cy, cx = pending.pop()
        current_z = float(elevation[cy, cx])
        for ny in range(max(0, cy - radius), min(height, cy + radius + 1)):
            for nx in range(max(0, cx - radius), min(width, cx + radius + 1)):
                if (ny == cy and nx == cx) or connected[ny, nx]:
                    continue
                if not candidate_mask[ny, nx] or not np.isfinite(elevation[ny, nx]):
                    continue
                cell_distance = math.hypot(nx - cx, ny - cy)
                if cell_distance > radius:
                    continue
                distance = resolution * cell_distance
                max_dz = tan_limit * distance
                if abs(float(elevation[ny, nx]) - current_z) <= max_dz:
                    connected[ny, nx] = True
                    pending.append((ny, nx))
    return connected, seed


def collision_band_evidence(
        xyz, ix, iy, ground_reference, valid,
        min_height_m, max_height_m,
        min_points, min_fraction):
    """Accumulate per-cell obstacle support in a ground-relative height band."""
    if min_height_m < 0.0 or max_height_m <= min_height_m:
        raise ValueError('invalid collision-band height limits')
    if min_points < 1:
        raise ValueError('collision_band_min_points must be >= 1')
    if not (0.0 <= min_fraction <= 1.0):
        raise ValueError('collision_band_min_fraction must be within [0, 1]')

    height, width = valid.shape
    total = np.zeros((height, width), dtype=np.int32)
    band = np.zeros((height, width), dtype=np.int32)
    max_hag = np.full((height, width), -np.inf, dtype=np.float64)

    reference = ground_reference[iy, ix]
    usable = np.isfinite(reference)
    if np.any(usable):
        ux = ix[usable]
        uy = iy[usable]
        heights = xyz[usable, 2] - reference[usable]
        np.add.at(total, (uy, ux), 1)
        in_band = (
            (heights >= float(min_height_m))
            & (heights <= float(max_height_m)))
        if np.any(in_band):
            np.add.at(band, (uy[in_band], ux[in_band]), 1)
        np.maximum.at(max_hag, (uy, ux), heights)

    fraction = np.zeros((height, width), dtype=np.float64)
    np.divide(
        band, total, out=fraction, where=total > 0)

    obstacle = (
        valid
        & (band >= int(min_points))
        & (fraction >= float(min_fraction)))
    ambiguous = valid & (band > 0) & ~obstacle
    overhang_only = (
        valid & (total > 0) & (band == 0)
        & np.isfinite(max_hag)
        & (max_hag > float(max_height_m)))
    return {
        'total_points': total,
        'band_points': band,
        'band_fraction': fraction,
        'max_height_above_ground': max_hag,
        'obstacle': obstacle,
        'ambiguous': ambiguous,
        'overhang_only': overhang_only,
    }


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
    swept = trajectory_swept_mask(
        pgm.shape, origin, resolution, poses, front_m, rear_m, half_width_m)
    before = pgm.copy()
    pgm[swept] = 254
    return int(np.count_nonzero((before != 254) & swept))


def trajectory_swept_mask(shape, origin, resolution, poses,
                           front_m, rear_m, half_width_m):
    """Return the map-grid cells covered by the recorded body footprint.

    The grid uses map coordinates (origin at its lower-left), before the PGM
    row flip performed for disk output.  Keeping this mask explicit makes the
    trajectory QA evidence reproducible without assigning a cause to a
    conflict: a conflict is only a non-free map cell in a traversed corridor.
    """
    swept = np.zeros(shape, dtype=bool)
    if not poses:
        return swept
    min_x, min_y, _ = origin
    height, width = shape
    radius = math.hypot(max(front_m, rear_m), half_width_m)
    cells = int(math.ceil(radius / resolution)) + 1

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
                    swept[gy, gx] = True
    return swept


def trajectory_conflict_regions(conflicts, origin, resolution):
    """Cluster 8-connected conflict cells in deterministic grid scan order."""
    height, width = conflicts.shape
    seen = np.zeros_like(conflicts, dtype=bool)
    min_x, min_y, _ = origin
    regions = []
    for gy in range(height):
        for gx in range(width):
            if not conflicts[gy, gx] or seen[gy, gx]:
                continue
            seen[gy, gx] = True
            pending = [(gy, gx)]
            cells = []
            while pending:
                cy, cx = pending.pop()
                cells.append((cy, cx))
                for ny in range(max(0, cy - 1), min(height, cy + 2)):
                    for nx in range(max(0, cx - 1), min(width, cx + 2)):
                        if conflicts[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True
                            pending.append((ny, nx))
            cells.sort()
            grid_y = [cell[0] for cell in cells]
            grid_x = [cell[1] for cell in cells]
            centers = [
                (min_x + (x + 0.5) * resolution, min_y + (y + 0.5) * resolution)
                for y, x in cells
            ]
            gx0, gx1 = min(grid_x), max(grid_x)
            gy0, gy1 = min(grid_y), max(grid_y)
            regions.append({
                'id': len(regions) + 1,
                'cell_count': len(cells),
                'grid_bbox': {'min_x': gx0, 'min_y': gy0, 'max_x': gx1, 'max_y': gy1},
                'map_bbox_m': {
                    'min_x': min_x + gx0 * resolution,
                    'min_y': min_y + gy0 * resolution,
                    'max_x': min_x + (gx1 + 1) * resolution,
                    'max_y': min_y + (gy1 + 1) * resolution,
                },
                'centroid_m': {
                    'x': float(sum(point[0] for point in centers) / len(centers)),
                    'y': float(sum(point[1] for point in centers) / len(centers)),
                },
                'approximate_area_m2': float(len(cells) * resolution * resolution),
                'cells': [
                    {
                        'grid_x': x,
                        'grid_y': y,
                        'map_x_m': center[0],
                        'map_y_m': center[1],
                    }
                    for (y, x), center in zip(cells, centers)
                ],
            })
    return regions


def trajectory_conflict_debug_image(original, swept, conflicts):
    """Create a no-dependency QA overlay; see emitted YAML legend for values."""
    image = original.copy()
    image[swept] = 160       # swept corridor over original free/occupied/unknown
    image[conflicts] = 80    # suspected-artifact conflict; not a dynamic label
    return np.flipud(image)


def convert(xyz, resolution, margin, min_points, max_step, max_slope_deg,
            trajectory_poses=None, trajectory_front_m=0.40,
            trajectory_rear_m=0.72, trajectory_half_width_m=0.46,
            slope_surface_mode='legacy_min_z',
            ground_radius_cells=2,
            ground_height_tolerance_m=0.25,
            ground_connect_max_slope_deg=45.0,
            ground_connect_radius_cells=3,
            obstacle_mode='legacy_span',
            collision_band_min_height_m=0.15,
            collision_band_max_height_m=1.50,
            collision_band_min_points=2,
            collision_band_min_fraction=0.20):
    min_x = float(np.min(xyz[:, 0]) - margin)
    min_y = float(np.min(xyz[:, 1]) - margin)
    max_x = float(np.max(xyz[:, 0]) + margin)
    max_y = float(np.max(xyz[:, 1]) + margin)
    width = max(1, int(math.ceil((max_x - min_x) / resolution)))
    height = max(1, int(math.ceil((max_y - min_y) / resolution)))
    origin = [min_x, min_y, 0.0]
    ix = np.clip(((xyz[:, 0] - min_x) / resolution).astype(np.int64), 0, width - 1)
    iy = np.clip(((xyz[:, 1] - min_y) / resolution).astype(np.int64), 0, height - 1)

    count = np.zeros((height, width), dtype=np.int32)
    min_z = np.full((height, width), np.inf, dtype=np.float64)
    max_z = np.full((height, width), -np.inf, dtype=np.float64)
    np.add.at(count, (iy, ix), 1)
    np.minimum.at(min_z, (iy, ix), xyz[:, 2])
    np.maximum.at(max_z, (iy, ix), xyz[:, 2])

    valid = count >= int(min_points)
    raw_elevation = np.where(valid, min_z, np.nan)
    span = np.where(valid, max_z - min_z, np.nan)

    local_ground_candidate = valid.copy()
    ground_anchor_seed = np.zeros_like(valid, dtype=bool)
    if slope_surface_mode == 'legacy_min_z':
        slope_elevation = raw_elevation
        ground_confident = valid.copy()
        local_ground_reference = raw_elevation.copy()
    elif slope_surface_mode in {
            'ground_confidence', 'anchored_ground_confidence'}:
        local_trusted, local_ground_candidate, local_ground_reference = (
            trusted_ground_elevation(
                raw_elevation, valid, ground_radius_cells,
                ground_height_tolerance_m))
        if slope_surface_mode == 'ground_confidence':
            ground_confident = local_ground_candidate
        else:
            if not trajectory_poses:
                raise ValueError(
                    'anchored_ground_confidence requires trajectory poses')
            anchor_swept = trajectory_swept_mask(
                valid.shape, origin, resolution, trajectory_poses,
                trajectory_front_m, trajectory_rear_m,
                trajectory_half_width_m)
            ground_confident, ground_anchor_seed = anchored_ground_connectivity(
                local_ground_candidate, raw_elevation, anchor_swept,
                resolution, ground_connect_max_slope_deg,
                ground_connect_radius_cells)
        slope_elevation = np.where(
            ground_confident, raw_elevation, np.nan)
    else:
        raise ValueError(
            'slope_surface_mode must be legacy_min_z, ground_confidence, '
            'or anchored_ground_confidence')

    slope, filled = slope_from_elevation(slope_elevation, resolution)
    span_trigger = valid & (span > max_step)
    slope_trigger = ground_confident & (slope > max_slope_deg)

    collision = {
        'total_points': np.zeros(valid.shape, dtype=np.int32),
        'band_points': np.zeros(valid.shape, dtype=np.int32),
        'band_fraction': np.zeros(valid.shape, dtype=np.float64),
        'max_height_above_ground': np.full(
            valid.shape, -np.inf, dtype=np.float64),
        'obstacle': np.zeros(valid.shape, dtype=bool),
        'ambiguous': np.zeros(valid.shape, dtype=bool),
        'overhang_only': np.zeros(valid.shape, dtype=bool),
    }

    if obstacle_mode == 'legacy_span':
        obstacle = span_trigger | slope_trigger
        if slope_surface_mode in {
                'ground_confidence', 'anchored_ground_confidence'}:
            free = ground_confident & ~obstacle
        else:
            free = valid & ~obstacle
    elif obstacle_mode == 'ground_relative_band':
        if slope_surface_mode == 'legacy_min_z':
            raise ValueError(
                'ground_relative_band requires a ground-confidence slope mode')
        collision = collision_band_evidence(
            xyz, ix, iy, filled, valid,
            collision_band_min_height_m,
            collision_band_max_height_m,
            collision_band_min_points,
            collision_band_min_fraction)
        obstacle = slope_trigger | collision['obstacle']

        # Conservative ambiguity rule: if a cell contains collision-band
        # evidence but lacks enough support to call it occupied, keep it
        # unknown rather than free. Pure high-overhang evidence may be free
        # only when the cell itself has anchored ground support.
        free = (
            ground_confident
            & ~obstacle
            & ~collision['ambiguous'])
    else:
        raise ValueError(
            'obstacle_mode must be legacy_span or ground_relative_band')

    # Nav2 trinary map convention: black occupied, white free, gray unknown.
    pgm = np.full((height, width), 205, dtype=np.uint8)
    pgm[free] = 254
    pgm[obstacle] = 0

    occupancy_before_trajectory_carve = pgm.copy()
    trajectory_swept = trajectory_swept_mask(
        pgm.shape, origin, resolution, trajectory_poses,
        trajectory_front_m, trajectory_rear_m, trajectory_half_width_m)
    trajectory_conflicts = trajectory_swept & (occupancy_before_trajectory_carve != 254)
    pgm[trajectory_swept] = 254
    trajectory_cleared_cells = int(trajectory_conflicts.sum())

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
        'elevation': np.flipud(normalized_layer(raw_elevation)),
        'slope': np.flipud(normalized_layer(slope, invert=True)),
        'ground_confidence': np.flipud(
            np.where(ground_confident, 254, 205).astype(np.uint8)),
        'ground_local_candidate': np.flipud(
            np.where(local_ground_candidate, 254, 205).astype(np.uint8)),
        'ground_anchor_seed': np.flipud(
            np.where(ground_anchor_seed, 254, 205).astype(np.uint8)),
        'collision_band': np.flipud(
            np.where(
                collision['obstacle'], 0,
                np.where(collision['ambiguous'], 160,
                         np.where(collision['overhang_only'], 80, 254))
            ).astype(np.uint8)),
        'obstacle': np.flipud(obstacle_image),
        'shape': [height, width],
        # Historical field kept for compatibility: this is the final count of
        # non-unknown navigation cells after trajectory carve, not raw
        # count>=min_points validity.
        'valid_cells': int(known.sum()),
        'raw_valid_cells': int(valid.sum()),
        'final_known_cells': int(known.sum()),
        'occupied_cells': int(final_obstacle.sum()),
        'ground_local_candidate_cells': int(local_ground_candidate.sum()),
        'ground_anchor_seed_cells': int(ground_anchor_seed.sum()),
        'ground_confident_cells': int(ground_confident.sum()),
        'floating_ground_candidate_cells': int(
            (local_ground_candidate & ~ground_confident).sum()),
        'low_confidence_valid_cells': int((valid & ~ground_confident).sum()),
        'span_trigger_cells': int(span_trigger.sum()),
        'slope_trigger_cells': int(slope_trigger.sum()),
        'collision_band_obstacle_cells': int(collision['obstacle'].sum()),
        'collision_band_ambiguous_cells': int(collision['ambiguous'].sum()),
        'collision_band_overhang_only_cells': int(
            collision['overhang_only'].sum()),
        'trajectory_cleared_cells': trajectory_cleared_cells,
        'trajectory_swept_cells': int(trajectory_swept.sum()),
        'trajectory_conflicts': trajectory_conflicts,
        'trajectory_conflict_debug': trajectory_conflict_debug_image(
            occupancy_before_trajectory_carve, trajectory_swept, trajectory_conflicts),
        'trajectory_conflict_regions': trajectory_conflict_regions(
            trajectory_conflicts, origin, resolution),
        'trajectory_conflict_original_occupied_cells': int(
            (trajectory_conflicts & (occupancy_before_trajectory_carve == 0)).sum()),
        'trajectory_conflict_original_unknown_cells': int(
            (trajectory_conflicts & (occupancy_before_trajectory_carve == 205)).sum()),
    }


def generate_navigation_map(
    pcd: Path,
    output: Path,
    *,
    resolution: float = 0.10,
    margin: float = 1.0,
    min_points: int = 2,
    max_step: float = 0.22,
    max_slope_deg: float = 20.0,
    trajectory_poses_path: Path | None = None,
    trajectory_front_m: float = 0.40,
    trajectory_rear_m: float = 0.72,
    trajectory_half_width_m: float = 0.46,
    slope_surface_mode: str = 'legacy_min_z',
    ground_radius_cells: int = 2,
    ground_height_tolerance_m: float = 0.25,
    ground_connect_max_slope_deg: float = 45.0,
    ground_connect_radius_cells: int = 3,
    obstacle_mode: str = 'legacy_span',
    collision_band_min_height_m: float = 0.15,
    collision_band_max_height_m: float = 1.50,
    collision_band_min_points: int = 2,
    collision_band_min_fraction: float = 0.20,
) -> dict:
    """Generate one deterministic Nav2/terrain directory from a frozen PCD."""
    if (resolution <= 0 or margin < 0 or min_points < 1
            or trajectory_front_m < 0 or trajectory_rear_m < 0
            or trajectory_half_width_m < 0 or ground_radius_cells < 0
            or ground_height_tolerance_m < 0
            or not (0.0 < ground_connect_max_slope_deg < 90.0)
            or ground_connect_radius_cells < 1
            or collision_band_min_height_m < 0.0
            or collision_band_max_height_m <= collision_band_min_height_m
            or collision_band_min_points < 1
            or not (0.0 <= collision_band_min_fraction <= 1.0)):
        raise ValueError('invalid grid parameters')
    if obstacle_mode not in {'legacy_span', 'ground_relative_band'}:
        raise ValueError(
            'obstacle_mode must be legacy_span or ground_relative_band')
    if slope_surface_mode not in {
            'legacy_min_z', 'ground_confidence',
            'anchored_ground_confidence'}:
        raise ValueError(
            'slope_surface_mode must be legacy_min_z, ground_confidence, '
            'or anchored_ground_confidence')
    pcd = pcd.expanduser().resolve()
    output = output.expanduser().resolve()
    if not pcd.is_file():
        raise FileNotFoundError(pcd)
    trajectory_path = None
    trajectory_poses = None
    if trajectory_poses_path is not None:
        trajectory_path = trajectory_poses_path.expanduser().resolve()
        trajectory_poses = load_trajectory_poses(trajectory_path)

    output.mkdir(parents=True, exist_ok=True)
    layers = convert(
        load_xyz(pcd), resolution, margin, min_points, max_step, max_slope_deg,
        trajectory_poses=trajectory_poses,
        trajectory_front_m=trajectory_front_m,
        trajectory_rear_m=trajectory_rear_m,
        trajectory_half_width_m=trajectory_half_width_m,
        slope_surface_mode=slope_surface_mode,
        ground_radius_cells=ground_radius_cells,
        ground_height_tolerance_m=ground_height_tolerance_m,
        ground_connect_max_slope_deg=ground_connect_max_slope_deg,
        ground_connect_radius_cells=ground_connect_radius_cells,
        obstacle_mode=obstacle_mode,
        collision_band_min_height_m=collision_band_min_height_m,
        collision_band_max_height_m=collision_band_max_height_m,
        collision_band_min_points=collision_band_min_points,
        collision_band_min_fraction=collision_band_min_fraction,
    )
    write_pgm(output / 'map.pgm', layers['occupancy'])
    write_pgm(output / 'elevation.pgm', layers['elevation'])
    write_pgm(output / 'slope.pgm', layers['slope'])
    write_pgm(output / 'obstacle.pgm', layers['obstacle'])
    write_pgm(output / 'ground_confidence.pgm', layers['ground_confidence'])
    write_pgm(
        output / 'ground_local_candidate.pgm',
        layers['ground_local_candidate'])
    write_pgm(
        output / 'ground_anchor_seed.pgm',
        layers['ground_anchor_seed'])
    write_pgm(output / 'collision_band.pgm', layers['collision_band'])
    map_yaml = {
        'image': 'map.pgm',
        'mode': 'trinary',
        'resolution': float(resolution),
        'origin': layers['origin'],
        'negate': 0,
        'occupied_thresh': 0.65,
        # PGM value 205 denotes unknown. With negate=0 it maps to 50/255,
        # so this threshold must remain below it for Nav2 to preserve unknown.
        'free_thresh': 0.196,
    }
    (output / 'map.yaml').write_text(
        yaml.safe_dump(map_yaml, sort_keys=False), encoding='utf-8')
    trajectory_qa_status = 'NOT_RUN'
    if trajectory_poses:
        trajectory_qa_status = (
            'PASS' if layers['trajectory_cleared_cells'] == 0 else 'REVIEW'
        )

    conflict_count = int(layers['trajectory_conflicts'].sum())
    swept_count = int(layers['trajectory_swept_cells'])
    conflict_evidence = {
        'format_version': 1,
        'description': (
            'Trajectory conflict / suspected artifact evidence. A conflict means '
            'a recorded swept corridor crossed a non-free cell before the carve; '
            'it is not a dynamic-object classification.'),
        'trajectory_conflict_cells': conflict_count,
        'trajectory_swept_cells': swept_count,
        'trajectory_conflict_ratio_of_swept_cells': (
            float(conflict_count / swept_count) if swept_count else 0.0),
        'original_occupied_conflict_cells': layers['trajectory_conflict_original_occupied_cells'],
        'original_unknown_conflict_cells': layers['trajectory_conflict_original_unknown_cells'],
        'region_count': len(layers['trajectory_conflict_regions']),
        'debug_pgm': 'trajectory_conflicts.pgm',
        'legend': {
            '0': 'original occupied cell outside swept/conflict overlay',
            '205': 'original unknown cell outside swept/conflict overlay',
            '254': 'original free cell outside swept/conflict overlay',
            '160': 'trajectory swept corridor without conflict',
            '80': 'trajectory conflict / suspected artifact',
        },
        'regions': layers['trajectory_conflict_regions'],
    }
    if trajectory_poses:
        write_pgm(output / 'trajectory_conflicts.pgm', layers['trajectory_conflict_debug'])
        (output / 'trajectory_conflicts.yaml').write_text(
            yaml.safe_dump(conflict_evidence, sort_keys=False), encoding='utf-8')

    metadata = {
        'source_pcd': str(pcd),
        'resolution': float(resolution),
        'margin': float(margin),
        'min_points': int(min_points),
        'max_step': float(max_step),
        'max_slope_deg': float(max_slope_deg),
        'slope_surface_mode': slope_surface_mode,
        'ground_radius_cells': int(ground_radius_cells),
        'ground_height_tolerance_m': float(ground_height_tolerance_m),
        'ground_connect_max_slope_deg': float(ground_connect_max_slope_deg),
        'ground_connect_radius_cells': int(ground_connect_radius_cells),
        'obstacle_mode': obstacle_mode,
        'collision_band_min_height_m': float(collision_band_min_height_m),
        'collision_band_max_height_m': float(collision_band_max_height_m),
        'collision_band_min_points': int(collision_band_min_points),
        'collision_band_min_fraction': float(collision_band_min_fraction),
        'grid_shape': layers['shape'],
        'valid_cells': layers['valid_cells'],
        'raw_valid_cells': layers['raw_valid_cells'],
        'final_known_cells': layers['final_known_cells'],
        'occupied_cells': layers['occupied_cells'],
        'ground_local_candidate_cells': layers['ground_local_candidate_cells'],
        'ground_anchor_seed_cells': layers['ground_anchor_seed_cells'],
        'ground_confident_cells': layers['ground_confident_cells'],
        'floating_ground_candidate_cells': layers[
            'floating_ground_candidate_cells'],
        'low_confidence_valid_cells': layers['low_confidence_valid_cells'],
        'span_trigger_cells': layers['span_trigger_cells'],
        'slope_trigger_cells': layers['slope_trigger_cells'],
        'collision_band_obstacle_cells': layers[
            'collision_band_obstacle_cells'],
        'collision_band_ambiguous_cells': layers[
            'collision_band_ambiguous_cells'],
        'collision_band_overhang_only_cells': layers[
            'collision_band_overhang_only_cells'],
        'trajectory_poses': str(trajectory_path) if trajectory_path else '',
        'trajectory_pose_count': len(trajectory_poses) if trajectory_poses else 0,
        'trajectory_front_m': float(trajectory_front_m),
        'trajectory_rear_m': float(trajectory_rear_m),
        'trajectory_half_width_m': float(trajectory_half_width_m),
        # Every cleared cell was non-free before the swept-footprint evidence
        # was applied. A non-zero count is a useful review signal for ghost
        # obstacles / canopy / self returns intersecting a physically traversed corridor.
        'trajectory_cleared_cells': layers['trajectory_cleared_cells'],
        'trajectory_conflict_cells_before_carve': conflict_count,
        'trajectory_conflict_ratio_of_swept_cells': conflict_evidence['trajectory_conflict_ratio_of_swept_cells'],
        'trajectory_conflict_region_count': conflict_evidence['region_count'],
        'trajectory_conflict_original_occupied_cells': (
            conflict_evidence['original_occupied_conflict_cells']),
        'trajectory_conflict_original_unknown_cells': (
            conflict_evidence['original_unknown_conflict_cells']),
        'trajectory_conflict_evidence': (
            'trajectory_conflicts.yaml' if trajectory_poses else ''),
        'trajectory_conflict_debug_pgm': (
            'trajectory_conflicts.pgm' if trajectory_poses else ''),
        'trajectory_qa_status': trajectory_qa_status,
        'warning': 'Demo V1 thresholds are not final; verify slope/edge behavior on the real Bunker.',
    }
    (output / 'converter_metadata.yaml').write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
    return metadata


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
    parser.add_argument(
        '--slope-surface-mode',
        choices=(
            'legacy_min_z', 'ground_confidence',
            'anchored_ground_confidence'),
        default='legacy_min_z',
        help=(
            'legacy min-z, local ground-confidence, or MQ2-A.1 '
            'trajectory-anchored ground-confidence slope'))
    parser.add_argument(
        '--ground-radius-cells', type=int, default=2,
        help='MQ2-A local low-surface neighborhood radius in grid cells')
    parser.add_argument(
        '--ground-height-tolerance-m', type=float, default=0.25,
        help='MQ2-A max min-z height above local low surface to trust as ground')
    parser.add_argument(
        '--ground-connect-max-slope-deg', type=float, default=45.0,
        help=(
            'MQ2-A.1 max slope used only to propagate '
            'trajectory-anchored ground support'))
    parser.add_argument(
        '--ground-connect-radius-cells', type=int, default=3,
        help=(
            'MQ2-A.1 candidate-graph bridge radius in cells; allows sparse '
            'ground observations to connect without crossing steep height jumps'))
    parser.add_argument(
        '--obstacle-mode',
        choices=('legacy_span', 'ground_relative_band'),
        default='legacy_span',
        help='legacy vertical-span obstacle rule or MQ2-B collision-band evidence')
    parser.add_argument(
        '--collision-band-min-height-m', type=float, default=0.15,
        help='MQ2-B lower height-above-ground bound for blocking returns')
    parser.add_argument(
        '--collision-band-max-height-m', type=float, default=1.50,
        help='MQ2-B upper height-above-ground bound for blocking returns')
    parser.add_argument(
        '--collision-band-min-points', type=int, default=2,
        help='MQ2-B minimum returns in collision height band')
    parser.add_argument(
        '--collision-band-min-fraction', type=float, default=0.20,
        help='MQ2-B minimum in-band return fraction per cell')
    parser.add_argument('--trajectory-poses', default='',
                        help='optional FAST-LIO poses.txt used as traversed free-space evidence')
    parser.add_argument('--trajectory-front-m', type=float, default=0.40,
                        help='body-frame forward swept-footprint clear distance')
    parser.add_argument('--trajectory-rear-m', type=float, default=0.72,
                        help='body-frame rear swept-footprint clear distance')
    parser.add_argument('--trajectory-half-width-m', type=float, default=0.46,
                        help='body-frame swept-footprint half width')
    args = parser.parse_args(argv)
    pcd = Path(args.pcd).expanduser().resolve()
    out = Path(args.output).expanduser().resolve()
    try:
        generate_navigation_map(
            pcd,
            out,
            resolution=args.resolution,
            margin=args.margin,
            min_points=args.min_points,
            max_step=args.max_step,
            max_slope_deg=args.max_slope_deg,
            trajectory_poses_path=(Path(args.trajectory_poses) if args.trajectory_poses else None),
            trajectory_front_m=args.trajectory_front_m,
            trajectory_rear_m=args.trajectory_rear_m,
            trajectory_half_width_m=args.trajectory_half_width_m,
            slope_surface_mode=args.slope_surface_mode,
            ground_radius_cells=args.ground_radius_cells,
            ground_height_tolerance_m=args.ground_height_tolerance_m,
            ground_connect_max_slope_deg=args.ground_connect_max_slope_deg,
            ground_connect_radius_cells=args.ground_connect_radius_cells,
            obstacle_mode=args.obstacle_mode,
            collision_band_min_height_m=args.collision_band_min_height_m,
            collision_band_max_height_m=args.collision_band_max_height_m,
            collision_band_min_points=args.collision_band_min_points,
            collision_band_min_fraction=args.collision_band_min_fraction,
        )
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(f'Wrote Nav2 map package to {out}')


if __name__ == '__main__':
    main()
