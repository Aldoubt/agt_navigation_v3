"""Analyze Nav2 occupancy deltas against frozen source-PCD evidence.

This tool is evidence-only. It never modifies the reference map, candidate map,
source PCD, or trajectory. The default selected transition is:
candidate occupied -> reference free.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
from pathlib import Path
import sys

import numpy as np
import yaml

from .pcd_to_nav_map import (
    load_trajectory_poses,
    load_xyz,
    trajectory_conflict_regions,
    trajectory_swept_mask,
    write_pgm,
)
from .validate_nav_map import read_pgm_header, validate


TRINARY_VALUES = {'occupied': 0, 'unknown': 205, 'free': 254}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _load_yaml(path: Path, label: str) -> dict:
    if not path.is_file():
        raise ValueError(f'{label} missing: {path}')
    value = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    if not isinstance(value, dict):
        raise ValueError(f'{label} must be a YAML mapping: {path}')
    return value


def _load_map(directory: Path):
    errors = validate(directory)
    if errors:
        raise ValueError(
            f'{directory}: map validation failed: ' + '; '.join(errors))
    nav = _load_yaml(directory / 'map.yaml', 'map.yaml')
    image = directory / str(nav.get('image', 'map.pgm'))
    width, height, payload = read_pgm_header(image)
    disk = np.frombuffer(payload, dtype=np.uint8).reshape((height, width)).copy()
    if not bool(np.isin(disk, list(TRINARY_VALUES.values())).all()):
        raise ValueError(f'{image}: expected trinary values 0/205/254 only')
    return nav, image, np.flipud(disk)


def _same_geometry(reference_nav, candidate_nav, reference_grid, candidate_grid):
    if reference_grid.shape != candidate_grid.shape:
        raise ValueError(
            f'map shape mismatch: {reference_grid.shape} != {candidate_grid.shape}')
    reference_resolution = float(reference_nav['resolution'])
    candidate_resolution = float(candidate_nav['resolution'])
    if abs(reference_resolution - candidate_resolution) > 1.0e-12:
        raise ValueError(
            f'map resolution mismatch: {reference_resolution} != {candidate_resolution}')
    reference_origin = [float(v) for v in reference_nav['origin']]
    candidate_origin = [float(v) for v in candidate_nav['origin']]
    if any(abs(a - b) > 1.0e-9 for a, b in zip(reference_origin, candidate_origin)):
        raise ValueError(
            f'map origin mismatch: {reference_origin} != {candidate_origin}')
    return reference_resolution, reference_origin


def _transition_counts(reference_grid, candidate_grid):
    result = {}
    for candidate_name, candidate_value in TRINARY_VALUES.items():
        for reference_name, reference_value in TRINARY_VALUES.items():
            key = f'candidate_{candidate_name}_to_reference_{reference_name}'
            result[key] = int(np.count_nonzero(
                (candidate_grid == candidate_value)
                & (reference_grid == reference_value)))
    return result


def _dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return mask.copy()
    height, width = mask.shape
    result = np.zeros_like(mask, dtype=bool)
    for dy in range(-radius, radius + 1):
        y_src0 = max(0, -dy)
        y_src1 = min(height, height - dy)
        y_dst0 = y_src0 + dy
        y_dst1 = y_src1 + dy
        for dx in range(-radius, radius + 1):
            x_src0 = max(0, -dx)
            x_src1 = min(width, width - dx)
            x_dst0 = x_src0 + dx
            x_dst1 = x_src1 + dx
            result[y_dst0:y_dst1, x_dst0:x_dst1] |= (
                mask[y_src0:y_src1, x_src0:x_src1])
    return result


def _point_groups(xyz, origin, resolution, shape, support_mask):
    height, width = shape
    ix = np.floor((xyz[:, 0] - origin[0]) / resolution).astype(np.int64)
    iy = np.floor((xyz[:, 1] - origin[1]) / resolution).astype(np.int64)
    inside = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    xyz = xyz[inside]
    ix = ix[inside]
    iy = iy[inside]
    keep = support_mask[iy, ix]
    groups = {}
    for gx, gy, z in zip(ix[keep], iy[keep], xyz[keep, 2]):
        groups.setdefault(int(gy * width + gx), []).append(float(z))
    return {
        key: np.asarray(values, dtype=np.float64)
        for key, values in groups.items()
    }


def _cell_z(groups, gx, gy, width):
    return groups.get(int(gy * width + gx), np.empty(0, dtype=np.float64))


def _percentile(values, q):
    return float(np.percentile(values, q)) if len(values) else None


def _cell_evidence(
        groups, gx, gy, shape, resolution, origin, *,
        ground_radius_cells, ground_percentile,
        near_ground_height_m, low_obstacle_height_m, overhang_height_m):
    height, width = shape
    z = _cell_z(groups, gx, gy, width)
    neighborhood = []
    for ny in range(max(0, gy - ground_radius_cells),
                    min(height, gy + ground_radius_cells + 1)):
        for nx in range(max(0, gx - ground_radius_cells),
                        min(width, gx + ground_radius_cells + 1)):
            values = _cell_z(groups, nx, ny, width)
            if len(values):
                neighborhood.append(values)
    local_values = (
        np.concatenate(neighborhood)
        if neighborhood else np.empty(0, dtype=np.float64))
    local_ground = _percentile(local_values, ground_percentile)
    heights = (
        z - local_ground
        if local_ground is not None else np.empty(0, dtype=np.float64))

    row = {
        'grid_x': int(gx),
        'grid_y': int(gy),
        'map_x_m': float(origin[0] + (gx + 0.5) * resolution),
        'map_y_m': float(origin[1] + (gy + 0.5) * resolution),
        'point_count': int(len(z)),
        'local_ground_z': local_ground,
    }
    if len(z):
        p05 = _percentile(z, 5)
        p95 = _percentile(z, 95)
        row.update({
            'z_min': float(np.min(z)),
            'z_p05': p05,
            'z_median': _percentile(z, 50),
            'z_p95': p95,
            'z_max': float(np.max(z)),
            'z_span': float(np.max(z) - np.min(z)),
            'z_robust_span_p95_p05': float(p95 - p05),
        })
    else:
        for key in (
                'z_min', 'z_p05', 'z_median', 'z_p95', 'z_max',
                'z_span', 'z_robust_span_p95_p05'):
            row[key] = None

    if len(heights):
        row.update({
            'height_above_ground_p05_m': _percentile(heights, 5),
            'height_above_ground_median_m': _percentile(heights, 50),
            'height_above_ground_p95_m': _percentile(heights, 95),
            'height_above_ground_max_m': float(np.max(heights)),
            'near_ground_fraction': float(
                np.mean(heights <= near_ground_height_m)),
            'low_obstacle_fraction': float(np.mean(
                (heights > near_ground_height_m)
                & (heights <= low_obstacle_height_m))),
            'tall_return_fraction': float(
                np.mean(heights > low_obstacle_height_m)),
            'overhang_return_fraction': float(
                np.mean(heights > overhang_height_m)),
        })
    else:
        for key in (
                'height_above_ground_p05_m',
                'height_above_ground_median_m',
                'height_above_ground_p95_m',
                'height_above_ground_max_m',
                'near_ground_fraction',
                'low_obstacle_fraction',
                'tall_return_fraction',
                'overhang_return_fraction'):
            row[key] = None
    return row


def _aggregate(rows, key, mode):
    values = [float(row[key]) for row in rows if row.get(key) is not None]
    if not values:
        return None
    if mode == 'mean':
        return float(np.mean(values))
    if mode == 'median':
        return float(np.median(values))
    if mode == 'max':
        return float(np.max(values))
    raise ValueError(mode)


def _write_csv(path: Path, rows: list[dict], fieldnames: list[str]):
    with path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: row.get(name) for name in fieldnames})


def analyze_map_delta(
        reference_directory: Path,
        candidate_directory: Path,
        source_pcd: Path,
        trajectory_poses: Path,
        output: Path,
        *,
        selected_reference_state: str = 'free',
        selected_candidate_state: str = 'occupied',
        expansion_steps_m=(0.10, 0.20, 0.30, 0.40, 0.50),
        ground_radius_cells: int = 2,
        ground_percentile: float = 10.0,
        near_ground_height_m: float = 0.25,
        low_obstacle_height_m: float = 1.0,
        overhang_height_m: float = 2.0) -> dict:
    reference_directory = reference_directory.expanduser().resolve()
    candidate_directory = candidate_directory.expanduser().resolve()
    source_pcd = source_pcd.expanduser().resolve()
    trajectory_poses = trajectory_poses.expanduser().resolve()
    output = output.expanduser().resolve()
    if output.exists():
        raise FileExistsError(f'analysis output already exists: {output}')
    if selected_reference_state not in TRINARY_VALUES:
        raise ValueError(f'unknown reference state: {selected_reference_state}')
    if selected_candidate_state not in TRINARY_VALUES:
        raise ValueError(f'unknown candidate state: {selected_candidate_state}')
    if ground_radius_cells < 0:
        raise ValueError('ground_radius_cells must be >= 0')
    if not (0.0 <= ground_percentile <= 100.0):
        raise ValueError('ground_percentile must be within [0, 100]')
    if not source_pcd.is_file():
        raise FileNotFoundError(source_pcd)
    if not trajectory_poses.is_file():
        raise FileNotFoundError(trajectory_poses)

    reference_nav, reference_map_path, reference_grid = _load_map(
        reference_directory)
    candidate_nav, candidate_map_path, candidate_grid = _load_map(
        candidate_directory)
    resolution, origin = _same_geometry(
        reference_nav, candidate_nav, reference_grid, candidate_grid)

    selected = (
        (reference_grid == TRINARY_VALUES[selected_reference_state])
        & (candidate_grid == TRINARY_VALUES[selected_candidate_state]))
    selected_count = int(selected.sum())
    if selected_count == 0:
        raise ValueError(
            'selected map transition has zero cells; choose another transition')

    regions = trajectory_conflict_regions(selected, origin, resolution)
    region_id_grid = np.full(selected.shape, -1, dtype=np.int32)
    for index, region in enumerate(regions, 1):
        for cell in region['cells']:
            region_id_grid[int(cell['grid_y']), int(cell['grid_x'])] = index

    meta_path = candidate_directory / 'converter_metadata.yaml'
    candidate_meta = (
        _load_yaml(meta_path, 'candidate converter metadata')
        if meta_path.is_file() else {})
    front_m = float(candidate_meta.get('trajectory_front_m', 0.40))
    rear_m = float(candidate_meta.get('trajectory_rear_m', 0.72))
    half_width_m = float(candidate_meta.get('trajectory_half_width_m', 0.46))
    poses = load_trajectory_poses(trajectory_poses)

    coverage = []
    expansion_masks = []
    for expansion in (0.0, *tuple(float(v) for v in expansion_steps_m)):
        mask = trajectory_swept_mask(
            selected.shape, origin, resolution, poses,
            front_m + expansion, rear_m + expansion,
            half_width_m + expansion)
        covered = int((selected & mask).sum())
        coverage.append({
            'expansion_m': float(expansion),
            'selected_cells_covered': covered,
            'selected_cells_total': selected_count,
            'coverage_ratio': float(covered / selected_count),
        })
        expansion_masks.append((float(expansion), mask))

    support_mask = _dilate(selected, ground_radius_cells)
    groups = _point_groups(
        load_xyz(source_pcd), origin, resolution, selected.shape, support_mask)

    cell_rows = []
    for gy, gx in np.argwhere(selected):
        row = _cell_evidence(
            groups, int(gx), int(gy), selected.shape, resolution, origin,
            ground_radius_cells=ground_radius_cells,
            ground_percentile=ground_percentile,
            near_ground_height_m=near_ground_height_m,
            low_obstacle_height_m=low_obstacle_height_m,
            overhang_height_m=overhang_height_m)
        row['region_id'] = int(region_id_grid[gy, gx])
        row['minimum_trajectory_expansion_m'] = None
        for expansion, mask in expansion_masks:
            if bool(mask[gy, gx]):
                row['minimum_trajectory_expansion_m'] = expansion
                break
        cell_rows.append(row)

    cells_by_region = {}
    for row in cell_rows:
        cells_by_region.setdefault(int(row['region_id']), []).append(row)

    region_rows = []
    for region in regions:
        rid = int(region['id'])
        rows = cells_by_region.get(rid, [])
        region_rows.append({
            'region_id': rid,
            'cell_count': int(region['cell_count']),
            'area_m2': float(region['approximate_area_m2']),
            'bbox_min_x_m': float(region['map_bbox_m']['min_x']),
            'bbox_min_y_m': float(region['map_bbox_m']['min_y']),
            'bbox_max_x_m': float(region['map_bbox_m']['max_x']),
            'bbox_max_y_m': float(region['map_bbox_m']['max_y']),
            'centroid_x_m': float(region['centroid_m']['x']),
            'centroid_y_m': float(region['centroid_m']['y']),
            'cells_with_points': int(sum(r['point_count'] > 0 for r in rows)),
            'pcd_points': int(sum(r['point_count'] for r in rows)),
            'median_points_per_cell': _aggregate(rows, 'point_count', 'median'),
            'median_z_robust_span_m': _aggregate(
                rows, 'z_robust_span_p95_p05', 'median'),
            'median_local_ground_z': _aggregate(
                rows, 'local_ground_z', 'median'),
            'median_height_above_ground_p95_m': _aggregate(
                rows, 'height_above_ground_p95_m', 'median'),
            'max_height_above_ground_p95_m': _aggregate(
                rows, 'height_above_ground_p95_m', 'max'),
            'mean_near_ground_fraction': _aggregate(
                rows, 'near_ground_fraction', 'mean'),
            'mean_low_obstacle_fraction': _aggregate(
                rows, 'low_obstacle_fraction', 'mean'),
            'mean_tall_return_fraction': _aggregate(
                rows, 'tall_return_fraction', 'mean'),
            'mean_overhang_return_fraction': _aggregate(
                rows, 'overhang_return_fraction', 'mean'),
        })
    region_rows.sort(
        key=lambda row: (-int(row['cell_count']), int(row['region_id'])))

    output.mkdir(parents=True)

    cell_fields = [
        'region_id', 'grid_x', 'grid_y', 'map_x_m', 'map_y_m',
        'minimum_trajectory_expansion_m', 'point_count',
        'local_ground_z', 'z_min', 'z_p05', 'z_median', 'z_p95', 'z_max',
        'z_span', 'z_robust_span_p95_p05',
        'height_above_ground_p05_m', 'height_above_ground_median_m',
        'height_above_ground_p95_m', 'height_above_ground_max_m',
        'near_ground_fraction', 'low_obstacle_fraction',
        'tall_return_fraction', 'overhang_return_fraction']
    region_fields = [
        'region_id', 'cell_count', 'area_m2',
        'bbox_min_x_m', 'bbox_min_y_m', 'bbox_max_x_m', 'bbox_max_y_m',
        'centroid_x_m', 'centroid_y_m', 'cells_with_points', 'pcd_points',
        'median_points_per_cell', 'median_z_robust_span_m',
        'median_local_ground_z', 'median_height_above_ground_p95_m',
        'max_height_above_ground_p95_m', 'mean_near_ground_fraction',
        'mean_low_obstacle_fraction', 'mean_tall_return_fraction',
        'mean_overhang_return_fraction']
    _write_csv(output / 'delta_cells.csv', cell_rows, cell_fields)
    _write_csv(output / 'delta_regions.csv', region_rows, region_fields)
    (output / 'delta_cells.yaml').write_text(
        yaml.safe_dump({'cells': cell_rows}, sort_keys=False), encoding='utf-8')
    (output / 'delta_regions.yaml').write_text(
        yaml.safe_dump({'regions': region_rows}, sort_keys=False), encoding='utf-8')

    write_pgm(
        output / 'delta_mask.pgm',
        np.where(np.flipud(selected), 0, 254).astype(np.uint8))
    context = np.full(selected.shape, 254, dtype=np.uint8)
    context[expansion_masks[0][1]] = 160
    context[selected] = 0
    write_pgm(output / 'delta_vs_trajectory.pgm', np.flipud(context))

    report = {
        'format_version': 1,
        'analysis_kind': 'map_delta_source_pcd_evidence',
        'selected_transition': {
            'candidate_state': selected_candidate_state,
            'reference_state': selected_reference_state,
            'candidate_value': TRINARY_VALUES[selected_candidate_state],
            'reference_value': TRINARY_VALUES[selected_reference_state],
            'cell_count': selected_count,
        },
        'geometry': {
            'grid_shape': [
                int(reference_grid.shape[0]), int(reference_grid.shape[1])],
            'resolution_m': float(resolution),
            'origin': origin,
        },
        'provenance': {
            'reference_directory': str(reference_directory),
            'candidate_directory': str(candidate_directory),
            'reference_map_pgm': str(reference_map_path),
            'candidate_map_pgm': str(candidate_map_path),
            'reference_map_sha256': _sha256(reference_map_path),
            'candidate_map_sha256': _sha256(candidate_map_path),
            'source_pcd': str(source_pcd),
            'source_pcd_sha256': _sha256(source_pcd),
            'trajectory_poses': str(trajectory_poses),
            'trajectory_poses_sha256': _sha256(trajectory_poses),
        },
        'transition_counts': _transition_counts(
            reference_grid, candidate_grid),
        'trajectory': {
            'pose_count': len(poses),
            'front_m': front_m,
            'rear_m': rear_m,
            'half_width_m': half_width_m,
            'coverage_by_expansion': coverage,
        },
        'source_pcd_evidence': {
            'ground_radius_cells': int(ground_radius_cells),
            'ground_radius_m': float(ground_radius_cells * resolution),
            'local_ground_percentile': float(ground_percentile),
            'near_ground_height_m': float(near_ground_height_m),
            'low_obstacle_height_m': float(low_obstacle_height_m),
            'overhang_height_m': float(overhang_height_m),
            'region_count': len(region_rows),
            'largest_regions': region_rows[:20],
            'interpretation_note': (
                'Evidence only: height fractions do not classify a cell as '
                'dynamic, vegetation, canopy, slope, or a true obstacle.'),
        },
        'artifacts': {
            'delta_cells_csv': 'delta_cells.csv',
            'delta_cells_yaml': 'delta_cells.yaml',
            'delta_regions_csv': 'delta_regions.csv',
            'delta_regions_yaml': 'delta_regions.yaml',
            'delta_mask_pgm': 'delta_mask.pgm',
            'delta_vs_trajectory_pgm': 'delta_vs_trajectory.pgm',
            'delta_vs_trajectory_legend': {
                '0': 'selected map-delta cell',
                '160': 'current trajectory swept footprint',
                '254': 'other cell',
            },
        },
    }
    (output / 'map_delta_analysis.yaml').write_text(
        yaml.safe_dump(report, sort_keys=False), encoding='utf-8')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Analyze Nav2 map delta against source-PCD/trajectory evidence.')
    parser.add_argument('--reference', required=True)
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--source-pcd', required=True)
    parser.add_argument('--trajectory-poses', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument(
        '--reference-state', choices=sorted(TRINARY_VALUES), default='free')
    parser.add_argument(
        '--candidate-state', choices=sorted(TRINARY_VALUES), default='occupied')
    parser.add_argument('--ground-radius-cells', type=int, default=2)
    parser.add_argument('--ground-percentile', type=float, default=10.0)
    args = parser.parse_args(argv)
    try:
        result = analyze_map_delta(
            Path(args.reference), Path(args.candidate),
            Path(args.source_pcd), Path(args.trajectory_poses),
            Path(args.output),
            selected_reference_state=args.reference_state,
            selected_candidate_state=args.candidate_state,
            ground_radius_cells=args.ground_radius_cells,
            ground_percentile=args.ground_percentile)
    except (OSError, ValueError) as exc:
        print(f'MAP DELTA ANALYSIS FAIL: {exc}', file=sys.stderr)
        raise SystemExit(2)
    selected = result['selected_transition']
    print(
        'MAP DELTA ANALYSIS PASS '
        f"candidate_{selected['candidate_state']} -> "
        f"reference_{selected['reference_state']}: "
        f"{selected['cell_count']} cells")
    print(
        f"report={Path(args.output).expanduser().resolve() / 'map_delta_analysis.yaml'}")


if __name__ == '__main__':
    main()
