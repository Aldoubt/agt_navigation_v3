"""Read-only analysis for bounded MapTracker failure artifacts.

The tracker writes query clouds in base_link and crop clouds in map.  This tool
uses the saved T_map_base_prediction to compare them in map coordinates.  It
does not invoke a matcher or write to a map asset.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path


def _pcd_points(path: Path):
    raw = path.read_bytes()
    marker = b'DATA '
    start = raw.find(marker)
    if start < 0:
        raise ValueError(f'{path}: no PCD DATA header')
    end = raw.find(b'\n', start) + 1
    header = raw[:end].decode('ascii', errors='replace').splitlines()
    values = {line.split(maxsplit=1)[0].upper(): line.split(maxsplit=1)[1]
              for line in header if len(line.split(maxsplit=1)) == 2}
    fields = values.get('FIELDS', '').split()
    sizes = [int(v) for v in values.get('SIZE', '').split()]
    count = int(values.get('POINTS', values.get('WIDTH', '0')))
    data = values['DATA'].strip().lower()
    offsets, offset = {}, 0
    for field, size in zip(fields, sizes):
        offsets[field], offset = offset, offset + size
    if not all(name in offsets for name in ('x', 'y', 'z')):
        raise ValueError(f'{path}: x/y/z fields are required')
    points = []
    if data == 'ascii':
        for line in raw[end:].decode('ascii', errors='replace').splitlines():
            values = line.split()
            if len(values) >= len(fields):
                points.append(tuple(float(values[fields.index(axis)]) for axis in ('x', 'y', 'z')))
    elif data == 'binary':
        stride = offset
        for index in range(count):
            base = end + index * stride
            points.append(tuple(struct.unpack_from('<f', raw, base + offsets[axis])[0] for axis in ('x', 'y', 'z')))
    else:
        raise ValueError(f'{path}: unsupported PCD DATA {data}')
    return points


def _quat_matrix(x, y, z, w):
    return ((1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def _transform(points, data):
    rotation = _quat_matrix(*(data[f'T_map_base_prediction_{k}'] for k in ('qx', 'qy', 'qz', 'qw')))
    translation = [data[f'T_map_base_prediction_{k}'] for k in ('x', 'y', 'z')]
    return [tuple(sum(rotation[row][col] * point[col] for col in range(3)) + translation[row]
                  for row in range(3)) for point in points]


def _stats(points):
    axes = list(zip(*points))
    bbox = [[min(axis), max(axis)] for axis in axes]
    centroid = [sum(axis)/len(axis) for axis in axes]
    z = sorted(axes[2])
    percentile = lambda q: z[round((len(z)-1)*q)]
    return {'point_count': len(points), 'bbox': bbox, 'centroid': centroid,
            'z_range': bbox[2][1] - bbox[2][0],
            'height_distribution': {'p05': percentile(.05), 'p50': percentile(.5), 'p95': percentile(.95)}}


def _bbox_overlap(a, b):
    # XY bbox overlap only: a transparent coarse evidence metric, not GICP overlap.
    ix = max(0.0, min(a[0][1], b[0][1]) - max(a[0][0], b[0][0]))
    iy = max(0.0, min(a[1][1], b[1][1]) - max(a[1][0], b[1][0]))
    area = max(1e-12, (a[0][1]-a[0][0]) * (a[1][1]-a[1][0]))
    return ix * iy / area


def _classify(data, query, crop, overlap, centroid_distance):
    message = data.get('backend_message', '')
    if message == 'GICP did not converge':
        if overlap < 0.20:
            return 'GICP_NOT_CONVERGED:CROP_OVERLAP_INSUFFICIENT'
        if min(query['z_range'], crop['z_range']) < 0.30 or min(query['point_count'], crop['point_count']) < 1500:
            return 'GICP_NOT_CONVERGED:GEOMETRY_INSUFFICIENT'
        if centroid_distance > 10.0:
            return 'GICP_NOT_CONVERGED:INITIAL_GUESS_SUSPICIOUS'
        return 'GICP_NOT_CONVERGED:UNRESOLVED'
    if data.get('reason') == 'innovation_gate':
        dx, dy, dz = (abs(float(data.get(f'translation_innovation_d{k}_m', 0))) for k in ('x', 'y', 'z'))
        yaw = abs(float(data.get('yaw_innovation_signed_deg', 0)))
        return 'INNOVATION_HIGH:' + ('Z_JUMP' if dz >= max(dx, dy) else 'YAW_JUMP' if yaw > 1.0 else 'TRANSLATION_JUMP')
    return str(data.get('reason_codes', ['UNCLASSIFIED'])[0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug-dir', default='~/.ros/agt_map_tracker_debug')
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    out = Path(args.output_dir); out.mkdir(parents=True, exist_ok=True)
    samples = []
    for artifact in sorted(Path(args.debug_dir).expanduser().glob('attempt_*')):
        result = artifact / 'registration_result.yaml'; query_file = artifact / 'query_cloud.pcd'; crop_file = artifact / 'local_map_crop.pcd'
        if not all(path.exists() for path in (result, query_file, crop_file)):
            continue
        data = json.loads(result.read_text(encoding='utf-8'))
        query = _stats(_transform(_pcd_points(query_file), data)); crop = _stats(_pcd_points(crop_file))
        distance = math.dist(query['centroid'], crop['centroid'])
        overlap = _bbox_overlap(query['bbox'], crop['bbox'])
        samples.append({'artifact': str(artifact), 'attempt_id': data.get('attempt_id'), 'query': query, 'local_map': crop,
                        'centroid_distance_m': distance, 'bbox_overlap_estimate': overlap,
                        'classification': _classify(data, query, crop, overlap, distance),
                        'innovation': {key: data.get(key) for key in ('translation_innovation_dx_m', 'translation_innovation_dy_m', 'translation_innovation_dz_m', 'yaw_innovation_signed_deg')},
                        'fitness': data.get('fitness')})
    def top(key): return max(samples, key=key) if samples else None
    result = {'sample_count': len(samples), 'samples': samples,
              'top_samples': {'translation_innovation': top(lambda s: math.sqrt(sum((s['innovation'].get(k) or 0)**2 for k in ('translation_innovation_dx_m','translation_innovation_dy_m','translation_innovation_dz_m')))),
                              'z_innovation': top(lambda s: abs(s['innovation'].get('translation_innovation_dz_m') or 0)),
                              'gicp_failed': next((s for s in samples if s['classification'].startswith('GICP_NOT_CONVERGED')), None),
                              'lowest_fitness': min((s for s in samples if s['fitness'] is not None), key=lambda s: s['fitness'], default=None)}}
    (out / 'artifact_analysis.json').write_text(json.dumps(result, indent=2) + '\n')
    markers = [{'id': index, 'frame_id': 'map', 'query_centroid': sample['query']['centroid'],
                'crop_centroid': sample['local_map']['centroid'], 'text': sample['classification']}
               for index, sample in enumerate(samples)]
    (out / 'rviz_markers.yaml').write_text(json.dumps({'markers': markers}, indent=2) + '\n')


if __name__ == '__main__':
    main()
