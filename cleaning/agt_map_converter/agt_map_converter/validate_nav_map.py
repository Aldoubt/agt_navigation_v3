from __future__ import annotations

import argparse
from pathlib import Path
import sys
import yaml


UNKNOWN_PGM_VALUE = 205
UNKNOWN_OCCUPANCY_PROBABILITY = 50.0 / 255.0


def read_pgm_header(path: Path):
    with path.open('rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError(f'{path.name}: expected binary PGM P5, got {magic!r}')
        line = f.readline().strip()
        while line.startswith(b'#'):
            line = f.readline().strip()
        width, height = [int(v) for v in line.split()]
        maxval = int(f.readline().strip())
        if maxval != 255:
            raise ValueError(f'{path.name}: expected maxval 255, got {maxval}')
        payload = f.read()
    if len(payload) != width * height:
        raise ValueError(
            f'{path.name}: payload size {len(payload)} does not match {width}x{height}')
    return width, height, payload


def validate(directory: Path) -> list[str]:
    errors: list[str] = []
    required = ['map.yaml', 'map.pgm', 'elevation.pgm', 'slope.pgm', 'obstacle.pgm']
    for name in required:
        if not (directory / name).is_file():
            errors.append(f'missing {name}')
    if errors:
        return errors

    try:
        nav = yaml.safe_load((directory / 'map.yaml').read_text(encoding='utf-8')) or {}
        if nav.get('image') != 'map.pgm':
            errors.append('map.yaml image must be map.pgm')
        resolution = float(nav.get('resolution', 0.0))
        if resolution <= 0.0:
            errors.append('map.yaml resolution must be > 0')
        origin = nav.get('origin')
        if not isinstance(origin, list) or len(origin) != 3:
            errors.append('map.yaml origin must be [x, y, yaw]')
        free_thresh = float(nav.get('free_thresh', -1.0))
        occupied_thresh = float(nav.get('occupied_thresh', -1.0))
        if not (0.0 <= free_thresh < occupied_thresh <= 1.0):
            errors.append('map.yaml thresholds must satisfy 0 <= free < occupied <= 1')
    except Exception as exc:
        errors.append(f'map.yaml parse failed: {exc}')

    dims = None
    payloads = {}
    for name in required[1:]:
        try:
            w, h, payload = read_pgm_header(directory / name)
            if dims is None:
                dims = (w, h)
            elif dims != (w, h):
                errors.append(f'{name}: dimensions {(w, h)} do not match {dims}')
            payloads[name] = payload
        except Exception as exc:
            errors.append(str(exc))

    map_payload = payloads.get('map.pgm', b'')
    if map_payload:
        occupied = sum(1 for v in map_payload if v <= 64)
        free = sum(1 for v in map_payload if v >= 250)
        unknown = len(map_payload) - occupied - free
        if occupied == 0:
            errors.append('map.pgm contains no occupied cells')
        if free == 0:
            errors.append('map.pgm contains no free cells')
        total = len(map_payload)
        print(
            f'map cells={total} free={free} ({free/total:.1%}) '
            f'occupied={occupied} ({occupied/total:.1%}) unknown={unknown} ({unknown/total:.1%})')
        if (
            UNKNOWN_PGM_VALUE in map_payload
            and int(nav.get('negate', 0)) == 0
            and float(nav.get('free_thresh', 0.25)) > UNKNOWN_OCCUPANCY_PROBABILITY
        ):
            errors.append('map.yaml free_thresh makes PGM value 205 free instead of unknown')

    metadata = directory / 'converter_metadata.yaml'
    if metadata.exists():
        try:
            meta = yaml.safe_load(metadata.read_text(encoding='utf-8')) or {}
            print(f"source_pcd={meta.get('source_pcd', '')}")
        except Exception as exc:
            errors.append(f'converter_metadata.yaml parse failed: {exc}')
    else:
        print('warning: converter_metadata.yaml not present')

    return errors


def acceptance_result(directory: Path) -> tuple[str, list[str]]:
    """Return the explicit MAP gate result for a generated acceptance map.

    Ordinary validation remains intentionally permissive for draft maps.  The
    acceptance gate is stricter: it requires trajectory QA provenance and its
    reproducible evidence.  Any non-zero conflict stays REVIEW; this tool never
    converts REVIEW into PASS based on an assumed dynamic-object explanation.
    """
    errors = validate(directory)
    if errors:
        return 'FAIL', errors

    metadata_path = directory / 'converter_metadata.yaml'
    if not metadata_path.is_file():
        return 'FAIL', ['acceptance metadata missing: converter_metadata.yaml']
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
    except Exception as exc:
        return 'FAIL', [f'acceptance metadata parse failed: {exc}']

    status = metadata.get('trajectory_qa_status')
    pose_count = metadata.get('trajectory_pose_count')
    poses_path = metadata.get('trajectory_poses')
    conflict_count = metadata.get('trajectory_conflict_cells_before_carve')
    evidence_name = metadata.get('trajectory_conflict_evidence')
    debug_name = metadata.get('trajectory_conflict_debug_pgm')
    if status == 'NOT_RUN' or not poses_path or not isinstance(pose_count, int) or pose_count <= 0:
        return 'FAIL', ['acceptance trajectory QA was not run (poses.txt is required)']
    if status not in {'PASS', 'REVIEW'}:
        return 'FAIL', [f'acceptance trajectory_qa_status is invalid: {status!r}']
    if not isinstance(conflict_count, int) or conflict_count < 0:
        return 'FAIL', ['acceptance trajectory conflict count is missing or invalid']
    if not evidence_name or not debug_name:
        return 'FAIL', ['acceptance trajectory conflict evidence artifacts are not declared']

    evidence_path = directory / str(evidence_name)
    debug_path = directory / str(debug_name)
    if not evidence_path.is_file() or not debug_path.is_file():
        return 'FAIL', ['acceptance trajectory conflict evidence artifact is missing']
    try:
        evidence = yaml.safe_load(evidence_path.read_text(encoding='utf-8')) or {}
        evidence_count = evidence.get('trajectory_conflict_cells')
        swept_count = evidence.get('trajectory_swept_cells')
        evidence_ratio = evidence.get('trajectory_conflict_ratio_of_swept_cells')
        regions = evidence.get('regions')
        region_count = evidence.get('region_count')
        if evidence_count != conflict_count:
            return 'FAIL', ['acceptance conflict count disagrees with trajectory_conflicts.yaml']
        if not isinstance(swept_count, int) or swept_count <= 0:
            return 'FAIL', ['acceptance swept-cell count is missing or invalid']
        expected_ratio = conflict_count / swept_count
        if (not isinstance(evidence_ratio, (int, float))
                or abs(float(evidence_ratio) - expected_ratio) > 1.0e-12
                or abs(float(metadata.get('trajectory_conflict_ratio_of_swept_cells', -1.0))
                       - expected_ratio) > 1.0e-12):
            return 'FAIL', ['acceptance trajectory conflict ratio disagrees with evidence']
        if metadata.get('trajectory_cleared_cells') != conflict_count:
            return 'FAIL', ['acceptance cleared-cell count disagrees with conflict evidence']
        if not isinstance(regions, list) or region_count != len(regions):
            return 'FAIL', ['acceptance conflict region evidence is missing or inconsistent']
        if sum(region.get('cell_count', -1) for region in regions) != conflict_count:
            return 'FAIL', ['acceptance conflict region cell counts disagree with metadata']
        if metadata.get('trajectory_conflict_region_count') != region_count:
            return 'FAIL', ['acceptance conflict region count disagrees with metadata']
        # Confirm the artifact is a structurally valid PGM without interpreting
        # its palette as a Nav2 occupancy map.
        map_width, map_height, _ = read_pgm_header(directory / 'map.pgm')
        debug_width, debug_height, _ = read_pgm_header(debug_path)
        if (debug_width, debug_height) != (map_width, map_height):
            return 'FAIL', ['acceptance trajectory conflict debug image dimensions disagree with map.pgm']
    except Exception as exc:
        return 'FAIL', [f'acceptance trajectory conflict evidence is unreadable: {exc}']

    if conflict_count == 0:
        if status != 'PASS':
            return 'FAIL', ['trajectory QA status must be PASS when conflict count is zero']
        return 'PASS', []
    if status != 'REVIEW':
        return 'FAIL', ['trajectory QA status must be REVIEW when conflicts exist']
    return 'REVIEW', []


def main(argv=None):
    parser = argparse.ArgumentParser(description='Validate AGT Nav2 map-converter output.')
    parser.add_argument('directory', help='Directory containing map.yaml/map.pgm terrain layers')
    parser.add_argument('--acceptance', action='store_true',
                        help='require trajectory QA evidence and emit MAP PASS/REVIEW/FAIL')
    args = parser.parse_args(argv)
    directory = Path(args.directory).expanduser().resolve()
    if args.acceptance:
        status, errors = acceptance_result(directory)
    else:
        status, errors = 'PASS', validate(directory)
    if errors:
        print('MAP VALIDATION FAILED', file=sys.stderr)
        for error in errors:
            print(f' - {error}', file=sys.stderr)
        raise SystemExit(2)
    if args.acceptance:
        print(f'MAP ACCEPTANCE {status}')
    else:
        print('MAP VALIDATION PASS')


if __name__ == '__main__':
    main()
