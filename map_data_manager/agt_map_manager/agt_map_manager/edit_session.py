"""Map-edit session lifecycle and immutable Nav2 grid geometry contract.

The HMI may edit occupancy cells and topology, but it must never change the
world-to-grid geometry of a released map.  This module is intentionally pure
Python so its invariants can be unit-tested without ROS or robot hardware.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import secrets
import shutil
import tempfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Tuple

import yaml

from .map_package import PackageInfo


SESSION_SCHEMA_VERSION = 1
_FLOAT_ABS_TOL = 1e-12
_SESSION_ID_RE = re.compile(r'^[A-Za-z0-9_.-]+$')
_ALLOWED_MODES = {'trinary', 'scale', 'raw'}


@dataclass(frozen=True)
class NavigationContract:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    mode: str
    negate: int
    occupied_thresh: float
    free_thresh: float

    def geometry_dict(self) -> Dict[str, Any]:
        return {
            'frame_id': 'map',
            'width': int(self.width),
            'height': int(self.height),
            'resolution': float(self.resolution),
            'origin': [
                float(self.origin_x),
                float(self.origin_y),
                float(self.origin_yaw),
            ],
        }

    def contract_dict(self) -> Dict[str, Any]:
        data = self.geometry_dict()
        data.update({
            'mode': self.mode,
            'negate': int(self.negate),
            'occupied_thresh': float(self.occupied_thresh),
            'free_thresh': float(self.free_thresh),
        })
        return data

    @property
    def geometry_fingerprint(self) -> str:
        return _fingerprint(self.geometry_dict())

    @property
    def contract_fingerprint(self) -> str:
        return _fingerprint(self.contract_dict())


@dataclass(frozen=True)
class EditSessionInfo:
    session_id: str
    state: str
    session_path: Path
    navigation_map_yaml: Path
    base_map_id: str
    base_map_version: str
    base_metadata_path: Path
    geometry_fingerprint: str
    contract_fingerprint: str
    reason: str = ''


def _fingerprint(data: Dict[str, Any]) -> str:
    canonical = json.dumps(
        data, sort_keys=True, separators=(',', ':'), ensure_ascii=True,
        allow_nan=False,
    ).encode('utf-8')
    return hashlib.sha256(canonical).hexdigest()


def _load_map_yaml(path: Path) -> Tuple[Dict[str, Any], Path]:
    path = path.expanduser().resolve()
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f'navigation_map_yaml_read_failed:{exc}') from exc
    if not isinstance(data, dict):
        raise ValueError('navigation_map_yaml_must_be_mapping')
    image_name = str(data.get('image', '')).strip()
    if not image_name:
        raise ValueError('navigation_map_yaml_missing_image')
    image_path = Path(image_name).expanduser()
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    image_path = image_path.resolve()
    if not image_path.is_file():
        raise ValueError(f'navigation_map_image_missing:{image_path}')
    return data, image_path


def _read_pgm_size(path: Path) -> Tuple[int, int]:
    """Read only the PGM header, supporting P2/P5 and comment lines."""
    tokens = []
    token = bytearray()
    in_comment = False
    with path.open('rb') as stream:
        while len(tokens) < 4:
            value = stream.read(1)
            if not value:
                if token:
                    tokens.append(bytes(token))
                break
            byte = value[0]
            if in_comment:
                if byte in (10, 13):
                    in_comment = False
                continue
            if byte == 35:  # '#'
                if token:
                    tokens.append(bytes(token))
                    token.clear()
                    if len(tokens) >= 4:
                        break
                in_comment = True
                continue
            if chr(byte).isspace():
                if token:
                    tokens.append(bytes(token))
                    token.clear()
                continue
            token.append(byte)
    if len(tokens) < 4:
        raise ValueError('pgm_header_incomplete')
    if tokens[0] not in (b'P2', b'P5'):
        raise ValueError(f'unsupported_map_image_format:{tokens[0].decode("ascii", "replace")}')
    try:
        width = int(tokens[1])
        height = int(tokens[2])
        max_value = int(tokens[3])
    except ValueError as exc:
        raise ValueError('pgm_header_invalid_numeric_value') from exc
    if width <= 0 or height <= 0 or max_value <= 0:
        raise ValueError('pgm_header_invalid_dimensions')
    return width, height


def read_navigation_contract(map_yaml: Path, require_zero_yaw: bool = True) -> NavigationContract:
    data, image_path = _load_map_yaml(map_yaml)
    width, height = _read_pgm_size(image_path)
    try:
        resolution = float(data['resolution'])
        origin = list(data['origin'])
        if len(origin) < 3:
            raise ValueError
        origin_x = float(origin[0])
        origin_y = float(origin[1])
        origin_yaw = float(origin[2])
        negate = int(data.get('negate', 0))
        occupied_thresh = float(data.get('occupied_thresh', 0.65))
        free_thresh = float(data.get('free_thresh', 0.25))
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('navigation_map_geometry_invalid') from exc

    mode = str(data.get('mode', 'trinary')).strip().lower() or 'trinary'
    if mode not in _ALLOWED_MODES:
        raise ValueError(f'navigation_map_mode_invalid:{mode}')
    if resolution <= 0.0 or not math.isfinite(resolution):
        raise ValueError('navigation_map_resolution_invalid')
    for name, value in (
        ('origin_x', origin_x), ('origin_y', origin_y), ('origin_yaw', origin_yaw),
        ('occupied_thresh', occupied_thresh), ('free_thresh', free_thresh),
    ):
        if not math.isfinite(value):
            raise ValueError(f'navigation_map_{name}_not_finite')
    if negate not in (0, 1):
        raise ValueError('navigation_map_negate_must_be_0_or_1')
    if not 0.0 <= free_thresh <= 1.0 or not 0.0 <= occupied_thresh <= 1.0:
        raise ValueError('navigation_map_threshold_out_of_range')
    if free_thresh >= occupied_thresh:
        raise ValueError('navigation_map_free_thresh_must_be_less_than_occupied_thresh')
    if require_zero_yaw and not math.isclose(origin_yaw, 0.0, rel_tol=0.0, abs_tol=_FLOAT_ABS_TOL):
        raise ValueError(f'navigation_map_origin_yaw_not_supported:{origin_yaw}')

    return NavigationContract(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
        mode=mode,
        negate=negate,
        occupied_thresh=occupied_thresh,
        free_thresh=free_thresh,
    )


def _float_equal(lhs: float, rhs: float) -> bool:
    return math.isclose(lhs, rhs, rel_tol=0.0, abs_tol=_FLOAT_ABS_TOL)


def assert_contract_compatible(base: NavigationContract, edited: NavigationContract) -> None:
    exact_fields = ('width', 'height', 'mode', 'negate')
    for field in exact_fields:
        before = getattr(base, field)
        after = getattr(edited, field)
        if before != after:
            raise ValueError(f'navigation_contract_mismatch:{field}:{before}!={after}')
    float_fields = (
        'resolution', 'origin_x', 'origin_y', 'origin_yaw',
        'occupied_thresh', 'free_thresh',
    )
    for field in float_fields:
        before = float(getattr(base, field))
        after = float(getattr(edited, field))
        if not _float_equal(before, after):
            raise ValueError(f'navigation_contract_mismatch:{field}:{before}!={after}')


def assert_navigation_maps_compatible(base_yaml: Path, edited_yaml: Path) -> Tuple[NavigationContract, NavigationContract]:
    base = read_navigation_contract(base_yaml, require_zero_yaw=True)
    edited = read_navigation_contract(edited_yaml, require_zero_yaw=True)
    assert_contract_compatible(base, edited)
    return base, edited


def _atomic_write_yaml(path: Path, data: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f'.{path.name}.', suffix='.tmp', dir=str(path.parent))
    tmp_path = Path(tmp_name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp_path, path)
    finally:
        if tmp_path.exists():
            tmp_path.unlink()


def _new_session_id() -> str:
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    return f'edit_{timestamp}_{secrets.token_hex(4)}'


def _safe_session_dir(edit_root: Path, session_id: str) -> Path:
    session_id = str(session_id).strip()
    if not session_id or not _SESSION_ID_RE.fullmatch(session_id):
        raise ValueError('invalid_edit_session_id')
    root = edit_root.expanduser().resolve()
    candidate = (root / session_id).resolve()
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise ValueError('edit_session_path_escape') from exc
    return candidate


def create_edit_session(base: PackageInfo, edit_root: Path) -> EditSessionInfo:
    if not base.valid:
        raise ValueError(f'base_map_package_invalid:{base.reason}')
    source_yaml = Path(base.asset_path('navigation_map')).resolve()
    source_data, source_image = _load_map_yaml(source_yaml)
    base_contract = read_navigation_contract(source_yaml, require_zero_yaw=True)

    edit_root = edit_root.expanduser().resolve()
    edit_root.mkdir(parents=True, exist_ok=True)
    for _ in range(8):
        session_id = _new_session_id()
        session_dir = _safe_session_dir(edit_root, session_id)
        try:
            session_dir.mkdir(parents=False, exist_ok=False)
            break
        except FileExistsError:
            continue
    else:
        raise RuntimeError('failed_to_allocate_unique_edit_session')

    session_yaml = session_dir / 'map.yaml'
    session_image = session_dir / 'map.pgm'
    session_data = dict(source_data)
    session_data['image'] = 'map.pgm'
    _atomic_write_yaml(session_yaml, session_data)
    shutil.copy2(source_image, session_image)

    source_topology = source_yaml.with_suffix('.topology')
    if source_topology.is_file():
        shutil.copy2(source_topology, session_dir / 'map.topology')

    # Verify the staged representation itself before exposing it to the HMI.
    staged_contract = read_navigation_contract(session_yaml, require_zero_yaw=True)
    assert_contract_compatible(base_contract, staged_contract)

    now = datetime.now(timezone.utc).isoformat()
    metadata = {
        'schema_version': SESSION_SCHEMA_VERSION,
        'session_id': session_id,
        'state': 'open',
        'created_utc': now,
        'updated_utc': now,
        'base_map_id': base.map_id,
        'base_map_version': base.map_version,
        'base_metadata_path': str(base.metadata_path),
        'base_package_path': str(base.package_path),
        'navigation_map_yaml': str(session_yaml),
        'geometry_fingerprint': base_contract.geometry_fingerprint,
        'contract_fingerprint': base_contract.contract_fingerprint,
        'navigation_contract': base_contract.contract_dict(),
        'reason': 'ready_for_hmi_edit',
    }
    _atomic_write_yaml(session_dir / 'session.yaml', metadata)
    return load_edit_session(edit_root, session_id)


def load_edit_session(edit_root: Path, session_id: str) -> EditSessionInfo:
    session_dir = _safe_session_dir(edit_root, session_id)
    metadata_path = session_dir / 'session.yaml'
    try:
        data = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f'edit_session_read_failed:{exc}') from exc
    if not isinstance(data, dict):
        raise ValueError('edit_session_metadata_must_be_mapping')
    if int(data.get('schema_version', -1)) != SESSION_SCHEMA_VERSION:
        raise ValueError('unsupported_edit_session_schema')
    if str(data.get('session_id', '')).strip() != str(session_id).strip():
        raise ValueError('edit_session_id_mismatch')
    nav_yaml = session_dir / 'map.yaml'
    if not nav_yaml.is_file():
        raise ValueError('edit_session_navigation_map_missing')
    base_metadata = Path(str(data.get('base_metadata_path', ''))).expanduser()
    if not str(base_metadata):
        raise ValueError('edit_session_base_metadata_missing')
    return EditSessionInfo(
        session_id=str(session_id).strip(),
        state=str(data.get('state', '')).strip(),
        session_path=session_dir,
        navigation_map_yaml=nav_yaml,
        base_map_id=str(data.get('base_map_id', '')).strip(),
        base_map_version=str(data.get('base_map_version', '')).strip(),
        base_metadata_path=base_metadata.resolve(),
        geometry_fingerprint=str(data.get('geometry_fingerprint', '')).strip(),
        contract_fingerprint=str(data.get('contract_fingerprint', '')).strip(),
        reason=str(data.get('reason', '')).strip(),
    )


def validate_edit_session_for_publish(edit_root: Path, session_id: str) -> EditSessionInfo:
    info = load_edit_session(edit_root, session_id)
    if info.state != 'open':
        raise ValueError(f'edit_session_not_open:{info.state}')
    metadata_path = info.session_path / 'session.yaml'
    data = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
    snapshot = data.get('navigation_contract')
    if not isinstance(snapshot, dict):
        raise ValueError('edit_session_navigation_contract_missing')
    try:
        origin = list(snapshot['origin'])
        base = NavigationContract(
            width=int(snapshot['width']),
            height=int(snapshot['height']),
            resolution=float(snapshot['resolution']),
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=float(origin[2]),
            mode=str(snapshot['mode']).strip().lower(),
            negate=int(snapshot['negate']),
            occupied_thresh=float(snapshot['occupied_thresh']),
            free_thresh=float(snapshot['free_thresh']),
        )
    except (KeyError, IndexError, TypeError, ValueError) as exc:
        raise ValueError('edit_session_navigation_contract_invalid') from exc
    edited = read_navigation_contract(info.navigation_map_yaml, require_zero_yaw=True)
    assert_contract_compatible(base, edited)
    if base.geometry_fingerprint != info.geometry_fingerprint:
        raise ValueError('edit_session_geometry_fingerprint_corrupt')
    if base.contract_fingerprint != info.contract_fingerprint:
        raise ValueError('edit_session_contract_fingerprint_corrupt')
    return info


def set_edit_session_state(edit_root: Path, session_id: str, state: str, reason: str) -> EditSessionInfo:
    if state not in {'open', 'published', 'cancelled'}:
        raise ValueError(f'invalid_edit_session_state:{state}')
    info = load_edit_session(edit_root, session_id)
    metadata_path = info.session_path / 'session.yaml'
    data = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
    data['state'] = state
    data['reason'] = str(reason)
    data['updated_utc'] = datetime.now(timezone.utc).isoformat()
    _atomic_write_yaml(metadata_path, data)
    return load_edit_session(edit_root, session_id)
