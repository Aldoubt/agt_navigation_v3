"""Local RViz operator scratch drafts. NOT a published MapRef/RouteRef asset."""

import hashlib
import json
import os
import tempfile
from pathlib import Path

from .route_editor import RouteEditor

MAX_BYTES = 256000
FORMAT_VERSION = 1


def _document(points, revision, context):
    if type(revision) is not int or revision < 0:
        raise ValueError('invalid draft revision')
    if not isinstance(context, dict) or any(not context.get(k) for k in
                                            ('map_id', 'map_version', 'robot_profile', 'frame')):
        raise ValueError('map/version/profile/frame must be selected')
    if not isinstance(context.get('map_observation'), list):
        raise ValueError('map observation not received')
    clean = RouteEditor().validate(points)
    if len(clean) != len(points):
        raise ValueError('draft contains collapsed duplicate points')
    if clean and revision == 0:
        raise ValueError('nonempty draft requires nonzero revision')
    return dict(format_version=FORMAT_VERSION, state='DRAFT', context=context,
                revision=revision, points=[list(p) for p in clean])


def _read(path):
    if path.is_symlink():
        raise ValueError('symlink draft files are prohibited')
    if path.stat().st_size > MAX_BYTES:
        raise ValueError('draft file too large')
    raw = path.read_bytes()
    if len(raw) > MAX_BYTES:
        raise ValueError('draft file too large')
    return raw, hashlib.sha256(raw).hexdigest()


def load_draft(filename, context):
    path = Path(filename)
    if not path.is_absolute():
        raise ValueError('draft_path must be absolute')
    raw, disk_token = _read(path)
    try:
        document = json.loads(raw)
    except (UnicodeError, ValueError) as exc:
        raise ValueError('invalid draft JSON') from exc
    if not isinstance(document, dict) or set(document) != {
            'format_version', 'state', 'context', 'revision', 'points'}:
        raise ValueError('unexpected draft fields')
    if type(document['format_version']) is not int or document['format_version'] != FORMAT_VERSION:
        raise ValueError('unsupported draft format version')
    if document['state'] != 'DRAFT':
        raise ValueError('only local DRAFT state may be loaded')
    if document['context'] != context:
        raise ValueError('map/version/profile/grid observation mismatch: draft not loaded')
    points = document['points']
    if not isinstance(points, list):
        raise ValueError('draft points must be a list')
    clean_document = _document(points, document['revision'], context)
    return clean_document['points'], document['revision'], disk_token


def save_draft(filename, points, revision, context, known_disk_token=None):
    """Atomic save, refusing to overwrite unknown or concurrently changed drafts."""
    path = Path(filename)
    if not path.is_absolute():
        raise ValueError('draft_path must be absolute')
    document = _document(points, revision, context)
    raw = (json.dumps(document, ensure_ascii=False, allow_nan=False,
                      separators=(',', ':'), sort_keys=True) + '\n').encode('utf-8')
    if len(raw) > MAX_BYTES:
        raise ValueError('draft file too large')
    if path.exists() or path.is_symlink():
        _, actual = _read(path)
        if known_disk_token is None or known_disk_token != actual:
            raise ValueError('draft exists/changed: load before overwriting')
    elif known_disk_token is not None:
        raise ValueError('loaded draft disappeared: refusing overwrite')
    if not path.parent.is_dir():
        raise ValueError('draft parent directory does not exist')
    temp = None
    try:
        with tempfile.NamedTemporaryFile(mode='wb', dir=path.parent,
                                         prefix='.rviz-draft-', delete=False) as stream:
            temp = stream.name
            os.chmod(temp, 0o600)
            stream.write(raw)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
        temp = None
    finally:
        if temp is not None:
            os.unlink(temp)
    return hashlib.sha256(raw).hexdigest()
