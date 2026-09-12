"""Small atomic YAML registry for immutable Map Packages."""

from __future__ import annotations

import os
import tempfile
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Optional

import yaml

from .map_package import PackageInfo, sha256_tree


class MapState:
    CANDIDATE = 'CANDIDATE'
    EDITING = 'EDITING'
    VALIDATING = 'VALIDATING'
    VALIDATED = 'VALIDATED'
    ACTIVE = 'ACTIVE'
    ARCHIVED = 'ARCHIVED'
    FAILED = 'FAILED'


TERMINAL = {MapState.ARCHIVED}
ALLOWED = {
    MapState.CANDIDATE: {MapState.EDITING, MapState.VALIDATING, MapState.FAILED, MapState.ARCHIVED},
    MapState.EDITING: {MapState.VALIDATING, MapState.FAILED, MapState.CANDIDATE},
    MapState.VALIDATING: {MapState.VALIDATED, MapState.FAILED},
    MapState.VALIDATED: {MapState.ACTIVE, MapState.ARCHIVED, MapState.FAILED},
    MapState.ACTIVE: {MapState.ARCHIVED},
    MapState.FAILED: {MapState.CANDIDATE, MapState.ARCHIVED},
    MapState.ARCHIVED: set(),
}


@dataclass
class MapRecord:
    map_id: str
    map_version: str
    status: str
    created_time: str
    package_path: str
    package_hash: str = ''
    reason: str = ''


class MapRegistry:
    def __init__(self, path: Path):
        self.path = Path(path).expanduser().resolve()
        self.records: dict[tuple[str, str], MapRecord] = {}
        self.load()

    def load(self) -> None:
        if not self.path.is_file():
            return
        data = yaml.safe_load(self.path.read_text(encoding='utf-8')) or {}
        for raw in data.get('maps', []) if isinstance(data, dict) else []:
            if not isinstance(raw, dict):
                continue
            record = MapRecord(**{key: str(raw.get(key, '')) for key in MapRecord.__dataclass_fields__})
            if record.map_id and record.map_version:
                self.records[(record.map_id, record.map_version)] = record

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = {'schema_version': 1, 'maps': [asdict(x) for x in sorted(
            self.records.values(), key=lambda r: (r.map_id, r.map_version))]}
        fd, tmp_name = tempfile.mkstemp(prefix=f'.{self.path.name}.', dir=str(self.path.parent))
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(payload, stream, sort_keys=False)
                stream.flush(); os.fsync(stream.fileno())
            os.replace(tmp_name, self.path)
        finally:
            if os.path.exists(tmp_name):
                os.unlink(tmp_name)

    def sync(self, packages: Iterable[PackageInfo], active: Optional[tuple[str, str]] = None) -> None:
        now = datetime.now(timezone.utc).isoformat()
        if active is not None:
            for record in self.records.values():
                if record.status == MapState.ACTIVE and (record.map_id, record.map_version) != active:
                    record.status = MapState.ARCHIVED
        for package in packages:
            key = package.key
            old = self.records.get(key)
            status = old.status if old else MapState.CANDIDATE
            if active == key:
                status = MapState.ACTIVE
            package_hash = old.package_hash if old and old.package_path == str(package.package_path) else ''
            self.records[key] = MapRecord(
                package.map_id, package.map_version, status,
                old.created_time if old else now, str(package.package_path),
                package_hash or (sha256_tree(package.package_path) if package.valid else ''),
                old.reason if old else package.reason)
        self.save()

    def get(self, map_id: str, version: str) -> Optional[MapRecord]:
        return self.records.get((str(map_id), str(version)))

    def transition(self, map_id: str, version: str, state: str, reason: str = '') -> MapRecord:
        record = self.get(map_id, version)
        if record is None:
            raise ValueError('map_not_registered')
        state = str(state).upper()
        if state not in ALLOWED.get(record.status, set()):
            raise ValueError(f'invalid_map_transition:{record.status}->{state}')
        record.status = state
        record.reason = reason
        self.save()
        return record

    def set_active(self, map_id: str, version: str) -> MapRecord:
        target = self.get(map_id, version)
        if target is None:
            raise ValueError('map_not_registered')
        if target.status not in (MapState.VALIDATED, MapState.ACTIVE, MapState.ARCHIVED):
            raise ValueError(f'map_must_be_validated_before_active:{target.status}')
        for record in self.records.values():
            if record.status == MapState.ACTIVE and record is not target:
                record.status = MapState.ARCHIVED
        target.status = MapState.ACTIVE
        target.reason = 'rollback' if target.status == MapState.ARCHIVED else 'activated'
        self.save()
        return target
