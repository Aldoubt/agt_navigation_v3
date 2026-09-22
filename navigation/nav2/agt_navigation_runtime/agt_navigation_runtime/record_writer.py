from __future__ import annotations

from pathlib import Path
import csv
import datetime as dt
import json
import os
import shutil
import threading
import time


class RecordWriter:
    # Keep the existing CSV contract; richer outcome details go to JSONL/events.
    FIELDS = [
        'mission_id', 'map_id', 'point_id', 'view_tag', 'image_path',
        'image_sec', 'image_nanosec', 'pose_valid', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
        'rtk_valid', 'rtk_age_sec', 'latitude', 'longitude', 'altitude', 'navsat_status',
        'gimbal_heading', 'gimbal_roll', 'gimbal_pitch', 'camera_error_code'
    ]
    TERMINAL = {'completed', 'failed', 'canceled', 'interrupted'}

    def __init__(self, root: str, mission):
        self._lock = threading.RLock()
        self._started = time.monotonic()
        self.last_event_error = ''
        if (not mission.mission_id or Path(mission.mission_id).name != mission.mission_id
                or mission.mission_id in {'.', '..'} or '\\' in mission.mission_id):
            raise ValueError('mission_id must be a single safe directory component')
        base = Path(os.path.expanduser(root))
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.directory = base / f'{mission.mission_id}_{stamp}'
        self.directory.mkdir(parents=True, exist_ok=False)
        self.images_dir = self.directory / 'images'
        self.images_dir.mkdir()
        shutil.copy2(mission.source_file, self.directory / 'mission.yaml')
        self.csv_path = self.directory / 'captures.csv'
        self.jsonl_path = self.directory / 'captures.jsonl'
        self.events_path = self.directory / 'events.jsonl'
        self.manifest_path = self.directory / 'manifest.json'
        with self.csv_path.open('w', newline='', encoding='utf-8') as f:
            csv.DictWriter(f, fieldnames=self.FIELDS).writeheader()
        self.manifest = {
            'mission_id': mission.mission_id, 'map_id': mission.map_id,
            'source_file': mission.source_file, 'schema_version': 1,
            'created_local': stamp, 'started_at_utc': self._utc(),
            'status': 'running', 'completed_points': 0, 'capture_records': 0,
            'planned_points': len(mission.points),
            'planned_views': [
                {'point_id': p.id, 'view_tag': v.tag, 'required': v.required,
                 'save_image': v.save_image}
                for p in mission.points for v in p.views
            ],
        }
        self._write_manifest()
        self.event('mission_started')

    @staticmethod
    def _utc():
        return dt.datetime.now(dt.timezone.utc).isoformat()

    def _write_manifest(self, value=None):
        temporary = self.manifest_path.with_suffix('.json.tmp')
        with temporary.open('w', encoding='utf-8') as f:
            json.dump(self.manifest if value is None else value, f, ensure_ascii=False, indent=2)
            f.write('\n')
            f.flush()
            os.fsync(f.fileno())
        os.replace(temporary, self.manifest_path)

    def event(self, kind: str, **details):
        with self._lock:
            row = {'event': kind, 'at_utc': self._utc(),
                   'elapsed_sec': time.monotonic() - self._started, **details}
            with self.events_path.open('a', encoding='utf-8') as f:
                f.write(json.dumps(row, ensure_ascii=False, default=str) + '\n')
                f.flush()

    def adopt_image(self, source_path: str, point_id: str, view_tag: str) -> str:
        if not source_path:
            return ''
        source = Path(source_path).expanduser()
        if not source.is_file() or source.stat().st_size == 0:
            raise FileNotFoundError(f'camera image is missing/empty: {source}')
        suffix = source.suffix or '.jpg'
        safe_point = ''.join(c if c.isalnum() or c in '-_' else '_' for c in point_id)
        safe_view = ''.join(c if c.isalnum() or c in '-_' else '_' for c in view_tag)
        target_dir = self.images_dir / safe_point
        target_dir.mkdir(parents=True, exist_ok=True)
        target = target_dir / f'{safe_view}{suffix}'
        if target.exists():
            target = target_dir / f'{safe_view}_{time.time_ns()}{suffix}'
        try:
            os.link(source, target)
        except OSError:
            shutil.copy2(source, target)
        if not target.is_file() or target.stat().st_size == 0:
            raise OSError(f'image archive failed: {target}')
        return str(target)

    def append(self, record: dict) -> None:
        with self._lock:
            row = {key: record.get(key, '') for key in self.FIELDS}
            with self.csv_path.open('a', newline='', encoding='utf-8') as f:
                csv.DictWriter(f, fieldnames=self.FIELDS).writerow(row)
            with self.jsonl_path.open('a', encoding='utf-8') as f:
                f.write(json.dumps(record, ensure_ascii=False) + '\n')
            self.manifest['capture_records'] += 1

    def finalize(self, status, *, completed_points, error_code, message, **details):
        if status not in self.TERMINAL:
            raise ValueError(f'invalid terminal status: {status}')
        with self._lock:
            # Teardown/late callbacks cannot rewrite an already committed terminal result.
            if self.manifest['status'] in self.TERMINAL:
                return False
            updated = {**self.manifest,
                'status': status, 'finished_at_utc': self._utc(),
                'elapsed_sec': time.monotonic() - self._started,
                'completed_points': int(completed_points),
                'error_code': int(error_code), 'message': str(message), **details,
            }
            self._write_manifest(updated)
            self.manifest = updated
            # manifest.json is the authoritative, atomic terminal commit. A later
            # best-effort event append must not reverse an already committed result.
            try:
                self.event('mission_terminal', status=status, error_code=error_code,
                           message=message, completed_points=completed_points, **details)
            except OSError as exc:
                self.last_event_error = str(exc)
            return True
