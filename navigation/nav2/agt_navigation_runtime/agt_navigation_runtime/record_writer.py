from __future__ import annotations

from pathlib import Path
import csv
import json
import os
import shutil
import time


class RecordWriter:
    FIELDS = [
        'mission_id', 'map_id', 'point_id', 'view_tag', 'image_path',
        'image_sec', 'image_nanosec', 'pose_valid', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw',
        'rtk_valid', 'rtk_age_sec', 'latitude', 'longitude', 'altitude', 'navsat_status',
        'gimbal_heading', 'gimbal_roll', 'gimbal_pitch', 'camera_error_code'
    ]

    def __init__(self, root: str, mission):
        base = Path(os.path.expanduser(root))
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.directory = base / f'{mission.mission_id}_{stamp}'
        self.directory.mkdir(parents=True, exist_ok=False)
        self.images_dir = self.directory / 'images'
        self.images_dir.mkdir()
        shutil.copy2(mission.source_file, self.directory / 'mission.yaml')
        self.csv_path = self.directory / 'captures.csv'
        self.jsonl_path = self.directory / 'captures.jsonl'
        with self.csv_path.open('w', newline='', encoding='utf-8') as f:
            csv.DictWriter(f, fieldnames=self.FIELDS).writeheader()
        (self.directory / 'manifest.json').write_text(json.dumps({
            'mission_id': mission.mission_id,
            'map_id': mission.map_id,
            'source_file': mission.source_file,
            'schema_version': 1,
            'created_local': stamp,
        }, ensure_ascii=False, indent=2), encoding='utf-8')

    def adopt_image(self, source_path: str, point_id: str, view_tag: str) -> str:
        if not source_path:
            return ''
        source = Path(source_path).expanduser()
        if not source.exists() or not source.is_file():
            return str(source)
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
        return str(target)

    def append(self, record: dict) -> None:
        row = {key: record.get(key, '') for key in self.FIELDS}
        with self.csv_path.open('a', newline='', encoding='utf-8') as f:
            csv.DictWriter(f, fieldnames=self.FIELDS).writerow(row)
        with self.jsonl_path.open('a', encoding='utf-8') as f:
            f.write(json.dumps(record, ensure_ascii=False) + '\n')
