"""Build mapping candidates and atomically promote validated, immutable maps."""

from __future__ import annotations

import argparse
import fcntl
import os
import shutil
import tempfile
from datetime import datetime, timezone
from pathlib import Path

import yaml

from .create_map_package import build_package
from .map_package import sha256_file, sha256_tree, validate_package
from .promote_hmi_navigation_edit import _atomic_write_yaml, _activate


EXPERIMENTS_ROOT = Path('/home/yangxuan/ros2_ws/experiments')
DEFAULT_CANDIDATES = EXPERIMENTS_ROOT / 'mapping/candidates'
DEFAULT_MAP_ROOT = Path('/home/yangxuan/ros2_ws/maps')
STATES = ('MAPPING', 'BUILT', 'VALIDATING', 'VALIDATED',
          'REVIEW_REQUIRED', 'REJECTED', 'PROMOTED')


def build_candidate(map_id: str, version: str, source_pcd: Path,
                    navigation_dir: Path, relocalization_dir: Path,
                    candidate_root: Path = DEFAULT_CANDIDATES) -> Path:
    candidate_root = Path(candidate_root).expanduser().resolve()
    if not candidate_root.is_relative_to(
            EXPERIMENTS_ROOT.resolve()):
        raise ValueError('candidate root must be inside experiments/')
    path = build_package(candidate_root, map_id, version, source_pcd,
                         navigation_dir, relocalization_assets_dir=relocalization_dir)
    _atomic_write_yaml(path / 'candidate_state.yaml', {
        'schema_version': 1, 'state': 'BUILT',
        'built_utc': datetime.now(timezone.utc).isoformat(),
        'source_pcd_sha256': sha256_file(Path(source_pcd)),
    })
    return path


def _set_state(path: Path, state: str, error: str = '') -> None:
    if state not in STATES:
        raise ValueError(f'unknown candidate state: {state}')
    current = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    current.update({'state': state, 'updated_utc': datetime.now(timezone.utc).isoformat()})
    if error:
        current['error'] = error
    _atomic_write_yaml(path, current)



def _restore_active_state(path: Path, previous: bytes | None) -> None:
    """Restore exact pre-activation pointer bytes with an atomic replacement."""
    if previous is None:
        path.unlink(missing_ok=True)
        return
    fd, temporary = tempfile.mkstemp(prefix=f'.{path.name}.rollback.', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(previous)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def promote_candidate(candidate: Path, map_root: Path = DEFAULT_MAP_ROOT,
                      robot_profile: str = 'bunker_v1', activate: bool = False) -> Path:
    candidate = Path(candidate).expanduser().resolve()
    map_root = Path(map_root).expanduser().resolve()
    if not candidate.is_relative_to(EXPERIMENTS_ROOT.resolve()):
        raise ValueError('candidate must be inside experiments/')
    state_path = candidate / 'candidate_state.yaml'
    state = yaml.safe_load(state_path.read_text(encoding='utf-8')) or {}
    if state.get('state') not in ('BUILT', 'VALIDATED'):
        raise ValueError('candidate must be BUILT or VALIDATED before promotion')
    registered = False
    _set_state(state_path, 'VALIDATING')
    try:
        info = validate_package(candidate / 'metadata.yaml', verify_hashes=True)
        if not info.valid or not info.asset_path('relocalization_assets'):
            raise ValueError(f'candidate invalid or missing relocalization assets: {info.reason}')
        if (candidate / 'quality' / 'report.yaml').exists():
            quality = yaml.safe_load((candidate / 'quality' / 'report.yaml').read_text(encoding='utf-8'))
            if not isinstance(quality, dict) or quality.get('status') != 'pass':
                raise ValueError('candidate quality report must pass')
        if not robot_profile or '/' in robot_profile or robot_profile in ('.', '..'):
            raise ValueError('invalid robot profile')
        _set_state(state_path, 'VALIDATED')

        map_root.mkdir(parents=True, exist_ok=True)
        lock_path = map_root / '.promotion.lock'
        with lock_path.open('a+') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            parent = map_root / info.map_id
            parent.mkdir(parents=True, exist_ok=True)
            destination = parent / info.map_version
            if destination.exists():
                raise FileExistsError(f'published map version already exists: {destination}')
            staging = Path(tempfile.mkdtemp(prefix=f'.{info.map_version}.staging.', dir=parent))
            published = False
            try:
                shutil.copytree(candidate, staging, dirs_exist_ok=True)
                (staging / 'candidate_state.yaml').unlink()
                nav = staging / 'navigation'
                profile_nav = nav / robot_profile
                profile_nav.mkdir()
                for item in list(nav.iterdir()):
                    if item != profile_nav:
                        shutil.move(str(item), str(profile_nav / item.name))
                metadata_path = staging / 'metadata.yaml'
                metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
                nav_rel = f'navigation/{robot_profile}/map.yaml'
                metadata['assets']['navigation_map']['path'] = nav_rel
                metadata['assets']['navigation_map']['sha256'] = sha256_file(staging / nav_rel)
                for asset in metadata['assets'].values():
                    if asset['path'].startswith('navigation/') and asset['path'] != nav_rel:
                        asset['path'] = f'navigation/{robot_profile}/' + asset['path'][len('navigation/'):]
                metadata['compatibility'] = {'robot_profiles': [robot_profile]}
                metadata['navigation'] = {robot_profile: {'map': nav_rel}}
                _atomic_write_yaml(metadata_path, metadata)
                checked = validate_package(metadata_path, verify_hashes=True)
                if not checked.valid:
                    raise ValueError(f'promoted staging package invalid: {checked.reason}')
                package_hash = sha256_tree(staging)
                registry_path = map_root / 'registry.yaml'
                if registry_path.exists():
                    registry = yaml.safe_load(registry_path.read_text(encoding='utf-8')) or {}
                    if registry.get('schema_version') != 1 or not isinstance(registry.get('maps'), dict):
                        raise ValueError('existing registry is invalid')
                else:
                    registry = {'schema_version': 1, 'default_map': 'latest_validated',
                                'default_map_id': info.map_id, 'maps': {}}
                entry = registry['maps'].setdefault(
                    info.map_id, {'active': '', 'latest_validated': '', 'versions': {}})
                versions = entry.setdefault('versions', {})
                if info.map_version in versions:
                    raise FileExistsError('map version already registered')
                versions[info.map_version] = {
                    'status': 'VALIDATED',
                    'validated_utc': datetime.now(timezone.utc).isoformat(),
                    'package_sha256': package_hash,
                }
                entry['latest_validated'] = info.map_version
                previous_active = entry.get('active', '')
                os.rename(staging, destination)
                published = True
                try:
                    _atomic_write_yaml(registry_path, registry)
                except Exception:
                    shutil.rmtree(destination)
                    raise
                registered = True
                if activate:
                    # A published version stays VALIDATED even if activation fails.
                    # Commit the active registry pointer only after the state file.
                    active_path = map_root / 'active_map.yaml'
                    previous_state = (active_path.read_bytes()
                                      if active_path.exists() else None)
                    try:
                        _activate(validate_package(
                            destination / 'metadata.yaml', verify_hashes=True),
                            active_path)
                        entry['active'] = info.map_version
                        _atomic_write_yaml(registry_path, registry)
                    except Exception as exc:
                        rollback_errors = []
                        try:
                            current_state = (active_path.read_bytes()
                                             if active_path.exists() else None)
                            if current_state != previous_state:
                                _restore_active_state(active_path, previous_state)
                        except Exception as restore_exc:
                            rollback_errors.append(f'active pointer: {restore_exc}')
                        try:
                            saved = yaml.safe_load(registry_path.read_text(
                                encoding='utf-8')) or {}
                            saved_entry = (saved.get('maps') or {}).get(info.map_id) or {}
                            if saved_entry.get('active') != previous_active:
                                entry['active'] = previous_active
                                _atomic_write_yaml(registry_path, registry)
                        except Exception as restore_exc:
                            rollback_errors.append(f'registry: {restore_exc}')
                        if rollback_errors:
                            raise RuntimeError(
                                'map activation failed; rollback incomplete; '
                                'manual reconciliation required: '
                                + '; '.join(rollback_errors)) from exc
                        raise
            finally:
                if not published and staging.exists():
                    shutil.rmtree(staging)
        _set_state(state_path, 'PROMOTED')
        return destination
    except Exception as exc:
        if state_path.exists():
            _set_state(state_path, 'PROMOTED' if registered else 'REJECTED',
                       str(exc))
        raise


def main_build(argv=None):
    parser = argparse.ArgumentParser(description='Build a candidate Map Package in experiments/')
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--map-version', required=True)
    parser.add_argument('--source-pcd', required=True)
    parser.add_argument('--navigation-dir', required=True)
    parser.add_argument('--relocalization-assets-dir', required=True)
    parser.add_argument('--candidate-root', default=str(DEFAULT_CANDIDATES))
    args = parser.parse_args(argv)
    print(build_candidate(args.map_id, args.map_version, Path(args.source_pcd),
                          Path(args.navigation_dir), Path(args.relocalization_assets_dir),
                          Path(args.candidate_root)))


def main_promote(argv=None):
    parser = argparse.ArgumentParser(description='Promote a validated mapping candidate')
    parser.add_argument('--candidate', required=True)
    parser.add_argument('--map-root', default=str(DEFAULT_MAP_ROOT))
    parser.add_argument('--robot', default='bunker_v1')
    parser.add_argument('--activate', action='store_true')
    args = parser.parse_args(argv)
    print(promote_candidate(Path(args.candidate), Path(args.map_root), args.robot,
                            bool(args.activate)))
