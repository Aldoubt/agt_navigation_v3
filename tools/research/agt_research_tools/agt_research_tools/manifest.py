"""Create immutable experiment locks and verify every referenced asset byte."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from datetime import datetime, timezone

import yaml

REQUIRED_ASSETS = ('geometry_map', 'navigation_map', 'relocalization_assets', 'stable_mask', 'corridors',
                   'routes', 'parameters', 'calibration', 'sensor_geometry')


def digest(path):
    p = Path(path).resolve(strict=True)
    files = [p] if p.is_file() else sorted(x for x in p.rglob('*') if x.is_file())
    if not files:
        raise ValueError(f'asset is empty: {p}')
    entries = []
    for item in files:
        resolved = item.resolve(strict=True)
        if p.is_dir() and not resolved.is_relative_to(p):
            raise ValueError(f'asset symlink leaves frozen tree: {item}')
        h = hashlib.sha256()
        with resolved.open('rb') as stream:
            for block in iter(lambda: stream.read(1024*1024), b''):
                h.update(block)
        entries.append({'path': item.name if p.is_file() else item.relative_to(p).as_posix(),
                        'sha256': h.hexdigest(), 'bytes': item.stat().st_size})
    tree_hash = hashlib.sha256(json.dumps(entries, sort_keys=True,
                                         separators=(',', ':')).encode()).hexdigest()
    return {'path': str(p), 'sha256': tree_hash, 'files': entries}


def atomic_yaml(path, value):
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=p.parent, delete=False) as stream:
        tmp = Path(stream.name)
        try:
            yaml.safe_dump(value, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        except Exception:
            tmp.unlink(missing_ok=True)
            raise
    try:
        # Immutable publication; an existing lock is never replaced.
        os.link(tmp, p)
    finally:
        tmp.unlink(missing_ok=True)


def git(repo, *args):
    return subprocess.run(['git', '-C', str(repo), *args], check=True,
                          capture_output=True).stdout


def navigation_map_file(record, selected):
    root=Path(record['path'])
    if not root.is_dir():
        raise ValueError('navigation_map asset must include a directory containing YAML and image bytes')
    if not isinstance(selected,str) or not selected:
        raise ValueError('exact navigation_map_yaml required')
    path=Path(selected).expanduser()
    path=(path if path.is_absolute() else root/path).resolve(strict=True)
    if not path.is_relative_to(root) or path.relative_to(root).as_posix() not in {x['path'] for x in record['files']}:
        raise ValueError('navigation map YAML leaves frozen asset tree')
    config=yaml.safe_load(path.read_text())
    image=Path(config['image']).expanduser()
    image=(image if image.is_absolute() else path.parent/image).resolve(strict=True)
    if not image.is_relative_to(root) or image.relative_to(root).as_posix() not in {x['path'] for x in record['files']}:
        raise ValueError('navigation map image leaves frozen asset tree')
    return str(path)


def freeze(spec_file, output, allow_dirty=False):
    spec_path = Path(spec_file).resolve(strict=True)
    spec = yaml.safe_load(spec_path.read_text())
    if not isinstance(spec, dict):
        raise ValueError('experiment spec must be a mapping')
    for key in ('experiment_id', 'map_id', 'map_version', 'robot_profile'):
        if not isinstance(spec.get(key), str) or not spec[key].strip() or spec[key] == 'auto':
            raise ValueError(f'exact {key} required')
    if spec.get('schema_version') != 1:
        raise ValueError('unsupported spec version')
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f'frozen lock already exists: {output}')
    software_root = output.parent/(output.stem+'.software')
    if software_root.exists():
        raise FileExistsError(f'software snapshot already exists: {software_root}')
    # Validate every destination before writing snapshots: generated files may
    # not become part of the source tree they are pinning.
    for entry in spec.get('repositories', []):
        repo = Path(entry['path']).expanduser().resolve(strict=True)
        if output.is_relative_to(repo) or software_root.is_relative_to(repo):
            raise ValueError('experiment locks and software snapshots must be outside pinned repositories')
    assets = {}
    for name in REQUIRED_ASSETS:
        value = spec.get('assets', {}).get(name)
        if not isinstance(value, str) or not value:
            raise ValueError(f'assets.{name} required')
        path = Path(value).expanduser()
        path = path if path.is_absolute() else spec_path.parent/path
        assets[name] = digest(path)
        if name == 'geometry_map' and not path.is_file():
            raise ValueError('geometry_map must identify the exact frozen PCD file')
        if path.is_dir() and output.is_relative_to(path.resolve()):
            raise ValueError('manifest output may not be inside a hashed asset tree')
    repositories = []
    nav_yaml=navigation_map_file(assets['navigation_map'],spec.get('navigation_map_yaml'))
    names = set()
    for entry in spec.get('repositories', []):
        name = entry.get('name', '')
        if not name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in name) or name in names:
            raise ValueError('unique safe repository name required')
        names.add(name)
        repo = Path(entry['path']).expanduser().resolve(strict=True)
        dirty = bool(git(repo, 'status', '--porcelain').strip())
        if dirty and not allow_dirty:
            raise ValueError(f'dirty repository {repo}; pin clean code or explicitly use --allow-dirty')
        record = {'name': name, 'path': str(repo), 'commit': git(repo, 'rev-parse', 'HEAD').decode().strip(),
                  'dirty': dirty}
        if dirty:
            artifacts = software_root/name
            artifacts.mkdir(parents=True, exist_ok=False)
            patch = artifacts/'tracked.patch'
            patch.write_bytes(git(repo, 'diff', '--binary', 'HEAD'))
            untracked = git(repo, 'ls-files', '--others', '--exclude-standard', '-z').split(b'\0')
            for raw in untracked:
                if not raw:
                    continue
                relative = Path(os.fsdecode(raw))
                src = repo/relative
                if not src.resolve().is_relative_to(repo):
                    raise ValueError('untracked symlink leaves repository')
                dst = artifacts/'untracked'/relative
                dst.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(src, dst)
            record['snapshot'] = digest(artifacts)
        repositories.append(record)
    if not repositories:
        raise ValueError('at least one pinned software repository is required')
    value = {'schema_version': 1,
             'created_utc': datetime.now(timezone.utc).isoformat(),
             **{k: spec[k] for k in ('experiment_id', 'map_id', 'map_version', 'robot_profile')},
             'map_hash': assets['geometry_map']['files'][0]['sha256'],
             'navigation_map_yaml':nav_yaml,
             'assets': assets, 'repositories': repositories,
             'spec_sha256': digest(spec_path)['sha256']}
    value['bundle_hash'] = hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
    for name, record in assets.items():
        if digest(record['path'])['sha256'] != record['sha256']:
            raise ValueError(f'asset changed while freezing: {name}')
    atomic_yaml(output, value)
    return value


def verify(path, check_software=True):
    lock = yaml.safe_load(Path(path).read_text())
    if lock.get('schema_version') != 1 or not isinstance(lock.get('bundle_hash'), str):
        raise ValueError('invalid experiment manifest')
    content = dict(lock)
    expected = content.pop('bundle_hash')
    if hashlib.sha256(json.dumps(content, sort_keys=True).encode()).hexdigest() != expected:
        raise ValueError('manifest content hash mismatch')
    for name in REQUIRED_ASSETS:
        record = lock['assets'][name]
        if digest(record['path'])['sha256'] != record['sha256']:
            raise ValueError(f'asset hash mismatch: {name}')
    if lock.get('map_hash') != lock['assets']['geometry_map']['files'][0]['sha256']:
        raise ValueError('raw geometry map hash mismatch')
    navigation_map_file(lock['assets']['navigation_map'],lock.get('navigation_map_yaml'))
    for repo in lock.get('repositories', []):
        if 'snapshot' in repo and digest(repo['snapshot']['path'])['sha256'] != repo['snapshot']['sha256']:
            raise ValueError(f'software snapshot modified: {repo["name"]}')
        if check_software:
            if git(repo['path'], 'rev-parse', 'HEAD').decode().strip() != repo['commit']:
                raise ValueError(f'software commit changed: {repo["name"]}')
            dirty = bool(git(repo['path'], 'status', '--porcelain').strip())
            if dirty != repo['dirty']:
                raise ValueError(f'software dirty state changed: {repo["name"]}')
            if dirty:
                snapshot = Path(repo['snapshot']['path'])
                if git(repo['path'], 'diff', '--binary', 'HEAD') != (snapshot/'tracked.patch').read_bytes():
                    raise ValueError(f'working patch changed: {repo["name"]}')
                current = [os.fsdecode(x) for x in git(repo['path'], 'ls-files', '--others', '--exclude-standard', '-z').split(b'\0') if x]
                stored_dir = snapshot/'untracked'
                stored = sorted(p.relative_to(stored_dir).as_posix() for p in stored_dir.rglob('*') if p.is_file())
                if sorted(current) != stored:
                    raise ValueError(f'untracked software files changed: {repo["name"]}')
                for relative in current:
                    if digest(Path(repo['path'])/relative)['files'][0]['sha256'] != digest(stored_dir/relative)['files'][0]['sha256']:
                        raise ValueError(f'untracked software content changed: {relative}')
    return lock


def freeze_main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', required=True); parser.add_argument('--output', required=True)
    parser.add_argument('--allow-dirty', action='store_true')
    args = parser.parse_args()
    try:
        value = freeze(args.spec, args.output, args.allow_dirty)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as exc:
        parser.exit(2, f'{exc}\n')
    print(json.dumps({'manifest': str(Path(args.output).resolve()), 'bundle_hash': value['bundle_hash']}))


def verify_main():
    parser = argparse.ArgumentParser(description='Verify immutable experiment inputs')
    parser.add_argument('manifest'); parser.add_argument('--assets-only', action='store_true')
    args = parser.parse_args()
    try:
        value = verify(args.manifest, not args.assets_only)
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as exc:
        parser.exit(2, f'{exc}\n')
    print(json.dumps({'valid': True, 'bundle_hash': value['bundle_hash']}))
