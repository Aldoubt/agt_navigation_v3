#!/usr/bin/env python3
"""Audit whether every ROS package manifest is included in a Git checkout."""

import argparse
from pathlib import Path
import subprocess


def run(*args, cwd=None):
    return subprocess.run(args, cwd=cwd, text=True, capture_output=True, check=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    args = parser.parse_args()
    root = args.source_root.expanduser().resolve()
    listing = run('colcon', 'list', '--base-paths', str(root), '--paths')
    if listing.returncode:
        parser.error(listing.stderr.strip() or 'colcon list failed')

    failures = []
    repositories = set()
    packages = 0
    for line in listing.stdout.splitlines():
        name, package_path, *_ = line.split('\t')
        packages += 1
        package = Path(package_path)
        owner = run('git', 'rev-parse', '--show-toplevel', cwd=package)
        if owner.returncode:
            failures.append(f'UNVERSIONED {name}: {package}')
            continue
        git_root = Path(owner.stdout.strip())
        repositories.add(git_root)
        manifest = package / 'package.xml'
        if manifest.is_symlink():
            # Livox ROS 2's bootstrap creates a package.xml link to its tracked
            # package_ROS2.xml. Check the versioned target, not the local link.
            manifest = manifest.resolve()
        elif not manifest.is_file():
            # colcon also discovers plain CMake packages such as Sophus/BBS.
            manifest = package / 'CMakeLists.txt'
        if not manifest.is_relative_to(git_root):
            failures.append(f'EXTERNAL_MANIFEST {name}: {manifest}')
            continue
        tracked = run('git', 'ls-files', '--error-unmatch', '--',
                      str(manifest.relative_to(git_root)), cwd=git_root)
        if tracked.returncode:
            failures.append(f'UNTRACKED_MANIFEST {name}: {manifest}')

    print(f'ROS packages: {packages}; Git roots: {len(repositories)}')
    for problem in failures:
        print(problem)
    if failures:
        print('SOURCE_OWNERSHIP FAIL: commit and publish source before migration')
        return 1
    print('SOURCE_OWNERSHIP PASS: manifests tracked locally; check push and clean clone separately')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
