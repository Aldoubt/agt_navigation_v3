"""Select a verified Map Package without requiring a running ROS graph."""

from __future__ import annotations

import argparse
from pathlib import Path

from .map_package import discover_packages, validate_package
from .promote_hmi_navigation_edit import DEFAULT_ACTIVE_STATE, DEFAULT_MAP_ROOT, _activate


def select(map_root: Path, map_id: str, map_version: str, active_state_file: Path) -> Path:
    """Verify exactly one package and atomically make it the active selection."""
    packages = list(discover_packages(map_root, verify_hashes=False))
    matches = [
        package for package in packages
        if package.map_id == map_id and package.map_version == map_version
    ]
    if len(matches) != 1:
        available = [
            f'{package.map_id}/{package.map_version}' for package in packages
            if package.valid and package.map_id and package.map_version
        ]
        available_text = ', '.join(available) if available else 'none'
        raise ValueError(
            f'expected one package for {map_id}/{map_version}, found {len(matches)}. '
            f'Available valid packages: {available_text}')

    info = validate_package(matches[0].metadata_path, verify_hashes=True)
    if not info.valid:
        raise ValueError(f'cannot select invalid package: {info.reason}')
    _activate(info, active_state_file.expanduser().resolve())
    return info.package_path


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='Validate and select an immutable AGT Map Package for the next navigation launch.')
    parser.add_argument('--map-id', required=True)
    parser.add_argument('--map-version', required=True)
    parser.add_argument('--map-root', default=str(DEFAULT_MAP_ROOT))
    parser.add_argument('--active-state-file', default=str(DEFAULT_ACTIVE_STATE))
    args = parser.parse_args(argv)
    try:
        package = select(
            Path(args.map_root).expanduser().resolve(),
            str(args.map_id).strip(),
            str(args.map_version).strip(),
            Path(args.active_state_file),
        )
    except ValueError as exc:
        parser.error(str(exc))
    print(f'Active map selected: {package}')


def main_list(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='List Map Packages available for selection without starting a ROS node.')
    parser.add_argument('--map-root', default=str(DEFAULT_MAP_ROOT))
    parser.add_argument(
        '--include-invalid', action='store_true',
        help='Also show metadata files that are not valid Map Packages.')
    args = parser.parse_args(argv)
    packages = list(discover_packages(
        Path(args.map_root).expanduser().resolve(), verify_hashes=False))
    shown = 0
    for package in packages:
        if not args.include_invalid and not package.valid:
            continue
        shown += 1
        print(f'{package.map_id}/{package.map_version}\t{package.reason}\t{package.package_path}')
    if shown == 0:
        print('No valid Map Packages found.')


if __name__ == '__main__':
    main()
