"""Interactive parent process for persistent sensor and AGT mode sessions."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from datetime import datetime

from .processes import ManagedProcess
from .profile import Profile, load_profile, mode_is_ready, readiness_rows


DEFAULT_ACTIVE_STATE = Path('/home/yangxuan/ros2_ws/agt_data/maps/active_map.yaml')
DEFAULT_MAPPING_ROOT = Path('/home/yangxuan/ros2_ws/agt_data/mapping_runs')


def pgo_save_command(destination: Path) -> tuple[str, ...]:
    request = f"{{file_path: '{destination}', save_patches: true}}"
    return ('ros2', 'service', 'call', '/pgo/save_maps', 'interface/srv/SaveMaps', request)


def relocalization_assets_command(source_pcd: Path, output: Path) -> tuple[str, ...]:
    return (
        'ros2', 'run', 'agt_global_relocalization_native', 'build_relocalization_assets',
        '--map', str(source_pcd), '--output', str(output),
    )


def relocalization_candidates_command(mapping_dir: Path, output: Path) -> tuple[str, ...]:
    return (
        'ros2', 'run', 'agt_global_relocalization_native', 'build_relocalization_candidates',
        '--map-dir', str(mapping_dir), '--output', str(output),
    )


def package_generation_command(
    profile: Profile,
    map_id: str,
    map_version: str,
    source_pcd: Path,
    relocalization_assets: Path,
    trajectory_poses: Path,
) -> tuple[str, ...]:
    return (
        'ros2', 'run', 'agt_map_manager', 'generate_map_package',
        '--map-root', str(profile.map_root),
        '--map-id', map_id,
        '--map-version', map_version,
        '--source-pcd', str(source_pcd),
        '--pipeline-config', str(profile.pipeline_config),
        '--relocalization-assets-dir', str(relocalization_assets),
        '--trajectory-poses', str(trajectory_poses),
    )


def map_selection_command(profile: Profile, map_id: str, map_version: str) -> tuple[str, ...]:
    return (
        'ros2', 'run', 'agt_map_manager', 'select_map_package',
        '--map-root', str(profile.map_root),
        '--active-state-file', str(DEFAULT_ACTIVE_STATE),
        '--map-id', map_id,
        '--map-version', map_version,
    )


def map_list_command(profile: Profile) -> tuple[str, ...]:
    return (
        'ros2', 'run', 'agt_map_manager', 'list_map_packages',
        '--map-root', str(profile.map_root),
    )


def map_editor_command(profile: Profile, map_id: str, map_version: str) -> tuple[str, ...]:
    return (
        'ros2', 'launch', 'agt_system_bringup', 'hmi_map_editor.launch.py',
        f'map_id:={map_id}', f'map_version:={map_version}',
        f'map_root:={profile.map_root}',
    )


def parse_map_list(output: str) -> list[tuple[str, str]]:
    """Parse the stable tab-delimited list_map_packages CLI output."""
    packages: list[tuple[str, str]] = []
    for line in output.splitlines():
        key = line.split('\t', 1)[0].strip()
        if '/' not in key:
            continue
        map_id, map_version = key.rsplit('/', 1)
        if map_id and map_version:
            packages.append((map_id, map_version))
    return packages


def wait_for_stable_files(paths: list[Path], timeout_sec: float = 20.0) -> None:
    """Require two consecutive unchanged samples before using PGO output."""
    deadline = time.monotonic() + timeout_sec
    previous = None
    while time.monotonic() < deadline:
        if all(path.is_file() for path in paths):
            snapshot = tuple((path.stat().st_size, path.stat().st_mtime_ns) for path in paths)
            if snapshot == previous and all(size > 0 for size, _ in snapshot):
                return
            previous = snapshot
        time.sleep(0.5)
    missing = [str(path) for path in paths if not path.is_file()]
    raise RuntimeError(f'PGO artifacts did not stabilize; missing={missing}')


class OperatorConsole:
    def __init__(self, profile: Profile, *, input_fn=input) -> None:
        self.profile = profile
        self.input = input_fn
        self.sensor_processes: list[ManagedProcess] = []
        self._interrupt_requested = False

    def start_sensors(self) -> None:
        if self.sensor_processes:
            return
        for driver in self.profile.drivers:
            if not driver.enabled:
                continue
            process = ManagedProcess(driver.name, driver.command)
            print(f'[sensor] start {driver.name}: {" ".join(driver.command)}')
            process.start()
            self.sensor_processes.append(process)

    def stop_sensors(self) -> None:
        for process in reversed(self.sensor_processes):
            print(f'[sensor] stop {process.name}')
            process.stop()
        self.sensor_processes.clear()

    def preflight(self, mode: str, *, run_mode_checks: bool = True) -> bool:
        from .preflight import run_preflight

        results = run_preflight(self.profile.checks)
        rows = readiness_rows(self.profile.checks, results, mode)
        print('\nSensor preflight')
        print(f"{'Sensor':<24} {'State':<9} Reason")
        print('-' * 72)
        for row in rows:
            print(f'{row.name:<24} {row.state:<9} {row.reason}')
        ready = mode_is_ready(rows)
        print('RESULT: READY' if ready else 'RESULT: BLOCKED')
        if ready and run_mode_checks:
            for command in self.profile.mode_preflight_commands.get(mode, ()):
                print(f'[preflight] {" ".join(command)}')
                subprocess.run(command, check=True)
        return ready

    def _run_child(self, name: str, command: tuple[str, ...], *, mapping: bool = False) -> str:
        child = ManagedProcess(name, command)
        self._interrupt_requested = False

        def on_sigint(_signum, _frame) -> None:
            self._interrupt_requested = True

        previous = signal.signal(signal.SIGINT, on_sigint)
        try:
            child.start()
            print(f'[{name}] running; Ctrl-C requests an operator decision.')
            while child.running():
                time.sleep(0.1)
                if not self._interrupt_requested:
                    continue
                self._interrupt_requested = False
                if not mapping:
                    child.stop()
                    return 'stopped'
                decision = self._mapping_interrupt_menu()
                if decision == 'continue':
                    continue
                if decision == 'save':
                    self._save_mapping()
                    child.stop()
                    return 'saved'
                child.stop()
                return 'discarded'
            return 'exited'
        finally:
            signal.signal(signal.SIGINT, previous)
            child.stop()

    def _mapping_interrupt_menu(self) -> str:
        while True:
            answer = self.input('\nMapping interrupted: [s]ave, [d]iscard, [c]ontinue: ').strip().lower()
            if answer in ('s', 'save'):
                return 'save'
            if answer in ('d', 'discard'):
                return 'discard'
            if answer in ('c', 'continue', ''):
                return 'continue'
            print('Choose s, d, or c.')

    def _save_mapping(self) -> None:
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        default_dir = DEFAULT_MAPPING_ROOT / stamp
        raw_dir = self.input(f'PGO output directory [{default_dir}]: ').strip()
        output_dir = Path(raw_dir).expanduser() if raw_dir else default_dir
        output_dir.mkdir(parents=True, exist_ok=True)

        print('[mapping] requesting final PGO save')
        subprocess.run(pgo_save_command(output_dir), check=True)
        source_pcd = output_dir / 'map.pcd'
        poses = output_dir / 'poses.txt'
        wait_for_stable_files([source_pcd, poses])

        relocalization = output_dir / 'relocalization'
        print('[mapping] building relocalization assets from final PGO PCD')
        subprocess.run(relocalization_assets_command(source_pcd, relocalization), check=True)
        patches = output_dir / 'patches'
        if not patches.is_dir() or not any(patches.glob('*.pcd')):
            raise RuntimeError(
                'PGO save did not produce patches/*.pcd required for relocalization candidates')
        print('[mapping] building relocalization candidate database from PGO keyframes')
        subprocess.run(relocalization_candidates_command(output_dir, relocalization), check=True)

        map_id = self.input('Map ID: ').strip()
        map_version = self.input('Generated map version [v001-generated]: ').strip() or 'v001-generated'
        if not map_id:
            raise RuntimeError('Map ID is required; generated package was not published')
        if not self.profile.pipeline_config.is_file():
            raise RuntimeError(f'pipeline config does not exist: {self.profile.pipeline_config}')

        print('[mapping] generating immutable, non-active Map Package')
        subprocess.run(
            package_generation_command(
                self.profile, map_id, map_version, source_pcd, relocalization, poses),
            check=True,
        )
        print(
            f'[mapping] package {map_id}/{map_version} is ready but not active. '
            'Create an HMI Edit Session, publish an approved version, then select it before navigation.')

    def mapping(self) -> int:
        if not self.preflight('mapping'):
            return 2
        result = self._run_child('mapping', self.profile.mode_commands['mapping'], mapping=True)
        print(f'[mapping] {result}')
        return 0 if result in ('saved', 'discarded', 'stopped', 'exited') else 1

    def navigation(
        self,
        mode: str = 'navigation',
        *,
        map_id: str = '',
        map_version: str = '',
    ) -> int:
        if not self.preflight(mode):
            return 2
        if bool(map_id) != bool(map_version):
            raise RuntimeError('--map-id and --map-version must be supplied together')
        if not map_id:
            map_id, map_version = self._choose_map()
        subprocess.run(map_selection_command(self.profile, map_id, map_version), check=True)
        subprocess.run(
            ('ros2', 'run', 'agt_map_manager', 'validate_active_map',
             '--active-state-file', str(DEFAULT_ACTIVE_STATE)),
            check=True,
        )
        result = self._run_child(mode, self.profile.mode_commands[mode])
        print(f'[{mode}] {result}')
        return 0

    def _choose_map(self) -> tuple[str, str]:
        """Require an exact operator choice before map-dependent operation."""
        completed = subprocess.run(
            map_list_command(self.profile), check=True, text=True, capture_output=True)
        packages = parse_map_list(completed.stdout)
        if not packages:
            raise RuntimeError('no valid Map Packages are available for navigation')

        print('\nPrepared Map Packages')
        for index, (candidate_id, candidate_version) in enumerate(packages, start=1):
            print(f'  {index}. {candidate_id}/{candidate_version}')
        while True:
            answer = self.input('Select map number (q to cancel): ').strip().lower()
            if answer in ('q', 'quit', 'cancel'):
                raise RuntimeError('navigation map selection cancelled by operator')
            try:
                selected = int(answer)
            except ValueError:
                selected = 0
            if 1 <= selected <= len(packages):
                return packages[selected - 1]
            print(f'Choose a number between 1 and {len(packages)}, or q to cancel.')

    def edit(self, *, map_id: str = '', map_version: str = '') -> int:
        if bool(map_id) != bool(map_version):
            raise RuntimeError('--map-id and --map-version must be supplied together')
        if not map_id:
            map_id, map_version = self._choose_map()
        result = self._run_child(
            'map_editor', map_editor_command(self.profile, map_id, map_version))
        print(f'[map_editor] {result}')
        return 0

    def sensors(self, watch_interval_sec: float) -> int:
        """Own drivers in this terminal and keep reporting connection readiness."""
        self.start_sensors()
        try:
            while True:
                self.preflight('mapping', run_mode_checks=False)
                print(
                    '[sensors] Next connection check in '
                    f'{watch_interval_sec:g}s. Ctrl-C stops the sensor session.')
                time.sleep(watch_interval_sec)
        except KeyboardInterrupt:
            print('\n[sensors] Stop requested.')
            return 0

    def close(self) -> None:
        self.stop_sensors()


def _default_profile_path() -> Path:
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory('agt_operator_console')) / 'config' / 'operator_profile.yaml'


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='Start the sensor session, mapping, or an AGT field mode.')
    parser.add_argument('--profile', type=Path, default=_default_profile_path())
    parser.add_argument('--map-id', default='', help='Exact package ID to select before navigation.')
    parser.add_argument('--map-version', default='', help='Exact package version to select before navigation.')
    parser.add_argument(
        '--watch-interval-sec', type=float, default=8.0,
        help='Sensor connection status refresh interval (default: 8).')
    parser.add_argument(
        'mode', nargs='?', choices=('sensors', 'mapping', 'edit', 'navigation', 'inspection'), default='sensors')
    args = parser.parse_args(argv)
    if args.watch_interval_sec <= 0:
        parser.error('--watch-interval-sec must be greater than zero')
    try:
        console = OperatorConsole(load_profile(args.profile))
        try:
            if args.mode == 'sensors':
                result = console.sensors(args.watch_interval_sec)
            elif args.mode == 'mapping':
                result = console.mapping()
            elif args.mode == 'edit':
                result = console.edit(map_id=args.map_id.strip(), map_version=args.map_version.strip())
            else:
                result = console.navigation(
                    args.mode, map_id=args.map_id.strip(), map_version=args.map_version.strip())
        finally:
            console.close()
    except (OSError, RuntimeError, subprocess.CalledProcessError, ValueError) as exc:
        print(f'operator console failed: {exc}', file=sys.stderr)
        raise SystemExit(2)
    raise SystemExit(result)
