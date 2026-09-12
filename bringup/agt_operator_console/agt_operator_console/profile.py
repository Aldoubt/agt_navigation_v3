"""Declarative profile parsing and readiness policy for operator modes."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


VALID_MODES = frozenset({'mapping', 'navigation', 'inspection'})
VALID_CHECK_KINDS = frozenset({'topic', 'action'})


@dataclass(frozen=True)
class DriverSpec:
    name: str
    command: tuple[str, ...]
    enabled: bool = True


@dataclass(frozen=True)
class CheckSpec:
    name: str
    kind: str
    ros_name: str
    ros_type: str
    required_for: frozenset[str]
    timeout_sec: float
    min_messages: int = 1


@dataclass(frozen=True)
class Profile:
    drivers: tuple[DriverSpec, ...]
    checks: tuple[CheckSpec, ...]
    mode_commands: dict[str, tuple[str, ...]]
    mode_preflight_commands: dict[str, tuple[tuple[str, ...], ...]]
    map_root: Path
    pipeline_config: Path


@dataclass(frozen=True)
class CheckResult:
    name: str
    ready: bool
    reason: str


@dataclass(frozen=True)
class ReadinessRow:
    name: str
    state: str
    required: bool
    reason: str


def _command(value: Any, label: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not value or not all(isinstance(item, str) and item for item in value):
        raise ValueError(f'{label} must be a non-empty list of command arguments')
    return tuple(value)


def _modes(value: Any, label: str, *, allow_empty: bool = False) -> frozenset[str]:
    if not isinstance(value, list) or (not value and not allow_empty):
        raise ValueError(f'{label} must list at least one mode')
    result = frozenset(str(item).strip() for item in value)
    unknown = result - VALID_MODES
    if unknown or '' in result:
        raise ValueError(f'{label} has unsupported modes: {sorted(unknown or {""})}')
    return result


def load_profile(path: Path) -> Profile:
    path = path.expanduser().resolve()
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f'operator profile is unreadable: {exc}') from exc
    if not isinstance(data, dict) or data.get('schema_version') != 1:
        raise ValueError('operator profile requires schema_version: 1')

    drivers: list[DriverSpec] = []
    names: set[str] = set()
    for entry in data.get('drivers', []):
        if not isinstance(entry, dict):
            raise ValueError('every driver entry must be a mapping')
        name = str(entry.get('name', '')).strip()
        if not name or name in names:
            raise ValueError(f'invalid or duplicate driver name: {name!r}')
        names.add(name)
        drivers.append(DriverSpec(
            name=name,
            command=_command(entry.get('command'), f'driver {name} command'),
            enabled=bool(entry.get('enabled', True)),
        ))

    checks: list[CheckSpec] = []
    names.clear()
    for entry in data.get('checks', []):
        if not isinstance(entry, dict):
            raise ValueError('every check entry must be a mapping')
        name = str(entry.get('name', '')).strip()
        kind = str(entry.get('kind', '')).strip()
        ros_name = str(entry.get('name_or_topic', '')).strip()
        ros_type = str(entry.get('type', '')).strip()
        if not name or name in names:
            raise ValueError(f'invalid or duplicate check name: {name!r}')
        if kind not in VALID_CHECK_KINDS:
            raise ValueError(f'check {name} kind must be one of {sorted(VALID_CHECK_KINDS)}')
        if not ros_name.startswith('/') or not ros_type:
            raise ValueError(f'check {name} requires an absolute name_or_topic and ROS type')
        timeout = float(entry.get('timeout_sec', 5.0))
        minimum = int(entry.get('min_messages', 1))
        if timeout <= 0 or minimum < 1:
            raise ValueError(f'check {name} has invalid timeout_sec or min_messages')
        names.add(name)
        checks.append(CheckSpec(
            name=name,
            kind=kind,
            ros_name=ros_name,
            ros_type=ros_type,
            required_for=_modes(
                entry.get('required_for', []), f'check {name} required_for', allow_empty=True),
            timeout_sec=timeout,
            min_messages=minimum,
        ))

    modes_raw = data.get('mode_commands')
    if not isinstance(modes_raw, dict):
        raise ValueError('operator profile mode_commands must be a mapping')
    mode_commands: dict[str, tuple[str, ...]] = {}
    for mode, command in modes_raw.items():
        mode = str(mode).strip()
        if mode not in VALID_MODES:
            raise ValueError(f'unsupported mode command: {mode!r}')
        mode_commands[mode] = _command(command, f'mode {mode} command')
    for required in ('mapping', 'navigation'):
        if required not in mode_commands:
            raise ValueError(f'operator profile is missing mode command: {required}')

    preflight_raw = data.get('mode_preflight_commands', {})
    if not isinstance(preflight_raw, dict):
        raise ValueError('operator profile mode_preflight_commands must be a mapping')
    mode_preflight_commands: dict[str, tuple[tuple[str, ...], ...]] = {}
    for mode, commands in preflight_raw.items():
        mode = str(mode).strip()
        if mode not in VALID_MODES or not isinstance(commands, list):
            raise ValueError(f'invalid preflight command list for mode: {mode!r}')
        mode_preflight_commands[mode] = tuple(
            _command(command, f'mode {mode} preflight command') for command in commands)

    map_root = Path(str(data.get('map_root', '')).strip()).expanduser()
    pipeline_config = Path(str(data.get('pipeline_config', '')).strip()).expanduser()
    if not str(map_root) or not str(pipeline_config):
        raise ValueError('operator profile requires map_root and pipeline_config')
    return Profile(
        drivers=tuple(drivers),
        checks=tuple(checks),
        mode_commands=mode_commands,
        mode_preflight_commands=mode_preflight_commands,
        map_root=map_root,
        pipeline_config=pipeline_config,
    )


def readiness_rows(
    checks: Iterable[CheckSpec],
    results: Iterable[CheckResult],
    mode: str,
) -> list[ReadinessRow]:
    if mode not in VALID_MODES:
        raise ValueError(f'unsupported mode: {mode}')
    by_name = {result.name: result for result in results}
    rows: list[ReadinessRow] = []
    for check in checks:
        result = by_name.get(check.name, CheckResult(check.name, False, 'no_result'))
        required = mode in check.required_for
        state = 'READY' if result.ready else ('BLOCKED' if required else 'WARN')
        rows.append(ReadinessRow(check.name, state, required, result.reason))
    return rows


def mode_is_ready(rows: Iterable[ReadinessRow]) -> bool:
    return all(row.state != 'BLOCKED' for row in rows)
