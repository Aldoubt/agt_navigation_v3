from pathlib import Path

import pytest

from agt_operator_console.profile import (
    CheckResult,
    CheckSpec,
    load_profile,
    mode_is_ready,
    readiness_rows,
)


def _check(name, required_for):
    return CheckSpec(
        name=name, kind='topic', ros_name=f'/{name}', ros_type='std_msgs/msg/String',
        required_for=frozenset(required_for), timeout_sec=1.0, min_messages=1,
    )


def test_required_failure_blocks_only_required_mode():
    checks = [_check('lidar', ['mapping', 'navigation']), _check('rtk', [])]
    results = [CheckResult('lidar', True, 'ok'), CheckResult('rtk', False, 'missing')]
    rows = readiness_rows(checks, results, 'mapping')
    assert [(row.name, row.state) for row in rows] == [('lidar', 'READY'), ('rtk', 'WARN')]
    assert mode_is_ready(rows)


def test_missing_required_result_blocks_mode():
    rows = readiness_rows([_check('lidar', ['mapping'])], [], 'mapping')
    assert rows[0].state == 'BLOCKED'
    assert not mode_is_ready(rows)


def test_profile_rejects_shell_command(tmp_path: Path):
    profile = tmp_path / 'bad.yaml'
    profile.write_text(
        'schema_version: 1\n'
        'drivers: []\nchecks: []\n'
        'mode_commands:\n  mapping: "ros2 launch x y"\n  navigation: [ros2, launch, x, y]\n'
        'map_root: /tmp/maps\npipeline_config: /tmp/pipeline.yaml\n',
        encoding='utf-8',
    )
    with pytest.raises(ValueError, match='mode mapping command'):
        load_profile(profile)
