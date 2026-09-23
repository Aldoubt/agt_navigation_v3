"""Read-only ROS domain and Nav2 lifecycle diagnosis."""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List

from .ros_graph_discovery import RosGraph, _run_ros2, discover_ros_graph


@dataclass
class RuntimeDiagnosis:
    ros_domain_id: str
    ros_domain_id_source: str
    lifecycle_nodes: List[str] = field(default_factory=list)
    lifecycle_states: Dict[str, str] = field(default_factory=dict)
    node_statuses: Dict[str, str] = field(default_factory=dict)
    node_details: Dict[str, str] = field(default_factory=dict)
    base_status: str = 'MISSING'
    localization_status: str = 'MISSING'
    nav2_status: str = 'MISSING'
    command_errors: Dict[str, str] = field(default_factory=dict)


def _last_state_line(output: str) -> str:
    for line in reversed(output.splitlines()):
        line = line.strip()
        if line:
            return line
    return ''


def _state_is_inactive(state: str) -> bool:
    state = state.lower()
    return any(value in state for value in ('unconfigured', 'inactive', 'activating', 'deactivating', 'error', 'finalized'))


def _state_is_active(state: str) -> bool:
    normalized = state.lower().strip().split()[0] if state.strip() else ''
    return normalized == 'active'


def _matching_topic(graph: RosGraph, suffixes: Iterable[str]) -> bool:
    return any(topic == suffix or topic.endswith(suffix) for topic in graph.topics for suffix in suffixes)


def diagnose_ros_runtime(graph: RosGraph | None = None) -> RuntimeDiagnosis:
    graph = graph or discover_ros_graph()
    configured_domain = os.environ.get('ROS_DOMAIN_ID')
    domain_id = configured_domain if configured_domain is not None else '0'
    domain_source = 'environment' if configured_domain is not None else 'ROS 2 default'
    lifecycle_output, lifecycle_error, lifecycle_returncode = _run_ros2('lifecycle', 'nodes')
    lifecycle_nodes = sorted({line.strip() for line in lifecycle_output.splitlines() if line.strip().startswith('/')})
    errors = dict(graph.command_errors)
    if lifecycle_returncode:
        errors['ros2 lifecycle nodes'] = lifecycle_error or f'exit code {lifecycle_returncode}'

    expected_nodes = ('planner_server', 'controller_server', 'bt_navigator', 'map_server')
    statuses = {}
    details = {}
    states = {}
    for expected in expected_nodes:
        matches = [node for node in graph.nodes if node.strip('/').split('/')[-1] == expected]
        if not matches:
            statuses[expected] = 'MISSING'
            details[expected] = 'not visible in ros2 node list'
            continue
        lifecycle_match = next((node for node in lifecycle_nodes if node in matches), None)
        if lifecycle_match is None:
            statuses[expected] = 'FOUND'
            details[expected] = f'node visible as {matches[0]}, lifecycle state not listed'
            continue
        state_output, state_error, state_returncode = _run_ros2('lifecycle', 'get', lifecycle_match)
        state = _last_state_line(state_output)
        states[lifecycle_match] = state or 'UNKNOWN'
        if state_returncode:
            errors[f'ros2 lifecycle get {lifecycle_match}'] = state_error or f'exit code {state_returncode}'
        if _state_is_active(state):
            statuses[expected] = 'FOUND'
        else:
            statuses[expected] = 'INACTIVE'
        details[expected] = f'{lifecycle_match}: {state or "unknown"}'

    nav2_values = [statuses[name] for name in expected_nodes]
    if all(value == 'FOUND' for value in nav2_values):
        nav2_status = 'PASS'
    elif any(value == 'INACTIVE' for value in nav2_values):
        nav2_status = 'INACTIVE'
    else:
        nav2_status = 'MISSING'
    base_status = 'PASS' if _matching_topic(graph, ('/cmd_vel', '/odom', '/odometry/local', '/chassis/odometry')) else 'MISSING'
    localization_status = 'PASS' if _matching_topic(graph, ('/tf', '/tf_static', '/odom', '/odometry/local')) else 'MISSING'
    return RuntimeDiagnosis(
        ros_domain_id=domain_id,
        ros_domain_id_source=domain_source,
        lifecycle_nodes=lifecycle_nodes,
        lifecycle_states=states,
        node_statuses=statuses,
        node_details=details,
        base_status=base_status,
        localization_status=localization_status,
        nav2_status=nav2_status,
        command_errors=errors,
    )


def write_runtime_diagnosis_report(diagnosis: RuntimeDiagnosis, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', encoding='utf-8') as stream:
        stream.write('# ROS Runtime Diagnosis\n\n')
        stream.write('Read-only diagnosis of the current ROS graph and lifecycle state.\n\n')
        stream.write('## ROS Domain\n\n')
        stream.write(f'- ROS_DOMAIN_ID: `{diagnosis.ros_domain_id}` ({diagnosis.ros_domain_id_source})\n')
        stream.write('- A domain mismatch can make an otherwise running navigation stack invisible to this process.\n\n')

        stream.write('## Lifecycle Nodes\n\n')
        stream.write(f'- `ros2 lifecycle nodes`: {", ".join(f"`{item}`" for item in diagnosis.lifecycle_nodes) or "MISSING"}\n')
        for node, state in diagnosis.lifecycle_states.items():
            stream.write(f'- `{node}`: `{state}`\n')
        stream.write('\n')

        stream.write('## Critical Nav2 Nodes\n\n')
        stream.write('| node | status | detail |\n|---|---|---|\n')
        for node in ('planner_server', 'controller_server', 'bt_navigator', 'map_server'):
            stream.write(f'| `{node}` | **{diagnosis.node_statuses[node]}** | {diagnosis.node_details[node]} |\n')
        stream.write('\n')

        stream.write('## Navigation Stack\n\n')
        stream.write(f'- Base: **{diagnosis.base_status}**\n')
        stream.write(f'- Localization: **{diagnosis.localization_status}**\n')
        stream.write(f'- Nav2: **{diagnosis.nav2_status}**\n\n')

        stream.write('## Diagnosis\n\n')
        if diagnosis.nav2_status == 'MISSING':
            stream.write('Nav2 bringup is not visible in current ROS graph.\n\n')
            stream.write('Possible causes:\n\n- launch not started\n- namespace mismatch\n- ROS_DOMAIN_ID mismatch\n')
        elif diagnosis.nav2_status == 'INACTIVE':
            stream.write('Nav2 nodes are visible, but at least one lifecycle node is not active.\n\n')
            stream.write('Check lifecycle bringup transitions and activation failures before evaluating planner/controller behavior.\n')
        else:
            stream.write('Nav2 critical nodes are visible and active.\n')
        if diagnosis.command_errors:
            stream.write('\n## Command Diagnostics\n\n')
            for command, error in diagnosis.command_errors.items():
                stream.write(f'- `{command}`: `{error}`\n')
