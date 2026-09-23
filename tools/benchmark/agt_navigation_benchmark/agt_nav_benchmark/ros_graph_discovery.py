"""Read-only discovery of the live ROS 2 navigation graph."""

from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List


@dataclass
class RosGraph:
    nodes: List[str] = field(default_factory=list)
    topic_types: Dict[str, str] = field(default_factory=dict)
    actions: List[str] = field(default_factory=list)
    services: List[str] = field(default_factory=list)
    command_errors: Dict[str, str] = field(default_factory=dict)

    @property
    def topics(self) -> List[str]:
        return sorted(self.topic_types)


def _run_ros2(*arguments: str):
    command = ['ros2', *arguments]
    result = subprocess.run(command, check=False, capture_output=True, text=True)
    return result.stdout, result.stderr.strip(), result.returncode


def _lines(output: str) -> List[str]:
    return sorted({line.strip() for line in output.splitlines() if line.strip().startswith('/')})


def discover_ros_graph() -> RosGraph:
    graph = RosGraph()
    output, error, returncode = _run_ros2('node', 'list')
    graph.nodes = _lines(output)
    if returncode:
        graph.command_errors['ros2 node list'] = error or f'exit code {returncode}'

    output, error, returncode = _run_ros2('topic', 'list', '-t')
    for line in output.splitlines():
        match = re.match(r'^\s*(/\S+)\s+\[([^]]+)\]\s*$', line)
        if match:
            graph.topic_types[match.group(1)] = match.group(2)
        elif line.strip().startswith('/'):
            graph.topic_types[line.strip().split()[0]] = 'unknown'
    if returncode:
        graph.command_errors['ros2 topic list -t'] = error or f'exit code {returncode}'

    output, error, returncode = _run_ros2('action', 'list')
    graph.actions = _lines(output)
    if returncode:
        graph.command_errors['ros2 action list'] = error or f'exit code {returncode}'

    output, error, returncode = _run_ros2('service', 'list')
    graph.services = _lines(output)
    if returncode:
        graph.command_errors['ros2 service list'] = error or f'exit code {returncode}'
    return graph


def _node_found(nodes: Iterable[str], node_name: str) -> bool:
    expected = node_name.strip('/')
    return any(node.strip('/').split('/')[-1] == expected for node in nodes)


def _topic_matches(topics: Iterable[str], patterns: Iterable[str]) -> Dict[str, List[str]]:
    from fnmatch import fnmatchcase
    topics = sorted(set(topics))
    return {pattern: [topic for topic in topics if fnmatchcase(topic, pattern)] for pattern in patterns}


def write_navigation_stack_discovery_report(graph: RosGraph, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    node_expectations = ('planner_server', 'controller_server', 'bt_navigator', 'map_server')
    standard_patterns = (
        '*/plan', '*/cmd_vel', '*/map', '*/global_costmap/*',
        '*/local_costmap/*', '*/tf', '*/tf_static', '*/parameter_events',
    )
    topic_matches = _topic_matches(graph.topics, standard_patterns)
    standard_topics = {topic for matches in topic_matches.values() for topic in matches}
    custom_topics = [topic for topic in graph.topics if topic not in standard_topics]
    with output_path.open('w', encoding='utf-8') as stream:
        stream.write('# Navigation Stack ROS Graph Discovery\n\n')
        stream.write('Read-only snapshot from `ros2 node list`, `ros2 topic list -t`, `ros2 action list`, and `ros2 service list`.\n\n')
        stream.write('## Nodes\n\n')
        stream.write('| node | status |\n|---|---|\n')
        for node in node_expectations:
            stream.write(f'| `{node}` | **{"FOUND" if _node_found(graph.nodes, node) else "MISSING"}** |\n')
        stream.write('\nDiscovered nodes: ' + (', '.join(f'`{node}`' for node in graph.nodes) or 'MISSING') + '\n\n')

        stream.write('## Topics\n\n')
        stream.write('| profile pattern | status | discovered topics |\n|---|---|---|\n')
        for pattern, matches in topic_matches.items():
            stream.write(f'| `{pattern}` | **{"FOUND" if matches else "MISSING"}** | {", ".join(f"`{item}`" for item in matches) or "—"} |\n')
        stream.write('\n### Custom topics\n\n')
        stream.write(', '.join(f'`{topic}` ({graph.topic_types[topic]})' for topic in custom_topics) or 'MISSING')
        stream.write('\n\n')

        stream.write('## Actions\n\n')
        for action in ('/navigate_to_pose', '/navigate_through_poses'):
            found = [item for item in graph.actions if item == action or item.endswith(action)]
            stream.write(f'- `{action}`: **{"FOUND" if found else "MISSING"}**' + (f' ({", ".join(f"`{item}`" for item in found)})' if found else '') + '\n')
        stream.write('\nDiscovered actions: ' + (', '.join(f'`{item}`' for item in graph.actions) or 'MISSING') + '\n\n')

        stream.write('## Services\n\n')
        stream.write(f'- Discovered service count: `{len(graph.services)}`\n')
        stream.write('- Service list is retained for runtime namespace diagnostics.\n\n')
        if graph.command_errors:
            stream.write('## Discovery Errors\n\n')
            for command, error in graph.command_errors.items():
                stream.write(f'- `{command}`: `{error}`\n')
