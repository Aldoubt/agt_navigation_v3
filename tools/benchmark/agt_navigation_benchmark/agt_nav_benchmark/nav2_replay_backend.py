"""Isolated real Nav2 planner_server backend for offline A/B experiments."""

from __future__ import annotations

import csv
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import yaml


@dataclass
class ReplayPose:
    x: float
    y: float
    yaw: float
    frame_id: str = 'map'


@dataclass
class ReplayOutput:
    path: object
    planning_time_sec: float
    namespace: str
    log_path: Path


def _quaternion_from_yaw(yaw: float):
    import math
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _yaw_from_quaternion(q) -> float:
    import math
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def write_plan_csv(path_message, output_path: Path) -> None:
    """Persist a real nav_msgs/Path result as timestamp,x,y,yaw CSV."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['timestamp', 'x', 'y', 'yaw'])
        stamp = path_message.header.stamp
        timestamp = int(stamp.sec) + int(stamp.nanosec) / 1e9
        for index, pose in enumerate(path_message.poses):
            position = pose.pose.position
            quaternion = pose.pose.orientation
            yaw = _yaw_from_quaternion(quaternion)
            # NavFn may return identity orientations. For a benchmark CSV,
            # use the actual path tangent in that case so yaw is meaningful.
            if abs(quaternion.x) < 1e-9 and abs(quaternion.y) < 1e-9 and abs(quaternion.z) < 1e-9 and abs(quaternion.w - 1.0) < 1e-9:
                next_pose = path_message.poses[min(index + 1, len(path_message.poses) - 1)]
                if index == len(path_message.poses) - 1 and index > 0:
                    next_pose = path_message.poses[index - 1]
                dx = float(next_pose.pose.position.x) - float(position.x)
                dy = float(next_pose.pose.position.y) - float(position.y)
                if index == len(path_message.poses) - 1:
                    dx, dy = -dx, -dy
                if abs(dx) > 1e-9 or abs(dy) > 1e-9:
                    import math
                    yaw = math.atan2(dy, dx)
            writer.writerow([
                f'{timestamp:.9f}', f'{float(position.x):.9f}', f'{float(position.y):.9f}', f'{yaw:.9f}',
            ])


class Nav2PlannerReplayBackend:
    """Launch only the Nav2 map/planner lifecycle nodes in a private namespace."""

    def __init__(
        self,
        map_yaml: Path,
        start: ReplayPose,
        goal: ReplayPose,
        planner_plugin: str = 'nav2_navfn_planner/NavfnPlanner',
        planner_id: str = 'GridBased',
        namespace: str = 'agt_nav2_replay',
        output_dir: Path = Path('.'),
        timeout_sec: float = 30.0,
        inflation_radius: float = 0.25,
    ):
        self.map_yaml = Path(map_yaml).resolve()
        self.start = start
        self.goal = goal
        self.planner_plugin = planner_plugin
        self.planner_id = planner_id
        self.namespace = namespace.strip('/')
        self.output_dir = Path(output_dir)
        self.timeout_sec = timeout_sec
        self.inflation_radius = inflation_radius
        self.processes: List[subprocess.Popen] = []
        self.log_stream = None

    def params(self):
        plugin_params = {
            'plugin': self.planner_plugin,
            'tolerance': 0.5,
            'allow_unknown': True,
        }
        if 'navfn' in self.planner_plugin.lower():
            plugin_params['use_astar'] = False
        elif 'smac' in self.planner_plugin.lower():
            plugin_params.update({
                'downsample_costmap': False,
                'max_iterations': -1,
                'max_on_approach_iterations': 1000,
                'motion_model_for_search': 'DUBIN',
                'minimum_turning_radius': 0.4,
            })
        elif 'theta' in self.planner_plugin.lower():
            plugin_params.update({
                'how_many_corners': 4,
                'w_euc_cost': 1.0,
                'w_traversal_cost': 2.0,
            })
        # Use fully-qualified node keys because the replay nodes run inside a
        # private namespace and are launched with ros2 run.
        prefix = f'/{self.namespace}'
        return {
            f'{prefix}/map_server': {'ros__parameters': {
                'use_sim_time': False,
                'yaml_filename': str(self.map_yaml),
                'topic_name': 'map',
                'frame_id': 'map',
            }},
            f'{prefix}/planner_server': {'ros__parameters': {
                'use_sim_time': False,
                'expected_planner_frequency': 5.0,
                'planner_plugins': [self.planner_id],
                self.planner_id: plugin_params,
            }},
            f'{prefix}/global_costmap': {'global_costmap': {'ros__parameters': {
                'use_sim_time': False,
                'global_frame': 'map',
                # This backend calls ComputePathToPose with use_start=true and
                # has no live robot TF. map->map keeps the offline costmap
                # lifecycle independent of the running navigation stack.
                'robot_base_frame': 'map',
                'update_frequency': 1.0,
                'publish_frequency': 1.0,
                'track_unknown_space': True,
                'rolling_window': False,
                'plugins': ['static_layer', 'inflation_layer'],
                'static_layer': {
                    'plugin': 'nav2_costmap_2d::StaticLayer',
                    'map_subscribe_transient_local': True,
                    'subscribe_to_updates': False,
                },
                'inflation_layer': {
                    'plugin': 'nav2_costmap_2d::InflationLayer',
                    'inflation_radius': self.inflation_radius,
                    'cost_scaling_factor': 3.0,
                },
                'always_send_full_costmap': True,
                'robot_radius': 0.20,
            }}},
            f'{prefix}/lifecycle_manager_replay': {'ros__parameters': {
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'planner_server'],
                'bond_timeout': 5.0,
            }},
        }

    def write_planner_params(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(self.params(), sort_keys=False), encoding='utf-8')

    def _command(self, package: str, executable: str, node_name: str, params_path: Path):
        command = [
            'ros2', 'run', package, executable, '--ros-args',
            '-r', f'__ns:=/{self.namespace}', '-r', f'__node:={node_name}',
            '--params-file', str(params_path),
        ]
        if package == 'nav2_lifecycle_manager':
            # Lifecycle manager parameter matching is affected by the
            # __node remap. Inline parameters are unambiguous in ros2 run.
            command.extend([
                '-p', 'use_sim_time:=false',
                '-p', 'autostart:=true',
                '-p', "node_names:=['map_server','planner_server']",
                '-p', 'bond_timeout:=5.0',
            ])
        return command

    def _start(self, params_path: Path, log_path: Path) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.log_stream = log_path.open('w', encoding='utf-8')
        commands = [
            self._command('nav2_map_server', 'map_server', 'map_server', params_path),
            self._command('nav2_planner', 'planner_server', 'planner_server', params_path),
            self._command('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_replay', params_path),
        ]
        for command in commands:
            process = subprocess.Popen(
                command, stdout=self.log_stream, stderr=subprocess.STDOUT,
                env=os.environ.copy(), start_new_session=True,
            )
            self.processes.append(process)
            time.sleep(0.5)

    def _stop(self) -> None:
        for process in reversed(self.processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + 5.0
        for process in reversed(self.processes):
            remaining = max(0.0, deadline - time.monotonic())
            try:
                process.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        if self.log_stream:
            self.log_stream.close()
            self.log_stream = None
        self.processes.clear()

    def compute_path(self):
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import ComputePathToPose
        from rclpy.action import ActionClient
        from rclpy.node import Node

        node = Node('agt_nav2_replay_client', namespace=self.namespace)
        client = ActionClient(node, ComputePathToPose, f'/{self.namespace}/compute_path_to_pose')
        try:
            if not client.wait_for_server(timeout_sec=self.timeout_sec):
                raise RuntimeError(f'planner action unavailable: /{self.namespace}/compute_path_to_pose')

            def pose_stamped(pose: ReplayPose):
                message = PoseStamped()
                message.header.frame_id = pose.frame_id
                message.header.stamp = node.get_clock().now().to_msg()
                message.pose.position.x = pose.x
                message.pose.position.y = pose.y
                message.pose.orientation.z, message.pose.orientation.w = _quaternion_from_yaw(pose.yaw)
                return message

            goal = ComputePathToPose.Goal()
            goal.start = pose_stamped(self.start)
            goal.goal = pose_stamped(self.goal)
            goal.planner_id = self.planner_id
            goal.use_start = True
            send_future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, send_future, timeout_sec=self.timeout_sec)
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError('planner rejected ComputePathToPose goal')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=self.timeout_sec)
            wrapped_result = result_future.result()
            if wrapped_result is None:
                raise RuntimeError('planner returned no result')
            if wrapped_result.status != 4:  # action_msgs/msg/GoalStatus STATUS_SUCCEEDED
                raise RuntimeError(f'planner action failed with status {wrapped_result.status}')
            result = wrapped_result.result
            if len(result.path.poses) < 2:
                raise RuntimeError('planner returned a path with fewer than two poses')
            planning_time = result.planning_time.sec + result.planning_time.nanosec / 1e9
            return ReplayOutput(result.path, planning_time, self.namespace, self.output_dir / 'nav2_replay.log')
        finally:
            node.destroy_node()

    def run(self, output_csv: Path, planner_yaml: Optional[Path] = None) -> ReplayOutput:
        import rclpy
        owns_rclpy = not rclpy.ok()
        if owns_rclpy:
            rclpy.init()
        params_path = planner_yaml or (self.output_dir / 'planner_runtime_params.yaml')
        self.write_planner_params(params_path)
        log_path = self.output_dir / 'nav2_replay.log'
        try:
            self._start(params_path, log_path)
            output = self.compute_path()
            write_plan_csv(output.path, output_csv)
            time.sleep(1.0)
            return ReplayOutput(output.path, output.planning_time_sec, self.namespace, log_path)
        finally:
            self._stop()
            if owns_rclpy and rclpy.ok():
                rclpy.shutdown()


def extract_experiment_poses(bag_path: Path):
    """Use the first valid recorded /plan as the A/B start and goal."""
    from .bag_reader import RosbagReader
    reader = RosbagReader(str(bag_path))
    for record in reader.records(['/plan']):
        if len(record.message.poses) < 2:
            continue
        first = record.message.poses[0]
        last = record.message.poses[-1]
        frame_id = record.message.header.frame_id or first.header.frame_id or 'map'
        return (
            ReplayPose(first.pose.position.x, first.pose.position.y, _yaw_from_quaternion(first.pose.orientation), frame_id),
            ReplayPose(last.pose.position.x, last.pose.position.y, _yaw_from_quaternion(last.pose.orientation), frame_id),
        )
    raise RuntimeError('No valid /plan with at least two poses found in bag')


def pose_dict(start: ReplayPose, goal: ReplayPose):
    return {
        'start': {'x': float(start.x), 'y': float(start.y), 'yaw': float(start.yaw), 'frame_id': start.frame_id},
        'goal': {'x': float(goal.x), 'y': float(goal.y), 'yaw': float(goal.yaw), 'frame_id': goal.frame_id},
    }
