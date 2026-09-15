#!/usr/bin/env python3
"""Explicit MQ4 planner-fixture state machine; the harness owns lifecycle."""
import argparse
import csv
import hashlib
import math
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from action_msgs.msg import GoalStatus
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


class Smoke(Node):
    def __init__(self):
        super().__init__('mq4_fixture_smoke')
        self.map = None
        self.costmap = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.broadcaster = TransformBroadcaster(self)
        self.synthetic_pose = (0.0, 0.0, 0.0)
        self.create_timer(0.05, self._broadcast_synthetic_tf)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        costmap_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, map_qos)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._costmap_callback, costmap_qos)

    def _map_callback(self, message):
        self.map = message

    def _costmap_callback(self, message):
        self.costmap = message

    def _broadcast_synthetic_tf(self):
        x, y, yaw = self.synthetic_pose
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.z = math.sin(yaw / 2.0)
        transform.transform.rotation.w = math.cos(yaw / 2.0)
        self.broadcaster.sendTransform(transform)

    def set_synthetic_pose(self, x, y, yaw):
        self.synthetic_pose = (x, y, yaw)

    def wait_for_synthetic_pose(self, expected, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                transform = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time()).transform
                actual_yaw = yaw_of(transform.rotation)
                yaw_error = abs(math.atan2(math.sin(actual_yaw - expected[2]),
                                             math.cos(actual_yaw - expected[2])))
                if math.hypot(transform.translation.x - expected[0], transform.translation.y - expected[1]) < 1e-3 and yaw_error < 1e-3:
                    return transform, actual_yaw
            except Exception:
                pass
        raise RuntimeError('TF_CONVERGENCE_TIMEOUT')

    def _wait_for_service(self, client, timeout=30):
        return client.wait_for_service(timeout_sec=timeout)

    def get_state(self, name):
        client = self.create_client(GetState, f'/{name}/get_state')
        if not self._wait_for_service(client):
            raise RuntimeError(f'{name.upper()}_LIFECYCLE_SERVICE_TIMEOUT')
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)
        if not future.result():
            raise RuntimeError(f'{name.upper()}_GET_STATE_FAILED')
        return future.result().current_state.label

    def ensure_active(self, name, status):
        """Directly configure/activate exactly one node, idempotently."""
        record = status['nodes'][name]
        state = self.get_state(name)
        record['initial_state'] = state
        if state == 'finalized':
            raise RuntimeError(f'{name.upper()}_FINALIZED')
        if state not in ('unconfigured', 'inactive', 'active'):
            raise RuntimeError(f'{name.upper()}_TRANSITIONAL_STATE_{state}')
        client = self.create_client(ChangeState, f'/{name}/change_state')
        if not self._wait_for_service(client):
            raise RuntimeError(f'{name.upper()}_LIFECYCLE_SERVICE_TIMEOUT')

        def transition(identifier, label, target):
            record[f'{label}_requested'] = True
            request = ChangeState.Request()
            request.transition.id = identifier
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10)
            record[f'{label}_success'] = bool(future.result() and future.result().success)
            if not record[f'{label}_success']:
                raise RuntimeError(f'{name.upper()}_{label.upper()}_FAILED')
            actual = self.get_state(name)
            record[f'post_{label}_state'] = actual
            if actual != target:
                raise RuntimeError(f'{name.upper()}_{label.upper()}_STATE_MISMATCH_{actual}')

        if state == 'unconfigured':
            transition(Transition.TRANSITION_CONFIGURE, 'configure', 'inactive')
            state = 'inactive'
        if state == 'inactive':
            transition(Transition.TRANSITION_ACTIVATE, 'activate', 'active')
        record['final_state'] = self.get_state(name)
        if record['final_state'] != 'active':
            raise RuntimeError(f'{name.upper()}_NOT_ACTIVE')


def pose(x, y, yaw):
    result = PoseStamped()
    result.header.frame_id = 'map'
    result.pose.position.x = x
    result.pose.position.y = y
    result.pose.orientation.z = math.sin(yaw / 2.0)
    result.pose.orientation.w = math.cos(yaw / 2.0)
    return result


def yaw_of(quaternion):
    return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def world_cell(grid, x, y):
    col = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
    row = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
    if col < 0 or row < 0 or col >= grid.info.width or row >= grid.info.height:
        return None
    return grid.data[row * grid.info.width + col]


def in_bounds(grid, x, y):
    return world_cell(grid, x, y) is not None


def samples(path, step):
    points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    result = []
    distance = 0.0
    for first, second in zip(points, points[1:]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length = math.hypot(dx, dy)
        distance += length
        count = max(1, math.ceil(length / step))
        result.extend((first[0] + dx * i / count, first[1] + dy * i / count,
                       length / count) for i in range(count))
    if points:
        result.append((points[-1][0], points[-1][1], 0.0))
    return result, distance


def audit_raw(grid, path, step):
    result = dict(sample_count=0, free_samples=0, occupied_samples=0,
                  unknown_samples=0, outside_samples=0, unknown_distance_m=0.0)
    sampled, _ = samples(path, step)
    for x, y, segment_length in sampled:
        value = world_cell(grid, x, y)
        result['sample_count'] += 1
        if value is None:
            result['outside_samples'] += 1
        elif value == 0:
            result['free_samples'] += 1
        elif value == 100:
            result['occupied_samples'] += 1
        else:
            result['unknown_samples'] += 1
            result['unknown_distance_m'] += segment_length
    result['unknown_fraction'] = (result['unknown_samples'] / result['sample_count']
                                  if result['sample_count'] else 0.0)
    return result


def audit_costmap(grid, path, step):
    result = dict(sample_count=0, traversable_samples=0, inscribed_samples=0,
                  lethal_samples=0, unknown_samples=0, outside_samples=0,
                  occupancy_values={})
    sampled, _ = samples(path, step)
    for x, y, _ in sampled:
        value = world_cell(grid, x, y)
        result['sample_count'] += 1
        if value is None:
            result['outside_samples'] += 1
        elif value == -1:
            result['unknown_samples'] += 1
        elif value == 100:  # Nav2 OccupancyGrid translation of LETHAL_OBSTACLE (254).
            result['lethal_samples'] += 1
        elif value == 99:  # Translation of INSCRIBED_INFLATED_OBSTACLE (253).
            result['inscribed_samples'] += 1
        else:
            result['traversable_samples'] += 1
        if value is not None:
            result['occupancy_values'][str(value)] = result['occupancy_values'].get(str(value), 0) + 1
    return result


def wait_for_message(node, attribute, timeout, failure):
    deadline = time.monotonic() + timeout
    while getattr(node, attribute) is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if getattr(node, attribute) is None:
        raise RuntimeError(failure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--params', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--start', nargs=3, type=float, required=True, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--goal', nargs=3, type=float, required=True, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--pair-id', default='pair_000')
    parser.add_argument('--source-pair-id', default='short_forward_0')
    parser.add_argument('--path-csv')
    parser.add_argument('--send-action', action='store_true')
    args = parser.parse_args()
    status = {
        'candidate_map': args.map_yaml,
        'nav2_params': args.params,
        'nav2_params_sha256': hashlib.sha256(open(args.params, 'rb').read()).hexdigest(),
        'lifecycle_authority': 'mq4_harness_direct',
        'state': 'PROCESS_STARTED',
        'nodes': {'map_server': {}, 'planner_server': {}},
        'request': {'pair_id': args.pair_id, 'source_pair_id': args.source_pair_id,
                    'planner_id': 'GridBased', 'use_start': True,
                    'start': {'x': args.start[0], 'y': args.start[1], 'yaw': args.start[2]},
                    'goal': {'x': args.goal[0], 'y': args.goal[1], 'yaw': args.goal[2]}},
    }
    rclpy.init()
    node = Smoke()
    try:
        node.set_synthetic_pose(*args.start)
        status['state'] = 'WAIT_MAP_LIFECYCLE_SERVICES'
        node.ensure_active('map_server', status)
        status['state'] = 'WAIT_MAP'
        wait_for_message(node, 'map', 30, 'MAP_NOT_AVAILABLE')
        status['map'] = {
            'width': node.map.info.width, 'height': node.map.info.height,
            'resolution': node.map.info.resolution,
            'origin': [node.map.info.origin.position.x, node.map.info.origin.position.y],
            'frame': node.map.header.frame_id,
        }
        status['map_received'] = True
        status['state'] = 'WAIT_TF'
        node.wait_for_synthetic_pose(args.start)
        status['tf_valid'] = True
        transform, tf_yaw = node.wait_for_synthetic_pose(args.start)
        status['synthetic_tf'] = {'x': transform.translation.x, 'y': transform.translation.y,
                                  'yaw': tf_yaw}
        if (math.hypot(transform.translation.x - args.start[0], transform.translation.y - args.start[1]) > 1e-3
                or abs(math.atan2(math.sin(tf_yaw - args.start[2]), math.cos(tf_yaw - args.start[2]))) > 1e-3):
            raise RuntimeError('TF_START_POSE_MISMATCH')
        status['state'] = 'WAIT_PLANNER_LIFECYCLE_SERVICES'
        node.ensure_active('planner_server', status)
        status['state'] = 'WAIT_ACTION'
        action = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        if not action.wait_for_server(timeout_sec=30):
            raise RuntimeError('ACTION_NOT_AVAILABLE')
        status['action_available'] = True
        status['state'] = 'WAIT_GLOBAL_COSTMAP'
        wait_for_message(node, 'costmap', 30, 'GLOBAL_COSTMAP_NOT_AVAILABLE')
        status['global_costmap'] = {
            'topic': '/global_costmap/costmap',
            'width': node.costmap.info.width,
            'height': node.costmap.info.height,
            'resolution': node.costmap.info.resolution,
            'origin': [node.costmap.info.origin.position.x, node.costmap.info.origin.position.y],
            'frame': node.costmap.header.frame_id,
        }
        status['global_costmap_ready'] = True
        if not in_bounds(node.costmap, args.start[0], args.start[1]) or not in_bounds(node.costmap, args.goal[0], args.goal[1]):
            raise RuntimeError('GLOBAL_COSTMAP_BOUNDS_INVALID')
        status['global_costmap_bounds_valid'] = True
        status['fixture_ready'] = True
        status['state'] = 'READY'
        if not args.send_action:
            return
        goal = ComputePathToPose.Goal()
        goal.start = pose(*args.start)
        goal.goal = pose(*args.goal)
        goal.use_start = True
        goal.planner_id = 'GridBased'
        started = time.monotonic()
        future = action.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        handle = future.result()
        status['result'] = {'goal_accepted': bool(handle and handle.accepted), 'error_code': None}
        if not handle or not handle.accepted:
            raise RuntimeError('ACTION_REJECTED')
        future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        wrapped = future.result()
        if not wrapped:
            raise RuntimeError('ACTION_TIMEOUT')
        status['result']['action_status'] = wrapped.status
        status['result']['planner_success'] = wrapped.status == GoalStatus.STATUS_SUCCEEDED
        status['result']['request_wall_time_ms'] = (time.monotonic() - started) * 1000
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('PLANNER_RESULT_FAILURE')
        path = wrapped.result.path
        path_samples, path_length = samples(path, min(0.05, node.map.info.resolution / 2.0))
        status['result']['nav2_planning_time_ms'] = (wrapped.result.planning_time.sec * 1000.0 + wrapped.result.planning_time.nanosec / 1e6)
        status['result']['path_pose_count'] = len(path.poses)
        status['result']['path_length_m'] = path_length
        status['raw_map_audit'] = audit_raw(node.map, path, min(0.05, node.map.info.resolution / 2.0))
        status['global_costmap_audit'] = audit_costmap(node.costmap, path, min(0.05, node.map.info.resolution / 2.0))
        if args.path_csv:
            with open(args.path_csv, 'w', newline='') as stream:
                writer = csv.writer(stream); writer.writerow(['index', 'x', 'y', 'yaw'])
                for index, item in enumerate(path.poses):
                    writer.writerow([index, item.pose.position.x, item.pose.position.y, yaw_of(item.pose.orientation)])
        if len(path.poses) <= 1 or path_length <= 0 or status['raw_map_audit']['outside_samples'] or status['global_costmap_audit']['outside_samples']:
            raise RuntimeError('PLANNER_RESULT_FAILURE')
        status['verdict'] = ('PAIR_SMOKE_SEMANTIC_REVIEW' if status['raw_map_audit']['occupied_samples'] or status['global_costmap_audit']['lethal_samples'] else 'PAIR_SMOKE_PASS')
    except Exception as error:
        status['failure_stage'] = status.get('state')
        status['state'] = 'FAILED'
        status['failure_reason'] = str(error)
        status['verdict'] = 'FIXTURE_FAILURE' if str(error) in ('TF_START_POSE_MISMATCH', 'GLOBAL_COSTMAP_BOUNDS_INVALID') or status.get('failure_stage', '').startswith('WAIT_') else 'PLANNER_FAILURE'
    finally:
        with open(args.output, 'w') as stream:
            yaml.safe_dump(status, stream, sort_keys=False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
