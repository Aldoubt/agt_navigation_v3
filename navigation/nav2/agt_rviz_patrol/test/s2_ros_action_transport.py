"""S2 integration smoke: real generated FollowRoute Action, isolated /test/s2 endpoint.

This is a TEST ONLY action server, with no navigation backend or vehicle topics.
Execute as a standalone program in an independently verified empty ROS domain.
"""
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from agt_navigation_interfaces.action import FollowRoute
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

ENDPOINT = '/test/s2/navigation/follow_route'


def checked_wait(future, label, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.01)
    raise AssertionError(f'{label} timed out')


class MockFollowServer(Node):
    def __init__(self):
        super().__init__('s2_mock_follow_server')
        group = ReentrantCallbackGroup()
        self.server = ActionServer(self, FollowRoute, ENDPOINT,
                                   execute_callback=self.execute,
                                   goal_callback=self.on_goal,
                                   cancel_callback=self.on_cancel,
                                   callback_group=group)
        self.seen = []

    def on_goal(self, request):
        self.seen.append(('goal', request.path.header.frame_id))
        return (GoalResponse.REJECT if request.path.header.frame_id == 'reject'
                else GoalResponse.ACCEPT)

    def on_cancel(self, _handle):
        self.seen.append(('cancel', True))
        return CancelResponse.ACCEPT

    def execute(self, handle):
        mode = handle.request.path.header.frame_id
        steps = 40 if mode == 'cancel' else 4
        for i in range(steps):
            if handle.is_cancel_requested:
                handle.canceled()
                result = FollowRoute.Result()
                result.success, result.error_code = False, 'MOCK_CANCELED'
                result.message = 'isolated mock canceled (not physical stop)'
                return result
            feedback = FollowRoute.Feedback()
            feedback.state = 'RUNNING'
            feedback.distance_remaining = float(steps-i)
            handle.publish_feedback(feedback)
            time.sleep(0.03)
        result = FollowRoute.Result()
        if mode == 'abort':
            handle.abort()
            result.success, result.error_code, result.message = False, 'MOCK_ABORT', 'intentional mock failure'
        else:
            handle.succeed()
            result.success, result.error_code, result.message = True, '', 'mock completion only'
        return result


def new_goal(mode):
    path = Path()
    path.header.frame_id = mode
    for x in (0., 1.):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.orientation.w = 1.
        path.poses.append(pose)
    msg = FollowRoute.Goal()
    msg.path = path
    return msg


def run():
    rclpy.init()
    server = MockFollowServer()
    node = Node('s2_mock_action_client')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    client = ActionClient(node, FollowRoute, ENDPOINT)
    absent = ActionClient(node, FollowRoute, '/test/s2/not_started/follow_route')
    try:
        assert not absent.wait_for_server(timeout_sec=0.3)
        print('PASS: unavailable server timeout (0.3s)', flush=True)
        assert client.wait_for_server(timeout_sec=3.)
        for mode, expected_status in [('reject', None), ('success', GoalStatus.STATUS_SUCCEEDED),
                                      ('abort', GoalStatus.STATUS_ABORTED), ('cancel', GoalStatus.STATUS_CANCELED)]:
            feedback = []
            handle = checked_wait(client.send_goal_async(new_goal(mode),
                                  feedback_callback=lambda f: feedback.append(f.feedback.state)),
                                  mode + ' goal response')
            if mode == 'reject':
                assert not handle.accepted
                print('PASS: goal rejected', flush=True)
                continue
            assert handle.accepted, mode
            if mode == 'cancel':
                deadline = time.monotonic()+2.
                while not feedback and time.monotonic()<deadline:
                    time.sleep(.01)
                assert feedback, 'cancel scenario must be running before cancel request'
                ack = checked_wait(handle.cancel_goal_async(), 'cancel ack')
                assert ack.goals_canceling, 'cancel not acknowledged'
            wrapped = checked_wait(handle.get_result_async(), mode+' result')
            assert wrapped.status == expected_status, (mode, wrapped.status, expected_status)
            assert feedback, 'missing Action feedback: '+mode
            assert wrapped.result.success == (mode == 'success')
            print('PASS:', mode, 'accepted, feedback, final status', wrapped.status,
                  'message=', wrapped.result.message, flush=True)
        print('RESULT: REAL ROS ACTION TRANSPORT PASS; NO MOTION; endpoint='+ENDPOINT, flush=True)
    finally:
        absent.destroy()
        client.destroy()
        executor.shutdown(timeout_sec=2.)
        server.server.destroy()
        server.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin.join(timeout=2.)


if __name__ == '__main__':
    run()
