"""No-ROS mock harness executing the REAL Workbench callback logic.

Test-only stand-ins implement the unchanged FollowRoute.action fields; no ROS
publisher, CAN, TF, Navigation Capability server or vehicle graph is started.
"""
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from types import SimpleNamespace as NS


def stub(fullname, **symbols):
    mod = types.ModuleType(fullname)
    mod.__dict__.update(symbols)
    sys.modules[fullname] = mod
    return mod


class GoalStatus:
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


class FollowRoute:
    class Goal:
        def __init__(self):
            self.path = None
    class Result:
        def __init__(self, success=False, error_code='', message=''):
            self.success, self.error_code, self.message = success, error_code, message
    class Feedback:
        def __init__(self, state='', distance_remaining=0.):
            self.state, self.distance_remaining = state, distance_remaining


class String:
    def __init__(self, data=''):
        self.data = data


stub('action_msgs'); stub('action_msgs.msg', GoalStatus=GoalStatus)
stub('agt_navigation_interfaces'); stub('agt_navigation_interfaces.action', FollowRoute=FollowRoute)
stub('agt_navigation_interfaces.msg', NavigationHealth=type('NavigationHealth', (), {'READY': 1}))
stub('geometry_msgs'); stub('geometry_msgs.msg', Point=object)
stub('nav_msgs'); stub('nav_msgs.msg', OccupancyGrid=object)
stub('nav2_msgs'); stub('nav2_msgs.srv', IsPathValid=object)
stub('rclpy'); stub('rclpy.qos', QoSProfile=object, DurabilityPolicy=object, ReliabilityPolicy=object)
stub('std_msgs'); stub('std_msgs.msg', String=String)
stub('std_srvs'); stub('std_srvs.srv', Trigger=object)
stub('visualization_msgs'); stub('visualization_msgs.msg', Marker=object)
stub('agt_rviz_patrol.path_tool', RvizPathTool=object, interpolate_polyline=lambda *a: [])

from agt_rviz_patrol.route_editor import RouteEditor
from agt_rviz_patrol.workbench import RouteWorkbench


class Future:
    def __init__(self, value):
        self.value, self.callback = value, None

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callback = callback

    def complete(self):
        assert self.callback is not None
        self.callback(self)


class Handle:
    def __init__(self, accepted):
        self.accepted, self.cancels = accepted, 0
        self.result_future = Future(None)

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancels += 1
        class Ack(Future):
            def add_done_callback(self, callback):
                callback(self)
        return Ack(NS(goals_canceling=[42]))


class Client:
    def __init__(self):
        self.endpoint = '/test/s2/navigation/follow_route'
        self.goals = []
        self.goal_future = None

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback):
        assert isinstance(goal, FollowRoute.Goal)
        self.goals.append(goal)
        self.feedback_callback = feedback_callback
        self.goal_future = Future(None)
        return self.goal_future

    def respond(self, handle):
        self.goal_future.value = handle
        self.goal_future.complete()


class CallbackLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory(dir=os.environ.get('AGT_S2_RUNS'))
        self.addCleanup(self.dir.cleanup)
        self.filename = str(Path(self.dir.name) / 'route_draft.json')
        self.params = {'preview_only': True, 'offline_hard_lock': False,
                       'map_id': 'test-s2-map',
                       'map_version': 'rev1', 'robot_profile': 'bunker_v1',
                       'draft_path': self.filename, 'preview_ttl': 30.0}
        self.node = self.new_node()

    def new_node(self):
        n = RouteWorkbench.__new__(RouteWorkbench)
        n.editor = RouteEditor()
        n.preview = None
        n.validating, n.validation_token = False, 0
        n.start_pending, n.pause_pending, n.cancel_pending = False, False, False
        n.active_goal, n.active_path, n.paused_path = None, None, None
        n.map_key = ('map', 100, 100, .05, 'origin', 123456)
        n.global_frame = 'map'
        n.saved_disk_token = n.saved_revision = n.saved_context = None
        n.follow_client = Client()
        n.status_text = ''
        n.publish_status = lambda text: setattr(n, 'status_text', text)
        n.publish_state = lambda: None
        n.publish_draft = lambda: None
        n.get_parameter = lambda key: NS(value=self.params[key])
        n.lio_dirty, n.wheel_dirty = False, False
        n.progress = 0
        n.robot_pose = lambda: (0., 0., 0.)
        n.current_map_pose = n.robot_pose
        n.health_ok = lambda: True
        return n

    def response(self):
        return NS(success=False, message='')

    def test_offline_execute_and_resume_never_send_mock_goal(self):
        n = self.node
        n.preview = {'resume': False}
        self.assertFalse(n.start_preview(self.response(), False).success)
        self.assertFalse(n.start_preview(self.response(), True).success)
        self.assertEqual(n.follow_client.goals, [])

    def test_accept_feedback_success(self):
        n = self.node
        path = NS(frame_id='map')
        self.assertEqual(n.follow_client.endpoint, '/test/s2/navigation/follow_route')
        self.assertTrue(n.submit_path(path))
        self.assertTrue(n.start_pending)
        self.assertIs(n.follow_client.goals[0].path, path)
        handle = Handle(accepted=True)
        n.follow_client.respond(handle)
        self.assertFalse(n.start_pending)
        self.assertIs(n.active_goal, handle)
        self.assertEqual(n.status_text, 'RUNNING')
        n.on_feedback(NS(feedback=FollowRoute.Feedback('RUNNING', 1.25)))
        self.assertEqual(n.feedback_remaining, 1.25)
        self.assertEqual(n.feedback_state, 'RUNNING')
        handle.result_future.value = NS(status=GoalStatus.STATUS_SUCCEEDED,
                                        result=FollowRoute.Result(True, '', 'completed'))
        handle.result_future.complete()
        self.assertFalse(n.busy())
        self.assertIn('completed', n.status_text)

    def test_rejected_goal_unlocks_without_executing(self):
        n = self.node
        n.submit_path(NS(frame_id='map'))
        n.follow_client.respond(Handle(accepted=False))
        self.assertFalse(n.busy())
        self.assertIn('rejected', n.status_text)

    def test_action_aborted_and_false_success_are_failures(self):
        for status, success in [(GoalStatus.STATUS_ABORTED, False),
                                (GoalStatus.STATUS_SUCCEEDED, False)]:
            with self.subTest(status=status):
                n = self.new_node()
                n.submit_path(NS(frame_id='map'))
                handle = Handle(accepted=True)
                n.follow_client.respond(handle)
                handle.result_future.value = NS(status=status,
                                                result=FollowRoute.Result(success, 'MOCK_FAIL', 'failed'))
                handle.result_future.complete()
                self.assertFalse(n.busy())
                self.assertIn('FAILED', n.status_text)
                self.assertIn('UNVERIFIED', n.status_text)

    def test_pending_cancel_ack_and_terminal_cancel(self):
        n = self.node
        n.submit_path(NS(frame_id='map'))
        result = n.on_cancel(None, self.response())
        self.assertTrue(result.success)
        self.assertTrue(n.busy())
        handle = Handle(accepted=True)
        n.follow_client.respond(handle)
        self.assertEqual(handle.cancels, 1)
        self.assertTrue(n.busy())
        handle.result_future.value = NS(status=GoalStatus.STATUS_CANCELED,
                                        result=FollowRoute.Result(False, 'CANCELED', 'operator cancel'))
        handle.result_future.complete()
        self.assertFalse(n.busy())
        self.assertIn('CANCELED', n.status_text)

    def test_unknown_goal_response_remains_locked(self):
        n = self.node
        n.submit_path(NS(frame_id='map'))
        n.on_goal_response(NS(result=lambda: (_ for _ in ()).throw(RuntimeError('transport lost'))))
        self.assertTrue(n.busy())
        self.assertIn('UNKNOWN', n.status_text)

    def test_validation_timeout_invalidates_preview(self):
        import time
        n = self.node
        n.validating = True
        n.validation_started = time.monotonic() - 4.
        n.preview = {'time': time.monotonic(), 'resume': False}
        n.tick()
        self.assertFalse(n.validating)
        self.assertIsNone(n.preview)
        self.assertIn('TIMEOUT', n.status_text)

    def test_3point_save_restart_load_offline_preview(self):
        n = self.node
        n.editor.replace([(0., 0.), (1., 0.), (1., 1.)], 0)
        n.editor.replace([(0., 0.), (1.1, 0.), (1., 1.)], 1)
        saved = n.on_save_draft(None, self.response())
        self.assertTrue(saved.success, saved.message)
        restarted = self.new_node()
        loaded = restarted.on_load_draft(None, self.response())
        self.assertTrue(loaded.success, loaded.message)
        self.assertEqual(restarted.editor.points, n.editor.points)
        self.assertEqual(restarted.editor.revision, 2)
        self.assertIsNone(restarted.preview)
        def fake_preview(points, robot):
            return NS(poses=[NS(pose=NS(position=NS(x=x, y=y)))
                             for x, y in [(0., 0.), *points]])
        restarted.build_preview = fake_preview
        restarted.route_pub = NS(publish=lambda _: None)
        self.assertTrue(restarted.on_preview(None, self.response()).success)
        self.assertIsNotNone(restarted.preview)
        self.assertIn('OFFLINE', restarted.status_text)
        self.assertFalse(restarted.on_start(None, self.response()).success)
        self.assertEqual(restarted.follow_client.goals, [])
        self.assertTrue(restarted.on_save_draft(None, self.response()).success)

    def test_reopen_rejects_changed_version_without_mutation(self):
        n = self.node
        n.editor.replace([(0., 0.), (1., 0.), (1., 1.)], 0)
        self.assertTrue(n.on_save_draft(None, self.response()).success)
        restarted = self.new_node()
        self.params['map_version'] = 'rev2'
        outcome = restarted.on_load_draft(None, self.response())
        self.assertFalse(outcome.success)
        self.assertEqual(restarted.editor.points, [])
        self.assertIn('mismatch', outcome.message)


if __name__ == '__main__':
    unittest.main()
