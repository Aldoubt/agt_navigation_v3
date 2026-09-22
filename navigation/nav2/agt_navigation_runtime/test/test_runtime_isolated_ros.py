"""Opt-in real-executor tests with fake Actions only; never launch hardware.

AGT_RUN_ISOLATED_ROS_TESTS=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=<nonzero>
All action/topic/service/TF endpoints use a per-fixture random test prefix.
"""
import base64
import csv
import json
import os
from pathlib import Path
import threading
import time
import uuid

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from agt_robot_interfaces.action import ExecuteInspectionMission
from agt_robot_interfaces.msg import MissionStatus
from camera_gimbal_interfaces.action import AcquireView
from agt_navigation_runtime.mission_runtime import MissionRuntime

pytestmark = pytest.mark.skipif(
    os.environ.get('AGT_RUN_ISOLATED_ROS_TESTS') != '1',
    reason='explicit isolated-domain opt-in required; default tests create no ROS nodes')

# A generated 1x1 PNG fixture, not a camera/field image.
PNG = base64.b64decode(
    'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAusB9Wl6'
    'qQAAAABJRU5ErkJggg==')


def wait(predicate, timeout=5.0, label='condition'):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError(f'timed out waiting for {label}')


class FakePeers(Node):
    def __init__(self, ctx, prefix, remaps, root, map_frame, base_frame):
        super().__init__('fake_peers_' + prefix.rsplit('_', 1)[-1], context=ctx, cli_args=remaps)
        self.group = ReentrantCallbackGroup()
        self.root = root
        self.map_frame, self.base_frame = map_frame, base_frame
        self.nav_mode = 'success'
        self.camera_mode = 'success'
        self.camera_accept_delay = 0.0
        self.nav_accept_delay = 0.0
        self.fail_camera_tag = None
        self.reject_nav = False
        self.nav_fail_at = None
        self.reject_cancel = False
        self.ignore_cancel_terminal = False
        self.always_moving = False
        self.odom_enabled = True
        self.closed = threading.Event()
        self.release = threading.Event()
        self.nav_goals, self.camera_goals = [], []
        self.cancel_requests = 0
        self.active_nav = 0
        self._x = 0.0
        self._last_tick = time.monotonic()
        self.nav = ActionServer(self, NavigateToPose, prefix + '/nav', self.nav_execute,
            goal_callback=self.nav_goal, cancel_callback=self.cancel_goal,
            callback_group=self.group)
        self.camera = ActionServer(self, AcquireView, prefix + '/camera', self.camera_execute,
            goal_callback=self.camera_goal, cancel_callback=self.cancel_goal,
            callback_group=self.group)
        self.odom = self.create_publisher(Odometry, prefix + '/odom', 20)
        self.create_timer(0.02, self.publish_odom, callback_group=self.group)
        self.tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = map_frame
        transform.child_frame_id = base_frame
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)

    def nav_goal(self, _request):
        if self.nav_accept_delay:
            time.sleep(self.nav_accept_delay)
        return GoalResponse.REJECT if self.reject_nav else GoalResponse.ACCEPT

    def camera_goal(self, _request):
        if self.camera_accept_delay:
            time.sleep(self.camera_accept_delay)
        return GoalResponse.ACCEPT

    def cancel_goal(self, _goal):
        self.cancel_requests += 1
        return CancelResponse.REJECT if self.reject_cancel else CancelResponse.ACCEPT

    def publish_odom(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        if not self.odom_enabled:
            return
        moving = self.always_moving or self.active_nav > 0
        if moving:
            self._x += 0.15 * dt
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'test_odom'
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 0.15 if moving else 0.0
        self.odom.publish(msg)

    def _execute(self, handle, result, mode, success_seconds=0.05):
        started = time.monotonic()
        while not self.closed.is_set() and time.monotonic() - started < 5.0:
            if handle.is_cancel_requested and not self.ignore_cancel_terminal:
                handle.canceled()
                return result
            if self.release.is_set() or (mode != 'hang' and time.monotonic() - started >= success_seconds):
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.succeed()
                return result
            time.sleep(0.005)
        if handle.is_cancel_requested:
            handle.canceled()
        elif handle.is_active:
            handle.abort()
        return result

    def nav_execute(self, handle):
        self.nav_goals.append(handle)
        self.active_nav += 1
        try:
            if self.nav_fail_at is not None and len(self.nav_goals)==self.nav_fail_at:
                handle.abort()
                return NavigateToPose.Result()
            return self._execute(handle, NavigateToPose.Result(), self.nav_mode)
        finally:
            self.active_nav -= 1

    def camera_execute(self, handle):
        self.camera_goals.append(handle)
        result = AcquireView.Result()
        fail = self.camera_mode == 'fail' or handle.request.tag == self.fail_camera_tag
        if fail:
            result.success = False
            result.error_code = 301
            result.message = 'SIMULATED_NO_FRESH_IMAGE'
            handle.abort()
            return result
        result.success = self.camera_mode != 'bad_success_flag'
        result.error_code = 0
        result.message = 'synthetic test frame only'
        result.actual_heading = handle.request.heading
        result.actual_roll = handle.request.roll
        result.actual_pitch = handle.request.pitch
        result.image_stamp = self.get_clock().now().to_msg()
        if self.camera_mode == 'missing_image':
            result.image_path = str(self.root / 'nonexistent.png')
        elif handle.request.save_image:
            target = self.root / ('fake_' + uuid.uuid4().hex + '.png')
            target.write_bytes(PNG)
            result.image_path = str(target)
        return self._execute(handle, result, self.camera_mode)


class Rig:
    def __init__(self, tmp_path):
        assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'local-only ROS required'
        assert os.environ.get('ROS_DOMAIN_ID') not in {None, '', '0'}, 'non-production test domain required'
        self.root = tmp_path
        self.ctx = Context()
        rclpy.init(context=self.ctx)
        self.prefix = '/agt_camera_fix_' + uuid.uuid4().hex[:10]
        key = self.prefix.rsplit('_', 1)[-1]
        self.map_frame, self.base_frame = 'test_map_' + key, 'test_base_' + key
        remaps = ['--ros-args']
        for source, dest in {
            '/agt/mission/status': self.prefix + '/status',
            '/agt/task/start': self.prefix + '/start',
            '/agt/task/pause': self.prefix + '/pause',
            '/agt/task/cancel': self.prefix + '/cancel',
            '/agt/mission/reset_fault': self.prefix + '/reset_fault',
            '/tf': self.prefix + '/tf', '/tf_static': self.prefix + '/tf_static',
            '/clock': self.prefix + '/clock',
        }.items():
            remaps += ['-r', source + ':=' + dest]
        params = {
            'mission_action': self.prefix + '/mission',
            'nav_action': self.prefix + '/nav',
            'acquire_view_action': self.prefix + '/camera',
            'local_odom_topic': self.prefix + '/odom',
            'navsat_topic': self.prefix + '/gnss',
            'hmi_task_request_topic': self.prefix + '/request',
            'hmi_task_status_topic': self.prefix + '/hmi_status',
            'global_frame': self.map_frame, 'base_frame': self.base_frame,
            'record_root': str(tmp_path / 'records'),
            'nav_action_timeout_sec': 0.7, 'camera_action_timeout_sec': 0.7,
            'nav_server_timeout_sec': 0.4, 'camera_server_timeout_sec': 0.4,
            'goal_response_timeout_sec': 0.2, 'cancel_confirm_timeout_sec': 0.3,
            'runtime_poll_interval_sec': 0.01,
            'stationary_hold_sec': 0.06, 'stationary_timeout_sec': 0.4,
            'odom_freshness_sec': 0.15, 'tf_lookup_timeout_sec': 0.1,
        }
        self.runtime = MissionRuntime(context=self.ctx, cli_args=remaps,
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        assert self.runtime.get_parameter('nav_action').value.startswith(self.prefix)
        assert self.runtime.get_parameter('acquire_view_action').value.startswith(self.prefix)
        self.peers = FakePeers(self.ctx, self.prefix, remaps, tmp_path,
                              self.map_frame, self.base_frame)
        self.client_node = Node('test_client_' + key, context=self.ctx, cli_args=remaps)
        self.client = ActionClient(self.client_node, ExecuteInspectionMission, self.prefix + '/mission')
        self.statuses, self.hmi_statuses = [], []
        self.client_node.create_subscription(MissionStatus, self.prefix + '/status', self.statuses.append, 20)
        self.client_node.create_subscription(String, self.prefix + '/hmi_status',
            lambda m: self.hmi_statuses.append(json.loads(m.data)), 20)
        self.executor = MultiThreadedExecutor(num_threads=8, context=self.ctx)
        for node in (self.runtime, self.peers, self.client_node):
            self.executor.add_node(node)
        self.spin_error = None
        def spin():
            try:
                self.executor.spin()
            except ExternalShutdownException:
                pass
            except Exception as exc:
                self.spin_error = exc
        self.thread = threading.Thread(target=spin, daemon=True)
        self.thread.start()
        wait(lambda: self.client.server_is_ready() and self.runtime.nav_client.server_is_ready()
             and self.runtime.camera_client.server_is_ready(), timeout=5, label='isolated fake Actions')
        wait(lambda: self.runtime.latest_pose_linear_mps is not None, label='fake odometry')
        self.handles = []

    def parameter(self, name, value):
        result = self.runtime.set_parameters([Parameter(name, value=value)])
        assert result[0].successful

    def mission(self, views=1, points=1, return_home=False, optional_first=False):
        name = 'test_' + uuid.uuid4().hex[:10]
        entries = []
        for i in range(points):
            entries.append({'id':f'P{i+1:03d}',
                'pose':{'x':0.0,'y':0.0,'yaw':0.0,'frame_id':self.map_frame},
                'settle_time':0.01,
                'views':[{'tag':f'v{j}', 'heading':0.0, 'roll':0.0,'pitch':0.0,
                          'required':not (optional_first and j==0), 'save_image':True}
                         for j in range(views)]})
        if return_home:
            entries.append({'id':'RETURN_HOME','pose':{'x':0.0,'y':0.0,'frame_id':self.map_frame},
                            'settle_time':0.0,'views':[]})
        path=self.root/(name+'.yaml')
        path.write_text(yaml.safe_dump({'version':1,'mission_id':name,'map_id':'mock_map', 'points':entries}))
        return name,path

    def submit(self, path, resume=False):
        goal=ExecuteInspectionMission.Goal();goal.mission_file=str(path);goal.resume=resume
        future=self.client.send_goal_async(goal)
        wait(future.done,label='parent goal response')
        handle=future.result()
        if handle.accepted:self.handles.append(handle)
        return handle

    def result(self, handle, timeout=5):
        future=handle.get_result_async();wait(future.done,timeout,label='parent result')
        return future.result()

    def cancel(self,handle):
        future=handle.cancel_goal_async();wait(future.done,label='parent cancel acknowledgement')
        assert future.result().goals_canceling

    def service(self,name):
        client=self.client_node.create_client(Trigger,self.prefix+'/'+name)
        wait(client.service_is_ready,label='service '+name)
        future=client.call_async(Trigger.Request());wait(future.done,label='service reply '+name)
        value=future.result();self.client_node.destroy_client(client);return value

    def record(self,name):
        matches=list((self.root/'records').glob(name+'_*'));assert len(matches)==1
        root=matches[0]
        return root,json.loads((root/'manifest.json').read_text())

    def terminal_status_seen(self,expected):
        wait(lambda:any(s.state==expected for s in self.statuses),label='terminal typed status')
        wait(lambda:any(s.get('terminal') and s['state']==expected for s in self.hmi_statuses),
             label='terminal HMI status')

    def close(self):
        self.peers.always_moving=False;self.peers.odom_enabled=True
        self.peers.ignore_cancel_terminal=False;self.peers.reject_cancel=False;self.peers.release.set()
        for handle in self.handles:
            if handle.status not in (4,5,6):
                try:handle.cancel_goal_async()
                except Exception:pass
        deadline=time.monotonic()+1.5
        while self.runtime.active_goal_handle is not None and time.monotonic()<deadline:
            time.sleep(0.02)
        self.runtime.prepare_shutdown();self.peers.closed.set()
        self.executor.shutdown(timeout_sec=3)
        self.thread.join(timeout=3)
        for node in (self.runtime,self.peers,self.client_node):node.destroy_node()
        self.ctx.try_shutdown()
        assert not self.thread.is_alive(), 'test executor did not stop'
        assert self.spin_error is None, repr(self.spin_error)


@pytest.fixture
def rig(tmp_path):
    instance=Rig(tmp_path)
    try:yield instance
    finally:instance.close()


def test_ros_three_views_and_return_home_complete(rig):
    name,path=rig.mission(views=3,return_home=True)
    result=rig.result(rig.submit(path))
    assert result.status==GoalStatus.STATUS_SUCCEEDED and result.result.success
    assert result.result.completed_points==2
    directory,manifest=rig.record(name)
    assert manifest['status']=='completed' and manifest['capture_records']==3
    assert len(manifest['planned_views'])==3
    rows=list(csv.DictReader((directory/'captures.csv').open()))
    assert len(rows)==3 and all(Path(x['image_path']).is_file() for x in rows)
    assert all(x['pose_valid']=='True' for x in rows)
    assert len(rig.peers.nav_goals)==2 and len(rig.peers.camera_goals)==3
    rig.terminal_status_seen(MissionStatus.COMPLETED)


def test_ros_required_camera_failure_persists_native_error_before_abort(rig):
    rig.peers.camera_mode='fail'
    name,path=rig.mission(views=3,points=2,return_home=True)
    result=rig.result(rig.submit(path))
    assert result.status==GoalStatus.STATUS_ABORTED and result.result.error_code==301
    directory,m=rig.record(name)
    assert m['status']=='failed' and m['capture_records']==1 and m['completed_points']==0
    record=json.loads((directory/'captures.jsonl').read_text().splitlines()[0])
    assert record['camera_error_code']==301 and record['camera_error_message']=='SIMULATED_NO_FRESH_IMAGE'
    assert not record['pose_valid']
    assert len(rig.peers.nav_goals)==1 and len(rig.peers.camera_goals)==1
    rig.terminal_status_seen(MissionStatus.ERROR)


def test_ros_optional_failure_remains_explicit_but_can_continue(rig):
    rig.peers.fail_camera_tag='v0'
    name,path=rig.mission(views=2,optional_first=True)
    result=rig.result(rig.submit(path))
    assert result.result.success
    directory,m=rig.record(name)
    rows=list(csv.DictReader((directory/'captures.csv').open()))
    assert m['status']=='completed' and [x['camera_error_code'] for x in rows]==['301','0']


def test_ros_nav_result_timeout_cancels_and_confirms_stop(rig):
    rig.peers.nav_mode='hang';rig.parameter('nav_action_timeout_sec',0.2)
    name,path=rig.mission(points=2)
    started=time.monotonic();result=rig.result(rig.submit(path))
    assert time.monotonic()-started<2.0
    assert result.status==GoalStatus.STATUS_ABORTED and result.result.error_code==1200
    _,m=rig.record(name)
    assert m['status']=='failed' and m['child_actions_terminal'] and m['base_stationary']
    assert len(rig.peers.nav_goals)==1 and not rig.peers.camera_goals
    assert rig.peers.cancel_requests>=1


def test_ros_camera_result_timeout_is_bounded_and_recorded(rig):
    rig.peers.camera_mode='hang';rig.parameter('camera_action_timeout_sec',0.2)
    name,path=rig.mission(views=2,points=2)
    result=rig.result(rig.submit(path))
    assert result.status==GoalStatus.STATUS_ABORTED and result.result.error_code==1201
    directory,m=rig.record(name)
    assert m['status']=='failed' and m['capture_records']==1 and not m['fault_latched']
    assert json.loads((directory/'captures.jsonl').read_text().splitlines()[0])['camera_error_code']==1201
    assert len(rig.peers.nav_goals)==1 and len(rig.peers.camera_goals)==1


@pytest.mark.parametrize('during',['nav','camera'])
def test_ros_action_cancel_is_canceled_not_required_photo_abort(rig,during):
    if during=='nav':rig.peers.nav_mode='hang'
    else:rig.peers.camera_mode='hang'
    name,path=rig.mission(views=2,points=2)
    handle=rig.submit(path)
    wait(lambda:bool(rig.peers.nav_goals if during=='nav' else rig.peers.camera_goals),label='child started')
    rig.cancel(handle);result=rig.result(handle)
    assert result.status==GoalStatus.STATUS_CANCELED and result.result.error_code==400
    _,m=rig.record(name)
    assert m['status']=='canceled' and m['base_stationary']
    assert len(rig.peers.nav_goals)==1
    rig.terminal_status_seen(MissionStatus.CANCELED)


def test_ros_hmi_cancel_uses_parent_action_cancel_protocol(rig):
    rig.peers.nav_mode='hang'
    name,path=rig.mission()
    handle=rig.submit(path);wait(lambda:bool(rig.peers.nav_goals))
    assert rig.service('cancel').success
    result=rig.result(handle)
    assert result.status==GoalStatus.STATUS_CANCELED and result.result.error_code==400
    assert rig.record(name)[1]['status']=='canceled'


@pytest.mark.parametrize('reject_cancel',[False,True])
def test_ros_unconfirmed_cancel_latches_and_rejects_new_mission(rig,reject_cancel):
    rig.peers.nav_mode='hang';rig.peers.ignore_cancel_terminal=True
    rig.peers.reject_cancel=reject_cancel
    rig.parameter('nav_action_timeout_sec',0.16)
    rig.parameter('cancel_confirm_timeout_sec',0.12)
    name,path=rig.mission()
    result=rig.result(rig.submit(path))
    assert result.result.error_code==1300 and result.status==GoalStatus.STATUS_ABORTED
    assert rig.record(name)[1]['fault_latched']
    _,next_path=rig.mission()
    assert not rig.submit(next_path).accepted
    assert not rig.service('reset_fault').success
    rig.peers.release.set()
    wait(lambda:all(op.termination_confirmed() for op in rig.runtime._fault_operations))
    time.sleep(0.12)
    assert rig.service('reset_fault').success
    rig.peers.nav_mode='success';rig.peers.ignore_cancel_terminal=False;rig.peers.reject_cancel=False
    rig.parameter('nav_action_timeout_sec',0.7)
    assert rig.result(rig.submit(next_path)).result.success


def test_ros_late_goal_acceptance_is_canceled_after_handshake_timeout(rig):
    rig.peers.camera_mode='hang';rig.peers.camera_accept_delay=0.45
    rig.parameter('goal_response_timeout_sec',0.08)
    rig.parameter('cancel_confirm_timeout_sec',0.1)
    name,path=rig.mission()
    result=rig.result(rig.submit(path))
    assert result.result.error_code==1300 and rig.record(name)[1]['fault_latched']
    wait(lambda:rig.peers.cancel_requests>0,label='late accepted goal cancellation')
    wait(lambda:all(op.termination_confirmed() for op in rig.runtime._fault_operations))
    assert rig.peers.camera_goals[0].status == GoalStatus.STATUS_CANCELED


def test_ros_stale_odom_does_not_allow_capture_or_fault_reset(rig):
    rig.peers.odom_enabled=False
    time.sleep(0.2)
    name,path=rig.mission()
    result=rig.result(rig.submit(path))
    assert result.result.error_code==1301 and not rig.peers.camera_goals
    assert rig.record(name)[1]['fault_latched']
    assert not rig.service('reset_fault').success


def test_ros_missing_image_cannot_be_success(rig):
    rig.peers.camera_mode='missing_image'
    name,path=rig.mission()
    result=rig.result(rig.submit(path))
    assert result.status==GoalStatus.STATUS_ABORTED and result.result.error_code==1400
    assert rig.record(name)[1]['status']=='failed'


def test_ros_missing_camera_action_has_discovery_timeout(rig):
    # Change only the client binding in the isolated test; no real camera endpoint is used.
    rig.runtime.camera_client.destroy()
    rig.runtime.camera_client=ActionClient(rig.runtime,AcquireView,rig.prefix+'/absent_camera',
                                           callback_group=rig.runtime._control_group)
    rig.parameter('camera_server_timeout_sec',0.1)
    name,path=rig.mission()
    result=rig.result(rig.submit(path))
    assert result.result.error_code==1201 and not rig.peers.camera_goals
    assert rig.record(name)[1]['status']=='failed'


def test_ros_paused_stage_can_be_canceled_without_asyncio_loop(rig):
    name,path=rig.mission()
    handle=rig.submit(path)
    wait(lambda:rig.runtime.active_goal_handle is not None)
    # The public pause is intentionally a stage-boundary pause, not emergency stop.
    assert rig.service('pause').success
    wait(lambda:any(s.state==MissionStatus.PAUSED for s in rig.statuses),label='paused boundary')
    rig.cancel(handle);result=rig.result(handle)
    assert result.status==GoalStatus.STATUS_CANCELED
    assert rig.record(name)[1]['status']=='canceled'


def test_ros_overlapping_mission_and_unimplemented_resume_are_rejected(rig):
    rig.peers.nav_mode='hang'
    _,path=rig.mission();handle=rig.submit(path)
    wait(lambda:bool(rig.peers.nav_goals))
    _,second=rig.mission()
    assert not rig.submit(second).accepted
    rig.cancel(handle);rig.result(handle)
    assert not rig.submit(second,resume=True).accepted


def test_ros_cancel_during_handshake_cancels_the_late_accepted_goal(rig):
    rig.peers.camera_mode='hang';rig.peers.camera_accept_delay=0.22
    rig.parameter('goal_response_timeout_sec',0.5)
    rig.parameter('cancel_confirm_timeout_sec',0.5)
    name,path=rig.mission()
    handle=rig.submit(path)
    wait(lambda:rig.runtime._active_operation is not None
         and rig.runtime._active_operation.label=='camera'
         and rig.runtime._active_operation.send_requested
         and not rig.runtime._active_operation.send_processed, label='pending camera handshake')
    rig.cancel(handle);result=rig.result(handle)
    assert result.status==GoalStatus.STATUS_CANCELED
    assert rig.record(name)[1]['status']=='canceled'
    assert rig.peers.camera_goals[0].status==GoalStatus.STATUS_CANCELED


def test_ros_rejected_nav_goal_is_terminal_failure_without_camera(rig):
    rig.peers.reject_nav=True
    name,path=rig.mission()
    result=rig.result(rig.submit(path))
    assert result.result.error_code==1000 and result.status==GoalStatus.STATUS_ABORTED
    assert not rig.peers.camera_goals and rig.record(name)[1]['status']=='failed'


def test_ros_total_timeout_works_when_simulation_clock_is_not_running(rig):
    rig.parameter('use_sim_time',True)
    rig.parameter('camera_action_timeout_sec',0.16)
    rig.peers.camera_mode='hang'
    name,path=rig.mission()
    started=time.monotonic();result=rig.result(rig.submit(path))
    assert time.monotonic()-started<2.0
    assert result.result.error_code==1201 and rig.record(name)[1]['status']=='failed'


def test_ros_invalid_or_mid_mission_timeout_changes_are_rejected(rig):
    for value in [float('nan'),float('inf'),-1.0,0.0]:
        result=rig.runtime.set_parameters([Parameter('camera_action_timeout_sec',value=value)])
        assert not result[0].successful
    rig.peers.nav_mode='hang'
    _,path=rig.mission();handle=rig.submit(path)
    wait(lambda:bool(rig.peers.nav_goals))
    changed=rig.runtime.set_parameters([Parameter('camera_action_timeout_sec',value=9999.0)])
    assert not changed[0].successful
    rig.cancel(handle);rig.result(handle)


def test_ros_inconsistent_camera_failure_flag_is_not_error_zero(rig):
    rig.peers.camera_mode='bad_success_flag'
    name,path=rig.mission()
    result=rig.result(rig.submit(path))
    assert not result.result.success and result.result.error_code==202
    assert rig.record(name)[1]['status']=='failed'


def test_ros_return_home_failure_cannot_be_validated_as_photo_success(rig):
    from agt_navigation_runtime.validate_records import validate
    from agt_navigation_runtime.generate_demo_report import build_report
    rig.peers.nav_fail_at=2
    name,path=rig.mission(views=3,return_home=True)
    result=rig.result(rig.submit(path))
    assert result.status==GoalStatus.STATUS_ABORTED and result.result.error_code==1000
    directory,m=rig.record(name)
    assert m['capture_records']==3 and m['completed_points']==1 and m['status']=='failed'
    errors=validate(directory,expected_points=1,views_per_point=3,require_rtk=False)
    assert any('mission status=failed' in x for x in errors)
    assert 'NOT a certified complete mission' in build_report(directory)
