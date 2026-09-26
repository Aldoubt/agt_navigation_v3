import json
import math
import time
from types import SimpleNamespace as NS

import pytest
import rclpy
from agt_navigation_interfaces.msg import NavigationHealth
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

from agt_rviz_patrol.route_editor import RouteEditor
from agt_rviz_patrol.workbench import RouteWorkbench


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = RouteWorkbench()
    yield n
    # Tests can replace the action client with a stub.
    if hasattr(n.follow_client, 'destroy'):
        n.follow_client.destroy()
    n.tf_listener.unregister()
    n.destroy_node()


def call(fn):
    return fn(Trigger.Request(), Trigger.Response())


def edit(n, points, revision=None):
    n.on_edit(String(data=json.dumps(dict(frame='map', revision=n.editor.revision if revision is None else revision,
                                         points=points))))


def prepare(n):
    n.map_key = ('test_map',)
    n.robot_pose = lambda: (0., 0., 0.)
    edit(n, [(0.,0.), (1.,0.), (1.,1.)])


def live_preview(n):
    prepare(n)
    assert call(n.on_preview).success
    n.set_parameters([Parameter('preview_only', value=False)])
    n.health_ok = lambda: True


def test_editor_undo_redo_and_revision():
    e=RouteEditor();e.replace([(0,0),(1,1)],0);e.undo();assert e.points==[]
    assert e.revision==2;e.redo();assert e.points==[(0,0),(1,1)]
    with pytest.raises(ValueError):e.replace([],0)


@pytest.mark.parametrize('points', [[(math.nan,0)],[(math.inf,1)],[(100001,0)],[(0,0),(201,0)],[(0,)],[(i*.01,0) for i in range(2001)]])
def test_editor_rejects_bad_input(points):
    e=RouteEditor()
    with pytest.raises(ValueError):e.replace(points,0)
    assert e.revision==0 and e.points==[]


def test_duplicate_points_are_coalesced():
    e=RouteEditor();e.replace([(0,0),(0,0),(1,0)],0)
    assert e.points==[(0,0),(1,0)]


def test_history_is_bounded():
    e=RouteEditor()
    for i in range(50):e.replace([(i,0)],e.revision)
    assert len(e.history)==20


def test_default_offline_never_executes(node):
    prepare(node)
    assert call(node.on_preview).success
    assert node.preview is not None
    assert not call(node.on_start).success
    assert not call(node.on_resume).success


def test_wheel_diagnostic_cannot_block_route_health(node):
    node.health = NS(status=NavigationHealth.READY, platform_ready=True,
                     lidar_alive=True, imu_alive=True, base_alive=False,
                     odom_alive=True, localized=True, nav2_active=True)
    node.health_received = time.monotonic()
    assert node.health_ok()
    node.health.odom_alive = False
    assert not node.health_ok()


def test_stale_edit_and_nan_leave_draft_intact(node):
    prepare(node);before=node.editor.points[:]
    edit(node,[(3,3)],revision=0);assert node.editor.points==before
    edit(node,[(math.nan,0)]);assert node.editor.points==before


def test_single_vertex_marker_and_empty_path(node):
    edit(node,[(0.,0.)]);assert node.editor.points==[(0.,0.)]
    # Vertex Marker, not an invalid one-pose route, provides first-point feedback.
    assert node.marker_pub.get_subscription_count()==0
    assert len(node.route_path().poses)==0


def test_busy_and_paused_draft_locked(node):
    prepare(node);old=node.editor.points[:]
    node.start_pending=True;edit(node,[(0.,0.),(3.,0.)]);assert node.editor.points==old
    assert not call(node.on_undo).success
    node.start_pending=False;node.paused_path=node.route_path()
    assert not call(node.on_clear_route).success
    call(node.on_cancel);assert call(node.on_clear_route).success


def test_preview_requires_map_and_bounded_connector(node):
    node.robot_pose=lambda:(0.,0.,0.);edit(node,[(10,5),(11,5)])
    assert not call(node.on_preview).success
    node.map_key=('map',)
    assert not call(node.on_preview).success
    assert node.preview is None


def test_preview_connector_densely_sampled(node):
    node.robot_pose=lambda:(0.,0.,0.);node.map_key=('map',)
    edit(node,[(.5,.5),(1.,.5)]);assert call(node.on_preview).success
    poses=node.preview['path'].poses
    assert poses[0].pose.position.x==0.
    assert all(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y)<=.050001 for a,b in zip(poses,poses[1:]))


def test_edit_invalidates_preview(node):
    prepare(node);assert call(node.on_preview).success
    edit(node,[(0,0),(2,0)]);assert node.preview is None


def test_map_update_invalidates_preview(node):
    prepare(node);assert call(node.on_preview).success
    m=OccupancyGrid();m.header.frame_id='map';m.info.width=1;m.info.height=1;m.info.resolution=.05;m.data=[0]
    node.on_map(m);assert node.preview is None


def test_validation_token_ignores_stale_response(node):
    prepare(node);node.validating=True;token=node.validation_token
    node.invalidate()
    node.on_validated(NS(result=lambda:NS(is_valid=True)),token,{'bad':'snapshot'})
    assert node.preview is None


def test_changed_pose_rejects_execute(node):
    live_preview(node);node.robot_pose=lambda:(.2,0.,0.)
    assert not call(node.on_start).success


def test_execute_uses_exact_preview(node):
    live_preview(node);path=node.preview['path'];saved=[]
    node.submit_path=lambda p:saved.append(p) or True
    assert call(node.on_start).success
    assert saved[0]==path and saved[0] is not path
    assert node.preview is None


def test_expired_preview_rejected(node):
    live_preview(node);node.preview['time']-=31
    assert not call(node.on_start).success


def test_missing_tf_nonblocking(node):
    start=time.perf_counter();assert node.current_map_pose() is None
    assert time.perf_counter()-start < .05


class GoalHandle:
    accepted=True
    def __init__(self):self.cancels=0
    def get_result_async(self):return NS(add_done_callback=lambda cb:None)
    def cancel_goal_async(self):
        self.cancels+=1
        return NS(add_done_callback=lambda cb:cb(NS(result=lambda:NS(goals_canceling=[1]))))


def test_pending_cancel_latched_until_acceptance(node):
    node.start_pending=True
    assert call(node.on_cancel).success
    assert node.cancel_pending and node.start_pending
    handle=GoalHandle();node.on_goal_response(NS(result=lambda:handle))
    assert handle.cancels==1 and node.active_goal is handle and node.busy()


def test_pending_pause_latched_until_acceptance(node):
    node.start_pending=True;assert call(node.on_pause).success
    handle=GoalHandle();node.on_goal_response(NS(result=lambda:handle))
    assert handle.cancels==1 and node.pause_pending


def test_resume_requires_repreview_and_rejects_relocation(node):
    prepare(node);node.paused_path=node.route_path();node.paused_pose=(0.,0.,0.)
    node.robot_pose=lambda:(0.,3.,0.)
    assert not call(node.on_preview).success
    assert not call(node.on_resume).success


def test_resume_preview_is_dense_and_progress_constrained(node):
    prepare(node);node.paused_path=node.route_path();node.progress=10
    node.paused_pose=(.5,0.,0.);node.robot_pose=lambda:(.5,.1,0.)
    assert call(node.on_preview).success
    assert node.preview['resume']
    p=node.preview['path'].poses
    assert all(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y)<=.050001 for a,b in zip(p,p[1:]))


def test_trails_are_bounded_and_not_published_in_odom_callback(node):
    class Publisher:
        def publish(self,_):raise AssertionError('no synchronous publish per odometry sample')
    node.lio_pub=Publisher()
    for i in range(1500):
        node.current_map_pose=lambda i=i:(i*.1,0.,0.)
        node.on_lio(None)
    assert len(node.lio_points)==1000 and node.lio_dirty


@pytest.mark.parametrize('payload', ['[]', '{}', 'null', '{"frame":"map","revision":true,"points":[]}'])
def test_malformed_edit_rejected_without_callback_exception(node, payload):
    node.on_edit(String(data=payload))
    assert node.editor.revision == 0 and node.editor.points == []


def test_ambiguous_crossing_resume_fails_closed(node):
    prepare(node)
    samples=[(i*.05,0.,0.) for i in range(30)]+[(0.,0.,0.),(.2,0.,0.)]
    node.paused_path=node.make_path(samples);node.progress=0;node.paused_pose=(0.,0.,0.)
    response=call(node.on_preview)
    assert not response.success and 'ambiguous' in response.message



def test_unknown_goal_acceptance_stays_locked(node):
    def failure():raise RuntimeError('transport failure')
    node.start_pending=True
    node.on_goal_response(NS(result=failure))
    assert node.busy() and node.start_pending
    assert not call(node.on_clear_route).success



def test_preview_length_includes_robot_connector(node):
    node.robot_pose=lambda:(0.,0.,0.);node.map_key=('map',)
    edit(node,[(.5,0.),(.5,.5)])
    assert call(node.on_preview).success
    assert node.preview['length_m'] == pytest.approx(1.0)
    assert math.dist(node.editor.points[0],node.editor.points[1]) == pytest.approx(.5)
