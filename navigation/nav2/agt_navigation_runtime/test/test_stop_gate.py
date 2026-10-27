import math
from agt_navigation_runtime.stop_gate import StopGate


def gate(speed=0., command=0., drift=0.):
    g = StopGate()
    for i in range(11):
        t = 10+i*.1
        g.pose(t, t, drift*i/10, .003*math.sin(i), 0., 'odom')
    g.wheel = (11., 11., speed, 0.)
    g.command = (11., command, 0.)
    return g


def test_stopped_wheels_and_bounded_pose_noise():
    assert gate().evaluate(11., 11.)[0]


def test_motion_and_commands_and_sliding_rejected():
    for g in (gate(speed=.02), gate(command=.03), gate(drift=.04)):
        assert not g.evaluate(11., 11.)[0]


def test_stale_and_future_feedback_rejected():
    g = gate()
    assert not g.evaluate(12., 12.)[0]
    g.wheel = (12., 11., 0., 0.)
    assert not g.evaluate(11., 11.)[0]
    g.wheel = (11., 11., float('nan'), 0.)
    assert not g.evaluate(11., 11.)[0]


def test_out_and_back_is_not_stationary():
    g = gate()
    g.poses.clear()
    for i in range(11):
        g.pose(10+i*.1, 10+i*.1, .08*math.sin(math.pi*i/10), 0., 0., 'odom')
    assert not g.evaluate(11., 11.)[0]


def test_clock_jump_frame_change_and_gap_restart_window():
    for t, frame in [(10., 'odom'), (11.1, 'different'), (12., 'odom')]:
        g = gate(); g.pose(t, t, 0., 0., 0., frame)
        assert len(g.poses) == 1
        assert not g.evaluate(t, t)[0]


def test_rotation_and_missing_command_rejected():
    g = gate(); g.pose(11.1, 11.1, 0., 0., .08, 'odom')
    assert not g.evaluate(11.1, 11.1)[0]
    g = gate(); g.command = None
    assert not g.evaluate(11., 11.)[0]
