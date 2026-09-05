import time

import rclpy
from agt_base_control.cmd_vel_guard import CmdVelGuard
from agt_robot_interfaces.msg import LocalizationStatus
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


EPS = 1.0e-6


class Probe(Node):
    def __init__(self, input_topic, output_topic, status_topic):
        super().__init__('agt_cmd_vel_guard_acceptance_probe')
        self.cmd_pub = self.create_publisher(Twist, input_topic, 20)
        self.status_pub = self.create_publisher(LocalizationStatus, status_topic, 20)
        self.samples = []
        self.create_subscription(Twist, output_topic, self._on_output, 50)

    def _on_output(self, msg):
        self.samples.append((time.monotonic(), float(msg.linear.x), float(msg.angular.z)))

    def status(self, state):
        msg = LocalizationStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.state = int(state)
        msg.local_odom_fresh = True
        msg.global_correction_valid = state == LocalizationStatus.STATE_LOCALIZED
        msg.reason = 'guard_acceptance'
        return msg

    @staticmethod
    def cmd(linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        return msg


def run_phase(executor, probe, duration, state, cmd=None):
    end = time.monotonic() + duration
    next_pub = 0.0
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_pub:
            probe.status_pub.publish(probe.status(state))
            if cmd is not None:
                probe.cmd_pub.publish(cmd)
            next_pub = now + 0.02
        executor.spin_once(timeout_sec=0.005)


def abs_motion(sample):
    return max(abs(sample[1]), abs(sample[2]))


def main():
    rclpy.init()
    guard = CmdVelGuard()
    probe = Probe(
        str(guard.get_parameter('input_topic').value),
        str(guard.get_parameter('output_topic').value),
        str(guard.get_parameter('localization_status_topic').value),
    )
    executor = SingleThreadedExecutor()
    executor.add_node(guard)
    executor.add_node(probe)

    # DDS discovery and a known-safe initial state.
    run_phase(executor, probe, 0.6, LocalizationStatus.STATE_LOCALIZED, Probe.cmd())

    probe.samples.clear()
    run_phase(
        executor, probe, 1.0, LocalizationStatus.STATE_LOCALIZED,
        Probe.cmd(linear=0.30, angular=0.30),
    )
    opened_peak = max((abs_motion(s) for s in probe.samples), default=0.0)
    if opened_peak < 0.10:
        raise RuntimeError(f'guard never opened for fresh localized command: peak={opened_peak:.4f}')

    # Keep feeding non-zero upstream commands while forcing LOST. The guard must
    # hard-stop and must not cache those commands for later replay.
    lost_at = time.monotonic()
    probe.status_pub.publish(probe.status(LocalizationStatus.STATE_LOST))
    run_phase(
        executor, probe, 0.45, LocalizationStatus.STATE_LOST,
        Probe.cmd(linear=0.30, angular=0.30),
    )
    lost_samples = [s for s in probe.samples if s[0] >= lost_at]
    first_zero = next((s for s in lost_samples if abs_motion(s) <= EPS), None)
    if first_zero is None:
        raise RuntimeError('LOST did not produce a zero output')
    stop_latency = first_zero[0] - lost_at
    settled = [s for s in lost_samples if s[0] >= lost_at + 0.08]
    lost_peak_after_settle = max((abs_motion(s) for s in settled), default=0.0)
    if lost_peak_after_settle > EPS:
        raise RuntimeError(
            f'non-zero /mux/cmd_vel persisted while LOST: peak={lost_peak_after_settle:.6f}')

    # Reopen localization without sending any new velocity command. This is the
    # stale-replay check: output must remain exactly zero beyond command timeout.
    reopen_at = time.monotonic()
    run_phase(executor, probe, 0.55, LocalizationStatus.STATE_LOCALIZED, cmd=None)
    reopen_samples = [s for s in probe.samples if s[0] >= reopen_at + 0.05]
    stale_replay_peak = max((abs_motion(s) for s in reopen_samples), default=0.0)
    if stale_replay_peak > EPS:
        raise RuntimeError(
            f'old command replayed after localization recovery: peak={stale_replay_peak:.6f}')

    # A new post-recovery command must work, proving that the gate is not stuck.
    fresh_at = time.monotonic()
    run_phase(
        executor, probe, 0.65, LocalizationStatus.STATE_LOCALIZED,
        Probe.cmd(linear=0.20, angular=-0.20),
    )
    fresh_samples = [s for s in probe.samples if s[0] >= fresh_at]
    fresh_peak = max((abs_motion(s) for s in fresh_samples), default=0.0)
    if fresh_peak < 0.08:
        raise RuntimeError(f'fresh command did not resume after recovery: peak={fresh_peak:.4f}')

    print(
        'GUARD_ACCEPTANCE PASS '
        f'opened_peak={opened_peak:.3f} '
        f'lost_stop_latency_ms={stop_latency * 1000.0:.1f} '
        f'lost_peak_after_80ms={lost_peak_after_settle:.6f} '
        f'stale_replay_peak={stale_replay_peak:.6f} '
        f'fresh_resume_peak={fresh_peak:.3f}',
        flush=True,
    )

    executor.remove_node(probe)
    executor.remove_node(guard)
    probe.destroy_node()
    guard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
