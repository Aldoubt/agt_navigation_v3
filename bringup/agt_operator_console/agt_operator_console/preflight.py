"""ROS graph readiness checks used by the terminal operator console."""

from __future__ import annotations

import time
from typing import Iterable

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_action, get_message

from .profile import CheckResult, CheckSpec


class PreflightProbe(Node):
    def __init__(self) -> None:
        super().__init__('agt_operator_preflight')

    def check(self, spec: CheckSpec) -> CheckResult:
        try:
            if spec.kind == 'topic':
                return self._check_topic(spec)
            return self._check_action(spec)
        except (AttributeError, ImportError, ModuleNotFoundError, TypeError, ValueError) as exc:
            return CheckResult(spec.name, False, f'type_or_graph_error:{exc}')

    def _check_topic(self, spec: CheckSpec) -> CheckResult:
        message_type = get_message(spec.ros_type)
        received = 0

        def callback(_msg) -> None:
            nonlocal received
            received += 1

        subscription = self.create_subscription(message_type, spec.ros_name, callback, 10)
        deadline = time.monotonic() + spec.timeout_sec
        while rclpy.ok() and time.monotonic() < deadline and received < spec.min_messages:
            rclpy.spin_once(self, timeout_sec=min(0.1, deadline - time.monotonic()))
        self.destroy_subscription(subscription)
        if received >= spec.min_messages:
            return CheckResult(spec.name, True, f'{received} messages')
        return CheckResult(spec.name, False, f'{received}/{spec.min_messages} messages before timeout')

    def _check_action(self, spec: CheckSpec) -> CheckResult:
        action_type = get_action(spec.ros_type)
        client = ActionClient(self, action_type, spec.ros_name)
        ready = client.wait_for_server(timeout_sec=spec.timeout_sec)
        client.destroy()
        return CheckResult(spec.name, ready, 'action server ready' if ready else 'action server unavailable')


def run_preflight(checks: Iterable[CheckSpec]) -> list[CheckResult]:
    rclpy.init(args=None)
    probe = PreflightProbe()
    try:
        return [probe.check(check) for check in checks]
    finally:
        probe.destroy_node()
        rclpy.shutdown()
