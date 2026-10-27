"""Executor-native, steady-clock waits. No asyncio event loop is required."""
from __future__ import annotations

import threading

from rclpy.clock import Clock, ClockType
from rclpy.task import Future


class RosWaiter:
    def __init__(self, node, callback_group):
        self.node = node
        self.callback_group = callback_group
        self.clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._lock = threading.RLock()
        self._pending = {}
        self._closed = False
        # Keep one timer alive for the node's lifetime. Creating/destroying a
        # timer for each await races Humble's MultiThreadedExecutor wait set.
        self._timer = node.create_timer(
            0.01, self._wake_ready, callback_group=callback_group, clock=self.clock)

    def _wake_ready(self):
        now_ns = self.clock.now().nanoseconds
        with self._lock:
            for future, deadline_ns in tuple(self._pending.items()):
                if not future.done() and now_ns >= deadline_ns:
                    future.set_result(None)

    async def sleep(self, seconds):
        if seconds <= 0:
            return
        future = Future(executor=self.node.executor)
        with self._lock:
            if self._closed:
                raise RuntimeError('ROS wait helper is closed')
            self._pending[future] = self.clock.now().nanoseconds + int(seconds * 1e9)
        try:
            await future
        finally:
            with self._lock:
                self._pending.pop(future, None)

    def close(self):
        """Release pending sleeps during teardown; caller supplies shutdown policy."""
        with self._lock:
            self._closed = True
            self._timer.cancel()
            for future in tuple(self._pending):
                if not future.done():
                    future.set_result(None)
