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
        self._pending = set()
        self._closed = False

    async def sleep(self, seconds):
        if seconds <= 0:
            return
        future = Future(executor=self.node.executor)
        with self._lock:
            if self._closed:
                raise RuntimeError('ROS wait helper is closed')
            self._pending.add(future)

        def wake():
            with self._lock:
                if not future.done():
                    future.set_result(None)

        timer = None
        try:
            timer = self.node.create_timer(
                max(0.001, float(seconds)), wake,
                callback_group=self.callback_group, clock=self.clock)
            await future
        finally:
            with self._lock:
                self._pending.discard(future)
            if timer is not None:
                try:
                    timer.cancel()
                    self.node.destroy_timer(timer)
                except Exception:
                    if not self._closed:
                        raise

    def close(self):
        """Release pending sleeps during teardown; caller supplies shutdown policy."""
        with self._lock:
            self._closed = True
            for future in tuple(self._pending):
                if not future.done():
                    future.set_result(None)
