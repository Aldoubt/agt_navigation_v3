"""Bounded ROS Action orchestration, independent of asyncio's event loop.

The caller supplies an executor-compatible async sleeper and a monotonic clock.
A cancel acknowledgement is NOT terminal confirmation. Late accepted goals are
retained and canceled even after the bounded caller has returned a fault.
"""
from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any, Callable

SUCCEEDED, CANCELED, ABORTED = 4, 5, 6
TERMINAL = {SUCCEEDED, CANCELED, ABORTED}


@dataclass
class ActionOutcome:
    state: str
    message: str
    termination_confirmed: bool = True
    wrapped_result: Any = None
    operation: Any = None

    @property
    def success(self):
        return self.state == 'succeeded'


class PendingAction:
    """Own every submitted goal until rejection or an observed terminal state."""

    def __init__(self, label: str, event: Callable | None = None):
        self.label = label
        self._event = event or (lambda *_args, **_kwargs: None)
        self._lock = threading.RLock()
        self.send_requested = False
        self.send_future = None
        self.send_processed = False
        self.handle = None
        self.result_future = None
        self.wrapped_result = None
        self.rejected = False
        self.send_error = ''
        self.result_error = ''
        self.cancel_error = ''
        self.cancel_requested = False
        self.cancel_future = None
        self.cancel_attempted = False
        self.cancel_acknowledged = False
        self._terminal_reported = False

    def emit(self, kind, **values):
        # Telemetry failure must not prevent canceling an already submitted goal.
        try:
            self._event(kind, action=self.label, **values)
        except Exception:
            pass

    def bind_send_future(self, future):
        with self._lock:
            self.send_requested = True
            self.send_future = future
        future.add_done_callback(self._on_send_done)

    def _on_send_done(self, future):
        with self._lock:
            if self.send_processed:
                return
            self.send_processed = True
            try:
                self.handle = future.result()
                if self.handle is None:
                    raise RuntimeError('goal response has no handle')
                self.rejected = not bool(self.handle.accepted)
                if self.rejected:
                    self.emit('action_rejected')
                    return
                self.emit('action_accepted')
                self.result_future = self.handle.get_result_async()
                self.result_future.add_done_callback(self._on_result_done)
            except Exception as exc:
                self.send_error = repr(exc)
                self.emit('action_goal_response_error', error=self.send_error)
            if self.cancel_requested:
                self.request_cancel()

    def _on_result_done(self, future):
        with self._lock:
            try:
                self.wrapped_result = future.result()
                if self.wrapped_result is None:
                    raise RuntimeError('empty action result')
                if not self._terminal_reported:
                    self._terminal_reported = True
                    self.emit('action_result', status=int(self.wrapped_result.status))
            except Exception as exc:
                self.result_error = repr(exc)
                self.emit('action_result_error', error=self.result_error)

    def refresh(self):
        # A Future may be done before its executor-scheduled done callback runs.
        with self._lock:
            if self.send_future is not None and self.send_future.done() and not self.send_processed:
                self._on_send_done(self.send_future)
            if (self.result_future is not None and self.result_future.done()
                    and self.wrapped_result is None and not self.result_error):
                self._on_result_done(self.result_future)

    def terminal_status(self):
        self.refresh()
        with self._lock:
            if self.wrapped_result is not None:
                status = int(self.wrapped_result.status)
                if status in TERMINAL:
                    return status
            if self.handle is not None and self.handle.accepted:
                status = int(getattr(self.handle, 'status', 0))
                if status in TERMINAL:
                    return status
        return None

    def termination_confirmed(self):
        self.refresh()
        return (not self.send_requested) or self.rejected or self.terminal_status() is not None

    def request_cancel(self):
        with self._lock:
            self.cancel_requested = True
            if (self.handle is None or self.rejected or self.cancel_attempted
                    or self.terminal_status() is not None):
                return
            self.cancel_attempted = True
            try:
                self.cancel_future = self.handle.cancel_goal_async()
                self.cancel_future.add_done_callback(self._on_cancel_done)
                self.emit('action_cancel_sent')
            except Exception as exc:
                self.cancel_error = repr(exc)
                self.emit('action_cancel_error', error=self.cancel_error)

    def _on_cancel_done(self, future):
        with self._lock:
            try:
                reply = future.result()
                self.cancel_acknowledged = bool(reply and reply.goals_canceling)
                self.emit('action_cancel_ack', accepted=self.cancel_acknowledged,
                          return_code=getattr(reply, 'return_code', None))
            except Exception as exc:
                self.cancel_error = repr(exc)
                self.emit('action_cancel_error', error=self.cancel_error)

    def snapshot(self):
        self.refresh()
        return {
            'action': self.label, 'goal_sent': self.send_requested,
            'goal_response_received': self.send_processed, 'rejected': self.rejected,
            'cancel_requested': self.cancel_requested,
            'cancel_acknowledged': self.cancel_acknowledged,
            'terminal_status': self.terminal_status(),
            'termination_confirmed': self.termination_confirmed(),
            'send_error': self.send_error, 'result_error': self.result_error,
            'cancel_error': self.cancel_error,
        }


async def cancel_and_confirm(op, *, sleep, timeout, poll_interval=0.05, now=time.monotonic):
    """Bounded grace period, keeping a late-response callback after expiry."""
    deadline = now() + timeout
    op.request_cancel()
    while True:
        op.refresh()
        op.request_cancel()  # Cancels a handle that became available during handshake.
        if op.termination_confirmed():
            return True
        remaining = deadline - now()
        if remaining <= 0:
            op.emit('action_cancel_unconfirmed', snapshot=op.snapshot())
            return False
        await sleep(min(poll_interval, remaining))


async def execute_action(client, goal, op, *, sleep, is_cancel_requested,
                         total_timeout, server_timeout, goal_response_timeout,
                         cancel_timeout, poll_interval=0.05, now=time.monotonic):
    """One total budget covers discovery, goal handshake and execution.

    Cancellation/timeout can use one additional bounded confirmation window.
    There is no unbounded await of a service/action Future.
    """
    deadline = now() + total_timeout
    discovery_deadline = min(deadline, now() + server_timeout)

    async def stop(state, message):
        confirmed = await cancel_and_confirm(
            op, sleep=sleep, timeout=cancel_timeout, poll_interval=poll_interval, now=now)
        return ActionOutcome(state, message, confirmed, op.wrapped_result, op)

    while not client.server_is_ready():
        if is_cancel_requested():
            return await stop('canceled', 'canceled before goal submission')
        remaining = discovery_deadline - now()
        if remaining <= 0:
            return ActionOutcome('timeout', 'action server discovery timed out', True, operation=op)
        await sleep(min(poll_interval, remaining))
    if is_cancel_requested():
        return await stop('canceled', 'canceled before goal submission')
    if now() >= deadline:
        return ActionOutcome('timeout', 'total action budget expired before submission', True, operation=op)

    op.send_requested = True
    op.emit('action_goal_sent', total_timeout_sec=total_timeout)
    try:
        op.bind_send_future(client.send_goal_async(goal))
    except Exception as exc:
        op.send_error = repr(exc)
        # A transport exception is not proof that no goal reached the server.
        return await stop('failed', 'goal submission raised: ' + repr(exc))

    response_deadline = min(deadline, now() + goal_response_timeout)
    while True:
        op.refresh()
        if is_cancel_requested():
            return await stop('canceled', 'canceled during action goal handshake')
        if op.send_processed:
            break
        remaining = response_deadline - now()
        if remaining <= 0:
            return await stop('timeout', 'action goal handshake timed out')
        await sleep(min(poll_interval, remaining))
    if op.rejected:
        return ActionOutcome('rejected', 'action goal rejected', True, operation=op)
    if op.send_error:
        return await stop('failed', 'action goal response failed: ' + op.send_error)

    while True:
        op.refresh()
        # Operator cancellation wins a race with a just-completed result: no next goal.
        if is_cancel_requested():
            return await stop('canceled', 'action canceled by operator')
        if op.wrapped_result is not None:
            status = int(op.wrapped_result.status)
            if status not in TERMINAL:
                return await stop('failed', f'non-terminal result status={status}')
            state = 'succeeded' if status == SUCCEEDED else 'failed'
            return ActionOutcome(state, f'action terminal status={status}', True,
                                 op.wrapped_result, op)
        if op.result_error:
            return await stop('failed', 'action result failed: ' + op.result_error)
        remaining = deadline - now()
        if remaining <= 0:
            return await stop('timeout', 'total action execution budget expired')
        await sleep(min(poll_interval, remaining))
