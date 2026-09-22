import asyncio
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from agt_navigation_runtime.action_execution import PendingAction, execute_action


class Clock:
    def __init__(self):
        self.value = 0.0
        self.jobs = []

    def now(self):
        return self.value

    def later(self, seconds, fn):
        self.jobs.append((self.value + seconds, fn))

    async def sleep(self, seconds):
        self.value += seconds
        while True:
            due = [job for job in self.jobs if job[0] <= self.value + 1e-9]
            if not due:
                break
            for job in due:
                self.jobs.remove(job)
                job[1]()

    def advance(self, seconds):
        asyncio.run(self.sleep(seconds))


class Handle:
    def __init__(self, clock, *, accepted=True, finish_after=None, status=4,
                 cancel_ack=True, finish_on_cancel=True):
        self.clock, self.accepted = clock, accepted
        self.status = 2 if accepted else 0
        self.result = Future()
        self.cancel_count = 0
        self.cancel_ack, self.finish_on_cancel = cancel_ack, finish_on_cancel
        if accepted and finish_after is not None:
            clock.later(finish_after, lambda: self.finish(status))

    def finish(self, status):
        self.status = status
        if not self.result.done():
            self.result.set_result(SimpleNamespace(status=status, result=SimpleNamespace(
                success=status == 4, error_code=0 if status == 4 else 301,
                message='native camera detail')))

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        self.cancel_count += 1
        f = Future()
        f.set_result(SimpleNamespace(return_code=0 if self.cancel_ack else 1,
                                    goals_canceling=[object()] if self.cancel_ack else []))
        if self.finish_on_cancel:
            self.clock.later(0.03, lambda: self.finish(5))
        return f


class Client:
    def __init__(self, clock, handle, *, ready_at=0.0, accept_after=0.0):
        self.clock, self.handle = clock, handle
        self.ready_at, self.accept_after = ready_at, accept_after
        self.sent = 0

    def server_is_ready(self):
        return self.clock.now() >= self.ready_at

    def send_goal_async(self, _goal):
        self.sent += 1
        f = Future()
        self.clock.later(self.accept_after, lambda: f.set_result(self.handle))
        return f


def run(clock, client, *, total=0.3, response=0.1, grace=0.12, cancel_at=None):
    events = []
    op = PendingAction('test', lambda kind, **kw: events.append((kind, kw)))
    result = asyncio.run(execute_action(
        client, object(), op, sleep=clock.sleep,
        is_cancel_requested=lambda: cancel_at is not None and clock.now() >= cancel_at,
        total_timeout=total, server_timeout=0.2, goal_response_timeout=response,
        cancel_timeout=grace, poll_interval=0.01, now=clock.now))
    return result, op, events


def test_success_preserves_result():
    clock = Clock();h = Handle(clock, finish_after=0.05)
    result, op, _ = run(clock, Client(clock, h))
    assert result.success and result.termination_confirmed
    assert result.wrapped_result.result.message == 'native camera detail'
    assert h.cancel_count == 0


def test_rejected_goal_is_known_not_running():
    clock = Clock();h = Handle(clock, accepted=False)
    result, _, _ = run(clock, Client(clock, h))
    assert result.state == 'rejected' and result.termination_confirmed
    assert h.cancel_count == 0


def test_aborted_result_keeps_native_error():
    clock = Clock();h = Handle(clock, finish_after=0.05, status=6)
    result, _, _ = run(clock, Client(clock, h))
    assert not result.success and result.termination_confirmed
    assert result.wrapped_result.result.error_code == 301


def test_missing_server_has_bounded_discovery_and_no_goal():
    clock = Clock();client = Client(clock, Handle(clock), ready_at=99)
    result, _, _ = run(clock, client)
    assert result.state == 'timeout' and result.termination_confirmed
    assert clock.now() <= 0.21 and client.sent == 0


def test_timeout_waits_for_terminal_not_only_cancel_ack():
    clock = Clock();h = Handle(clock)
    result, op, events = run(clock, Client(clock, h))
    assert result.state == 'timeout' and result.termination_confirmed
    assert op.cancel_acknowledged and op.terminal_status() == 5
    assert h.cancel_count == 1 and clock.now() >= 0.33 - 1e-8
    assert any(e[0] == 'action_cancel_ack' for e in events)


@pytest.mark.parametrize('ack',[True,False])
def test_cancel_ack_or_rejection_without_terminal_is_unconfirmed(ack):
    clock = Clock();h = Handle(clock, cancel_ack=ack, finish_on_cancel=False)
    result, op, _ = run(clock, Client(clock, h))
    assert not result.termination_confirmed
    assert op.cancel_acknowledged == ack
    assert clock.now() <= 0.43


def test_cancel_before_discovery_does_not_send_goal():
    clock = Clock();client = Client(clock, Handle(clock), ready_at=99)
    result, _, _ = run(clock, client, cancel_at=0)
    assert result.state == 'canceled' and result.termination_confirmed
    assert client.sent == 0


def test_cancel_during_execution_is_confirmed():
    clock = Clock();h = Handle(clock)
    result, _, _ = run(clock, Client(clock, h), cancel_at=0.08)
    assert result.state == 'canceled' and result.termination_confirmed
    assert h.cancel_count == 1


def test_late_accepted_goal_is_canceled_after_caller_timeout():
    clock = Clock();h = Handle(clock)
    result, op, _ = run(clock, Client(clock,h,accept_after=0.6),total=0.8)
    assert result.state == 'timeout' and not result.termination_confirmed
    assert h.cancel_count == 0
    clock.advance(0.6)
    assert h.cancel_count == 1
    clock.advance(0.04)
    assert op.termination_confirmed() and op.terminal_status() == 5


def test_cancel_during_handshake_cancels_later_handle():
    clock = Clock();h = Handle(clock)
    result, op, _ = run(clock,Client(clock,h,accept_after=0.09),cancel_at=0.03,response=0.2)
    assert result.state == 'canceled' and result.termination_confirmed
    assert h.cancel_count == 1 and op.terminal_status() == 5


def test_total_budget_includes_discovery_and_handshake():
    clock = Clock();h = Handle(clock)
    result, _, _ = run(clock,Client(clock,h,ready_at=0.15,accept_after=0.1),total=0.3,response=0.2)
    assert result.state == 'timeout' and result.termination_confirmed
    assert 0.3 <= clock.now() <= 0.35


def test_cancellation_wins_when_success_arrives_at_same_poll():
    clock = Clock();h = Handle(clock,finish_after=0.08)
    result, _, _ = run(clock,Client(clock,h),cancel_at=0.075)
    assert result.state == 'canceled' and result.termination_confirmed
