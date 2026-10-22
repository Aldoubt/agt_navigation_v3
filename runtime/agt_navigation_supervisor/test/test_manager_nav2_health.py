"""Lifecycle manager request completion regressions (no Nav2 or CAN needed)."""

from types import SimpleNamespace

from agt_navigation_supervisor.supervisor import NavigationSupervisor


class Future:
    def __init__(self, result=None, exception=None):
        self._result = result
        self._exception = exception

    def result(self):
        if self._exception:
            raise self._exception
        return self._result


def fresh_manager(generation=4):
    return {'generation': generation, 'active': False,
            'verified_at': 0.0, 'pending': object()}


def test_successful_manager_ack_recovers_readiness():
    state = fresh_manager()
    NavigationSupervisor._manager_state_result(
        Future(SimpleNamespace(success=True)), state, 4)
    assert state['active'] is True
    assert state['verified_at'] > 0.0
    assert state['pending'] is None


def test_manager_false_ack_is_fail_closed():
    state = fresh_manager()
    NavigationSupervisor._manager_state_result(
        Future(SimpleNamespace(success=False)), state, 4)
    assert state['active'] is False
    assert state['verified_at'] == 0.0


def test_stale_late_reply_cannot_override_newer_health_state():
    state = fresh_manager(generation=5)
    state['active'] = True
    state['verified_at'] = 123.0
    pending = state['pending']
    NavigationSupervisor._manager_state_result(
        Future(SimpleNamespace(success=False)), state, 4)
    assert state['active'] is True
    assert state['verified_at'] == 123.0
    assert state['pending'] is pending


def test_manager_service_exception_fails_closed():
    state = fresh_manager()
    NavigationSupervisor._manager_state_result(
        Future(exception=RuntimeError('timed out')), state, 4)
    assert state['active'] is False
    assert state['verified_at'] == 0.0
    assert state['pending'] is None
