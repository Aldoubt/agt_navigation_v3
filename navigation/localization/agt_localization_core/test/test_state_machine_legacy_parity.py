import pytest

from agt_localization_core.state_machine import LocalizationState, RecoveryStateMachine
from legacy_source import legacy_manager


def _snapshot(machine):
    return machine.state.value, machine.reason, machine.pending, machine.requested, machine.request_count


def test_state_names_match_legacy_internal_state_machine():
    assert {state.value for state in LocalizationState} == {
        state.value for state in legacy_manager.LocalizationState}


def test_tracking_failure_and_recovery_match_legacy():
    core = RecoveryStateMachine(cooldown_sec=5.0)
    legacy = legacy_manager._RecoveryStateMachine(cooldown_sec=5.0)

    core.tracking_failure('tracker_failure', auto_request=True)
    legacy.tracking_failure('tracker_failure', auto_request=True)
    assert _snapshot(core) == _snapshot(legacy)
    assert core.state is LocalizationState.LOST

    assert core.try_request(10.0) is legacy.try_request(10.0) is True
    assert _snapshot(core) == _snapshot(legacy)
    assert core.state is LocalizationState.RELOCALIZING

    core.tracking_failure('second_failure', auto_request=True)
    legacy.tracking_failure('second_failure', auto_request=True)
    assert core.try_request(12.0) is legacy.try_request(12.0) is False
    assert _snapshot(core) == _snapshot(legacy)
    assert core.cooldown_remaining(12.0) == pytest.approx(legacy.cooldown_remaining(12.0))

    assert core.try_request(15.0) is legacy.try_request(15.0) is True
    assert _snapshot(core) == _snapshot(legacy)


def test_global_pose_acceptance_matches_legacy():
    core = RecoveryStateMachine(cooldown_sec=5.0)
    legacy = legacy_manager._RecoveryStateMachine(cooldown_sec=5.0)
    core.global_pose_accepted()
    legacy.global_pose_accepted()
    assert _snapshot(core) == _snapshot(legacy)
    assert core.state is LocalizationState.LOCALIZED
