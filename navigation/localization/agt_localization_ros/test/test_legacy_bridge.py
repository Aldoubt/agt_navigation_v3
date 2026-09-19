import json

from agt_localization_core.state_machine import LocalizationState as CoreState
from agt_localization_ros.legacy_bridge import legacy_state_name, parse_tracker_status, public_v1_state
from agt_robot_interfaces.msg import LocalizationStatus


def test_legacy_wire_state_names_are_preserved():
    message = LocalizationStatus()
    message.state = LocalizationStatus.STATE_RELOCALIZING
    assert legacy_state_name(message) == 'RELOCALIZING'


def test_public_state_maps_only_to_patch1_vocabulary():
    assert public_v1_state(CoreState.BOOT, False) == 'UNINITIALIZED'
    assert public_v1_state(CoreState.WAIT_GLOBAL, True) == 'SEARCHING'
    assert public_v1_state(CoreState.TRACKING, True) == 'TRACKING'
    assert public_v1_state(CoreState.DEGRADED, True) == 'LOCALIZED'
    assert public_v1_state(CoreState.LOST, True) == 'LOST'


def test_tracker_json_is_read_only_and_defensive():
    assert parse_tracker_status(json.dumps({'state': 'RECOVERY_REQUIRED'})) == {'state': 'RECOVERY_REQUIRED'}
    assert parse_tracker_status('not-json') == {}
