from action_msgs.msg import GoalStatus

from agt_demo_task.navigate_capture_task import RUNNING_STATES, status_name


def test_running_states_include_action_lifecycle_before_completion():
    assert GoalStatus.STATUS_ACCEPTED in RUNNING_STATES
    assert GoalStatus.STATUS_EXECUTING in RUNNING_STATES
    assert GoalStatus.STATUS_SUCCEEDED not in RUNNING_STATES


def test_status_names_are_operator_readable():
    assert status_name(GoalStatus.STATUS_SUCCEEDED) == 'SUCCEEDED'
    assert status_name(GoalStatus.STATUS_CANCELED) == 'CANCELED'
