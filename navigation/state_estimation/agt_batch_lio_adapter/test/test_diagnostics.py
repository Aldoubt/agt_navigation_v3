from agt_batch_lio_adapter.diagnostics import AdapterState, classify_adapter_state


def test_fresh_input_is_fresh():
    assert classify_adapter_state(0.1, 0.1, 0.2, 1.0) is AdapterState.FRESH


def test_delayed_input_keeps_recent_pose_observable():
    assert classify_adapter_state(0.5, 0.5, 0.2, 1.0) is AdapterState.INPUT_DELAYED


def test_old_pose_is_stale():
    assert classify_adapter_state(2.0, 2.0, 0.2, 1.0) is AdapterState.STALE


def test_startup_without_input_is_no_input():
    assert classify_adapter_state(None, None, 0.2, 1.0) is AdapterState.NO_INPUT
