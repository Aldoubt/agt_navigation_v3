from agt_navigation_capability.policy import payload_permission_ok


def test_not_required_always_ok():
    assert payload_permission_ok(False, False, None, 5.0, 0.5)


def test_required_needs_fresh_true():
    assert payload_permission_ok(True, True, 5.0, 5.2, 0.5)
    assert not payload_permission_ok(True, True, 5.0, 5.6, 0.5)
    assert not payload_permission_ok(True, False, 5.0, 5.1, 0.5)
    assert not payload_permission_ok(True, True, None, 5.1, 0.5)
