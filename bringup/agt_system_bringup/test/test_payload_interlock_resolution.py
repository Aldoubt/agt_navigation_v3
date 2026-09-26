"""R3.1: navigation.launch.py interlock resolution never silently defaults to off."""
import importlib.util
from pathlib import Path

import pytest

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "navigation.launch.py"


def _mod():
    spec = importlib.util.spec_from_file_location("nav_launch_under_test", LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_empty_config_legacy_bunker_resolves_off():
    assert _mod().resolve_payload_interlock("auto", "", "bunker_v1") is False


def test_blocked_harvesting_config_raises_in_every_mode():
    # yhs_harvesting is BLOCKED (profile/gear unverified): launch must fail,
    # never resolve to interlock-off. The installed-arm -> True path is covered
    # with complete fake configs in agt_robot_bringup/test/test_robot_config.py.
    mod = _mod()
    for mode in ("auto", "true", "false"):
        with pytest.raises(Exception, match="BLOCKED"):
            mod.resolve_payload_interlock(mode, "yhs_harvesting", "")


def test_empty_config_unknown_profile_is_an_error_not_off():
    with pytest.raises(Exception):
        _mod().resolve_payload_interlock("auto", "", "yhs_tk_mid_v1")
