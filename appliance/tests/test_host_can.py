"""Controlled CAN commands: synthetic subprocess results, no kernel/CAN mutation."""

import json
from pathlib import Path
from types import SimpleNamespace
import pytest
from agt_field.contracts import ContractError
from agt_field import host_control


def test_can_rejects_network_interface_before_mutation(monkeypatch):
    calls = []
    monkeypatch.setattr(
        host_control,
        "load_profile",
        lambda _: {"base": {"can_interface": "eth0", "can_bitrate": 1000}},
    )

    def run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(
            returncode=0, stdout=json.dumps([{"linkinfo": {"info_kind": "ether"}}]), stderr=""
        )

    monkeypatch.setattr(host_control.subprocess, "run", run)
    with pytest.raises(ContractError, match="non-CAN"):
        host_control.can(Path("unused"), "up")
    assert len(calls) == 1
    assert calls[0][-3:] == ["link", "show", "eth0"]


def test_can_down_does_not_need_bitrate(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setenv("AGT_DATA_ROOT", str(tmp_path))
    monkeypatch.setattr(host_control.os, "geteuid", lambda: 0)
    monkeypatch.setattr(
        host_control,
        "load_profile",
        lambda _: {"base": {"can_interface": "can_test", "can_bitrate": None}},
    )

    def run(argv, **kwargs):
        calls.append(argv)
        return SimpleNamespace(
            returncode=0, stdout=json.dumps([{"linkinfo": {"info_kind": "can"}}]), stderr=""
        )

    monkeypatch.setattr(host_control.subprocess, "run", run)
    assert host_control.can(Path("unused"), "down")["operation"] == "down"
    assert calls[-1] == ["ip", "link", "set", "can_test", "down"]
    with pytest.raises(ContractError, match="CONFIG_REQUIRED"):
        host_control.can(Path("unused"), "up")
