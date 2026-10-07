import json
from pathlib import Path
import threading
import time
import pytest
from agt_field.appliance import Appliance
from agt_field.contracts import ContractError
from agt_field.server import serve, rpc
from agt_field.diagnostics import diagnostic_zip
from test_contracts import route_data


def wait(check, timeout=5):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return
        time.sleep(0.01)
    raise AssertionError("state timeout")


def test_mock_appliance_end_to_end(tmp_path):
    profile = Path(__file__).resolve().parents[2] / "profiles/mock_yhs"
    c = Appliance(tmp_path, profile, mock=True)
    server = serve(c)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    def call(command, **kw):
        return rpc(tmp_path, dict(command=command, **kw))

    try:
        status = call("STATUS")
        assert status["devices"]["LiDAR"]["state"] == "ONLINE"
        assert call("PREFLIGHT")["pass_"]
        call("START_MAPPING", map_bundle_id="mock_field", map_version="1")
        assert call("STATUS")["mapping"] == "RUNNING"
        call("STOP_MAPPING")
        wait(lambda: c.mapping == "REVIEW_REQUIRED" and not c.busy())
        call("REVIEW_MAP")
        call("CONFIRM_MAP")
        assert c.mapping == "READY"
        call("ACTIVATE_MAP", map_bundle_id="mock_field", map_version="1")
        call("START_NAVIGATION")
        assert c.localization == "READY"
        wait(lambda: c.mission.localization_ready)
        data = route_data(c.mission.binding)
        for point in data["waypoints"]:
            point["dwell_seconds"] = 0.05
        call("SAVE_ROUTE", route=data)
        assert call("LOAD_ROUTE", route_id="field_route") == data
        call("START")
        wait(lambda: c.mission.state == "COMPLETED")
        assert c.mission.index == 3
        states = [e["state"] for e in c.mission.events]
        assert states.count("DWELLING") == 3 and states[-1] == "COMPLETED"
        call("START_RECORDING")
        call("STOP_RECORDING")
        assert c.recording == "STOPPED"
        for fault in ["localization", "gateway", "nav"]:
            c.localization = "READY"
            c.gateway_ready = True
            c.mission.readiness(True, True)
            call("LOAD_ROUTE", route_id="field_route")
            call("START")
            call("MOCK_FAULT", fault=fault)
            assert c.mission.state == "ERROR"
        c.localization = "READY"
        c.gateway_ready = True
        c.mission.readiness(True, True)
        call("LOAD_ROUTE", route_id="field_route")
        call("START")
        call("CANCEL")
        assert c.mission.state == "CANCELLED"
        (c.bundle / "localization/polar_context.db").unlink()
        with pytest.raises(ValueError):
            call("START")
        call("STOP_ALL")
        assert c.mode == "IDLE"
        # doctor captures bounded configuration/logs; no bags.
        archive = diagnostic_zip(tmp_path, c.profile, c.status())
        assert archive.is_file()
        import zipfile

        with zipfile.ZipFile(archive) as z:
            assert not any(name.startswith("bags/") for name in z.namelist())
        evidence = dict(
            mapping_stages=c.mapping_history,
            mission_states=states,
            exceptions=[
                "missing localization asset",
                "localization lost",
                "Nav2 failure",
                "gateway disconnected",
                "mission cancelled",
            ],
            result="PASS_MOCK_ONLY",
        )
        (tmp_path / "acceptance.json").write_text(json.dumps(evidence, indent=2))
        print(json.dumps(evidence))
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)
        c.shutdown()


def test_mapping_mode_and_route_start_semantics(tmp_path):
    c = Appliance(tmp_path, Path(__file__).resolve().parents[2] / "profiles/mock_yhs", mock=True)
    try:
        c.command(dict(command="START_MAPPING", map_bundle_id="m", map_version="1"))
        with pytest.raises(ContractError):
            c.command(dict(command="START_NAVIGATION"))
        with pytest.raises(ContractError):
            c.command(dict(command="START_MAPPING", map_bundle_id="m", map_version="1"))
        assert c.localization == "STOPPED"
    finally:
        c.shutdown()


def test_stop_all_cancels_mapping_build(tmp_path):
    c = Appliance(tmp_path, Path(__file__).resolve().parents[2] / "profiles/mock_yhs", mock=True)
    try:
        c.command(dict(command="START_MAPPING", map_bundle_id="m", map_version="1"))
        c.command(dict(command="STOP_MAPPING"))
        c.command(dict(command="STOP_ALL"))
        wait(lambda: not c.busy())
        assert c.mapping == "CANCELLED"
        assert not (c.bundle / "manifest.yaml").exists()
    finally:
        c.shutdown()
