from pathlib import Path
import shutil

import pytest

from agt_field.appliance import Appliance
from agt_field.bundle import validate_bundle
from agt_field.contracts import ContractError, atomic_yaml, files, read_yaml, route
from agt_field.mock import bundle_fixture
from agt_field.navigation_edit import publish_navigation_edit


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(autouse=True)
def v3_validator_paths(monkeypatch):
    # Reuse production validators; no mocked image/hash validation.
    monkeypatch.syspath_prepend(str(ROOT / "cleaning/agt_map_converter"))
    monkeypatch.syspath_prepend(str(ROOT / "map_data_manager/agt_map_manager"))


def fixture(tmp_path):
    data = tmp_path / "data"
    source = bundle_fixture(data / "maps/mock_field/1")
    binding = validate_bundle(source, allow_mock=True)
    draft = data / "run/map_edits/draft"
    shutil.copytree(source / "navigation", draft)
    (draft / "map.pgm").chmod(0o644)
    (draft / "map.pgm").write_bytes(b"P5\n3 3\n255\n" + bytes([0, 205] + [254] * 7))
    nav = read_yaml(draft / "map.yaml")
    nav["free_thresh"] = 0.196
    nav["mode"] = "trinary"
    atomic_yaml(draft / "map.yaml", nav)
    return data, source, binding, draft


def test_publish_edited_map_and_v3_validation(tmp_path):
    from agt_map_manager.map_package import validate_package

    data, source, binding, draft = fixture(tmp_path)
    before = files(source)
    target = publish_navigation_edit(source, draft / "map.yaml", data, binding, {}, mock=True)
    edited = validate_bundle(target, allow_mock=True)
    assert edited["map_version"] != binding["map_version"]
    assert edited["bundle_sha256"] != binding["bundle_sha256"]
    assert files(source) == before
    assert files(target / "mapping") == files(source / "mapping")
    assert (target / "localization/polar_context.db").read_bytes() == (
        source / "localization/polar_context.db"
    ).read_bytes()
    assert read_yaml(target / "localization/metadata.yaml")["map"]["map_version"] == target.name
    assert validate_package(target / "metadata.yaml", verify_hashes=True).valid
    assert (target / "navigation/map.pgm").read_bytes().endswith(bytes([0, 205] + [254] * 7))
    with pytest.raises(ContractError):
        route(
            dict(
                schema_version=1,
                route_id="old",
                map=binding,
                waypoints=[
                    dict(
                        id="P1", x=0.0, y=0.0, yaw=0.0, sequence=0, dwell_seconds=0.0, action="wait"
                    )
                ],
            ),
            edited,
        )


@pytest.mark.parametrize(
    "change",
    [
        "origin",
        "resolution",
        "dimensions",
        "truncated",
        "unknown",
        "outside",
        "binding",
        "sourcehash",
        "symlink",
    ],
)
def test_edit_rejects_incompatible_maps(tmp_path, change):
    data, source, binding, draft = fixture(tmp_path)
    nav = read_yaml(draft / "map.yaml")
    if change == "origin":
        nav["origin"] = [1, 0, 0]
    if change == "resolution":
        nav["resolution"] = 0.2
    if change == "unknown":
        nav["free_thresh"] = 0.25
    atomic_yaml(draft / "map.yaml", nav)
    if change == "dimensions":
        (draft / "map.pgm").write_bytes(b"P5\n2 2\n255\n" + bytes([254] * 4))
    if change == "truncated":
        (draft / "map.pgm").write_bytes(b"P5\n3 3\n255\n" + bytes([254]))
    if change == "outside":
        draft = source / "navigation"
    if change == "binding":
        binding = dict(binding, map_version="wrong")
    if change == "sourcehash":
        (source / "mapping/map.pcd").chmod(0o644)
        (source / "mapping/map.pcd").write_bytes(b"changed")
    if change == "symlink":
        (draft / "map.pgm").unlink()
        (draft / "map.pgm").symlink_to(source / "navigation/map.pgm")
    with pytest.raises((ContractError, ValueError)):
        publish_navigation_edit(source, draft / "map.yaml", data, binding, {}, mock=True)
    assert len(list((data / "maps/mock_field").iterdir())) == 1


def test_runtime_save_activation_and_mission_gate(tmp_path):
    data, source, binding, draft = fixture(tmp_path)
    appliance = Appliance(data, ROOT / "profiles/mock_yhs", mock=True)
    try:
        appliance.activate(binding)
        command = dict(
            command="SAVE_NAVIGATION_EDIT",
            source_binding=binding,
            edited_map=str(draft / "map.yaml"),
            confirmed=True,
        )
        appliance.mission.state = "NAVIGATING"
        with pytest.raises(ContractError):
            appliance.command(command)
        appliance.mission.state = "READY"
        with pytest.raises(ContractError):
            appliance.command(dict(command, confirmed=False))
        result = appliance.command(command)
        assert appliance.mode == "IDLE" and appliance.localization == "STOPPED"
        assert result["binding"] == appliance.mission.binding
        assert (
            read_yaml(data / "run/active_map.yaml")["map_version"]
            == result["binding"]["map_version"]
        )
        with pytest.raises(ContractError):
            appliance.command(command)
    finally:
        appliance.stop_event.set()
        appliance.mock_thread.join(timeout=1)
