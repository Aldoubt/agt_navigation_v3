import copy
from pathlib import Path

import pytest

from agt_field.bundle import activate, seal, validate_bundle
from agt_field.contracts import ContractError, atomic_yaml, read_yaml, route
from agt_field.mock import bundle_fixture
from agt_field.profile import load_profile, require_real


def route_data(binding):
    return dict(
        schema_version=1,
        route_id="field_route",
        map=binding,
        waypoints=[
            dict(id=f"P{i + 1}", x=float(i), y=0, yaw=0, sequence=i, dwell_seconds=d, action="wait")
            for i, d in enumerate([5, 20, 0])
        ],
    )


def test_route_roundtrip(tmp_path):
    binding = validate_bundle(bundle_fixture(tmp_path / "bundle"), allow_mock=True)
    data = route(route_data(binding), binding)
    atomic_yaml(tmp_path / "route.yaml", data)
    assert route(read_yaml(tmp_path / "route.yaml"), binding) == data
    assert [p["dwell_seconds"] for p in data["waypoints"]] == [5, 20, 0]
    del data["waypoints"][0]["dwell_seconds"]
    assert route(data)["waypoints"][0]["dwell_seconds"] == 0


@pytest.mark.parametrize(
    "change", ["negative", "nan", "action", "duplicate", "sequence", "empty", "wrongmap"]
)
def test_invalid_routes(change):
    binding = dict(map_bundle_id="a", map_version="1", bundle_sha256="a" * 64)
    d = route_data(binding)
    if change == "negative":
        d["waypoints"][0]["dwell_seconds"] = -1
    if change == "nan":
        d["waypoints"][0]["x"] = float("nan")
    if change == "action":
        d["waypoints"][0]["action"] = "arm_task"
    if change == "duplicate":
        d["waypoints"][1]["id"] = "P1"
    if change == "sequence":
        d["waypoints"][1]["sequence"] = 5
    if change == "empty":
        d["waypoints"] = []
    if change == "wrongmap":
        binding = copy.deepcopy(binding)
        binding["map_version"] = "2"
    with pytest.raises(ContractError):
        route(d, binding)


@pytest.mark.parametrize(
    "asset",
    [
        "mapping/map.pcd",
        "mapping/poses.txt",
        "mapping/patches/0.pcd",
        "localization/polar_context.db",
        "localization/voxelmaps_coords/0.pcd",
        "navigation/map.pgm",
        "navigation/map.yaml",
        "navigation/review_status.yaml",
        "metadata.yaml",
        "manifest.yaml",
    ],
)
def test_hash_fail_closed(tmp_path, asset):
    b = bundle_fixture(tmp_path / "bundle")
    (b / asset).chmod(0o644)
    (b / asset).write_bytes(b"corrupt")
    with pytest.raises((ContractError, ValueError)):
        validate_bundle(b, allow_mock=True)


def test_missing_assets_and_mock_exclusion(tmp_path):
    b = bundle_fixture(tmp_path / "bundle")
    with pytest.raises(ContractError):
        validate_bundle(b)
    (b / "localization/polar_context.db").unlink()
    with pytest.raises(ContractError):
        activate(b, tmp_path / "active.yaml", allow_mock=True)
    assert not (tmp_path / "active.yaml").exists()


def test_unconfirmed(tmp_path):
    b = bundle_fixture(tmp_path / "bundle", confirmed=False)
    with pytest.raises(ContractError):
        seal(b, dict(map_bundle_id="mock_field", map_version="1"), {}, mock=True)


def test_activation_identity(tmp_path):
    b = bundle_fixture(tmp_path / "bundle")
    identity = activate(b, tmp_path / "active.yaml", allow_mock=True)
    state = read_yaml(tmp_path / "active.yaml")
    assert state["bundle_sha256"] == identity["bundle_sha256"]
    assert state["navigation_map_yaml"].startswith(str(b))
    assert state["localization_map_pcd"].startswith(str(b))
    activate(b, tmp_path / "active.yaml", allow_mock=True)
    assert read_yaml(tmp_path / "active.yaml")["generation"] == 2


def test_symlink(tmp_path):
    b = bundle_fixture(tmp_path / "bundle")
    (b / "escape").symlink_to("/etc/passwd")
    with pytest.raises(ContractError):
        validate_bundle(b, allow_mock=True)


def test_profile():
    root = Path(__file__).resolve().parents[2] / "profiles"
    yhs = load_profile(root / "yhs")
    with pytest.raises(ContractError, match="CALIBRATION_REQUIRED"):
        require_real(yhs, motion=True)
    require_real(load_profile(root / "mock_yhs"))
    yhs["base"]["can_bitrate"] = None


def test_invalid_can(tmp_path):
    import shutil

    src = Path(__file__).resolve().parents[2] / "profiles/yhs"
    shutil.copytree(src, tmp_path / "p")
    d = read_yaml(tmp_path / "p/base.yaml")
    d["can_interface"] = "can0;evil"
    atomic_yaml(tmp_path / "p/base.yaml", d)
    with pytest.raises(ContractError):
        load_profile(tmp_path / "p")
    d["can_interface"] = "can0"
    d["can_bitrate"] = -1
    atomic_yaml(tmp_path / "p/base.yaml", d)
    with pytest.raises(ContractError):
        load_profile(tmp_path / "p")


def test_real_profile_complete_and_missing_limits(tmp_path):
    """Synthetic unit fixture only: these numbers are never production defaults."""
    import shutil
    from agt_field.profile import MOTION_KEYS

    src = Path(__file__).resolve().parents[2] / "profiles/yhs"
    root = tmp_path / "profile"
    shutil.copytree(src, root)
    for name in ["lidar_extrinsics", "imu_extrinsics", "base_geometry", "calibration_version"]:
        atomic_yaml(
            root / "calibration" / (name + ".yaml"),
            dict(status="VERIFIED", verified_by="unit-test-fixture", values={"test": True}),
        )
    p = load_profile(root)
    for section, key in [
        ("robot", "urdf"),
        ("sensors", "livox_config"),
        ("localization", "fastlio_config"),
        ("mapping", "projection_config"),
    ]:
        (root / (key + ".test")).write_text("unit fixture only")
        p[section][key] = key + ".test"
    p["navigation"].update(
        footprint=[[0, 0], [1, 0], [0, 1]],
        footprint_padding=0,
        perception=dict(self_center_xyz=[0, 0, 0], self_size_xyz=[1, 1, 1], sensor_ground_z_m=0),
        motion_limits={k: 0.1 for k in MOTION_KEYS},
    )
    p["robot"].update(lidar_frame="test_lidar", imu_frame="test_imu", rotation_frame="test_base")
    p["sensors"].update(interface="test0", host_ip="127.0.0.1", sensor_ip="127.0.0.2")
    p["base"].update(
        stop_linear_mps=0.01,
        stop_angular_radps=0.01,
        stop_timeout_sec=1,
        command_timeout_sec=0.1,
        driver_watchdog_sec=0.1,
        can_interface="vcan0",
        can_bitrate=1000,
        driver_watchdog_verified=True,
    )
    p["topics"].update(
        ros1_cmd_vel="/test/cmd",
        ros1_odom="/test/odom",
        ros1_chassis="/test/chassis",
        ros1_estop="/test/estop",
    )
    require_real(p, motion=True)
    p["navigation"]["motion_limits"] = {}
    with pytest.raises(ContractError, match="motion limit"):
        require_real(p, motion=True)
    atomic_yaml(
        root / "calibration/lidar_extrinsics.yaml",
        dict(status="VERIFIED", verified_by="unit-test-fixture", values=None),
    )
    with pytest.raises(ContractError, match="CALIBRATION_REQUIRED"):
        require_real(p)
