"""Bounded, honest diagnostics. Hardware observations never inferred from mock."""

import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
from datetime import datetime, timezone
import zipfile

from .bundle import validate_bundle
from .contracts import files, read_yaml
from .profile import calibration_errors


def probe(argv, timeout=5):
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return dict(
            exit_code=result.returncode, stdout=result.stdout[-65536:], stderr=result.stderr[-4096:]
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return dict(exit_code=-1, error=str(exc))


def report(data, profile, status=None):
    data = Path(data)
    mock = profile["robot"]["profile"] == "mock_yhs"
    system = dict(
        os=platform.platform(),
        disk=shutil.disk_usage(data)._asdict(),
        memory=Path("/proc/meminfo").read_text()[:2000],
        network=probe(["ip", "-j", "address"]),
        docker=probe(["docker", "ps", "--format", "{{json .}}"]),
        compose=probe(["docker", "compose", "version"]),
    )
    try:
        from .host_control import request

        system["host_observations"] = request(data, "HOST_DIAGNOSTICS")
    except Exception as exc:
        system["host_observations"] = dict(state="UNAVAILABLE", reason=str(exc))
    base = profile["base"]
    can = (
        probe(["ip", "-details", "-json", "link", "show", base["can_interface"]])
        if base.get("can_interface")
        else dict(state="CONFIG_REQUIRED")
    )
    sensor = profile["sensors"]
    network = dict(
        interface=sensor.get("interface"),
        host_ip=sensor.get("host_ip"),
        sensor_ip=sensor.get("sensor_ip"),
        reachability=probe(["ping", "-c", "1", "-W", "1", sensor["sensor_ip"]])
        if sensor.get("sensor_ip")
        else dict(state="CONFIG_REQUIRED"),
    )
    ros = dict(
        nodes=probe(["ros2", "node", "list"]),
        topics=probe(["ros2", "topic", "list", "-t"]),
        tf=probe(["ros2", "topic", "echo", "/tf", "tf2_msgs/msg/TFMessage", "--once"], timeout=3),
    )
    # view_frames creates output files: run separately in diagnostics root, never in a frozen bundle.
    checks = dict(
        calibration="PASS_MOCK" if mock else ("FAIL" if calibration_errors(profile) else "PASS"),
        real_hardware="PENDING",
        can="MOCK_ONLY" if mock else "PENDING",
        tf_duplicate_publishers="PENDING",
        critical_topic_rates=(status or {}).get("devices", {}),
        tf_frames=(status or {}).get("tf", {}),
    )
    binding = None
    active = data / "run/active_map.yaml"
    if active.exists():
        try:
            state = read_yaml(active)
            binding = validate_bundle(state["package_path"], allow_mock=mock)
            checks["map_bundle"] = "PASS_MOCK" if mock else "PASS"
        except Exception as exc:
            checks["map_bundle"] = "FAIL: " + str(exc)
    else:
        checks["map_bundle"] = "PENDING: no active map"
    root = profile["root"]
    profile_hashes = files(root)
    return dict(
        schema_version=1,
        timestamp=datetime.now(timezone.utc).isoformat(),
        mock=mock,
        checks=checks,
        host=system,
        can=can,
        mid360=network,
        ros2=ros,
        runtime=status or {},
        active_map=binding,
        profile_errors=calibration_errors(profile),
        build_commits=dict(
            navigation=os.environ.get("AGT_NAVIGATION_COMMIT", "unknown"),
            hmi_patch_sha256=os.environ.get("AGT_HMI_PATCH_SHA256", "unknown"),
        ),
        configuration_hashes=profile_hashes,
    )


def diagnostic_zip(data, profile, status=None):
    data = Path(data)
    out = data / "diagnostics"
    out.mkdir(parents=True, exist_ok=True)
    info = report(data, profile, status)
    path = out / (
        "agt_diagnostic_" + datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f") + ".zip"
    )
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        archive.writestr("diagnostic.json", json.dumps(info, indent=2))
        for p in sorted(profile["root"].rglob("*.yaml")):
            archive.writestr("profile/" + p.relative_to(profile["root"]).as_posix(), p.read_bytes())
        for root, prefix in [(data / "logs", "logs"), (data / "run", "runtime")]:
            for p in sorted(root.glob("*")):
                if p.is_file() and not p.is_symlink():
                    with p.open("rb") as stream:
                        if p.stat().st_size > 2 * 1024 * 1024:
                            stream.seek(-2 * 1024 * 1024, os.SEEK_END)
                        archive.writestr(prefix + "/" + p.name, stream.read())
        active = data / "run/active_map.yaml"
        if active.exists():
            try:
                bundle = Path(read_yaml(active)["package_path"])
                for name in [
                    "manifest.yaml",
                    "metadata.yaml",
                    "localization/metadata.yaml",
                    "navigation/review_status.yaml",
                ]:
                    if (bundle / name).is_file():
                        archive.writestr("map/" + name, (bundle / name).read_bytes())
            except Exception:
                pass
        versions = Path("/opt/agt/versions.yaml")
        if versions.exists():
            archive.write(versions, "software_versions.yaml")
        commits = Path("/opt/agt/build_commits.yaml")
        if commits.exists():
            archive.write(commits, "build_commits.yaml")
    return path
