"""Image build helper: independent overlay workspaces; every clone commit-pinned."""

from pathlib import Path
import subprocess
import yaml


def run(args, cwd=None):
    subprocess.run(args, cwd=cwd, check=True)


def fetch(url, commit, path):
    if not path.exists():
        run(["git", "clone", "--filter=blob:none", "--no-checkout", url, str(path)])
    run(["git", "-C", str(path), "checkout", "--detach", commit])
    actual = subprocess.check_output(
        ["git", "-C", str(path), "rev-parse", "HEAD"], text=True
    ).strip()
    if actual != commit:
        raise RuntimeError("dependency commit mismatch")


def native(path, options):
    run(
        [
            "cmake",
            "-S",
            str(path),
            "-B",
            str(path / "build-agt"),
            "-DCMAKE_BUILD_TYPE=Release",
            "-DCMAKE_INSTALL_PREFIX=/usr/local",
            *options,
        ]
    )
    run(["cmake", "--build", str(path / "build-agt"), "--parallel", "2"])
    run(["cmake", "--install", str(path / "build-agt")])


nav = Path("/opt/nav_ws/src/agt_navigation_v3")
mapping = Path("/opt/mapping_ws/src/agt-lio-pgo-mapping")
# Navigation's dependency versions remain independent from Mapping's .repos.
manifest = yaml.safe_load((nav / "dependencies/agt_navigation.repos").read_text())["repositories"]
for name in [
    "external/Livox-SDK2",
    "external/Sophus",
    "external/3d_bbs",
    "external/small_gicp",
    "external/livox_ros_driver2",
    "external/fast_lio2_mapping",
]:
    entry = manifest[name]
    fetch(entry["url"], entry["version"], Path("/opt/nav_ws/src") / name)
external = Path("/opt/nav_ws/src/external")
native(external / "Sophus", ["-DBUILD_SOPHUS_TESTS=OFF"])
native(external / "Livox-SDK2", [])
native(external / "3d_bbs", ["-DBUILD_CUDA=OFF"])
native(
    external / "small_gicp",
    ["-DBUILD_HELPER=ON", "-DBUILD_TESTS=OFF", "-DBUILD_EXAMPLES=OFF", "-DBUILD_BENCHMARKS=OFF"],
)
for root, entries in [
    (
        Path("/opt/mapping_ws/src/external"),
        yaml.safe_load((mapping / ".repos").read_text())["repositories"],
    )
]:
    for name in ["fast_lio2_mapping", "livox_ros_driver2"]:
        entry = entries[name]
        fetch(entry["url"], entry["version"], root / name)
# Prepare driver ROS2 manifests exactly as upstream build helper, without invoking destructive helper.
for driver in [
    external / "livox_ros_driver2",
    Path("/opt/mapping_ws/src/external/livox_ros_driver2"),
]:
    if not (driver / "package.xml").exists():
        (driver / "package.xml").symlink_to("package_ROS2.xml")
    if not (driver / "launch").exists():
        (driver / "launch").symlink_to("launch_ROS2")
    # Driver embeds an old SDK in its CMake; system-pinned SDK headers/libraries satisfy build.
    (driver / "COLCON_IGNORE").unlink(missing_ok=True)
