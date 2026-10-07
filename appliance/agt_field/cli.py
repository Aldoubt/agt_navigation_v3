"""Field launcher: one CLI for install, Docker, CAN, diagnostics and local RPC."""

from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys

from .contracts import ContractError, read_yaml, atomic_yaml
from .profile import load_profile
from .server import rpc
from .diagnostics import report, diagnostic_zip

ROOT = Path(os.environ.get("AGT_REPO_ROOT", Path(__file__).resolve().parents[2])).resolve()
DATA = Path(os.environ.get("AGT_DATA_ROOT", str(Path.home() / "agt"))).expanduser().resolve()


def directories():
    for name in ["maps", "routes", "bags", "logs", "profiles", "diagnostics", "run", "sources"]:
        (DATA / name).mkdir(parents=True, exist_ok=True)
    os.chmod(DATA / "run", 0o700)
    for profile in ["yhs", "mock_yhs"]:
        if not (DATA / "profiles" / profile).exists():
            shutil.copytree(ROOT / "profiles" / profile, DATA / "profiles" / profile)
    description = DATA / "profiles/yhs/robot_description"
    if not description.exists():
        shutil.copytree(ROOT / "robot_description/yhs", description)


def compose(*args):
    env = dict(
        os.environ, AGT_DATA_ROOT=str(DATA), AGT_UID=str(os.getuid()), AGT_GID=str(os.getgid())
    )
    if (DATA / "run/launcher.env.yaml").exists():
        env.update({k: str(v) for k, v in read_yaml(DATA / "run/launcher.env.yaml").items()})
    subprocess.run(
        [
            "docker",
            "compose",
            "-p",
            "agt-yhs",
            "-f",
            str(ROOT / "appliance/docker-compose.yml"),
            *args,
        ],
        env=env,
        check=True,
    )


def display():
    if not os.environ.get("DISPLAY"):
        raise ContractError(
            "No DISPLAY. Use ./agt up --headless for runtime checks, or start from Linux desktop."
        )
    authority = DATA / "run/xauthority"
    authority.touch(mode=0o600, exist_ok=True)
    source = os.environ.get("XAUTHORITY", str(Path.home() / ".Xauthority"))
    result = subprocess.run(
        ["xauth", "-f", source, "nlist", os.environ["DISPLAY"]], capture_output=True, check=True
    )
    if not result.stdout.strip():
        raise ContractError(
            "X11 authorization cookie unavailable; launcher refuses an unauthenticated display"
        )
    normalized = b"\n".join(b"ffff" + line[4:] for line in result.stdout.splitlines()) + b"\n"
    subprocess.run(["xauth", "-f", str(authority), "nmerge", "-"], input=normalized, check=True)
    return str(authority)


def host_services():
    pidfile = DATA / "run/host_control.pid"
    alive = False
    if pidfile.exists():
        try:
            os.kill(int(pidfile.read_text()), 0)
            alive = True
        except (ProcessLookupError, ValueError):
            pass
    if not alive:
        with (DATA / "logs/host-control.log").open("ab") as log:
            proc = subprocess.Popen(
                [
                    sys.executable,
                    "-m",
                    "agt_field.host_control",
                    "--data-root",
                    str(DATA),
                    "--profile",
                    str(DATA / "profiles/yhs"),
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        pidfile.write_text(str(proc.pid))
    profile = load_profile(DATA / "profiles/yhs")
    if not profile["base"].get("ros1_master_uri") or any(
        not profile["topics"].get(k) for k in ["ros1_odom", "ros1_chassis", "ros1_estop"]
    ):
        return
    gateway_pid = DATA / "run/ros1_gateway.pid"
    if gateway_pid.exists():
        try:
            os.kill(int(gateway_pid.read_text()), 0)
            return
        except (ProcessLookupError, ValueError):
            pass
    if Path("/opt/ros/noetic/setup.bash").is_file():
        env = dict(os.environ)
        if profile["base"].get("ros1_master_uri"):
            env["ROS_MASTER_URI"] = profile["base"]["ros1_master_uri"]
        with (DATA / "logs/ros1-gateway.log").open("ab") as log:
            proc = subprocess.Popen(
                [
                    "bash",
                    str(ROOT / "appliance/scripts/with_ros1.sh"),
                    "--profile",
                    str(DATA / "profiles/yhs"),
                    "--data-root",
                    str(DATA),
                ],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        (DATA / "run/ros1_gateway.pid").write_text(str(proc.pid))


def install(args):
    if not sys.platform.startswith("linux"):
        raise ContractError("Field Appliance requires a Linux host with Docker Engine")
    directories()
    if not shutil.which("docker"):
        raise ContractError(
            "Install Docker Engine and Compose plugin for your OS; see docs/installation.md. No system changes were made."
        )
    subprocess.run(["docker", "info"], stdout=subprocess.DEVNULL, check=True)
    subprocess.run(["docker", "compose", "version"], check=True)
    if not args.skip_build:
        if subprocess.check_output(
            ["git", "-C", str(ROOT), "status", "--porcelain"], text=True
        ).strip():
            raise ContractError(
                "Refusing uncommitted appliance source: commit or use a clean checkout for reproducible image builds"
            )
        subprocess.run(
            [
                sys.executable,
                str(ROOT / "appliance/scripts/prepare_sources.py"),
                "--root",
                str(ROOT),
                "--source-root",
                str(DATA / "sources"),
            ],
            check=True,
        )
        secret = []
        if os.environ.get("CODEX_PROXY_CERT"):
            secret = ["--secret", "id=proxy_ca,src=" + os.environ["CODEX_PROXY_CERT"]]
        subprocess.run(
            [
                "docker",
                "build",
                *secret,
                "-f",
                str(ROOT / "appliance/Dockerfile.base"),
                "-t",
                "agt-yhs-base:v1",
                str(ROOT),
            ],
            check=True,
        )
        subprocess.run(
            [
                "docker",
                "build",
                *secret,
                "--build-arg",
                "NAVIGATION_COMMIT="
                + subprocess.check_output(
                    ["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True
                ).strip(),
                "--build-arg",
                "HMI_PATCH_SHA256="
                + read_yaml(ROOT / "appliance/repos.lock.yaml")["hmi"].get("patch_sha256", "none"),
                "--build-context",
                "mapping=" + str(DATA / "sources/mapping"),
                "--build-context",
                "hmi=" + str(DATA / "sources/hmi"),
                "-f",
                str(ROOT / "appliance/Dockerfile"),
                "-t",
                "agt-yhs-field:v1",
                str(ROOT),
            ],
            check=True,
        )
    applications = Path.home() / ".local/share/applications"
    applications.mkdir(parents=True, exist_ok=True)
    launcher = applications / "agt-yhs-control.desktop"
    path = str(ROOT / "appliance/scripts/desktop-launch.sh").replace('"', '\\"')
    launcher.write_text(
        '[Desktop Entry]\nType=Application\nName=AGT YHS Control\nExec="'
        + path
        + '"\nTerminal=false\nCategories=Science;Robotics;\n'
    )
    launcher.chmod(0o755)
    print(
        "Installed launcher:",
        launcher,
        "\nData root:",
        DATA,
        "\nFill profile:",
        DATA / "profiles/yhs",
    )


def main(argv=None):
    parser = argparse.ArgumentParser(prog="./agt")
    sub = parser.add_subparsers(dest="command", required=True)
    i = sub.add_parser("install")
    i.add_argument("--skip-build", action="store_true")
    u = sub.add_parser("up")
    u.add_argument("--mock", action="store_true")
    u.add_argument("--headless", action="store_true")
    for name in ["down", "restart", "status", "logs", "test"]:
        sub.add_parser(name)
    c = sub.add_parser("can")
    c.add_argument("operation", choices=["up", "down"])
    d = sub.add_parser("doctor")
    d.add_argument("--report", action="store_true")
    d.add_argument("--mock", action="store_true")
    r = sub.add_parser("rpc")
    r.add_argument("request", help="JSON command object")
    args = parser.parse_args(argv)
    try:
        if args.command == "install":
            install(args)
        elif args.command == "up":
            directories()
            host_services()
            authority = "/dev/null" if args.headless else display()
            atomic_yaml(
                DATA / "run/launcher.env.yaml",
                dict(AGT_XAUTHORITY=authority, AGT_RUNTIME_MODE="mock" if args.mock else "runtime"),
            )
            compose("up", "-d", "--wait", "runtime")
            if not args.headless:
                compose("up", "-d", "hmi")
            print(json.dumps(rpc(DATA, dict(command="STATUS")), ensure_ascii=False, indent=2))
        elif args.command == "down":
            try:
                rpc(DATA, dict(command="STOP_ALL"))
            except (ValueError, OSError):
                pass
            compose("down")
            for name in ["ros1_gateway", "host_control"]:
                path = DATA / "run" / (name + ".pid")
                if path.exists():
                    try:
                        os.kill(int(path.read_text()), signal.SIGTERM)
                    except ProcessLookupError:
                        pass
                    path.unlink()
        elif args.command == "restart":
            compose("restart")
        elif args.command == "status":
            print(json.dumps(rpc(DATA, dict(command="STATUS")), ensure_ascii=False, indent=2))
            compose("ps")
        elif args.command == "logs":
            compose("logs", "--tail", "200")
        elif args.command == "can":
            from .host_control import can

            print(can(DATA / "profiles/yhs", args.operation))
        elif args.command == "doctor":
            directories()
            profile = load_profile(DATA / "profiles" / ("mock_yhs" if args.mock else "yhs"))
            try:
                status = rpc(DATA, dict(command="STATUS"))
            except (ValueError, OSError):
                status = dict(runtime="OFFLINE")
            if status.get("runtime") != "OFFLINE":
                runtime_report = rpc(DATA, dict(command="DOCTOR", report=args.report), timeout=90)
                result = runtime_report["diagnostic"]
                if args.report:
                    print(
                        "Report:",
                        runtime_report["report_path"].replace("/data/", str(DATA) + "/", 1),
                    )
            else:
                result = report(DATA, profile, status)
                if args.report:
                    print("Report:", diagnostic_zip(DATA, profile, status))
            print(json.dumps(result, ensure_ascii=False, indent=2))
            if result["profile_errors"] or any(
                str(v).startswith("FAIL") for v in result["checks"].values()
            ):
                return 2
        elif args.command == "rpc":
            print(json.dumps(rpc(DATA, json.loads(args.request)), ensure_ascii=False, indent=2))
        elif args.command == "test":
            subprocess.run(
                [sys.executable, "-m", "pytest", "-q", str(ROOT / "appliance/tests")], check=True
            )
        return 0
    except (ContractError, ValueError, OSError, subprocess.CalledProcessError) as exc:
        print("AGT failed: " + str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
