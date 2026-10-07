"""Host-only CAN control, fixed argv and validated profile. Never executes shell text."""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import socketserver
import subprocess
import threading

from .contracts import ContractError
from .profile import load_profile


def can(profile, operation):
    p = load_profile(profile)
    base = p["base"]
    interface = base.get("can_interface")
    bitrate = base.get("can_bitrate")
    if not interface or not bitrate:
        raise ContractError("CONFIG_REQUIRED: CAN interface/bitrate in base.yaml")
    if operation == "up":
        commands = [
            ["ip", "link", "set", interface, "down"],
            ["ip", "link", "set", interface, "type", "can", "bitrate", str(bitrate)],
            ["ip", "link", "set", interface, "up"],
        ]
    elif operation == "down":
        commands = [["ip", "link", "set", interface, "down"]]
    else:
        raise ContractError("invalid CAN operation")
    log = Path(os.environ.get("AGT_DATA_ROOT", str(Path.home() / "agt"))) / "logs/can-control.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    for command in commands:
        if os.geteuid() != 0:
            command = ["sudo", "-n", *command]
        result = subprocess.run(command, capture_output=True, text=True, timeout=5)
        with log.open("a") as stream:
            stream.write(
                json.dumps(dict(argv=command, code=result.returncode, stderr=result.stderr)) + "\n"
            )
        if result.returncode:
            raise ContractError(
                "CAN control failed: "
                + result.stderr.strip()
                + ". Run ./agt can "
                + operation
                + " in a host terminal with configured sudo access."
            )
    return dict(operation=operation, interface=interface, bitrate=bitrate)


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(15)
        try:
            request = json.loads(self.rfile.readline(4096))
            command = request["command"]
            if command == "HOST_DIAGNOSTICS":
                from .diagnostics import probe
                import platform

                result = dict(
                    os=platform.platform(),
                    docker=probe(["docker", "ps", "--format", "{{json .}}"]),
                    compose=probe(["docker", "compose", "version"]),
                    network=probe(["ip", "-j", "address"]),
                    ros1_master_uri=load_profile(self.server.profile)["base"].get(
                        "ros1_master_uri"
                    ),
                    ros1_nodes=probe(["rosnode", "list"], timeout=2),
                    ros1_topics=probe(["rostopic", "list"], timeout=2),
                )
            elif command not in {"CAN_UP", "CAN_DOWN"}:
                raise ContractError("unsupported host command")
            else:
                result = can(self.server.profile, "up" if command == "CAN_UP" else "down")
            response = dict(ok=True, result=result)
        except Exception as exc:
            response = dict(ok=False, error=str(exc))
        self.wfile.write(json.dumps(response).encode() + b"\n")


def request(data, command):
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(30)
    try:
        sock.connect(str(Path(data) / "run/host_control.sock"))
        sock.sendall(json.dumps(dict(command=command)).encode() + b"\n")
        with sock.makefile("rb") as stream:
            result = json.loads(stream.readline(65536))
        if not result["ok"]:
            raise ContractError(result["error"])
        return result["result"]
    except OSError as exc:
        raise ContractError("host control unavailable; run ./agt up on host") from exc
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", required=True)
    parser.add_argument("--data-root", required=True)
    args = parser.parse_args()
    path = Path(args.data_root) / "run/host_control.sock"
    if path.exists():
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            probe.connect(str(path))
        except ConnectionRefusedError:
            path.unlink()
        else:
            return
        finally:
            probe.close()
    server = socketserver.ThreadingUnixStreamServer(str(path), Handler)
    server.daemon_threads = True
    server.profile = args.profile
    os.chmod(path, 0o600)

    def stop(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        server.serve_forever(poll_interval=0.1)
    finally:
        server.server_close()
        path.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
