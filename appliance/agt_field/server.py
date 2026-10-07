"""Local Unix RPC, JSON lines, closed command set. No Web HMI or shell RPC."""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import socketserver
import threading

from .appliance import Appliance
from .contracts import read_yaml


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        self.connection.settimeout(5)
        raw = self.rfile.readline(1048577)
        if len(raw) > 1048576:
            return
        try:
            request = json.loads(raw)
            result = self.server.controller.command(request)
            response = dict(ok=True, result=result)
        except Exception as exc:
            response = dict(ok=False, error=str(exc))
        self.wfile.write(json.dumps(response, allow_nan=False).encode() + b"\n")


def rpc(data, request, timeout=10):
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect(str(Path(data) / "run/appliance.sock"))
        sock.sendall(json.dumps(request, allow_nan=False).encode() + b"\n")
        with sock.makefile("rb") as stream:
            result = json.loads(stream.readline(1048576))
        if not result["ok"]:
            raise ValueError(result["error"])
        return result["result"]
    finally:
        sock.close()


def serve(controller):
    path = controller.data / "run/appliance.sock"
    if path.exists():
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            probe.connect(str(path))
        except ConnectionRefusedError:
            path.unlink()
        else:
            raise ValueError("appliance already running")
        finally:
            probe.close()
    server = Server(str(path), Handler)
    server.controller = controller
    os.chmod(path, 0o600)
    return server


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data-root", default=os.environ.get("AGT_DATA_ROOT", str(Path.home() / "agt"))
    )
    parser.add_argument(
        "--profile", default=os.environ.get("AGT_PROFILE_PATH", "/data/profiles/yhs")
    )
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args(argv)
    versions = (
        read_yaml("/opt/agt/versions.yaml") if Path("/opt/agt/versions.yaml").exists() else {}
    )
    c = Appliance(args.data_root, args.profile, mock=args.mock, versions=versions)
    server = serve(c)

    def stop(_signal, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        server.serve_forever(poll_interval=0.1)
    finally:
        c.shutdown()
        server.server_close()
        (c.data / "run/appliance.sock").unlink(missing_ok=True)


if __name__ == "__main__":
    main()
