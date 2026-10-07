"""Owned process groups, bounded shutdown and external-overlay entry points."""

import os
from pathlib import Path
import signal
import subprocess

from .contracts import ContractError


class Processes:
    def __init__(self, logs):
        self.logs = Path(logs)
        self.logs.mkdir(parents=True, exist_ok=True)
        self.owned = {}

    def start(self, name, argv, *, env=None):
        if name in self.owned and self.owned[name].poll() is None:
            raise ContractError(f"{name} already running")
        with (self.logs / (name + ".log")).open("ab") as log:
            p = subprocess.Popen(
                argv,
                stdout=log,
                stderr=subprocess.STDOUT,
                env=dict(os.environ, **(env or {})),
                start_new_session=True,
            )
        self.owned[name] = p
        return p

    def stop(self, name, timeout=15):
        p = self.owned.get(name)
        if not p or p.poll() is not None:
            return
        os.killpg(p.pid, signal.SIGINT)
        try:
            p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM)
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait(timeout=5)

    def shutdown(self):
        for name in reversed(list(self.owned)):
            self.stop(name)

    def running(self, name):
        return name in self.owned and self.owned[name].poll() is None


def overlay_command(overlay, argv):
    # All shell code is fixed. User-controlled paths and args are positional argv.
    script = Path(__file__).resolve().parents[1] / "scripts/with_overlay.sh"
    return ["bash", str(script), str(overlay), *argv]
