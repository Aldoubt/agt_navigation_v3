"""Small process-group wrapper so mode shutdown never kills sensor drivers."""

from __future__ import annotations

import os
import signal
import subprocess
import time
from dataclasses import dataclass
from typing import Sequence


@dataclass
class ManagedProcess:
    name: str
    command: tuple[str, ...]
    process: subprocess.Popen | None = None

    def start(self) -> None:
        if self.process is not None and self.process.poll() is None:
            raise RuntimeError(f'{self.name} is already running')
        self.process = subprocess.Popen(list(self.command), start_new_session=True)

    def running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def stop(self, timeout_sec: float = 8.0) -> None:
        if not self.running():
            return
        assert self.process is not None
        os.killpg(self.process.pid, signal.SIGINT)
        deadline = time.monotonic() + timeout_sec
        while self.process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)
            self.process.wait(timeout=2.0)


def ros_launch(package: str, launch_file: str, *arguments: str) -> tuple[str, ...]:
    return ('ros2', 'launch', package, launch_file, *arguments)
