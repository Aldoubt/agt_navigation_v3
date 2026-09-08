"""
Runtime apply orchestration layer.

This module coordinates applying a validated MapPackage generation.
It intentionally does not directly depend on Nav2 or localization backends.
Those are provided through adapters.
"""

from dataclasses import dataclass
from enum import Enum


class RuntimeApplyState(Enum):
    IDLE = "IDLE"
    VALIDATING = "VALIDATING"
    APPLYING = "APPLYING"
    WAITING_BACKENDS = "WAITING_BACKENDS"
    READY = "READY"
    ERROR = "ERROR"


@dataclass
class RuntimeApplyRequest:
    map_id: str
    version: str
    generation: int


@dataclass
class RuntimeApplyResult:
    success: bool
    state: RuntimeApplyState
    reason: str


class RuntimeApplyController:
    def __init__(self, nav2_adapter=None, localization_adapter=None):
        self.nav2_adapter = nav2_adapter
        self.localization_adapter = localization_adapter
        self.state = RuntimeApplyState.IDLE

    def apply(self, request: RuntimeApplyRequest) -> RuntimeApplyResult:
        self.state = RuntimeApplyState.VALIDATING

        if not request.map_id or not request.version:
            self.state = RuntimeApplyState.ERROR
            return RuntimeApplyResult(False, self.state, "invalid map identity")

        self.state = RuntimeApplyState.APPLYING

        if self.nav2_adapter:
            self.nav2_adapter.prepare(request)

        if self.localization_adapter:
            self.localization_adapter.prepare(request)

        self.state = RuntimeApplyState.WAITING_BACKENDS

        self.state = RuntimeApplyState.READY
        return RuntimeApplyResult(True, self.state, "runtime map applied")
