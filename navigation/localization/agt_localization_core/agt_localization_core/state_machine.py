"""Pure localization recovery state machine mirrored from the legacy manager."""

from __future__ import annotations

from enum import Enum
from typing import Optional


class LocalizationState(str, Enum):
    BOOT = 'BOOT'
    WAIT_GLOBAL = 'WAIT_GLOBAL'
    LOCALIZED = 'LOCALIZED'
    TRACKING = 'TRACKING'
    DEGRADED = 'DEGRADED'
    LOST = 'LOST'
    RECOVERY_REQUESTED = 'RECOVERY_REQUESTED'
    RELOCALIZING = 'RELOCALIZING'


class RecoveryStateMachine:
    """Recovery/cooldown semantics copied from legacy `_RecoveryStateMachine`.

    The short-lived ``RECOVERY_REQUESTED`` state is retained even though a
    successful request transitions to ``RELOCALIZING`` in the same call.
    """

    def __init__(self, cooldown_sec: float) -> None:
        self.cooldown_sec = max(0.0, float(cooldown_sec))
        self.last_request_sec: Optional[float] = None
        self.request_count = 0
        self.state = LocalizationState.BOOT
        self.pending = False
        self.requested = False
        self.reason = 'boot'

    def cooldown_remaining(self, now_sec: float) -> float:
        if self.last_request_sec is None:
            return 0.0
        return max(0.0, self.cooldown_sec - (float(now_sec) - self.last_request_sec))

    def request(self, now_sec: float) -> bool:
        if self.cooldown_remaining(now_sec) > 0.0:
            return False
        self.last_request_sec = float(now_sec)
        self.request_count += 1
        return True

    def tracking_failure(self, reason: str, auto_request: bool) -> None:
        self.state = LocalizationState.LOST
        self.reason = reason
        self.pending = bool(auto_request)
        self.requested = False

    def try_request(self, now_sec: float) -> bool:
        if not self.pending or self.requested or not self.request(now_sec):
            return False
        self.state = LocalizationState.RECOVERY_REQUESTED
        self.reason = 'global_relocalization_requested'
        self.pending = False
        self.requested = True
        self.state = LocalizationState.RELOCALIZING
        self.reason = 'relocalization_requested'
        return True

    def global_pose_accepted(self) -> None:
        self.state = LocalizationState.LOCALIZED
        self.reason = 'global_pose_accepted'
        self.pending = False
        self.requested = False
