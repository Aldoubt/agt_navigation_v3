"""Pure state classification for Batch-LIO adapter runtime diagnostics."""

from __future__ import annotations

from enum import IntEnum
from typing import Optional


class AdapterState(IntEnum):
    FRESH = 0
    INPUT_DELAYED = 1
    STALE = 2
    NO_INPUT = 3


def classify_adapter_state(
    input_age_sec: Optional[float],
    last_valid_pose_age_sec: Optional[float],
    max_input_age_sec: float,
    stale_pose_timeout_sec: float,
) -> AdapterState:
    """Classify adapter health without changing its acceptance policy.

    ``INPUT_DELAYED`` means an input exists but fails the existing freshness
    gate while a recently accepted pose is still known.  ``STALE`` means that
    no accepted pose remains within the watchdog window.  ``NO_INPUT`` is
    reserved for startup before any input has been received.
    """
    if input_age_sec is None:
        return AdapterState.NO_INPUT
    if last_valid_pose_age_sec is None or last_valid_pose_age_sec > stale_pose_timeout_sec:
        return AdapterState.STALE
    if input_age_sec > max_input_age_sec:
        return AdapterState.INPUT_DELAYED
    return AdapterState.FRESH
