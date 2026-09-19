"""Read-only adapters for frozen legacy localization interfaces."""

from __future__ import annotations

import json
from typing import Dict

from agt_localization_core.state_machine import LocalizationState as CoreState
from agt_robot_interfaces.msg import LocalizationStatus


_LEGACY_STATE_NAMES = {
    LocalizationStatus.STATE_BOOT: 'BOOT',
    LocalizationStatus.STATE_WAIT_LOCAL_ODOM: 'WAIT_LOCAL_ODOM',
    LocalizationStatus.STATE_WAIT_GLOBAL: 'WAIT_GLOBAL',
    LocalizationStatus.STATE_LOCALIZED: 'LOCALIZED',
    LocalizationStatus.STATE_DEGRADED: 'DEGRADED',
    LocalizationStatus.STATE_LOST: 'LOST',
    LocalizationStatus.STATE_RELOCALIZING: 'RELOCALIZING',
}


def legacy_state_name(message: LocalizationStatus) -> str:
    """Return the frozen wire state name without altering the legacy message."""
    return _LEGACY_STATE_NAMES.get(int(message.state), f'UNKNOWN({int(message.state)})')


def public_v1_state(state: CoreState, has_local_odom: bool) -> str:
    """Map richer core states to the Patch 1 v1 public state vocabulary."""
    if state is CoreState.BOOT:
        return 'UNINITIALIZED'
    if state is CoreState.WAIT_GLOBAL:
        return 'SEARCHING' if has_local_odom else 'UNINITIALIZED'
    if state is CoreState.LOCALIZED:
        return 'LOCALIZED'
    if state is CoreState.TRACKING:
        return 'TRACKING'
    if state is CoreState.DEGRADED:
        # Patch 1 LocalizationState.msg intentionally has no DEGRADED value.
        # The detailed core state remains visible in the JSON diagnostics.
        return 'LOCALIZED'
    if state is CoreState.LOST:
        return 'LOST'
    return 'SEARCHING'


def parse_tracker_status(payload: str) -> Dict[str, object]:
    """Parse legacy tracker JSON defensively; invalid payloads are inert."""
    try:
        parsed = json.loads(payload)
    except (TypeError, ValueError, json.JSONDecodeError):
        return {}
    return parsed if isinstance(parsed, dict) else {}
