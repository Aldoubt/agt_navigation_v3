"""Versioned JSON events exchanged between mapping, HMI and Map Manager."""

from __future__ import annotations

import json
from typing import Any, Dict


MAP_GENERATED = 'MAP_GENERATED'
MAP_ACTIVATED = 'MAP_ACTIVATED'
MAP_VALIDATED = 'MAP_VALIDATED'


def encode_event(event_type: str, **fields: Any) -> str:
    return json.dumps({'type': str(event_type), **fields}, sort_keys=True)


def decode_event(payload: str) -> Dict[str, Any]:
    data = json.loads(str(payload))
    if not isinstance(data, dict) or not str(data.get('type', '')).strip():
        raise ValueError('map event requires a JSON object with type')
    return data
