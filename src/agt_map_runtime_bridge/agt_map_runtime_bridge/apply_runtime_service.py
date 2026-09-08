"""Map runtime application service skeleton.

This module defines the orchestration boundary for applying a validated
MapPackage generation to runtime components.

The implementation intentionally does not directly control Nav2 or the
localization backend. Those are injected through adapters.
"""


class ApplyRuntimeService:
    def __init__(self, state_machine, adapters=None):
        self.state_machine = state_machine
        self.adapters = adapters or {}

    def apply(self, map_id: str, version: str):
        self.state_machine.transition("VALIDATING")

        result = {
            "map_id": map_id,
            "version": version,
            "success": False,
            "reason": "adapter_not_connected",
        }

        self.state_machine.transition("ERROR")
        return result
