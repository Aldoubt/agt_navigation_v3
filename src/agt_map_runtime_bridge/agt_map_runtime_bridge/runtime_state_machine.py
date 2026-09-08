from dataclasses import dataclass
from enum import Enum


class RuntimeState(Enum):
    IDLE = "idle"
    RECEIVED = "received"
    VALIDATING = "validating"
    APPLYING = "applying"
    READY = "ready"
    ERROR = "error"


@dataclass
class MapRuntimeContext:
    map_id: str = ""
    version: str = ""
    generation: int = 0
    state: RuntimeState = RuntimeState.IDLE
    reason: str = ""


class MapRuntimeStateMachine:
    """Small deterministic state holder for map runtime application.

    The state machine deliberately does not start Nav2 or localization.
    Those are adapters attached after the runtime contract is stable.
    """

    def __init__(self):
        self.context = MapRuntimeContext()

    def receive_map(self, map_id: str, version: str, generation: int):
        self.context.map_id = map_id
        self.context.version = version
        self.context.generation = generation
        self.context.state = RuntimeState.RECEIVED

    def set_state(self, state: RuntimeState, reason: str = ""):
        self.context.state = state
        self.context.reason = reason
