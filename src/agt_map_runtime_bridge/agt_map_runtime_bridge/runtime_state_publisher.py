"""Runtime state publisher skeleton.

This module defines the boundary for publishing map runtime state.
Actual ROS2 publisher wiring is added after interface package integration.
"""


class RuntimeStatePublisher:
    """Publish runtime lifecycle state to consumers.

    Consumers:
    - HMI
    - Mission Runtime
    - Diagnostics
    """

    def __init__(self):
        self.last_state = None

    def publish(self, state):
        self.last_state = state
        return state
