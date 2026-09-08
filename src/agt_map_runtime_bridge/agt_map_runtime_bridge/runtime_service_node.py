"""ROS2 service entry point for runtime map application.

This node intentionally keeps orchestration separate from Nav2 and
localization backends. Backend implementations are injected later through
adapter interfaces.
"""

from dataclasses import dataclass


@dataclass
class RuntimeApplyRequest:
    map_id: str
    version: str
    generation: int


class RuntimeServiceNode:
    """Framework-independent skeleton used before ROS2 binding.

    The ROS2 rclpy wrapper will expose this as /agt/map/runtime/apply.
    """

    def __init__(self, controller=None, state_publisher=None):
        self.controller = controller
        self.state_publisher = state_publisher

    def apply_map_runtime(self, request: RuntimeApplyRequest):
        if self.controller is None:
            return False, "controller_not_configured"

        result = self.controller.apply(
            request.map_id,
            request.version,
            request.generation,
        )

        if self.state_publisher:
            self.state_publisher.publish(result)

        return result
