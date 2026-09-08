"""ROS2 service node skeleton for applying validated map runtime packages.

This module intentionally keeps backend implementations behind adapters.
The node owns orchestration only:

Map Manager -> Runtime Bridge -> Backend adapters

Future service binding:
    /agt/map/runtime/apply

The implementation is kept backend-agnostic so Nav2 and localization
backends can evolve independently.
"""


class RuntimeApplyServiceNode:
    def __init__(self, controller=None):
        self.controller = controller

    def apply_map_runtime(self, request):
        """Handle ApplyMapRuntime request.

        Expected request fields:
          - map_id
          - version
          - generation
        """
        if self.controller is None:
            return {
                "success": False,
                "reason": "runtime controller unavailable",
            }

        return self.controller.apply(
            request.map_id,
            request.version,
            request.generation,
        )
