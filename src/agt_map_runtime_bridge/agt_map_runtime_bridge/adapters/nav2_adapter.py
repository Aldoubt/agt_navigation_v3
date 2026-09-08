"""
Nav2 runtime adapter boundary.

Implementation will later connect to Nav2 lifecycle manager and map_server.
"""


class Nav2Adapter:
    def prepare(self, runtime_request):
        """Prepare navigation stack for a new map generation."""
        return {
            "component": "nav2",
            "map_id": runtime_request.map_id,
            "version": runtime_request.version,
            "state": "prepared",
        }

    def activate(self):
        return True

    def deactivate(self):
        return True
