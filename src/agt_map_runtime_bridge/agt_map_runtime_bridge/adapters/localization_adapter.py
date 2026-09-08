"""
Global localization runtime adapter boundary.

This adapter will connect MapPackage localization assets to the
selected localization backend (SDK/FAST-LIO tracking pipeline).
"""


class LocalizationAdapter:
    def prepare(self, runtime_request):
        return {
            "component": "localization",
            "map_id": runtime_request.map_id,
            "version": runtime_request.version,
            "state": "prepared",
        }

    def activate(self):
        return True

    def deactivate(self):
        return True
