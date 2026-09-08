"""Nav2 lifecycle adapter skeleton.

This module keeps Nav2 runtime operations isolated from Map Manager.
The adapter will later call lifecycle services of map_server, costmap and
related Nav2 nodes after a MapPackage is validated.
"""


class Nav2LifecycleAdapter:
    def __init__(self):
        self.state = "UNINITIALIZED"

    def prepare(self, map_package):
        self.state = "PREPARED"
        return True

    def activate(self):
        self.state = "ACTIVE"
        return True

    def deactivate(self):
        self.state = "INACTIVE"
        return True
