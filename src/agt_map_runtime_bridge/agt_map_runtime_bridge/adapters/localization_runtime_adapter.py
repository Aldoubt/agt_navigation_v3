"""Localization runtime adapter skeleton.

Responsible for applying localization assets from a MapPackage.
The concrete backend can later be FAST-LIO2, Global Localization SDK,
or another localization implementation.
"""


class LocalizationRuntimeAdapter:
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
