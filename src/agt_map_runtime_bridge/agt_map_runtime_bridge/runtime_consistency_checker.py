"""Runtime map consistency checks.

The goal is to prevent navigation and localization from using different
map generations.
"""


class RuntimeConsistencyChecker:
    def validate(self, navigation_map, localization_map):
        required = ("map_id", "version", "generation")

        for field in required:
            if navigation_map.get(field) != localization_map.get(field):
                return {
                    "valid": False,
                    "reason": f"map mismatch on {field}",
                }

        return {
            "valid": True,
            "reason": "map generation consistent",
        }
