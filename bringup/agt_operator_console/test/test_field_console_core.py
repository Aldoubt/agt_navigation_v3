"""Desktop operator permissions and ROS-independent contract regressions."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from agt_operator_console.field_console_core import (
    MapPackage, discover_map_packages, health_gate, read_map_package,
    validate_distance,
)


def asset(root):
    p = Path(root)/"fwmini_mid360"/"v1"
    (p/"navigation").mkdir(parents=True)
    (p/"metadata.yaml").write_text(
        "map_id: fwmini_mid360\nmap_version: v1\n", encoding="utf-8")
    (p/"navigation/map.yaml").write_text(
        "image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n",
        encoding="utf-8")
    (p/"navigation/map.pgm").write_bytes(b"P5\n4 4\n255\n" + bytes([205]*16))
    return p/"metadata.yaml"


def snapshots(profile="fwmini"):
    now = time.monotonic()
    robot = "fwmini_20261021_field_trial" if profile == "fwmini" else "bunker_v1"
    h = {"at": now, "data": {
        "status": 1, "state": "NAV_READY",
        "localized": True, "nav2_active": True, "base_alive": True,
        "robot_profile": robot, "map_id": "fwmini_mid360", "map_version": "v1"}}
    e = {"at": now, "data": {
        "map_id": "fwmini_mid360", "map_version": "v1",
        "preview_only": False, "locked": False,
        "validating": False, "can_execute": True}}
    g = {"at": now, "data": {"state": "WAIT_COMMAND"}}
    return h, e, g, now


class OperatorGatesTest(unittest.TestCase):
    def test_valid_map_and_reject_tampered_identity(self):
        with tempfile.TemporaryDirectory() as d:
            meta = asset(d)
            rec = read_map_package(meta)
            self.assertEqual(rec.label, "fwmini_mid360 / v1")
            self.assertEqual(rec.resolution, 0.05)
            choices, errors = discover_map_packages(d)
            self.assertEqual(len(choices), 1)
            self.assertFalse(errors)
            meta.write_text("map_id: different\nmap_version: v1\n")
            _, errors = discover_map_packages(d)
            self.assertTrue(errors)

    def test_reject_unsafe_path_traversal(self):
        with tempfile.TemporaryDirectory() as d:
            meta = asset(d)
            (meta.parent/"navigation/map.yaml").write_text(
                "image: ../../../../../etc/passwd\nresolution: 0.05\norigin: [0,0,0]\n")
            with self.assertRaises(ValueError):
                read_map_package(meta)

    def test_fwmini_ready_gates(self):
        with tempfile.TemporaryDirectory() as d:
            m = read_map_package(asset(d))
            h, e, g, now = snapshots()
            self.assertEqual(health_gate("fwmini", m, h, e, g, now=now),
                             ("可以生成路线", True, True))
            h["data"]["map_version"] = "wrong"
            self.assertFalse(health_gate("fwmini", m, h, e, g, now=now)[1])

    def test_bunker_independent_of_fwmini_guard(self):
        with tempfile.TemporaryDirectory() as d:
            m = read_map_package(asset(d))
            h, e, g, now = snapshots("bunker")
            self.assertTrue(health_gate("bunker", m, h, e, None, now=now)[1])
            self.assertFalse(health_gate("fwmini", m, h, e, g, now=now)[1])

    def test_missing_or_stale_guard_blocks_fwmini(self):
        with tempfile.TemporaryDirectory() as d:
            m = read_map_package(asset(d))
            h, e, g, now = snapshots()
            self.assertFalse(health_gate("fwmini", m, h, e, None, now=now)[1])
            g["data"]["state"] = "OBSTACLE_STOP"
            self.assertFalse(health_gate("fwmini", m, h, e, g, now=now)[2])
            g["data"]["state"] = "WAIT_COMMAND"
            self.assertFalse(health_gate("fwmini", m, h, e, g, now=now+3)[1])

    def test_no_map_or_health_or_preview_only_never_execute(self):
        with tempfile.TemporaryDirectory() as d:
            m = read_map_package(asset(d))
            h, e, g, now = snapshots()
            self.assertFalse(health_gate("fwmini", None, h, e, g)[2])
            self.assertFalse(health_gate("fwmini", m, None, e, g)[2])
            e["data"]["preview_only"] = True
            self.assertFalse(health_gate("fwmini", m, h, e, g, now=now)[2])
            e["data"]["preview_only"] = False
            e["data"]["can_execute"] = False
            self.assertEqual(health_gate("fwmini", m, h, e, g, now=now),
                             ("可以生成路线", True, False))

    def test_distance_bounds(self):
        for distance in (5, 7, 10, 30):
            self.assertEqual(validate_distance(distance), distance)
        for distance in (-1, 0, 31, float("nan")):
            with self.assertRaises(ValueError):
                validate_distance(distance)


if __name__ == "__main__":
    unittest.main()
