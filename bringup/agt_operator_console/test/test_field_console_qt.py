"""Qt offscreen acceptance; no ROS, no CAN, no live action allowed."""
import os
import sys
import tempfile
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
try:
    from PyQt5.QtWidgets import QApplication
    from agt_operator_console.field_console import FieldConsoleWindow
    QT_AVAILABLE = True
except ImportError:
    QT_AVAILABLE = False


@unittest.skipUnless(QT_AVAILABLE, "python3-pyqt5 unavailable")
class QtFieldConsoleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication(["offline-test"])

    def test_both_robot_modes_stay_disarmed(self):
        with tempfile.TemporaryDirectory() as d:
            win = FieldConsoleWindow(map_root=d, offline_demo=True)
            win.show()
            for index in (0, 1):
                win.robot.setCurrentIndex(index)
                self.assertTrue(win.prepare.isEnabled())
                self.assertFalse(win.execute.isEnabled())
                self.assertFalse(win.cancel.isEnabled())
                win.distance.setValue(7.)
                win.prepare_route()
                self.assertIn("离线示意", win.route_label.text())
                self.assertFalse(win.execute.isEnabled())
            win.close()

    def test_repeated_offline_route_preview_does_not_use_deleted_qt_items(self):
        with tempfile.TemporaryDirectory() as d:
            win = FieldConsoleWindow(map_root=d, offline_demo=True)
            win.distance.setValue(5.0)
            win.prepare_route()
            win.distance.setValue(10.0)
            win.prepare_route()
            self.assertIn("10", win.route_label.text())
            self.assertFalse(win.execute.isEnabled())
            win.close()

    def test_ros_callback_does_not_reenter_executor(self):
        with tempfile.TemporaryDirectory() as d:
            win = FieldConsoleWindow(map_root=d, offline_demo=False)

            class FakeBridge:
                depth = 0
                max_depth = 0
                calls = 0
                def spin(self):
                    self.depth += 1
                    self.max_depth = max(self.depth, self.max_depth)
                    self.calls += 1
                    if self.calls == 1:
                        win._on_ros_event("guard", {
                            "at": 10.0, "data": {"state": "OBSTACLE_STOP"}})
                    self.depth -= 1

                def navigation_nodes_present(self):
                    return False

                def close(self):
                    pass

            fake = FakeBridge()
            win.bridge = fake
            win._tick()
            self.assertEqual(fake.max_depth, 1)
            self.assertEqual(fake.calls, 6)
            self.assertFalse(win.execute.isEnabled())
            win.bridge = None
            win.close()

    def test_real_pgm_asset_preview_and_world_to_pixel_projection(self):
        with tempfile.TemporaryDirectory() as d:
            map_dir = Path(d) / "field" / "v1"
            navigation = map_dir / "navigation"
            navigation.mkdir(parents=True)
            (map_dir / "metadata.yaml").write_text(
                "map_id: field\nmap_version: v1\n")
            (navigation / "map.yaml").write_text(
                "image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n")
            (navigation / "map.pgm").write_bytes(
                b"P5\n4 4\n255\n" + bytes([205] * 16))
            win = FieldConsoleWindow(map_root=d, offline_demo=True)
            self.assertEqual(win.map_select.count(), 2)
            win.map_select.setCurrentIndex(1)
            self.assertIsNotNone(win.current_map)
            self.assertIsNotNone(win.map_image)
            self.assertEqual(win._pixel(0, 0), (0.0, 3.0))
            self.assertEqual(win._pixel(0.10, 0), (2.0, 3.0))
            self.assertFalse(win.activate.isEnabled())
            win.close()

    def test_online_without_ros_stays_blocked(self):
        with tempfile.TemporaryDirectory() as d:
            win = FieldConsoleWindow(map_root=d, offline_demo=False)
            self.assertFalse(win.execute.isEnabled())
            self.assertFalse(win.prepare.isEnabled())
            win.close()


if __name__ == "__main__":
    unittest.main()
