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

    def test_online_without_ros_stays_blocked(self):
        with tempfile.TemporaryDirectory() as d:
            win = FieldConsoleWindow(map_root=d, offline_demo=False)
            self.assertFalse(win.execute.isEnabled())
            self.assertFalse(win.prepare.isEnabled())
            win.close()


if __name__ == "__main__":
    unittest.main()
