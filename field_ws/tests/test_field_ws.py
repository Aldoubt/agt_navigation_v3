import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("field_ws_tool", ROOT / "field_ws.py")
app = importlib.util.module_from_spec(spec)
spec.loader.exec_module(app)
spec2 = importlib.util.spec_from_file_location("relative_line_tool", ROOT / "relative_line.py")
line = importlib.util.module_from_spec(spec2)
spec2.loader.exec_module(line)


class FieldWorkspaceTests(unittest.TestCase):
    def test_lock_is_pinned_and_has_three_existing_named_sources(self):
        lock = app.lock_data()
        self.assertEqual(len(lock["repositories"]), 3)
        self.assertEqual(set(x["name"] for x in lock["repositories"]), {
            "agt_navigation_v3", "fwmini_navigation_lab", "agt-lio-pgo-mapping"
        })

    def test_init_dry_run_does_not_touch_disk(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "field_ws"
            app.init_workspace(root, dry_run=True)
            self.assertFalse(root.exists())

    def test_expected_duplicate_package_has_explicit_owner(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for repo in ("agt_navigation_v3", "fwmini_navigation_lab"):
                package = root / "src" / repo / "some_package"
                package.mkdir(parents=True)
                (package / "package.xml").write_text(
                    "<package format='3'><name>agt_navigation_interfaces</name>"
                    "<version>0.0.1</version><description>test</description>"
                    "<maintainer email='robot@example.com'>robot</maintainer>"
                    "<license>MIT</license></package>"
                )
            chosen = app.resolve_packages(root, ("agt_navigation_interfaces",))
            self.assertIn("agt_navigation_v3", str(chosen[0]))
            self.assertEqual(len(app.packages_in(root)["agt_navigation_interfaces"]), 2)

    def test_ignored_mirrored_packages(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            mirror = root / "src/fwmini_navigation_lab/integrations/v3_fwmini_ui/a"
            mirror.mkdir(parents=True)
            (mirror / "package.xml").write_text("<package><name>agt_rviz_patrol</name></package>")
            self.assertNotIn("agt_rviz_patrol", app.packages_in(root))

    def test_robot_builds_do_not_share_vendor_can_interfaces(self):
        fw = set(app.navigation_packages("fwmini"))
        bunker = set(app.navigation_packages("bunker"))
        self.assertIn("yhs_can_interfaces", fw)
        self.assertIn("fwmini_chassis_adapter", fw)
        self.assertNotIn("yhs_can_interfaces", bunker)
        self.assertNotIn("fwmini_chassis_adapter", bunker)
        self.assertIn("agt_base_runtime", bunker)
        self.assertIn("agt_operator_console", fw & bunker)
        with self.assertRaises(ValueError):
            app.navigation_packages("unknown")

    def test_5_7_10_m_lines_have_exact_endpoints(self):
        for d in (5., 7., 10.):
            for yaw in (0., math.pi/2, -math.pi/3):
                poses = line.generate_line(2., -1., yaw, d)
                self.assertAlmostEqual(poses[0]["x"], 2.)
                self.assertAlmostEqual(poses[0]["y"], -1.)
                self.assertAlmostEqual(
                    math.hypot(poses[-1]["x"] - 2., poses[-1]["y"] + 1.), d)
                self.assertEqual(len(poses), math.ceil(d/.05)+1)

    def test_line_geometry_does_not_auto_insert_connector(self):
        poses = line.generate_line(1., 2., 0., 5.)
        self.assertEqual((poses[0]["x"], poses[0]["y"]), (1., 2.))
        self.assertEqual((poses[-1]["x"], poses[-1]["y"]), (6., 2.))
        self.assertTrue(all(p["y"] == 2. for p in poses))

    def test_invalid_geometry_is_rejected(self):
        for d in (-1, 0, math.inf, math.nan, 31):
            with self.assertRaises(ValueError):
                line.generate_line(0, 0, 0, d)
        with self.assertRaises(ValueError):
            line.generate_line(0, 0, 0, 5, 0)
        with self.assertRaises(ValueError):
            line.generate_line(math.nan, 0, 0, 5)

    def test_terminal_error(self):
        self.assertAlmostEqual(line.terminal_error_m(3, 4, 0, 0), 5)


if __name__ == "__main__":
    unittest.main()
