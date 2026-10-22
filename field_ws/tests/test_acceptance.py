import importlib.util
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("field_acceptance", ROOT / "acceptance.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def nav_row(distance, run, error=0.1, src="total_station"):
    return dict(distance_m=distance, run_id=str(run), target_x_m=10,
                target_y_m=0, arrival_x_m=10 + error, arrival_y_m=0,
                measurement_source=src)


def speed_row(run, seconds=29):
    return dict(run_id=str(run), gate_distance_m=30,
                elapsed_s=seconds, measurement_source="manual_tape")


class AcceptanceTests(unittest.TestCase):
    def test_empty_reports_are_never_pass(self):
        r = module.combined_report([], [])
        self.assertEqual(r["result"], "INCOMPLETE")
        self.assertEqual(r["hardware_slope_and_endurance"], "NOT_EVALUATED")

    def test_full_reference_dataset_pass(self):
        nav = [nav_row(d, i) for d in (5, 7, 10) for i in range(5)]
        r = module.combined_report(nav, [speed_row(1), speed_row(2)])
        self.assertEqual(r["result"], "PASS")
        self.assertEqual([x["count"] for x in r["navigation"]["measurements"]], [5]*3)

    def test_fail_7m_average_endpoint_error(self):
        nav = [nav_row(d, i, error=0.16 if d == 7 else 0.01)
               for d in (5, 7, 10) for i in range(5)]
        r = module.combined_report(nav, [speed_row(1), speed_row(2)])
        self.assertEqual(r["result"], "FAIL")
        self.assertEqual(r["navigation"]["measurements"][1]["result"], "FAIL")

    def test_speed_below_1mps_fails_even_with_good_navigation(self):
        nav = [nav_row(d, i) for d in (5, 7, 10) for i in range(5)]
        r = module.combined_report(nav, [speed_row(1, 35), speed_row(2, 29)])
        self.assertEqual(r["result"], "FAIL")

    def test_one_missing_trial_is_incomplete(self):
        nav = [nav_row(d, i) for d in (5, 7, 10) for i in range(5)
               if (d, i) != (7, 4)]
        self.assertEqual(module.assess_navigation(nav)["status"], "INCOMPLETE")

    def test_duplicate_trial_id_rejected(self):
        with self.assertRaisesRegex(ValueError, "duplicate"):
            module.assess_navigation([nav_row(5, 1), nav_row(5, 1)])

    def test_slam_poses_cannot_masquerade_as_external_ground_truth(self):
        with self.assertRaisesRegex(ValueError, "measurement_source"):
            module.assess_navigation([nav_row(5, 1, src="fastlio2")])

    def test_invalid_speed_gate_and_time_rejected(self):
        with self.assertRaisesRegex(ValueError, "positive"):
            module.assess_speed([speed_row(1, 0)])
        with self.assertRaisesRegex(ValueError, "30m"):
            module.assess_speed([dict(speed_row(1), gate_distance_m=28)])

    def test_nonfinite_error_rejected(self):
        with self.assertRaisesRegex(ValueError, "nonfinite"):
            module.assess_navigation([dict(nav_row(5, 1), arrival_x_m=float("nan"))])


if __name__ == "__main__":
    unittest.main()
