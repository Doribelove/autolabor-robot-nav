#!/usr/bin/env python3

import os
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "scripts")
sys.path.insert(0, SCRIPT_DIR)

from cmd_vel_watchdog import evaluate_twist_values  # noqa: E402


class CommandValidationTest(unittest.TestCase):
    def test_accepts_bounded_planar_command(self):
        accepted, reason = evaluate_twist_values(
            (0.3, 0.0, 0.0, 0.0, 0.0, -0.6), 0.3, 0.6
        )
        self.assertTrue(accepted, reason)

    def test_rejects_nonfinite_command(self):
        accepted, _reason = evaluate_twist_values(
            (float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0), 0.3, 0.6
        )
        self.assertFalse(accepted)

    def test_rejects_speed_above_cap_instead_of_clamping(self):
        accepted, reason = evaluate_twist_values(
            (0.301, 0.0, 0.0, 0.0, 0.0, 0.0), 0.3, 0.6
        )
        self.assertFalse(accepted)
        self.assertIn("linear speed", reason)

    def test_rejects_nonplanar_component(self):
        accepted, reason = evaluate_twist_values(
            (0.1, 0.01, 0.0, 0.0, 0.0, 0.0), 0.3, 0.6
        )
        self.assertFalse(accepted)
        self.assertIn("unsupported", reason)

    def test_accepts_chassis_hard_limit_and_rejects_above_it(self):
        accepted, reason = evaluate_twist_values(
            (1.7, 0.0, 0.0, 0.0, 0.0, 0.0), 1.7, 0.6
        )
        self.assertTrue(accepted, reason)
        accepted, reason = evaluate_twist_values(
            (1.701, 0.0, 0.0, 0.0, 0.0, 0.0), 1.7, 0.6
        )
        self.assertFalse(accepted)
        self.assertIn("linear speed", reason)


if __name__ == "__main__":
    unittest.main()
