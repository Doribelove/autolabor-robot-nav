#!/usr/bin/env python3

import importlib.util
import math
import os
import unittest

from geometry_msgs.msg import Twist


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "hybrid_teb_command_mux.py",
)
SPEC = importlib.util.spec_from_file_location("hybrid_teb_command_mux", SCRIPT)
MUX = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MUX)


class HybridTebCommandMuxTest(unittest.TestCase):
    def test_fixed_gear_inference_rejects_mixed_path(self):
        self.assertEqual(1, MUX.infer_fixed_gear([
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
        ]))
        self.assertEqual(-1, MUX.infer_fixed_gear([
            (0.0, 0.0, 0.0), (-1.0, 0.0, 0.0)
        ]))
        with self.assertRaisesRegex(ValueError, "constant-gear"):
            MUX.infer_fixed_gear([
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.5, 0.0, 0.0),
            ])

    def test_teb_command_must_match_action_gear(self):
        command = Twist()
        command.linear.x = -0.3
        command.angular.z = 0.1
        checked = MUX.checked_teb_command(
            command, MUX.EnforcedPath.GEAR_REVERSE, 1.35
        )
        self.assertAlmostEqual(-0.3, checked.linear.x)
        with self.assertRaisesRegex(ValueError, "fixed-gear"):
            MUX.checked_teb_command(
                command, MUX.EnforcedPath.GEAR_FORWARD, 1.35
            )

    def test_teb_command_obeys_final_ackermann_curvature_bound(self):
        command = Twist()
        command.linear.x = 0.27
        command.angular.z = 0.60
        checked = MUX.checked_teb_command(
            command, MUX.EnforcedPath.GEAR_FORWARD, 1.35
        )
        self.assertTrue(math.isfinite(checked.angular.z))
        self.assertAlmostEqual(0.20, checked.angular.z)


if __name__ == "__main__":
    unittest.main()
