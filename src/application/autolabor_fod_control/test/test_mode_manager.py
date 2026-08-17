#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from autolabor_fod_control.mode_manager import (  # noqa: E402
    ENTER_FOD,
    FOD_SOURCE,
    GPS_SOURCE,
    KEEP_GPS,
    STOP_SOURCE,
    WAIT_FOR_FOD,
    CommandArbiter,
    FodEntryGate,
    stopped_sample_is_valid,
)


class CommandArbiterTest(unittest.TestCase):
    def test_only_selected_fresh_source_can_move(self):
        arbiter = CommandArbiter(0.5)
        arbiter.update(GPS_SOURCE, 1.2, 0.2, 10.0)
        arbiter.update(FOD_SOURCE, 0.2, -0.05, 10.0)

        self.assertEqual(
            arbiter.sample(GPS_SOURCE, 10.1)[:2],
            (1.2, 0.2),
        )
        self.assertEqual(
            arbiter.sample(FOD_SOURCE, 10.1)[:2],
            (0.2, -0.05),
        )

    def test_transition_stop_cannot_forward_a_cached_command(self):
        arbiter = CommandArbiter(0.5)
        arbiter.update(GPS_SOURCE, 2.7, 0.4, 10.0)

        self.assertEqual(arbiter.sample(STOP_SOURCE, 10.01)[:2], (0.0, 0.0))

    def test_stale_or_nonfinite_command_fails_to_zero(self):
        arbiter = CommandArbiter(0.5)
        arbiter.update(GPS_SOURCE, 1.0, 0.1, 10.0)
        self.assertEqual(arbiter.sample(GPS_SOURCE, 10.51)[:2], (0.0, 0.0))

        arbiter.update(FOD_SOURCE, math.nan, 0.0, 11.0)
        self.assertEqual(arbiter.sample(FOD_SOURCE, 11.1)[:2], (0.0, 0.0))

    def test_clearing_gps_prevents_pre_pause_command_reuse(self):
        arbiter = CommandArbiter(0.5)
        arbiter.update(GPS_SOURCE, 1.0, 0.0, 10.0)
        arbiter.clear(GPS_SOURCE)

        self.assertEqual(arbiter.sample(GPS_SOURCE, 10.1)[:2], (0.0, 0.0))


class StopConfirmationInputTest(unittest.TestCase):
    def test_fresh_low_speed_odometry_is_stopped(self):
        self.assertTrue(
            stopped_sample_is_valid(0.02, 0.04, 0.1, 0.6, 0.03, 0.05)
        )

    def test_stale_fast_or_nonfinite_odometry_is_not_stopped(self):
        self.assertFalse(
            stopped_sample_is_valid(0.0, 0.0, 0.61, 0.6, 0.03, 0.05)
        )
        self.assertFalse(
            stopped_sample_is_valid(0.031, 0.0, 0.1, 0.6, 0.03, 0.05)
        )
        self.assertFalse(
            stopped_sample_is_valid(0.0, math.nan, 0.1, 0.6, 0.03, 0.05)
        )


class FodEntryGateTest(unittest.TestCase):
    def test_nearest_target_strictly_inside_five_metres_enters_fod(self):
        gate = FodEntryGate(5.0, 1.0)
        gate.update((4.8, 2.1, 3.0), 10.0)

        decision = gate.evaluate(10.1, 10.0)

        self.assertEqual(decision.action, ENTER_FOD)
        self.assertAlmostEqual(decision.nearest_depth_m, 2.1)

    def test_target_at_or_beyond_five_metres_keeps_gps(self):
        for depth in (5.0, 7.5):
            with self.subTest(depth=depth):
                gate = FodEntryGate(5.0, 1.0)
                gate.update((depth,), 20.0)
                decision = gate.evaluate(20.1, 20.0)
                self.assertEqual(decision.action, KEEP_GPS)

    def test_one_second_without_valid_depth_keeps_gps(self):
        gate = FodEntryGate(5.0, 1.0)
        gate.update((math.nan, -1.0), 30.0)

        self.assertEqual(gate.evaluate(30.99, 30.0).action, WAIT_FOR_FOD)
        self.assertEqual(gate.evaluate(31.0, 30.0).action, KEEP_GPS)

    def test_stale_close_target_does_not_trigger_handoff(self):
        gate = FodEntryGate(5.0, 1.0)
        gate.update((2.0,), 39.0)

        self.assertEqual(gate.evaluate(40.1, 40.0).action, WAIT_FOR_FOD)
        self.assertEqual(gate.evaluate(41.1, 40.0).action, KEEP_GPS)


if __name__ == "__main__":
    unittest.main()
