#!/usr/bin/env python3

import math
from pathlib import Path
import sys
import unittest


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from autolabor_fod_control.mode_manager import (  # noqa: E402
    FOD_SOURCE,
    GPS_SOURCE,
    STOP_SOURCE,
    CommandArbiter,
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


if __name__ == "__main__":
    unittest.main()
