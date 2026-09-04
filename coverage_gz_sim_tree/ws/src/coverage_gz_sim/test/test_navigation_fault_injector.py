#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "navigation_fault_injector.py"
SPEC = importlib.util.spec_from_file_location("navigation_fault_injector", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class NavigationFaultInjectorTest(unittest.TestCase):
    def test_positive_cross_track_offset_matches_sweep_projection(self):
        for yaw in (0.0, 0.3, math.pi / 2.0, -2.1):
            dx, dy = MODULE.positive_cross_track_offset(yaw, 0.38)
            projected_cross = dx * math.sin(yaw) - dy * math.cos(yaw)
            projected_along = dx * math.cos(yaw) + dy * math.sin(yaw)
            self.assertAlmostEqual(0.38, projected_cross, places=12)
            self.assertAlmostEqual(0.0, projected_along, places=12)
            self.assertAlmostEqual(0.38, math.hypot(dx, dy), places=12)

    def test_pre_sweep_fault_waits_for_stopped_transition_handoff(self):
        injector = MODULE.NavigationFaultInjector.__new__(
            MODULE.NavigationFaultInjector
        )
        injector.target_region = "A区"
        injector.target_segment = 2
        injector.entry_trigger_distance = 0.30
        injector.status = SimpleNamespace(
            state="TRANSITING", current_region_name="A区", current_segment=1
        )
        injector.transition_target = {
            "start_x": 1.0, "start_y": 2.0, "yaw": 0.4
        }
        injector.pose = (1.20, 2.05, 0.4)
        injector.actual_velocity = 0.0
        self.assertTrue(injector._matches_entry_approach())

        injector.actual_velocity = 0.20
        self.assertFalse(injector._matches_entry_approach())
        injector.actual_velocity = 0.0
        injector.status.state = "SWEEPING"
        self.assertFalse(injector._matches_entry_approach())


if __name__ == "__main__":
    unittest.main()
