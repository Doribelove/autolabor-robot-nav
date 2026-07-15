#!/usr/bin/env python3

from pathlib import Path
import re
import unittest

import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
PROFILE_DIR = WORKSPACE / "config" / "teb_profiles"
BRINGUP_SCRIPT = WORKSPACE / "scripts" / "bringup.sh"
RECOVERY_CONFIG = (
    WORKSPACE / "src" / "scripts" / "robot_bringup" / "config" / "ackermann_recovery.yaml"
)


def load_teb_profile(name):
    with (PROFILE_DIR / name).open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    return document["TebLocalPlannerROS"]


class ObstacleProfileSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.obstacle = load_teb_profile("gps_obstacle.yaml")
        cls.cruise = load_teb_profile("gps_cruise.yaml")

    def test_obstacle_clearance_was_not_traded_for_speed(self):
        self.assertGreaterEqual(self.obstacle["min_obstacle_dist"], 0.35)
        self.assertGreaterEqual(self.obstacle["inflation_dist"], 0.70)
        self.assertGreaterEqual(self.obstacle["weight_obstacle"], 80.0)
        self.assertGreaterEqual(self.obstacle["weight_inflation"], 0.5)

    def test_static_obstacle_profile_uses_fast_resize_without_ignoring_points(self):
        self.assertFalse(self.obstacle["include_dynamic_obstacles"])
        self.assertEqual(self.obstacle["feasibility_check_no_poses"], 5)

    def test_acceleration_is_responsive_but_below_cruise_limit(self):
        self.assertGreaterEqual(self.obstacle["acc_lim_x"], 2.0)
        self.assertLess(self.obstacle["acc_lim_x"], self.cruise["acc_lim_x"])
        self.assertEqual(self.obstacle["acc_lim_theta"], 0.8)

    def test_tight_forward_turns_have_more_speed_authority(self):
        hardware_min_turning_radius = 1.2
        old_tight_turn_speed = 1.2 * hardware_min_turning_radius
        tuned_tight_turn_speed = (
            self.obstacle["max_vel_theta"] * hardware_min_turning_radius
        )
        self.assertGreaterEqual(self.obstacle["max_vel_theta"], 1.4)
        self.assertEqual(
            self.obstacle["min_vel_theta"], -self.obstacle["max_vel_theta"]
        )
        self.assertGreater(tuned_tight_turn_speed, old_tight_turn_speed)
        self.assertGreaterEqual(self.obstacle["weight_optimaltime"], 4.0)

    def test_homotopy_search_fits_the_control_cycle_budget(self):
        optimizer_budget = (
            self.obstacle["max_number_classes"]
            * self.obstacle["no_outer_iterations"]
            * self.obstacle["no_inner_iterations"]
        )
        self.assertLessEqual(optimizer_budget, 60)
        self.assertLessEqual(self.obstacle["roadmap_graph_no_samples"], 10)

    def test_old_topology_cannot_remain_locked_for_seconds(self):
        self.assertLessEqual(self.obstacle["selection_cost_hysteresis"], 1.0)
        self.assertGreaterEqual(self.obstacle["selection_prefer_initial_plan"], 0.9)
        self.assertLessEqual(self.obstacle["switching_blocking_period"], 1.0)

    def test_infeasible_cycle_fallback_recovers_promptly(self):
        self.assertTrue(self.obstacle["shrink_horizon_backup"])
        self.assertGreater(self.obstacle["shrink_horizon_min_duration"], 0.0)
        self.assertLessEqual(self.obstacle["shrink_horizon_min_duration"], 3.0)


class MotionEfficiencyDefaultsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with RECOVERY_CONFIG.open(encoding="utf-8") as stream:
            cls.recovery = yaml.safe_load(stream)
        cls.bringup = BRINGUP_SCRIPT.read_text(encoding="utf-8")

    def test_gps_reverse_default_is_faster_but_below_forward_test_speed(self):
        assignment = re.search(
            r'GPS_NAV_MAX_VEL_X_BACKWARDS="\$\{GPS_NAV_MAX_VEL_X_BACKWARDS:-([0-9.]+)\}"',
            self.bringup,
        )
        self.assertIsNotNone(assignment)
        reverse_limit = float(assignment.group(1))
        self.assertGreaterEqual(reverse_limit, 1.4)
        self.assertLess(reverse_limit, 2.2)

    def test_recovery_arcs_are_faster_without_relaxing_safety_envelope(self):
        reverse = dict(self.recovery["ackermann_reverse_arc"])
        forward = dict(self.recovery["ackermann_forward_arc"])
        reverse.pop("direction")
        forward.pop("direction")
        self.assertEqual(reverse, forward)

        for name, direction in (
            ("ackermann_reverse_arc", -1),
            ("ackermann_forward_arc", 1),
        ):
            with self.subTest(name=name):
                arc = self.recovery[name]
                self.assertEqual(arc["direction"], direction)
                self.assertEqual(arc["linear_speed"], 0.30)
                self.assertGreaterEqual(arc["acceleration_limit"], 0.60)
                self.assertGreaterEqual(arc["min_turning_radius"], 1.30)
                self.assertGreaterEqual(
                    arc["max_angular_speed"],
                    arc["linear_speed"] / arc["min_turning_radius"],
                )
                self.assertLessEqual(arc["max_distance"], 0.55)
                self.assertLessEqual(arc["max_duration"], 4.0)
                self.assertLessEqual(arc["sim_granularity"], 0.05)
                self.assertGreaterEqual(arc["safety_lookahead"], 0.30)
                self.assertLessEqual(
                    arc["linear_speed"] * arc["command_hold_timeout"],
                    arc["safety_lookahead"],
                )

                ramp_time = arc["linear_speed"] / arc["acceleration_limit"]
                ramp_distance = (
                    0.5 * arc["acceleration_limit"] * ramp_time * ramp_time
                )
                expected_duration = ramp_time + max(
                    0.0,
                    (arc["max_distance"] - ramp_distance) / arc["linear_speed"],
                )
                self.assertLess(expected_duration, arc["max_duration"])


if __name__ == "__main__":
    unittest.main()
