#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


WORKSPACE_ROOT = Path(__file__).resolve().parents[4]
PROFILE_DIR = WORKSPACE_ROOT / "config" / "teb_profiles"
NAVIGATION_LAUNCH = (
    WORKSPACE_ROOT / "src" / "scripts" / "robot_bringup" / "launch" / "navigation_arena.launch"
)
BRINGUP_SCRIPT = WORKSPACE_ROOT / "scripts" / "bringup.sh"


class GpsTebProfileCurvatureTest(unittest.TestCase):
    @staticmethod
    def load_planner_params(profile_name):
        profile_path = PROFILE_DIR / profile_name
        with profile_path.open(encoding="utf-8") as stream:
            return yaml.safe_load(stream)["TebLocalPlannerROS"]

    def test_gps_profiles_preserve_curvature_when_saturating_velocity(self):
        for profile_name in ("gps_cruise.yaml", "gps_obstacle.yaml"):
            with self.subTest(profile=profile_name):
                params = self.load_planner_params(profile_name)
                self.assertIs(params.get("use_proportional_saturation"), True)

    def test_gps_profiles_keep_steering_margin(self):
        for profile_name in ("gps_cruise.yaml", "gps_obstacle.yaml"):
            with self.subTest(profile=profile_name):
                params = self.load_planner_params(profile_name)
                self.assertGreaterEqual(params.get("min_turning_radius", 0.0), 1.3)


class GpsGoalDistanceConfigurationTest(unittest.TestCase):
    def test_launch_has_distinct_planner_and_hard_stop_defaults(self):
        root = ElementTree.parse(str(NAVIGATION_LAUNCH)).getroot()
        arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in root.findall("arg")
        }
        self.assertIn("0.3", arguments["xy_goal_tolerance"])
        self.assertEqual(arguments["goal_slowdown_hard_stop_distance"], "0.2")

        limiter = next(
            node for node in root.findall("node") if node.attrib.get("name") == "gps_goal_speed_limiter"
        )
        parameters = {
            element.attrib["name"]: element.attrib.get("value")
            for element in limiter.findall("param")
        }
        self.assertEqual(
            parameters["hard_stop_distance"],
            "$(arg goal_slowdown_hard_stop_distance)",
        )
        self.assertEqual(
            parameters["planner_xy_goal_tolerance"],
            "$(arg xy_goal_tolerance)",
        )
        self.assertEqual(parameters["action_goal_topic"], "/move_base/goal")
        self.assertEqual(parameters["status_topic"], "/move_base/status")

    def test_bringup_passes_both_goal_distance_settings(self):
        script = BRINGUP_SCRIPT.read_text(encoding="utf-8")
        self.assertIn('GPS_XY_GOAL_TOLERANCE="${GPS_XY_GOAL_TOLERANCE:-0.3}"', script)
        self.assertIn(
            'GPS_GOAL_HARD_STOP_DISTANCE="${GPS_GOAL_HARD_STOP_DISTANCE:-0.2}"',
            script,
        )
        self.assertIn('xy_goal_tolerance:="$GPS_XY_GOAL_TOLERANCE"', script)
        self.assertIn(
            'goal_slowdown_hard_stop_distance:="$GPS_GOAL_HARD_STOP_DISTANCE"',
            script,
        )


class BringupCommandRouteSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.script = BRINGUP_SCRIPT.read_text(encoding="utf-8")

    def test_clean_start_stops_an_old_goal_speed_limiter(self):
        self.assertIn("    /gps_goal_speed_limiter\n", self.script)

    def test_route_check_requires_exactly_one_command_publisher(self):
        self.assertIn("publisher_count == 1", self.script)
        self.assertIn("Expected exactly one publisher", self.script)


if __name__ == "__main__":
    unittest.main()
