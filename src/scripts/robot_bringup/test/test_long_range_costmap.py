#!/usr/bin/env python3

import math
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

import yaml


WORKSPACE = Path(__file__).resolve().parents[4]
BRINGUP_SCRIPT = WORKSPACE / "scripts" / "bringup.sh"
ROBOT_LAUNCH = (
    WORKSPACE
    / "src"
    / "scripts"
    / "robot_bringup"
    / "launch"
    / "navigation_arena.launch"
)
ARENA_ROOT = WORKSPACE / "src" / "navigation_arena" / "arena-rosnav-3D"
REAL_NAV_LAUNCH = ARENA_ROOT / "arena_bringup" / "launch" / "real_nav_nomap.launch"
MOVE_BASE_LAUNCH = (
    ARENA_ROOT
    / "arena_bringup"
    / "launch"
    / "sublaunch_testing"
    / "move_base"
    / "move_base_teb_nomap.launch"
)
BASE_GLOBAL_COSTMAP = (
    ARENA_ROOT
    / "arena_navigation"
    / "arena_local_planer"
    / "model_based"
    / "conventional"
    / "config"
    / "dingo"
    / "global_costmap_params_nomap.yaml"
)


def shell_default(script, variable):
    match = re.search(
        rf'^{variable}="\$\{{{variable}:-([^}}]+)\}}"$', script, re.MULTILINE
    )
    if match is None:
        raise AssertionError(f"missing shell default for {variable}")
    return float(match.group(1))


def launch_arg_names(path):
    root = ET.parse(path).getroot()
    return {element.attrib["name"] for element in root.findall("arg")}


def forwarded_arg_values(path):
    root = ET.parse(path).getroot()
    include = root.find("include")
    if include is None:
        raise AssertionError(f"missing include in {path}")
    return {
        element.attrib["name"]: element.attrib.get("value")
        for element in include.findall("arg")
    }


class LongRangeGlobalCostmapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.script = BRINGUP_SCRIPT.read_text(encoding="utf-8")
        cls.size = shell_default(cls.script, "GPS_GLOBAL_COSTMAP_SIZE")
        cls.resolution = shell_default(
            cls.script, "GPS_GLOBAL_COSTMAP_RESOLUTION"
        )

    def test_default_window_reaches_beyond_a_50_meter_goal(self):
        # Keep margin between the goal and the rolling-window edge so NavFn can
        # inflate the footprint and still terminate on an in-bounds cell.
        self.assertGreaterEqual(self.size / 2.0 - 50.0, 25.0)

    def test_default_grid_has_a_bounded_resource_cost(self):
        cells_per_side = math.ceil(self.size / self.resolution)
        self.assertLessEqual(cells_per_side * cells_per_side, 1_000_000)
        self.assertGreaterEqual(self.resolution, 0.2)

    def test_base_global_costmap_is_a_robot_centered_rolling_window(self):
        with BASE_GLOBAL_COSTMAP.open(encoding="utf-8") as stream:
            global_costmap = yaml.safe_load(stream)["global_costmap"]
        self.assertTrue(global_costmap["rolling_window"])
        self.assertFalse(global_costmap["static_map"])

    def test_bringup_exposes_validates_and_passes_both_overrides(self):
        self.assertIn('is_positive_number "$GPS_GLOBAL_COSTMAP_SIZE"', self.script)
        self.assertIn(
            'is_positive_number "$GPS_GLOBAL_COSTMAP_RESOLUTION"', self.script
        )
        self.assertIn('cells <= 1000000', self.script)
        self.assertIn(
            'global_costmap_size:="$GPS_GLOBAL_COSTMAP_SIZE"', self.script
        )
        self.assertIn(
            'global_costmap_resolution:="$GPS_GLOBAL_COSTMAP_RESOLUTION"',
            self.script,
        )

    def test_overrides_are_forwarded_through_both_launch_layers(self):
        for launch_file in (ROBOT_LAUNCH, REAL_NAV_LAUNCH):
            self.assertTrue(
                {"global_costmap_size", "global_costmap_resolution"}
                <= launch_arg_names(launch_file)
            )
            forwarded = forwarded_arg_values(launch_file)
            self.assertEqual(
                forwarded["global_costmap_size"], "$(arg global_costmap_size)"
            )
            self.assertEqual(
                forwarded["global_costmap_resolution"],
                "$(arg global_costmap_resolution)",
            )

    def test_move_base_applies_the_overrides_after_optional_profile(self):
        root = ET.parse(MOVE_BASE_LAUNCH).getroot()
        node = root.find("node")
        self.assertIsNotNone(node)
        parameters = {
            element.attrib["name"]: element.attrib.get("value")
            for element in node.findall("param")
        }
        self.assertEqual(
            parameters["global_costmap/width"], "$(arg global_costmap_size)"
        )
        self.assertEqual(
            parameters["global_costmap/height"], "$(arg global_costmap_size)"
        )
        self.assertEqual(
            parameters["global_costmap/resolution"],
            "$(arg global_costmap_resolution)",
        )
        expected_origin = "$(eval -float(arg('global_costmap_size')) / 2.0)"
        self.assertEqual(parameters["global_costmap/origin_x"], expected_origin)
        self.assertEqual(parameters["global_costmap/origin_y"], expected_origin)

        text = MOVE_BASE_LAUNCH.read_text(encoding="utf-8")
        self.assertLess(
            text.index('file="$(arg teb_profile_file)"'),
            text.index('name="global_costmap/width"'),
        )


if __name__ == "__main__":
    unittest.main()
