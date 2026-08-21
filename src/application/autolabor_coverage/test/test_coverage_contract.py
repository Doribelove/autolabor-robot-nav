#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]


class CoverageContractTest(unittest.TestCase):
    def test_plugin_and_static_map_launch_are_wired(self):
        plugin = ElementTree.parse(
            str(PACKAGE_ROOT / "coverage_global_planner_plugin.xml")
        ).getroot()
        planner = plugin.find("class")
        self.assertEqual("autolabor_coverage/CoverageGlobalPlanner",
                         planner.attrib["name"])
        navigation = (WORKSPACE_ROOT / "src" / "scripts" / "robot_bringup" /
                      "launch" / "navigation_j6m.launch").read_text(encoding="utf-8")
        self.assertIn("$(find autolabor_coverage)/launch/coverage.launch", navigation)
        self.assertIn('name="base_global_planner"', navigation)
        self.assertIn('value="autolabor_coverage/CoverageGlobalPlanner"', navigation)
        self.assertIn('if="$(arg use_static_map)"', navigation)

    def test_sweep_path_is_published_before_the_move_base_goal(self):
        manager = (PACKAGE_ROOT / "scripts" / "coverage_manager.py").read_text(
            encoding="utf-8")
        first_publish = manager.index("self.enforced_path_pub.publish(enforced)")
        first_goal = manager.index("self.move_base.send_goal(goal)")
        self.assertLess(first_publish, first_goal)
        self.assertIn('segment["type"] == "transit"', manager)
        self.assertIn("self.allow_reverse_transit", manager)
        plugin = (PACKAGE_ROOT / "src" / "coverage_global_planner.cpp").read_text(
            encoding="utf-8")
        self.assertIn("if (!path.active)", plugin)
        self.assertIn("if (active)", plugin)

    def test_fod_pause_does_not_reissue_a_coverage_endpoint(self):
        bridge = (WORKSPACE_ROOT / "src" / "platform" /
                  "autolabor_dual_host" / "scripts" /
                  "move_base_pause_bridge.py").read_text(encoding="utf-8")
        self.assertIn('"/coverage/active"', bridge)
        self.assertIn("not self.coverage_active", bridge)

    def test_j6m_deploy_builds_and_verifies_coverage_package(self):
        deploy = (WORKSPACE_ROOT / "scripts" / "deploy_j6m.sh").read_text(
            encoding="utf-8")
        self.assertIn("./src/application/autolabor_coverage", deploy)
        self.assertIn("fast_lio_localization\\\\;autolabor_coverage\\\\;robot_bringup",
                      deploy)
        self.assertIn("rospack find autolabor_coverage", deploy)
        self.assertIn("libcoverage_global_planner.so", deploy)


if __name__ == "__main__":
    unittest.main()
