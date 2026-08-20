#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]
KNOWN_MAP_SOURCE = (
    WORKSPACE_ROOT
    / "src"
    / "localization_fastlio"
    / "fast_lio_localization"
    / "src"
    / "fast_lio_map_localizer.cpp"
).read_text(encoding="utf-8")


class StaticMapNavigationContractTest(unittest.TestCase):
    def test_static_costmaps_share_fixed_map_frame(self):
        global_config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "global_costmap_static.yaml").read_text(
                encoding="utf-8"
            )
        )["global_costmap"]
        local_config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "local_costmap_static.yaml").read_text(
                encoding="utf-8"
            )
        )["local_costmap"]
        self.assertEqual("map", global_config["global_frame"])
        self.assertEqual("map", local_config["global_frame"])
        self.assertTrue(global_config["static_map"])
        self.assertFalse(local_config["static_map"])
        self.assertTrue(local_config["rolling_window"])

    def test_navigation_uses_map_server_and_localization_velocity_gate_without_amcl(self):
        launch_path = PACKAGE_ROOT / "launch" / "navigation_j6m.launch"
        root = ElementTree.parse(str(launch_path)).getroot()
        text = launch_path.read_text(encoding="utf-8")
        arguments = {item.attrib["name"] for item in root.findall("arg")}
        self.assertTrue({"use_static_map", "map_file"} <= arguments)
        self.assertIn('pkg="map_server" type="map_server"', text)
        self.assertIn('type="localization_cmd_vel_gate.py"', text)
        self.assertNotIn('pkg="amcl"', text)

    def test_known_map_localizer_uses_multiscale_icp_and_separate_map_tf(self):
        self.assertGreaterEqual(KNOWN_MAP_SOURCE.count("IterativeClosestPoint"), 2)
        self.assertIn("map_to_odom_", KNOWN_MAP_SOURCE)
        self.assertIn("initialPoseCallback", KNOWN_MAP_SOURCE)
        self.assertIn('state_ = "WAITING_INITIAL_POSE"', KNOWN_MAP_SOURCE)
        self.assertIn('"/fast_lio/localization_status"', KNOWN_MAP_SOURCE)

    def test_dual_host_launch_is_map_free_by_default(self):
        launch_path = (
            WORKSPACE_ROOT
            / "src"
            / "platform"
            / "autolabor_dual_host"
            / "launch"
            / "j6m_fastlio_navigation.launch"
        )
        root = ElementTree.parse(str(launch_path)).getroot()
        arguments = {
            item.attrib["name"]: item.attrib.get("default")
            for item in root.findall("arg")
        }
        self.assertEqual("false", arguments["use_static_map"])
        self.assertEqual("", arguments["map_file"])
        self.assertEqual("", arguments["fast_lio_map_file"])
        text = launch_path.read_text(encoding="utf-8")
        self.assertIn("$(find fast_lio_localization)/launch/known_map_localization.launch", text)
        self.assertNotIn('arg name="localization_enabled"', text)
        self.assertNotIn('arg name="map_file_path"', text)

    def test_startup_accepts_and_synchronizes_a_complete_map_set(self):
        start = (WORKSPACE_ROOT / "scripts" / "start_dual_host.sh").read_text(
            encoding="utf-8"
        )
        health = (WORKSPACE_ROOT / "scripts" / "health_check.sh").read_text(
            encoding="utf-8"
        )
        sync = (WORKSPACE_ROOT / "scripts" / "sync_static_map.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("--map-set", start)
        self.assertIn("--static-map-source", start)
        self.assertLess(start.index("sync_static_map.sh"), start.index("Starting the J6M ROS master"))
        self.assertIn("global_maps/map_sets", sync)
        self.assertIn("map_3d/map.pcd", sync)
        self.assertIn("map_fused_2d/map.yaml", sync)
        self.assertIn("runtime/run/map_mode.env", health)
        self.assertIn("runtime_nodes+=(/map_server /fast_lio_map_localizer", health)


if __name__ == "__main__":
    unittest.main()
