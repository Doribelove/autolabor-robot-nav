#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[2]


class StaticMapNavigationContractTest(unittest.TestCase):
    def test_amcl_corrects_fast_lio_frame_without_replacing_local_odometry(self):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "amcl_fast_lio.yaml").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual("map", config["global_frame_id"])
        self.assertEqual("camera_init", config["odom_frame_id"])
        self.assertEqual("base_link", config["base_frame_id"])
        self.assertTrue(config["tf_broadcast"])

    def test_static_global_and_smooth_rolling_local_costmaps_are_separate(self):
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
        self.assertTrue(global_config["static_map"])
        self.assertFalse(global_config["rolling_window"])
        self.assertEqual("costmap_2d::StaticLayer", global_config["plugins"][0]["type"])
        self.assertEqual("/map", global_config["static_layer"]["map_topic"])
        self.assertEqual("camera_init", local_config["global_frame"])
        self.assertFalse(local_config["static_map"])
        self.assertTrue(local_config["rolling_window"])

    def test_navigation_launch_owns_map_server_amcl_and_static_switch(self):
        launch_path = PACKAGE_ROOT / "launch" / "navigation_j6m.launch"
        root = ElementTree.parse(str(launch_path)).getroot()
        text = launch_path.read_text(encoding="utf-8")
        arguments = {item.attrib["name"] for item in root.findall("arg")}
        self.assertTrue({"use_static_map", "map_file", "start_amcl"} <= arguments)
        self.assertIn('pkg="map_server" type="map_server"', text)
        self.assertIn('pkg="amcl" type="amcl"', text)
        self.assertIn('global_costmap_static.yaml', text)
        self.assertIn('local_costmap_static.yaml', text)

    def test_dual_host_launch_enables_static_localization_by_default(self):
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
        self.assertEqual("true", arguments["use_static_map"])
        self.assertEqual(
            "/var/lib/autolabor/maps/current/map.yaml", arguments["map_file"]
        )

    def test_startup_synchronizes_latest_map_before_remote_navigation(self):
        start = (WORKSPACE_ROOT / "scripts" / "start_dual_host.sh").read_text(
            encoding="utf-8"
        )
        sync = (WORKSPACE_ROOT / "scripts" / "sync_static_map.sh").read_text(
            encoding="utf-8"
        )
        self.assertLess(start.index("sync_static_map.sh"), start.index('Starting the J6M ROS master'))
        self.assertIn("global_maps/static_maps/latest", sync)
        self.assertIn("/var/lib/autolabor/maps/current/map.yaml", sync)


if __name__ == "__main__":
    unittest.main()
