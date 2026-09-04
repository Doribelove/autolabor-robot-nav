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
MAPPING_SESSION = (WORKSPACE_ROOT / "scripts/global_mapping_session.sh").read_text(
    encoding="utf-8"
)
OFFLINE_MAPPING = (
    WORKSPACE_ROOT / "scripts/build_static_map_from_bag.sh"
).read_text(encoding="utf-8")
VOXEL_MAPPER_SOURCE = (
    PACKAGE_ROOT / "src/voxel_cloud_mapper.cpp"
).read_text(encoding="utf-8")
TEB_ROOT = (
    WORKSPACE_ROOT
    / "src/navigation_arena/forks/navigation/local_planner/teb"
)
TEB_CONFIG_HEADER = (
    TEB_ROOT / "include/teb_local_planner/teb_config.h"
).read_text(encoding="utf-8")
TEB_RECONFIGURE = (
    TEB_ROOT / "cfg/TebLocalPlannerReconfigure.cfg"
).read_text(encoding="utf-8")
TEB_OPTIMAL_PLANNER = (
    TEB_ROOT / "src/optimal_planner.cpp"
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
        self.assertEqual(20.0, local_config["width"])
        self.assertEqual(20.0, local_config["height"])
        self.assertEqual(0.10, local_config["resolution"])
        self.assertEqual(
            10.0,
            local_config["obstacles_layer"]["scan"]["obstacle_range"],
        )
        self.assertEqual(
            11.0,
            local_config["obstacles_layer"]["scan"]["raytrace_range"],
        )
        local_plugins = {
            plugin["name"]: plugin["type"] for plugin in local_config["plugins"]
        }
        self.assertEqual(
            "costmap_2d::StaticLayer", local_plugins["static_layer"]
        )
        self.assertEqual("/map", local_config["static_layer"]["map_topic"])
        self.assertTrue(local_config["static_layer"]["track_unknown_space"])
        for config in (global_config, local_config):
            self.assertEqual(-1, config["static_layer"]["unknown_cost_value"])
            self.assertEqual(
                "unknown_space_guard_layer", config["plugins"][-1]["name"]
            )
            self.assertEqual(
                "robot_bringup/UnknownSpaceGuardLayer",
                config["plugins"][-1]["type"],
            )
            self.assertTrue(config["unknown_space_guard_layer"]["enabled"])
            self.assertEqual(
                "/map", config["unknown_space_guard_layer"]["map_topic"]
            )

    def test_navigation_uses_map_server_and_localization_velocity_gate_without_amcl(self):
        launch_path = PACKAGE_ROOT / "launch" / "navigation_j6m.launch"
        root = ElementTree.parse(str(launch_path)).getroot()
        text = launch_path.read_text(encoding="utf-8")
        arguments = {
            item.attrib["name"]: item.attrib.get("default")
            for item in root.findall("arg")
        }
        self.assertTrue({"use_static_map", "map_file"} <= set(arguments))
        self.assertEqual(
            "$(eval '/odom' if arg('use_static_map') else arg('odom_topic'))",
            arguments["teb_odom_topic"],
        )
        self.assertEqual("4.0", arguments["static_teb_lookahead_dist"])
        self.assertEqual("/cmd_vel_teb", arguments["teb_command_topic"])
        self.assertEqual(
            "$(eval arg('teb_command_topic') if arg('use_static_map') else arg('cmd_vel_topic'))",
            arguments["move_base_cmd_vel_topic"],
        )
        move_base = root.find("./node[@name='move_base']")
        self.assertIsNotNone(move_base)
        move_base_parameters = {
            item.attrib["name"]: item.attrib.get("value")
            for item in move_base.findall("param")
        }
        move_base_remaps = {
            item.attrib["from"]: item.attrib.get("to")
            for item in move_base.findall("remap")
        }
        self.assertEqual(
            "$(arg teb_odom_topic)",
            move_base_remaps["odom"],
        )
        self.assertEqual(
            "$(arg move_base_cmd_vel_topic)",
            move_base_remaps["cmd_vel"],
        )
        self.assertEqual(
            "$(arg teb_odom_topic)",
            move_base_parameters["TebLocalPlannerROS/odom_topic"],
        )
        self.assertIn('pkg="map_server" type="map_server"', text)
        self.assertIn('type="localization_cmd_vel_gate.py"', text)
        self.assertIn('<param name="cancel_topic" value="/move_base/cancel"/>', text)
        self.assertIn('<param name="goal_topic" value="/move_base/goal"/>', text)
        self.assertIn(
            'name="TebLocalPlannerROS/feasibility_check_no_poses"', text
        )
        self.assertIn('type="int" value="50"', text)
        self.assertIn(
            'name="TebLocalPlannerROS/max_global_plan_lookahead_dist"', text
        )
        self.assertIn(
            'type="double" value="$(arg static_teb_lookahead_dist)"', text
        )
        self.assertIn(
            'name="TebLocalPlannerROS/control_look_ahead_poses"', text
        )
        self.assertIn('type="int" value="2"', text)
        self.assertIn(
            'name="TebLocalPlannerROS/treat_unknown_as_obstacle"', text
        )
        self.assertIn(
            'name="CoverageGlobalPlanner_navfn/allow_unknown"', text
        )
        self.assertIn(
            'name="CoverageGlobalPlanner/hybrid_minimum_turning_radius"',
            text,
        )
        self.assertIn(
            'name="CoverageGlobalPlanner/hybrid_planning_timeout"', text
        )
        self.assertIn(
            'name="CoverageGlobalPlanner/hybrid_cache_collision_check_horizon"',
            text,
        )
        self.assertNotIn('pkg="amcl"', text)

    def test_static_coverage_mux_preserves_localization_gate_and_live_tf_pose(self):
        navigation = (
            PACKAGE_ROOT / "launch" / "navigation_j6m.launch"
        ).read_text(encoding="utf-8")
        coverage_launch_path = (
            WORKSPACE_ROOT / "src" / "application" /
            "autolabor_coverage" / "launch" / "coverage.launch"
        )
        coverage_launch = coverage_launch_path.read_text(encoding="utf-8")
        coverage_config = yaml.safe_load(
            (
                WORKSPACE_ROOT / "src" / "application" /
                "autolabor_coverage" / "config" / "coverage.yaml"
            ).read_text(encoding="utf-8")
        )
        self.assertIn(
            '<arg name="command_topic" value="$(arg localization_gate_input_topic)"/>',
            navigation,
        )
        self.assertIn(
            '<arg name="teb_command_topic" value="$(arg teb_command_topic)"/>',
            navigation,
        )
        self.assertIn(
            'type="hybrid_teb_command_mux.py"', coverage_launch
        )
        self.assertIn(
            'name="hybrid_teb_command_mux" output="screen" required="true"',
            coverage_launch,
        )
        self.assertIn(
            '<param name="use_tf_pose" type="bool" value="$(arg use_tf_pose)"/>',
            coverage_launch,
        )
        self.assertIn(
            '<param name="command_topic" value="$(arg command_topic)"/>',
            coverage_launch,
        )
        self.assertIn(
            'value="/move_base/CoverageGlobalPlanner/hybrid_path_safe"',
            coverage_launch,
        )
        self.assertIn(
            '<param name="safety_timeout" value="1.50"/>',
            coverage_launch,
        )
        self.assertIn('type="double" value="3.00"', navigation)
        self.assertTrue(coverage_config["hierarchical_hybrid_on_demand"])
        self.assertTrue(coverage_config["direct_hybrid_to_final_goal"])
        self.assertTrue(coverage_config["entry_navfn_single_topology"])
        self.assertFalse(coverage_config["hybrid_execute_unsplit_cusps"])
        self.assertFalse(coverage_config["online_hybrid_without_precompute"])
        self.assertEqual(1.35, coverage_config["minimum_turning_radius_m"])
        self.assertEqual(
            2.0, coverage_config["hybrid_transit_lookahead_dist_m"]
        )

    def test_static_teb_cruise_objective_can_reach_the_dynamic_speed_ceiling(self):
        teb = yaml.safe_load(
            (
                WORKSPACE_ROOT / "src" / "navigation_arena" /
                "arena-rosnav-3D" / "arena_navigation" /
                "arena_local_planer" / "model_based" / "conventional" /
                "config" / "dingo" / "teb_local_planner_params_nomap.yaml"
            ).read_text(encoding="utf-8")
        )["TebLocalPlannerROS"]
        self.assertEqual(12.0, teb["weight_optimaltime"])
        self.assertEqual(10.0, teb["weight_acc_lim_x"])

    def test_static_navigation_rejects_unknown_without_changing_nomap_default(self):
        self.assertIn("bool treat_unknown_as_obstacle", TEB_CONFIG_HEADER)
        self.assertIn(
            "trajectory.treat_unknown_as_obstacle = false", TEB_CONFIG_HEADER
        )
        self.assertIn('grp_trajectory.add("treat_unknown_as_obstacle"',
                      TEB_RECONFIGURE)
        self.assertIn("cfg_->trajectory.treat_unknown_as_obstacle",
                      TEB_OPTIMAL_PLANNER)
        self.assertIn("footprint_cost == -2", TEB_OPTIMAL_PLANNER)
        self.assertIn(
            "footprint_cost == -3 && pose_index == 0", TEB_OPTIMAL_PLANNER
        )
        self.assertNotIn("footprint_cost < 0", TEB_OPTIMAL_PLANNER)
        self.assertIn("an out-of-rolling-map (-3) future sample", TEB_CONFIG_HEADER)

    def test_three_dimensional_and_two_dimensional_maps_share_fast_lio_odometry(self):
        self.assertIn("_input_topic:=/cloud_registered", MAPPING_SESSION)
        self.assertIn("_odom_topic:=/Odometry", MAPPING_SESSION)
        self.assertIn("--scan-topic /dual_lidar/scan", MAPPING_SESSION)
        self.assertIn("--odom-topic /Odometry", MAPPING_SESSION)
        self.assertIn("map_set_fuser.py", MAPPING_SESSION)

    def test_history_cloud_preview_is_subscriber_gated_and_matches_saved_pcd(self):
        for evidence in (
            'param<std::string>("history_topic", history_topic_',
            '"/static_mapping/history_cloud"',
            'param<double>("history_publish_period", history_publish_period_, 1.0)',
            "history_publisher_.getNumSubscribers() == 0U",
            "ros::WallTime::now()",
            "publishHistoryCloudIfRequested(cloud->header.stamp)",
            "modifier.setPointCloud2Fields(",
            'sensor_msgs::PointCloud2Iterator<std::uint32_t> observations(',
        ):
            self.assertIn(evidence, VOXEL_MAPPER_SOURCE)
        # Preview and the final PCD intentionally share exactly one persistent
        # voxel extraction path, rather than accumulating raw RViz scan history.
        self.assertIn("std::vector<PcdPoint> persistentPoints() const", VOXEL_MAPPER_SOURCE)
        self.assertGreaterEqual(
            VOXEL_MAPPER_SOURCE.count(
                "const std::vector<PcdPoint> points = persistentPoints();"
            ),
            2,
        )

    def test_mid360_static_slice_defaults_are_consistent(self):
        for script in (MAPPING_SESSION, OFFLINE_MAPPING):
            self.assertIn(
                'SLICE_CENTER_Z="${MAPPING_SLICE_CENTER_Z:--0.4}"', script
            )
            self.assertIn(
                'SLICE_HALF_WIDTH="${MAPPING_SLICE_HALF_WIDTH:-0.20}"', script
            )
            self.assertIn(
                'SLICE_SELF_CROP_ENABLED="${MAPPING_SLICE_SELF_CROP_ENABLED:-true}"',
                script,
            )
            self.assertIn('_slice_self_crop_enabled:="$SLICE_SELF_CROP_ENABLED"', script)
            self.assertIn('_slice_sweep_front:="$SLICE_SWEEP_FRONT"', script)
            self.assertIn('_slice_sweep_rear:="$SLICE_SWEEP_REAR"', script)
            self.assertIn('_slice_sweep_half_width:="$SLICE_SWEEP_HALF_WIDTH"', script)
            self.assertIn('_body_to_base_z:="$BASE_OFFSET_Z"', script)
        self.assertIn(
            'param<double>("slice_center_z", slice_center_z_, -0.4)',
            VOXEL_MAPPER_SOURCE,
        )
        self.assertIn(
            'param<double>("slice_half_width", slice_half_width_, 0.20)',
            VOXEL_MAPPER_SOURCE,
        )
        self.assertIn("sync_policies::ExactTime", VOXEL_MAPPER_SOURCE)
        self.assertIn("recordSweptCrop", VOXEL_MAPPER_SOURCE)
        self.assertIn("mapPointInsideBaseCrop", VOXEL_MAPPER_SOURCE)
        self.assertIn('<< "schema_version: 2\\n"', VOXEL_MAPPER_SOURCE)

    def test_offline_mapping_is_deterministic_and_waits_for_voxel_subscriptions(self):
        play_index = OFFLINE_MAPPING.index("rosbag play --clock")
        for command in (
            "wait_for_subscription /voxel_cloud_mapper /cloud_registered",
            "wait_for_subscription /voxel_cloud_mapper /Odometry",
        ):
            self.assertIn(command, OFFLINE_MAPPING)
            self.assertLess(OFFLINE_MAPPING.index(command), play_index)
        self.assertIn('fused_scan_mapper.py --bag "$BAG_PATH"', OFFLINE_MAPPING)
        self.assertNotIn("fused_scan_mapper.py --ros", OFFLINE_MAPPING)

    def test_static_navigation_obstacles_still_use_planar_fused_scan(self):
        common = yaml.safe_load(
            (PACKAGE_ROOT / "config/costmap_common_static.yaml").read_text(
                encoding="utf-8"
            )
        )
        local = yaml.safe_load(
            (PACKAGE_ROOT / "config/local_costmap_static.yaml").read_text(
                encoding="utf-8"
            )
        )["local_costmap"]
        scan_launch = ElementTree.parse(
            str(PACKAGE_ROOT / "launch/scan_mid360_avoidance.launch")
        ).getroot()
        scan_arguments = {
            item.attrib["name"]: item.attrib.get("default")
            for item in scan_launch.findall("arg")
        }

        self.assertEqual("0.25", scan_arguments["min_height"])
        self.assertEqual("0.8", scan_arguments["max_height"])
        for scan in (
            common["obstacles_layer"]["scan"],
            local["obstacles_layer"]["scan"],
        ):
            self.assertEqual("LaserScan", scan["data_type"])
            self.assertEqual("/scan", scan["topic"])
            self.assertEqual(-2.0, scan["min_obstacle_height"])

    def test_indoor_navigation_default_forward_speed_is_point_eight(self):
        launch_path = PACKAGE_ROOT / "launch" / "navigation_j6m.launch"
        root = ElementTree.parse(str(launch_path)).getroot()
        arguments = {
            item.attrib["name"]: item.attrib.get("default")
            for item in root.findall("arg")
        }
        self.assertEqual("0.8", arguments["max_vel_x"])
        self.assertEqual("0.3", arguments["max_vel_x_backwards"])
        self.assertEqual("0.6", arguments["max_vel_theta"])
        text = launch_path.read_text(encoding="utf-8")
        self.assertIn("TebLocalPlannerROS/max_vel_theta", text)

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
        self.assertIn("/coverage_manager", health)
        self.assertIn("/fast_lio_map_localizer/good_matches_required 2", health)
        self.assertIn("/fast_lio_localization_cmd_vel_gate/cancel_topic", health)


if __name__ == "__main__":
    unittest.main()
