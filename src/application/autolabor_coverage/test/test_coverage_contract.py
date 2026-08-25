#!/usr/bin/env python3

from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


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
        handoff_service = (PACKAGE_ROOT / "srv" / "SetEnforcedPath.srv").read_text(
            encoding="utf-8"
        )
        first_publish = manager.index("self.enforced_path_pub.publish(enforced)")
        first_goal = manager.index("self.move_base.send_goal(goal)")
        self.assertLess(first_publish, first_goal)
        synchronous_handoff = manager.index("self._set_enforced_path(enforced)")
        self.assertLess(synchronous_handoff, first_goal)
        self.assertIn("coverage_active=coverage_active", manager)
        self.assertIn("enforced_path=enforced", manager)
        self.assertIn("coverage_active=False", manager)
        self.assertIn("bool coverage_active", handoff_service)
        self.assertIn('segment["type"] == "transit"', manager)
        self.assertIn("self.allow_reverse_transit", manager)
        plugin = (PACKAGE_ROOT / "src" / "coverage_global_planner.cpp").read_text(
            encoding="utf-8")
        self.assertIn('advertiseService(', plugin)
        self.assertIn('"set_enforced_path"', plugin)
        self.assertIn("coverage_active_ = request.coverage_active", plugin)
        self.assertIn("refresh does not match the synchronously armed", plugin)
        self.assertNotIn('subscribe(\n      "/coverage/active"', plugin)
        self.assertIn("if (!path.active)", plugin)
        self.assertIn("if (!active)", plugin)
        self.assertIn("POINT_TO_POINT_NAVFN_TRANSIT", plugin)
        self.assertIn("fallback_.makePlan(start, goal, plan)", plugin)
        self.assertIn("goal yaw does not match enforced path endpoint", plugin)
        self.assertIn("world_model.footprintCost", plugin)
        self.assertIn(
            "executing point-to-point Navfn transit", manager
        )

        navigation = (
            WORKSPACE_ROOT / "src" / "scripts" / "robot_bringup" /
            "launch" / "navigation_j6m.launch"
        ).read_text(encoding="utf-8")
        self.assertIn('<arg name="planner_frequency" default="1.0"/>', navigation)
        self.assertIn(
            '<param name="planner_frequency" type="double" '
            'value="$(arg planner_frequency)"/>',
            navigation,
        )

    def test_fod_pause_does_not_reissue_a_coverage_endpoint(self):
        bridge = (WORKSPACE_ROOT / "src" / "platform" /
                  "autolabor_dual_host" / "scripts" /
                  "move_base_pause_bridge.py").read_text(encoding="utf-8")
        self.assertIn('"/coverage/active"', bridge)
        self.assertIn("not self.coverage_active", bridge)
        self.assertIn("self.paused or self.coverage_active", bridge)
        self.assertIn("simple navigation goal rejected: coverage owns move_base", bridge)
        self.assertIn('"/navigation_goal/accepted"', bridge)

        navigation = (WORKSPACE_ROOT / "src" / "scripts" / "robot_bringup" /
                      "launch" / "navigation_j6m.launch").read_text(encoding="utf-8")
        dual_host_launch = (WORKSPACE_ROOT / "src" / "platform" /
                            "autolabor_dual_host" / "launch" /
                            "j6m_fastlio_navigation.launch").read_text(encoding="utf-8")
        self.assertIn('from="/move_base_simple/goal"', navigation)
        self.assertIn("$(arg move_base_simple_goal_topic)", navigation)
        self.assertIn("/navigation_goal/accepted", dual_host_launch)

    def test_j6m_deploy_builds_and_verifies_coverage_package(self):
        deploy = (WORKSPACE_ROOT / "scripts" / "deploy_j6m.sh").read_text(
            encoding="utf-8")
        self.assertIn("./src/application/autolabor_coverage", deploy)
        self.assertIn("./src/navigation_arena/forks/navigation/local_planner/teb",
                      deploy)
        self.assertIn("teb_local_planner", deploy)
        self.assertIn("rospack find teb_local_planner", deploy)
        self.assertIn("libteb_local_planner.so", deploy)
        self.assertIn("fast_lio_localization\\\\;autolabor_coverage\\\\;robot_bringup",
                      deploy)
        self.assertIn("rospack find autolabor_coverage", deploy)
        self.assertIn("libcoverage_global_planner.so", deploy)

    def test_task_speed_is_bounded_and_applied_through_teb(self):
        start_service = (PACKAGE_ROOT / "srv" / "StartCoverage.srv").read_text(
            encoding="utf-8"
        )
        manager = (PACKAGE_ROOT / "scripts" / "coverage_manager.py").read_text(
            encoding="utf-8"
        )
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "coverage.yaml").read_text(encoding="utf-8")
        )
        self.assertIn("float64 max_speed_mps", start_service)
        self.assertEqual(0.80, config["default_max_speed_mps"])
        self.assertEqual(1.60, config["max_speed_limit_mps"])
        self.assertEqual(0.30, config["reverse_transit_speed_mps"])
        self.assertIn("requested_speed > self.watchdog_max_linear_speed", manager)
        self.assertIn('"max_vel_x": self.task_max_speed', manager)
        self.assertIn('"max_vel_x": configuration.get', manager)

    def test_ackermann_limits_match_live_vcu_precheck_and_teb(self):
        manager = (PACKAGE_ROOT / "scripts" / "coverage_manager.py").read_text(
            encoding="utf-8"
        )
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "coverage.yaml").read_text(encoding="utf-8")
        )
        teb_path = (
            WORKSPACE_ROOT / "src" / "navigation_arena" / "arena-rosnav-3D" /
            "arena_navigation" / "arena_local_planer" / "model_based" /
            "conventional" / "config" / "dingo" /
            "teb_local_planner_params_nomap.yaml"
        )
        teb = yaml.safe_load(teb_path.read_text(encoding="utf-8"))[
            "TebLocalPlannerROS"
        ]
        self.assertEqual(1.00, config["operation_width_m"])
        self.assertEqual(1.60, config["max_speed_limit_mps"])
        self.assertEqual(1.35, config["minimum_turning_radius_m"])
        self.assertEqual(0.65, config["expected_wheelbase_m"])
        self.assertAlmostEqual(0.488692, config["expected_max_steering_angle_rad"])
        self.assertEqual(config["minimum_turning_radius_m"],
                         teb["min_turning_radius"])
        self.assertEqual(config["expected_wheelbase_m"], teb["wheelbase"])
        self.assertIs(True, teb["use_proportional_saturation"])
        self.assertIn('"/m2_driver/chassis_parameter"', manager)
        self.assertIn("required_steering + self.steering_angle_margin", manager)
        self.assertIn("TEB minimum turning radius is below", manager)
        self.assertIn("treat_unknown_as_obstacle", manager)
        self.assertIn("CoverageGlobalPlanner_navfn/allow_unknown", manager)
        self.assertIn('"allow_init_with_backwards_motion": backwards > 0.0',
                      manager)

    def test_status_reports_route_and_verified_chassis_constraints(self):
        status = (PACKAGE_ROOT / "msg" / "CoverageStatus.msg").read_text(
            encoding="utf-8"
        )
        for field in (
            "operation_width_m",
            "lane_spacing_m",
            "minimum_turning_radius_m",
            "required_steering_angle_rad",
            "chassis_wheelbase_m",
            "chassis_max_steering_angle_rad",
            "chassis_max_speed_mps",
            "kinematics_verified",
            "kinematics_detail",
            "chassis_ready",
            "chassis_detail",
            "avoidance_ready",
            "avoidance_detail",
        ):
            self.assertIn(field, status)

    def test_coverage_requires_fresh_complete_obstacle_sensing(self):
        manager = (PACKAGE_ROOT / "scripts" / "coverage_manager.py").read_text(
            encoding="utf-8"
        )
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "coverage.yaml").read_text(encoding="utf-8")
        )
        self.assertIs(True, config["require_dual_lidar_for_coverage"])
        self.assertEqual(0.5, config["avoidance_scan_fresh_sec"])
        self.assertEqual(1.0, config["dual_lidar_fresh_sec"])
        self.assertEqual(0.2, config["avoidance_scan_future_tolerance_sec"])
        self.assertEqual("base_link", config["avoidance_scan_frame"])
        self.assertIn('"/scan", LaserScan', manager)
        self.assertIn('"/avoidance/dual_lidar_active"', manager)
        self.assertIn("_validate_avoidance_scan", manager)
        self.assertIn("math.isfinite(value) and value > 0.0", manager)
        self.assertIn("not self.avoidance_loss_paused", manager)
        self.assertIn("self._pause_for_avoidance_loss()", manager)
        self.assertIn("obstacle sensing is not ready", manager)

    def test_coverage_requires_fresh_fault_free_chassis_execution(self):
        manager = (PACKAGE_ROOT / "scripts" / "coverage_manager.py").read_text(
            encoding="utf-8"
        )
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "coverage.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(3.0, config["chassis_status_fresh_sec"])
        self.assertEqual(1.0, config["chassis_odom_fresh_sec"])
        self.assertEqual(3.0, config["chassis_monitor_fault_latch_sec"])
        self.assertIn('"/m2_driver/chassis_info"', manager)
        self.assertIn('"/m2_driver/chassis_monitor"', manager)
        self.assertIn('"/odom", Odometry', manager)
        self.assertIn("gamepad_emergency", manager)
        self.assertIn('(\"tcu\", \"TCU\")', manager)
        self.assertIn('prefix + "_state"', manager)
        self.assertIn("chassis execution is not ready", manager)
        self.assertIn("not self.chassis_fault_paused", manager)
        self.assertIn("self._pause_for_chassis_fault()", manager)
        m2_driver = (
            WORKSPACE_ROOT / "src" / "autolabor_core" /
            "autolabor_canbus_driver" / "autolabor_canbus_driver" /
            "src" / "m2_driver.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("three safety slots", m2_driver)
        self.assertIn("status_query_rate_limit_hz", m2_driver)
        self.assertIn("m2_status_query_rate_hz", m2_driver)
        self.assertIn("srv.request.requests.push_back(next_req)", m2_driver)
        self.assertNotIn("for (std::size_t index = 0;", m2_driver)


if __name__ == "__main__":
    unittest.main()
