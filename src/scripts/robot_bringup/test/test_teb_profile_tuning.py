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
ARENA_BRINGUP = WORKSPACE_ROOT / "src" / "navigation_arena" / "arena-rosnav-3D" / "arena_bringup"
REAL_NAV_LAUNCH = ARENA_BRINGUP / "launch" / "real_nav_nomap.launch"
MOVE_BASE_TEB_LAUNCH = (
    ARENA_BRINGUP
    / "launch"
    / "sublaunch_testing"
    / "move_base"
    / "move_base_teb_nomap.launch"
)
GLOBAL_COSTMAP_CONFIG = (
    WORKSPACE_ROOT
    / "src"
    / "navigation_arena"
    / "arena-rosnav-3D"
    / "arena_navigation"
    / "arena_local_planer"
    / "model_based"
    / "conventional"
    / "config"
    / "dingo"
    / "global_costmap_params_nomap.yaml"
)
LONG_RANGE_GOAL_LAUNCH = (
    WORKSPACE_ROOT
    / "src"
    / "application"
    / "gps_module"
    / "launch"
    / "gps_long_range_goal.launch"
)
FOD_CONTROL_ROOT = (
    WORKSPACE_ROOT / "src" / "application" / "autolabor_fod_control"
)
FOD_INTEGRATED_LAUNCH = (
    FOD_CONTROL_ROOT / "launch" / "gps_visual_recovery_standby.launch"
)
FOD_VISUAL_LAUNCH = FOD_CONTROL_ROOT / "launch" / "visual_recovery.launch"
FOD_MODE_SCRIPT = WORKSPACE_ROOT / "scripts" / "fod_mode.sh"


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


class GpsLocalCostmapHorizonTest(unittest.TestCase):
    @staticmethod
    def load_profile(profile_name):
        profile_path = PROFILE_DIR / profile_name
        with profile_path.open(encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def test_cruise_uses_compact_costmap_with_explicit_safe_horizon(self):
        profile = self.load_profile("gps_cruise.yaml")
        costmap = profile["local_costmap"]
        planner = profile["TebLocalPlannerROS"]

        self.assertEqual(costmap["width"], 16.0)
        self.assertEqual(costmap["height"], 16.0)
        teb_map_limit = 0.85 * min(costmap["width"], costmap["height"]) / 2.0
        self.assertAlmostEqual(teb_map_limit, 6.8)
        self.assertEqual(planner["max_global_plan_lookahead_dist"], 6.5)
        self.assertLess(planner["max_global_plan_lookahead_dist"], teb_map_limit)

    def test_obstacle_profile_retains_larger_costmap_and_horizon(self):
        profile = self.load_profile("gps_obstacle.yaml")
        costmap = profile["local_costmap"]
        planner = profile["TebLocalPlannerROS"]

        self.assertEqual(costmap["width"], 24.0)
        self.assertEqual(costmap["height"], 24.0)
        self.assertEqual(planner["max_global_plan_lookahead_dist"], 10.0)
        teb_map_limit = 0.85 * min(costmap["width"], costmap["height"]) / 2.0
        self.assertLess(planner["max_global_plan_lookahead_dist"], teb_map_limit)


class GpsGeofenceIntegrationContractTest(unittest.TestCase):
    def test_both_gps_profiles_place_geofence_before_inflation(self):
        for profile_name in ("gps_cruise.yaml", "gps_obstacle.yaml"):
            with self.subTest(profile=profile_name):
                with (PROFILE_DIR / profile_name).open(encoding="utf-8") as stream:
                    profile = yaml.safe_load(stream)
                for costmap_name in ("global_costmap", "local_costmap"):
                    plugins = profile[costmap_name]["plugins"]
                    names = [plugin["name"] for plugin in plugins]
                    self.assertEqual(
                        names,
                        ["obstacles_layer", "gps_geofence_layer", "inflater_layer"],
                    )
                    self.assertEqual(
                        plugins[1]["type"],
                        "robot_bringup/GpsGeofenceLayer",
                    )

    def test_navigation_launch_loads_same_file_into_both_costmaps(self):
        root = ElementTree.parse(str(NAVIGATION_LAUNCH)).getroot()
        arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in root.findall("arg")
        }
        self.assertEqual(
            arguments["gps_geofence_enabled"],
            "$(eval arg('localization_source') == 'gps')",
        )
        group = next(
            element
            for element in root.findall("group")
            if element.attrib.get("if") == "$(arg gps_geofence_enabled)"
        )
        namespaces = [element.attrib["ns"] for element in group.findall("rosparam")]
        self.assertEqual(
            namespaces,
            [
                "move_base/global_costmap/gps_geofence_layer",
                "move_base/local_costmap/gps_geofence_layer",
            ],
        )

    def test_gps_bringup_enables_persistent_fence_by_default(self):
        script = BRINGUP_SCRIPT.read_text(encoding="utf-8")
        self.assertIn(
            'GPS_GEOFENCE_ENABLED="${GPS_GEOFENCE_ENABLED:-true}"',
            script,
        )
        self.assertIn('gps_geofence_file:="$GPS_GEOFENCE_FILE"', script)

class GpsCruiseLateralStabilityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with (PROFILE_DIR / "gps_cruise.yaml").open(encoding="utf-8") as stream:
            cls.cruise = yaml.safe_load(stream)["TebLocalPlannerROS"]
        cls.script = BRINGUP_SCRIPT.read_text(encoding="utf-8")

    def test_cruise_uses_damped_path_correction_limits(self):
        self.assertEqual(self.cruise["control_look_ahead_poses"], 2)
        self.assertGreaterEqual(self.cruise["global_plan_viapoint_sep"], 1.5)
        self.assertLessEqual(self.cruise["max_vel_theta"], 0.85)
        self.assertAlmostEqual(
            self.cruise["min_vel_theta"],
            -self.cruise["max_vel_theta"],
        )
        self.assertAlmostEqual(self.cruise["acc_lim_theta"], 0.70)
        self.assertGreaterEqual(self.cruise["weight_acc_lim_theta"], 250.0)
        self.assertGreaterEqual(self.cruise["min_turning_radius"], 1.35)
        self.assertIs(self.cruise["use_proportional_saturation"], True)
        self.assertLessEqual(self.cruise["weight_viapoint"], 6.0)
        self.assertLessEqual(self.cruise["weight_optimaltime"], 4.0)

    def test_quicker_turn_entry_does_not_cap_full_speed_straight_cruise(self):
        # max_vel_x remains the final bringup argument.  The overlay changes
        # only how quickly bounded yaw curvature may build.
        self.assertNotIn("max_vel_x", self.cruise)
        self.assertGreater(self.cruise["acc_lim_theta"], 0.45)

    def test_cruise_reduces_moving_gps_filter_lag_only_for_that_profile(self):
        cruise_start = self.script.index("    cruise)\n")
        obstacle_start = self.script.index("    obstacle)\n", cruise_start)
        profile_end = self.script.index("    *)\n", obstacle_start)
        cruise_block = self.script[cruise_start:obstacle_start]
        obstacle_block = self.script[obstacle_start:profile_end]

        self.assertIn(
            'GPS_POSITION_FILTER_ALPHA="${GPS_POSITION_FILTER_ALPHA:-0.70}"',
            cruise_block,
        )
        self.assertIn(
            'GPS_POSITION_FILTER_ALPHA="${GPS_POSITION_FILTER_ALPHA:-0.25}"',
            obstacle_block,
        )
        self.assertIn(
            'position_filter_alpha:="$GPS_POSITION_FILTER_ALPHA"',
            self.script,
        )

        # A first-order filter following constant 10 Hz, 2.7 m/s motion has
        # steady lag v*dt*(1-alpha)/alpha.  Keep the cruise default below 15 cm.
        cruise_lag_m = 2.7 * 0.1 * (1.0 - 0.70) / 0.70
        self.assertLess(cruise_lag_m, 0.15)

    def test_cruise_holds_a_stable_global_route_during_control(self):
        cruise_start = self.script.index("    cruise)\n")
        obstacle_start = self.script.index("    obstacle)\n", cruise_start)
        profile_end = self.script.index("    *)\n", obstacle_start)
        cruise_block = self.script[cruise_start:obstacle_start]
        obstacle_block = self.script[obstacle_start:profile_end]
        self.assertIn(
            'GPS_GLOBAL_PLANNER_FREQUENCY="${GPS_GLOBAL_PLANNER_FREQUENCY:-0.0}"',
            cruise_block,
        )
        self.assertIn(
            'GPS_GLOBAL_PLANNER_FREQUENCY="${GPS_GLOBAL_PLANNER_FREQUENCY:-1.0}"',
            obstacle_block,
        )
        self.assertIn(
            'planner_frequency:="$GPS_GLOBAL_PLANNER_FREQUENCY"',
            self.script,
        )

        nav_root = ElementTree.parse(str(NAVIGATION_LAUNCH)).getroot()
        real_root = ElementTree.parse(str(REAL_NAV_LAUNCH)).getroot()
        move_base_root = ElementTree.parse(str(MOVE_BASE_TEB_LAUNCH)).getroot()
        for root in (nav_root, real_root, move_base_root):
            arguments = {
                element.attrib["name"]: element.attrib.get("default")
                for element in root.findall("arg")
            }
            self.assertEqual(arguments["planner_frequency"], "1.0")

        nav_include_args = {
            element.attrib["name"]: element.attrib.get("value")
            for element in nav_root.find("include").findall("arg")
        }
        real_include_args = {
            element.attrib["name"]: element.attrib.get("value")
            for element in real_root.find("include").findall("arg")
        }
        self.assertEqual(nav_include_args["planner_frequency"], "$(arg planner_frequency)")
        self.assertEqual(real_include_args["planner_frequency"], "$(arg planner_frequency)")

        move_base_node = move_base_root.find("node")
        move_base_params = {
            element.attrib["name"]: element.attrib.get("value")
            for element in move_base_node.findall("param")
        }
        self.assertEqual(move_base_params["planner_frequency"], "$(arg planner_frequency)")


class GpsGoalDistanceConfigurationTest(unittest.TestCase):
    def test_launch_has_distinct_planner_and_hard_stop_defaults(self):
        root = ElementTree.parse(str(NAVIGATION_LAUNCH)).getroot()
        arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in root.findall("arg")
        }
        self.assertIn("0.3", arguments["xy_goal_tolerance"])
        self.assertEqual(arguments["goal_slowdown_hard_stop_distance"], "0.2")
        self.assertEqual(arguments["goal_speed_cap_enabled"], "false")
        self.assertEqual(arguments["goal_near_commit_distance"], "1.0")
        self.assertEqual(arguments["goal_near_timeout"], "15.0")
        self.assertEqual(arguments["goal_near_max_regression"], "0.5")

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
        self.assertEqual(
            parameters["speed_cap_enabled"],
            "$(arg goal_speed_cap_enabled)",
        )
        self.assertEqual(
            parameters["near_goal_commit_distance"],
            "$(arg goal_near_commit_distance)",
        )
        self.assertEqual(
            parameters["near_goal_timeout"],
            "$(arg goal_near_timeout)",
        )
        self.assertEqual(
            parameters["near_goal_max_regression"],
            "$(arg goal_near_max_regression)",
        )

    def test_bringup_passes_both_goal_distance_settings(self):
        script = BRINGUP_SCRIPT.read_text(encoding="utf-8")
        self.assertIn('GPS_XY_GOAL_TOLERANCE="${GPS_XY_GOAL_TOLERANCE:-0.3}"', script)
        self.assertIn(
            'GPS_GOAL_HARD_STOP_DISTANCE="${GPS_GOAL_HARD_STOP_DISTANCE:-0.2}"',
            script,
        )
        self.assertIn(
            'GPS_GOAL_SPEED_CAP_ENABLED="${GPS_GOAL_SPEED_CAP_ENABLED:-false}"',
            script,
        )
        self.assertIn('xy_goal_tolerance:="$GPS_XY_GOAL_TOLERANCE"', script)
        self.assertIn(
            'goal_slowdown_hard_stop_distance:="$GPS_GOAL_HARD_STOP_DISTANCE"',
            script,
        )
        self.assertIn(
            'goal_speed_cap_enabled:="$GPS_GOAL_SPEED_CAP_ENABLED"',
            script,
        )
        self.assertIn(
            'goal_near_commit_distance:="$GPS_GOAL_NEAR_COMMIT_DISTANCE"',
            script,
        )
        self.assertIn('goal_near_timeout:="$GPS_GOAL_NEAR_TIMEOUT"', script)
        self.assertIn(
            'goal_near_max_regression:="$GPS_GOAL_NEAR_MAX_REGRESSION"',
            script,
        )


class GpsLongRangeGoalConfigurationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.script = BRINGUP_SCRIPT.read_text(encoding="utf-8")
        cls.launch_root = ElementTree.parse(str(LONG_RANGE_GOAL_LAUNCH)).getroot()
        cls.launch_args = {
            element.attrib["name"]: element.attrib.get("default")
            for element in cls.launch_root.findall("arg")
        }

    def test_rolling_goal_defaults_leave_global_costmap_margin(self):
        with GLOBAL_COSTMAP_CONFIG.open(encoding="utf-8") as stream:
            global_costmap = yaml.safe_load(stream)["global_costmap"]
        half_width = min(
            global_costmap["width"],
            global_costmap["height"],
        ) / 2.0

        self.assertIs(global_costmap["rolling_window"], True)
        self.assertEqual(self.launch_args["lookahead_distance"], "15.0")
        self.assertEqual(self.launch_args["advance_distance"], "5.0")
        self.assertEqual(self.launch_args["max_lookahead_distance"], "18.0")
        self.assertLess(float(self.launch_args["lookahead_distance"]), half_width)
        self.assertLess(
            float(self.launch_args["max_lookahead_distance"]),
            half_width,
        )

    def test_launch_owns_final_gps_input_and_move_base_action_output(self):
        node = self.launch_root.find("node")
        self.assertEqual(node.attrib["name"], "gps_long_range_goal_manager")
        self.assertEqual(node.attrib["required"], "true")
        parameters = {
            element.attrib["name"]: element.attrib.get("value")
            for element in node.findall("param")
        }
        self.assertEqual(parameters["goal_fix_topic"], "/gps/goal_fix")
        self.assertEqual(parameters["odom_topic"], "$(arg odom_topic)")
        self.assertEqual(parameters["action_goal_topic"], "/move_base/goal")
        self.assertEqual(
            parameters["lookahead_distance"],
            "$(arg lookahead_distance)",
        )
        self.assertEqual(
            parameters["advance_distance"],
            "$(arg advance_distance)",
        )

    def test_gps_bringup_enables_and_validates_rolling_goals(self):
        self.assertIn(
            'GPS_LONG_RANGE_GOAL_ENABLED="${GPS_LONG_RANGE_GOAL_ENABLED:-true}"',
            self.script,
        )
        self.assertIn(
            'GPS_LONG_RANGE_LOOKAHEAD_DISTANCE="${GPS_LONG_RANGE_LOOKAHEAD_DISTANCE:-15.0}"',
            self.script,
        )
        self.assertIn(
            'GPS_LONG_RANGE_ADVANCE_DISTANCE="${GPS_LONG_RANGE_ADVANCE_DISTANCE:-5.0}"',
            self.script,
        )
        self.assertIn(
            'start_launch "GPS long-range goal manager" gps_module gps_long_range_goal.launch',
            self.script,
        )
        self.assertIn(
            'check_single_topic_subscriber "/gps/goal_fix" "/gps_long_range_goal_manager"',
            self.script,
        )
        self.assertIn(
            'check_topic_route "/move_base/goal" "/gps_long_range_goal_manager" "/move_base"',
            self.script,
        )
        self.assertIn(
            "advance < lookahead",
            self.script,
        )
        self.assertIn(
            "lookahead <= maximum",
            self.script,
        )

    def test_clean_start_stops_old_rolling_goal_manager(self):
        self.assertIn("    /gps_long_range_goal_manager\n", self.script)


class BringupCommandRouteSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.script = BRINGUP_SCRIPT.read_text(encoding="utf-8")

    def test_clean_start_stops_an_old_goal_speed_limiter(self):
        self.assertIn("    /gps_goal_speed_limiter\n", self.script)

    def test_route_check_requires_exactly_one_command_publisher(self):
        self.assertIn("publisher_count == 1", self.script)
        self.assertIn("Expected exactly one publisher", self.script)

    def test_embedded_rviz_can_be_selected_without_changing_legacy_default(self):
        self.assertIn('NAV_START_RVIZ="${NAV_START_RVIZ:-true}"', self.script)
        self.assertIn('start_rviz:="$NAV_START_RVIZ"', self.script)

        root = ElementTree.parse(str(NAVIGATION_LAUNCH)).getroot()
        arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in root.findall("arg")
        }
        self.assertEqual(arguments["start_rviz"], "true")

    def test_fast_lio_prefers_system_libusb_without_changing_other_launches(self):
        self.assertIn(
            'DEFAULT_FAST_LIO_SYSTEM_LIBRARY_DIR="/lib/aarch64-linux-gnu"',
            self.script,
        )
        self.assertIn(
            'DEFAULT_FAST_LIO_SYSTEM_LIBRARY_DIR="/lib/x86_64-linux-gnu"',
            self.script,
        )
        self.assertIn(
            'FAST_LIO_SYSTEM_LIBRARY_DIR="${FAST_LIO_SYSTEM_LIBRARY_DIR:-$DEFAULT_FAST_LIO_SYSTEM_LIBRARY_DIR}"',
            self.script,
        )
        self.assertIn(
            'env "LD_LIBRARY_PATH=$fast_lio_library_path"',
            self.script,
        )
        self.assertIn(
            'start_fast_lio_launch "FAST_LIO localization"',
            self.script,
        )
        self.assertIn(
            'start_fast_lio_launch "FAST_LIO point cloud registration"',
            self.script,
        )
        self.assertNotIn(
            'start_launch "FAST_LIO localization"',
            self.script,
        )
        self.assertNotIn(
            'start_launch "FAST_LIO point cloud registration"',
            self.script,
        )

    def test_can_one_shot_check_uses_its_exit_status_without_required_shutdown(self):
        self.assertIn(
            'rosrun robot_diagnostics check_can.py _port:="$CAN_PORT" _require_write:=true',
            self.script,
        )
        self.assertNotIn(
            "roslaunch robot_diagnostics check_can.launch",
            self.script,
        )

    def test_gps_odom_wait_allows_strict_heading_time_to_reach_fixed(self):
        self.assertIn(
            'GPS_ODOM_STARTUP_TIMEOUT="${GPS_ODOM_STARTUP_TIMEOUT:-120.0}"',
            self.script,
        )
        self.assertIn(
            'local timeout="${4:-15.0}"',
            self.script,
        )
        self.assertIn(
            'check_odom "/gps/odom" "camera_init" "base_link" "$GPS_ODOM_STARTUP_TIMEOUT"',
            self.script,
        )
        self.assertIn(
            "NARROW_FLOAT is intentionally rejected",
            self.script,
        )

    def test_heading_jump_guard_defaults_off_for_all_gps_profiles(self):
        self.assertNotIn(
            'GPS_HEADING_JUMP_GUARD_ENABLED="${GPS_HEADING_JUMP_GUARD_ENABLED:-true}"',
            self.script,
        )
        self.assertGreaterEqual(
            self.script.count(
                'GPS_HEADING_JUMP_GUARD_ENABLED="${GPS_HEADING_JUMP_GUARD_ENABLED:-false}"'
            ),
            2,
        )
        self.assertIn(
            'heading_jump_guard_enabled:="$GPS_HEADING_JUMP_GUARD_ENABLED"',
            self.script,
        )
        self.assertIn(
            'heading_recovery_samples:="$GPS_HEADING_RECOVERY_SAMPLES"',
            self.script,
        )


class GpsFodModeIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.script = BRINGUP_SCRIPT.read_text(encoding="utf-8")
        cls.integrated_root = ElementTree.parse(
            str(FOD_INTEGRATED_LAUNCH)
        ).getroot()
        cls.navigation_root = ElementTree.parse(
            str(NAVIGATION_LAUNCH)
        ).getroot()

    def test_navigation_output_can_be_routed_through_the_mode_manager(self):
        arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in self.navigation_root.findall("arg")
        }
        self.assertEqual(arguments["output_cmd_vel_topic"], "/cmd_vel")
        self.assertIn(
            "arg('output_cmd_vel_topic')",
            arguments["cmd_vel_topic"],
        )
        limiter = next(
            node
            for node in self.navigation_root.findall("node")
            if node.attrib.get("name") == "gps_goal_speed_limiter"
        )
        parameters = {
            element.attrib["name"]: element.attrib.get("value")
            for element in limiter.findall("param")
        }
        self.assertEqual(
            parameters["output_cmd_topic"],
            "$(arg output_cmd_vel_topic)",
        )

    def test_integrated_launch_has_one_external_chassis_publisher(self):
        include = self.integrated_root.find("include")
        include_args = {
            element.attrib["name"]: element.attrib.get("value")
            for element in include.findall("arg")
        }
        self.assertEqual(include_args["cmd_vel_topic"], "$(arg fod_cmd_topic)")
        self.assertEqual(
            include_args["expected_cmd_vel_subscriber_node"],
            "/fod_navigation_mode",
        )

        manager = next(
            node
            for node in self.integrated_root.findall("node")
            if node.attrib.get("name") == "fod_navigation_mode"
        )
        parameters = {
            element.attrib["name"]: element.attrib.get("value")
            for element in manager.findall("param")
        }
        self.assertEqual(parameters["gps_cmd_topic"], "$(arg gps_cmd_topic)")
        self.assertEqual(parameters["fod_cmd_topic"], "$(arg fod_cmd_topic)")
        self.assertEqual(
            parameters["output_cmd_topic"],
            "$(arg output_cmd_topic)",
        )

    def test_gps_bringup_defaults_to_safe_fod_standby_routes(self):
        self.assertIn(
            'FOD_RECOVERY_STANDBY_ENABLED="${FOD_RECOVERY_STANDBY_ENABLED:-true}"',
            self.script,
        )
        self.assertIn(
            'GPS_OUTPUT_CMD_VEL_TOPIC="/cmd_vel_gps"',
            self.script,
        )
        self.assertIn(
            'output_cmd_vel_topic:="$GPS_OUTPUT_CMD_VEL_TOPIC"',
            self.script,
        )
        self.assertIn(
            'check_cmd_vel_route "/cmd_vel_gps" "/gps_goal_speed_limiter" "/fod_navigation_mode"',
            self.script,
        )
        self.assertIn(
            'check_cmd_vel_route "/cmd_vel_fod" "/fod_visual_servo" "/fod_navigation_mode"',
            self.script,
        )
        self.assertIn(
            'check_cmd_vel_route "/cmd_vel" "/fod_navigation_mode" "/m2_driver"',
            self.script,
        )

    def test_retained_route_pause_and_operator_command_are_wired(self):
        long_range_root = ElementTree.parse(str(LONG_RANGE_GOAL_LAUNCH)).getroot()
        long_range_parameters = {
            element.attrib["name"]: element.attrib.get("value")
            for element in long_range_root.find("node").findall("param")
        }
        self.assertEqual(
            long_range_parameters["pause_service"],
            "$(arg pause_service)",
        )
        cli = FOD_MODE_SCRIPT.read_text(encoding="utf-8")
        self.assertIn(
            'MODE_SERVICE="/fod_navigation_mode/set_fod_enabled"',
            cli,
        )
        self.assertNotIn(
            'rosservice call /fod_visual_servo/set_enabled',
            cli,
        )

    def test_standalone_visual_launch_keeps_direct_driver_default(self):
        root = ElementTree.parse(str(FOD_VISUAL_LAUNCH)).getroot()
        arguments = {
            element.attrib["name"]: element.attrib.get("default")
            for element in root.findall("arg")
        }
        self.assertEqual(arguments["cmd_vel_topic"], "/cmd_vel")
        self.assertEqual(
            arguments["expected_cmd_vel_subscriber_node"],
            "/m2_driver",
        )


if __name__ == "__main__":
    unittest.main()
