#!/usr/bin/env python3

import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = Path(os.environ.get(
    "AUTOLABOR_SOURCE_WORKSPACE", "/home/slam/robot_j6m_ws"
)).resolve()


class CoverageContractTest(unittest.TestCase):
    def test_explicit_planner_modes_and_hybrid_precompute_interfaces(self):
        enforced = (
            PACKAGE_ROOT / "msg" / "EnforcedPath.msg"
        ).read_text(encoding="utf-8")
        for contract in (
            "uint8 MODE_ORDINARY_NAVFN=0",
            "uint8 MODE_COVERAGE_NAVFN=1",
            "uint8 MODE_HYBRID_TRANSIT=2",
            "uint8 MODE_ENFORCED_SWEEP=3",
            "int8 GEAR_REVERSE=-1",
            "int8 GEAR_AUTO=0",
            "int8 GEAR_FORWARD=1",
            "uint32 path_generation",
            "int8 expected_gear",
            "uint8 planner_mode",
        ):
            self.assertIn(contract, enforced)

        transition_request = (
            PACKAGE_ROOT / "msg" / "HybridTransitionRequest.msg"
        ).read_text(encoding="utf-8")
        for field in (
            "uint32 candidate_index",
            "uint32 transition_index",
            "geometry_msgs/PoseStamped start",
            "geometry_msgs/PoseStamped goal",
            "bool accept_goal_region",
        ):
            self.assertIn(field, transition_request)

        transition_result = (
            PACKAGE_ROOT / "msg" / "HybridTransitionResult.msg"
        ).read_text(encoding="utf-8")
        for field in (
            "uint8 OUTCOME_SUCCESS=0",
            "uint8 OUTCOME_TIMEOUT=1",
            "uint8 OUTCOME_NO_PATH=2",
            "nav_msgs/Path path",
            "uint64 expansions",
            "float64 estimated_time_sec",
        ):
            self.assertIn(field, transition_result)

        precompute_request, precompute_response = (
            PACKAGE_ROOT / "srv" / "PrecomputeTransitions.srv"
        ).read_text(encoding="utf-8").split("---")
        for field in (
            "string plan_id",
            "autolabor_coverage/HybridTransitionRequest[] transitions",
            "autolabor_coverage/TransitProfile transit_profile",
            "float64 total_timeout_sec",
        ):
            self.assertIn(field, precompute_request)
        self.assertIn(
            "autolabor_coverage/HybridTransitionResult[] results",
            precompute_response,
        )

        status = (
            PACKAGE_ROOT / "msg" / "CoverageStatus.msg"
        ).read_text(encoding="utf-8")
        for field in (
            "bool watchdog_motion_enabled",
            "float32 watchdog_max_linear_speed_mps",
            "float32 watchdog_max_angular_speed_rps",
        ):
            self.assertIn(field, status)

        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        for generated_interface in (
            "HybridTransitionRequest.msg",
            "HybridTransitionResult.msg",
            "PrecomputeTransitions.srv",
        ):
            self.assertIn(generated_interface, cmake)

        navigation = (
            WORKSPACE_ROOT / "src" / "scripts" / "robot_bringup" /
            "launch" / "navigation_j6m.launch"
        ).read_text(encoding="utf-8")
        for parameter in (
            "hybrid_use_obstacle_heuristic",
            "hybrid_use_nonholonomic_heuristic",
            "hybrid_use_analytic_expansion",
            "hybrid_analytic_improvement_timeout",
            "hybrid_analytic_gearchange_improvement_timeout",
            "hybrid_analytic_connector_improvement_timeout",
            "hybrid_analytic_expansion_interval",
            "hybrid_cache_max_deviation",
        ):
            self.assertIn(parameter, navigation)
        self.assertNotIn("hybrid_block_persistence", navigation)
        self.assertNotIn("hybrid_replan_cooldown", navigation)

    def test_batch_interfaces_and_single_plan_map_identity_are_generated(self):
        coverage_region = (
            PACKAGE_ROOT / "msg" / "CoverageRegion.msg"
        ).read_text(encoding="utf-8").splitlines()
        self.assertEqual([
            "string id",
            "string name",
            "geometry_msgs/PolygonStamped region",
        ], coverage_region)

        start_batch = (
            PACKAGE_ROOT / "srv" / "StartCoverageBatch.srv"
        ).read_text(encoding="utf-8")
        request, response = start_batch.split("---")
        for field in (
            "string client_request_id",
            "autolabor_coverage/CoverageRegion[] regions",
            "float32 operation_width_m",
            "float32 overlap_ratio",
            "bool allow_reverse_transit",
            "float64 max_speed_mps",
            "float64 reverse_speed_mps",
            "float64 max_angular_speed_rps",
            "float64 linear_accel_mps2",
            "float64 angular_accel_rps2",
            "float64 direction_change_penalty_sec",
            "float64 segment_handoff_penalty_sec",
            "float64 transit_replan_period_sec",
            "string map_digest",
        ):
            self.assertIn(field, request)
        for field in ("bool accepted", "string message", "string batch_id"):
            self.assertIn(field, response)

        cancel_request, cancel_response = (
            PACKAGE_ROOT / "srv" / "CancelCoverageBatch.srv"
        ).read_text(encoding="utf-8").split("---")
        self.assertEqual(
            ["string batch_id"], cancel_request.strip().splitlines()
        )
        self.assertEqual([
            "bool success",
            "bool cancellation_requested",
            "bool not_started",
            "string message",
            "string batch_id",
        ], cancel_response.strip().splitlines())

        owner_request, owner_response = (
            PACKAGE_ROOT / "srv" / "SetCoverageOwner.srv"
        ).read_text(encoding="utf-8").split("---")
        self.assertEqual(
            ["bool claim", "string owner_token"],
            owner_request.strip().splitlines(),
        )
        self.assertEqual([
            "bool success",
            "bool claimed",
            "string current_owner_token",
            "string message",
        ], owner_response.strip().splitlines())

        plan_request, plan_response = (
            PACKAGE_ROOT / "srv" / "PlanCoverage.srv"
        ).read_text(encoding="utf-8").split("---")
        self.assertIn("string map_digest", plan_request)
        self.assertIn("string map_digest", plan_response)
        for field in (
            "float64 max_speed_mps",
            "float64 reverse_speed_mps",
            "float64 max_angular_speed_rps",
            "float64 linear_accel_mps2",
            "float64 angular_accel_rps2",
            "float64 direction_change_penalty_sec",
            "float64 segment_handoff_penalty_sec",
            "float64 transit_replan_period_sec",
        ):
            self.assertIn(field, plan_request)
        for field in (
            "estimated_total_time_sec",
            "estimated_sweep_time_sec",
            "estimated_transit_time_sec",
            "estimated_reverse_transitions",
        ):
            self.assertIn(field, plan_response)

        status = (
            PACKAGE_ROOT / "msg" / "CoverageStatus.msg"
        ).read_text(encoding="utf-8")
        for field in (
            "map_digest", "batch_id", "batch_active",
            "batch_cancel_requested", "batch_current_index",
            "batch_total_regions", "batch_completed_regions",
            "batch_partial_regions", "batch_skipped_regions",
            "current_region_id", "current_region_name",
            "last_region_id", "last_region_name", "last_region_state",
        ):
            self.assertIn(field, status)

        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("CoverageRegion.msg", cmake)
        self.assertIn("StartCoverageBatch.srv", cmake)
        self.assertIn("CancelCoverageBatch.srv", cmake)
        self.assertIn("SetCoverageOwner.srv", cmake)
        self.assertIn("SetNavigationProfile.srv", cmake)
        self.assertIn("CoveragePlanningParameters.msg", cmake)
        self.assertIn("SetCoveragePlanningDefaults.srv", cmake)

    def test_batch_manager_owns_move_base_across_region_gaps(self):
        manager = (PACKAGE_ROOT / "scripts" / "coverage_manager.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"/coverage/start_batch"', manager)
        self.assertIn('"/coverage/cancel_batch"', manager)
        self.assertIn('"/coverage/skip_current"', manager)
        self.assertIn("self.active or self.batch_active", manager)
        self.assertIn("off, coverage_active=True", manager)
        self.assertIn("off, coverage_active=False", manager)
        self.assertIn("self.active_pub.publish(Bool(data=True))", manager)
        self.assertIn("self.batch_cancel_requested = True", manager)
        self.assertIn("self.batch_skip_requested = True", manager)
        self.assertIn("self.map_digest != self.batch_map_digest", manager)
        self.assertIn("self.batch_current_index = 0", manager)
        self.assertNotIn("cancel_all_goals(", manager)

    def test_manager_uses_atomic_navigation_owner_for_every_goal(self):
        manager = (PACKAGE_ROOT / "scripts" / "coverage_manager.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('"/navigation_pause/set_coverage_owner"', manager)
        self.assertIn("SetCoverageOwner", manager)
        self.assertIn('return "coverage-{}".format(uuid.uuid4().hex)', manager)
        self.assertIn("self._resolve_navigation_owner_claim(", manager)
        self.assertIn("self._set_navigation_owner(False, owner_token)", manager)
        self.assertIn("self.navigation_owner_releasing = True", manager)
        reclaim = manager.index(
            "owner_state, owner_detail = self._resolve_navigation_owner_claim(",
            manager.index("def _execute_segment"),
        )
        send_goal = manager.index(
            "self._send_move_base_goal_locked(goal)",
            manager.index("def _execute_segment"),
        )
        self.assertLess(reclaim, send_goal)

    def test_plugin_and_static_map_launch_are_wired(self):
        plugin = ElementTree.parse(
            str(PACKAGE_ROOT / "coverage_global_planner_plugin.xml")
        ).getroot()
        planner = plugin.find("class")
        self.assertEqual("autolabor_coverage/CoverageGlobalPlanner",
                         planner.attrib["name"])
        self.assertIn("Hybrid A*", planner.find("description").text)
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
        first_goal = manager.index("self._send_move_base_goal_locked(goal)")
        self.assertLess(first_publish, first_goal)
        synchronous_handoff = manager.index("self._set_enforced_path(enforced)")
        self.assertLess(synchronous_handoff, first_goal)
        self.assertIn("coverage_active=coverage_active", manager)
        self.assertIn("enforced_path=enforced", manager)
        self.assertIn("transit_profile=transit_profile", manager)
        self.assertIn("coverage_active=False", manager)
        self.assertIn("bool coverage_active", handoff_service)
        self.assertIn(
            "autolabor_coverage/TransitProfile transit_profile",
            handoff_service,
        )
        self.assertIn('segment["type"] == "transit"', manager)
        self.assertIn("self.allow_reverse_transit", manager)
        plugin = (PACKAGE_ROOT / "src" / "coverage_global_planner.cpp").read_text(
            encoding="utf-8")
        self.assertIn('advertiseService(', plugin)
        self.assertIn('"set_enforced_path"', plugin)
        self.assertIn("coverage_active_ = request.coverage_active", plugin)
        self.assertIn("refresh does not match the synchronously armed", plugin)
        self.assertNotIn('subscribe(\n      "/coverage/active"', plugin)
        self.assertIn("if (!active)", plugin)
        self.assertIn("MODE_ORDINARY_NAVFN", plugin)
        self.assertIn("MODE_COVERAGE_NAVFN", plugin)
        self.assertIn("MODE_HYBRID_TRANSIT", plugin)
        self.assertIn("MODE_ENFORCED_SWEEP", plugin)
        self.assertIn("KINEMATIC_HYBRID_ASTAR_TRANSIT", plugin)
        self.assertIn("hybrid_planner_.makePlan", plugin)
        self.assertIn("coverage Hybrid A* %s replan", plugin)
        self.assertIn('"1 Hz online" : "blocked/deviation"', plugin)
        self.assertIn("0.80 * replan_period", plugin)
        self.assertIn("if (cache_valid)", plugin)
        self.assertIn("retry cooldown", plugin)
        self.assertIn("snapshotCostmap", plugin)
        self.assertIn("live->getMutex()", plugin)
        self.assertIn("refusing any planner fallback", plugin)
        self.assertIn("fallback_.makePlan(start, goal, plan)", plugin)
        self.assertIn("goal yaw does not match enforced path endpoint", plugin)
        self.assertIn("world_model.footprintCost", plugin)
        self.assertIn("hybrid_cache_collision_check_horizon_", plugin)
        self.assertIn('advertise<std_msgs::Bool>', plugin)
        self.assertIn('"hybrid_path_safe"', plugin)
        self.assertIn("publishHybridPathSafety(false)", plugin)
        self.assertIn("publishHybridPathSafety(cache_valid)", plugin)
        self.assertIn("if (!hybrid_replan_every_cycle_)", plugin)
        self.assertIn("single path authority", plugin)
        self.assertIn("distance_from_start <=", plugin)
        self.assertIn("Hybrid transition made less than", manager)
        self.assertIn(
            "executing cached Hybrid A* swath transition", manager
        )
        self.assertIn("_split_hybrid_transition", manager)
        self.assertIn("max_global_plan_lookahead_dist", manager)
        self.assertIn("_precompute_hybrid_candidates", manager)
        self.assertIn("the swath was not skipped", manager)

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
        self.assertIn(
            'name="CoverageGlobalPlanner/hybrid_cache_collision_check_horizon"',
            navigation,
        )
        self.assertIn('type="double" value="3.00"', navigation)

    def test_fod_pause_does_not_reissue_a_coverage_endpoint(self):
        bridge = (WORKSPACE_ROOT / "src" / "platform" /
                  "autolabor_dual_host" / "scripts" /
                  "move_base_pause_bridge.py").read_text(encoding="utf-8")
        self.assertIn('"/coverage/active"', bridge)
        self.assertIn("not self.coverage_active", bridge)
        self.assertIn("if not (is_known_ai or is_bridge_simple)", bridge)
        self.assertIn("Coverage segments are also foreign here", bridge)
        self.assertIn("simple navigation goal rejected: coverage owns move_base", bridge)
        self.assertIn('"/navigation_goal/legacy_simple_input_disabled"', bridge)
        self.assertIn("_submit_simple_action_locked", bridge)

        navigation = (WORKSPACE_ROOT / "src" / "scripts" / "robot_bringup" /
                      "launch" / "navigation_j6m.launch").read_text(encoding="utf-8")
        dual_host_launch = (WORKSPACE_ROOT / "src" / "platform" /
                            "autolabor_dual_host" / "launch" /
                            "j6m_fastlio_navigation.launch").read_text(encoding="utf-8")
        self.assertIn('from="/move_base_simple/goal"', navigation)
        self.assertIn("$(arg move_base_simple_goal_topic)", navigation)
        self.assertIn(
            "/navigation_goal/legacy_simple_input_disabled", dual_host_launch)

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
        self.assertIn("libcoverage_hybrid_astar.so", deploy)

    def test_task_speed_is_bounded_and_applied_through_teb(self):
        start_service = (PACKAGE_ROOT / "srv" / "StartCoverage.srv").read_text(
            encoding="utf-8"
        )
        manager = (PACKAGE_ROOT / "scripts" / "coverage_manager.py").read_text(
            encoding="utf-8"
        )
        planner_plugin = (
            PACKAGE_ROOT / "src" / "coverage_global_planner.cpp"
        ).read_text(encoding="utf-8")
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "coverage.yaml").read_text(encoding="utf-8")
        )
        for field in (
            "float64 max_speed_mps",
            "bool allow_reverse_transit",
            "float64 reverse_speed_mps",
            "float64 max_angular_speed_rps",
            "float64 linear_accel_mps2",
            "float64 angular_accel_rps2",
            "float64 direction_change_penalty_sec",
            "float64 segment_handoff_penalty_sec",
            "float64 transit_replan_period_sec",
        ):
            self.assertIn(field, start_service)
        self.assertEqual(0.80, config["default_max_speed_mps"])
        self.assertEqual(1.60, config["max_speed_limit_mps"])
        self.assertEqual(0.30, config["reverse_transit_speed_mps"])
        self.assertEqual(0.60, config["default_max_angular_speed_rps"])
        self.assertEqual(1.00, config["default_linear_accel_mps2"])
        self.assertEqual(0.50, config["default_angular_accel_rps2"])
        self.assertEqual(1.00, config["default_transit_replan_period_sec"])
        self.assertEqual(2.00, config["hybrid_transit_lookahead_dist_m"])
        self.assertEqual(0.25, config["hybrid_cusp_position_tolerance_m"])
        self.assertEqual(
            100.0, config["hybrid_transit_weight_viapoint_lateral"]
        )
        self.assertEqual(
            50.0, config["hybrid_transit_weight_viapoint_heading"]
        )
        self.assertEqual(3.0, config["hybrid_no_progress_timeout_sec"])
        self.assertEqual(0.20, config["hybrid_cusp_yaw_tolerance_rad"])
        self.assertEqual(0.60, config["hybrid_cusp_max_forward_speed_mps"])
        self.assertEqual(0.30, config["hybrid_cusp_join_max_skip_m"])
        self.assertEqual(
            0.10, config["hybrid_cusp_join_chord_tolerance_rad"]
        )
        self.assertEqual(0.40, config["entry_position_tolerance_m"])
        self.assertEqual(0.436332, config["entry_yaw_tolerance_rad"])
        self.assertEqual(
            0.30, config["transition_completion_cross_track_tolerance_m"]
        )
        self.assertEqual(
            0.30, config["sweep_completion_cross_track_tolerance_m"]
        )
        self.assertEqual(
            0.30,
            config["sweep_entry_recovery_goal_position_tolerance_m"],
        )
        self.assertEqual(
            0.349066,
            config["sweep_entry_recovery_goal_yaw_tolerance_rad"],
        )
        self.assertEqual(0.60, config["transition_completion_start_gate_m"])
        self.assertEqual(0.60, config["transition_completion_max_overshoot_m"])
        self.assertEqual(3.00, config["hybrid_recovery_timeout_sec"])
        self.assertEqual(0.30, config["route_first_entry_distance_slack_m"])
        self.assertTrue(config["hierarchical_hybrid_on_demand"])
        self.assertTrue(config["direct_hybrid_to_final_goal"])
        self.assertTrue(config["entry_navfn_single_topology"])
        self.assertFalse(config["hybrid_execute_unsplit_cusps"])
        self.assertFalse(config["online_hybrid_without_precompute"])
        self.assertEqual(0.0, config["hybrid_transit_max_forward_speed_mps"])
        self.assertEqual(0.0, config["hybrid_transit_max_reverse_speed_mps"])
        self.assertEqual(0.0, config["hybrid_transit_max_angular_speed_rps"])
        self.assertIn("/coverage/hybrid_transition_path", manager)
        self.assertIn("_observe_transition_completion", manager)
        self.assertIn("_replan_remaining_hybrid_transition", manager)
        self.assertIn("check_join_after_completion", manager)
        self.assertIn("_rebase_joinable_hybrid_part", manager)
        self.assertIn("pose_acceptance = within_goal_pose", manager)
        self.assertNotIn("order_adjacent_boustrophedon", manager)
        self.assertIn("order_swaths", manager)
        self.assertIn("first_entry_connector_distance", manager)
        self.assertIn("route_first_entry_distance_slack", manager)
        self.assertIn("hybrid_execute_unsplit_cusps", manager)
        self.assertIn('"motion_direction_mode": (', manager)
        self.assertIn("request.accept_goal_region = direct_to_final", manager)
        self.assertIn(
            "requested_motion_speed > self.watchdog_max_linear_speed", manager
        )
        self.assertIn('"max_vel_x": maximum_forward_speed', manager)
        self.assertIn('"max_vel_x": configuration.get', manager)
        self.assertIn('"max_vel_theta": getattr(', manager)
        self.assertIn('"acc_lim_x": getattr(', manager)
        self.assertIn('"acc_lim_theta": getattr(', manager)
        self.assertIn("self.hybrid_cusp_position_tolerance", manager)
        self.assertIn("self.hybrid_cusp_yaw_tolerance", manager)
        self.assertIn("if hybrid_tracking and hybrid_cusp_goal", manager)
        self.assertIn("self.entry_position_tolerance", manager)
        self.assertIn("self.entry_yaw_tolerance", manager)
        self.assertIn(
            "index == nearest && nearest + 1 < poses.size()", planner_plugin
        )
        self.assertIn(
            'segment["type"] in ("entry", "transit")', manager
        )
        self.assertIn('"/coverage/set_navigation_profile"', manager)
        self.assertIn('"/coverage/set_planning_defaults"', manager)
        self.assertIn("_stage_atomic_yaml_update", manager)
        self.assertIn("_commit_staged_yaml", manager)
        self.assertIn("coverage_factory_defaults.yaml", (
            PACKAGE_ROOT / "launch" / "coverage.launch"
        ).read_text(encoding="utf-8"))
        navigation_profile = (
            PACKAGE_ROOT / "srv" / "SetNavigationProfile.srv"
        ).read_text(encoding="utf-8")
        for field in (
            "bool allow_reverse",
            "float64 max_forward_speed_mps",
            "float64 max_reverse_speed_mps",
            "float64 max_angular_speed_rps",
            "float64 linear_accel_mps2",
            "float64 angular_accel_rps2",
            "float64 transit_replan_period_sec",
        ):
            self.assertIn(field, navigation_profile)
        planning_defaults = (
            PACKAGE_ROOT / "srv" / "SetCoveragePlanningDefaults.srv"
        ).read_text(encoding="utf-8")
        self.assertIn("bool restore_factory_defaults", planning_defaults)
        self.assertIn("CoveragePlanningParameters parameters", planning_defaults)
        self.assertIn("CoveragePlanningParameters effective", planning_defaults)
        planning_message = (
            PACKAGE_ROOT / "msg" / "CoveragePlanningParameters.msg"
        ).read_text(encoding="utf-8")
        for field in (
            "operation_width_m",
            "overlap_ratio",
            "allow_reverse",
            "max_forward_speed_mps",
            "direction_change_penalty_sec",
            "transit_replan_period_sec",
        ):
            self.assertIn(field, planning_message)

    def test_exact_sweeps_enable_teb_cross_track_and_heading_costs_only_temporarily(self):
        manager = (PACKAGE_ROOT / "scripts" / "coverage_manager.py").read_text(
            encoding="utf-8"
        )
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "coverage.yaml").read_text(encoding="utf-8")
        )
        teb_cfg = (
            WORKSPACE_ROOT / "src" / "navigation_arena" / "forks" /
            "navigation" / "local_planner" / "teb" / "cfg" /
            "TebLocalPlannerReconfigure.cfg"
        ).read_text(encoding="utf-8")
        teb_edge = (
            WORKSPACE_ROOT / "src" / "navigation_arena" / "forks" /
            "navigation" / "local_planner" / "teb" / "include" /
            "teb_local_planner" / "g2o_types" / "edge_via_point.h"
        ).read_text(encoding="utf-8")

        self.assertEqual(0.30, config["sweep_viapoint_separation_m"])
        self.assertEqual(50.0, config["sweep_weight_viapoint"])
        self.assertEqual(200.0, config["sweep_weight_viapoint_lateral"])
        self.assertEqual(100.0, config["sweep_weight_viapoint_heading"])
        self.assertEqual(
            1000.0, config["sweep_weight_kinematics_forward_drive"]
        )
        self.assertIs(True, config["sweep_viapoints_all_candidates"])
        self.assertIn("weight_viapoint_lateral", teb_cfg)
        self.assertIn("weight_viapoint_heading", teb_cfg)
        self.assertIn("class EdgeViaPointDirection", teb_edge)
        self.assertIn('"max_vel_x_backwards": 0.0', manager)
        self.assertIn("target = copy.deepcopy(self.original_teb)", manager)

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
        self.assertEqual(0.20, config["hybrid_transit_min_obstacle_dist_m"])
        self.assertEqual(0.45, config["hybrid_transit_inflation_dist_m"])
        self.assertEqual(8.0, config["hybrid_transit_weight_viapoint"])
        self.assertEqual(
            1.0, config["hybrid_transit_selection_viapoint_cost_scale"]
        )
        self.assertIs(False, config["hybrid_transit_viapoints_all_candidates"])
        self.assertIn('"/m2_driver/chassis_parameter"', manager)
        self.assertIn("required_steering + self.steering_angle_margin", manager)
        self.assertIn("TEB minimum turning radius is below", manager)
        self.assertIn("treat_unknown_as_obstacle", manager)
        self.assertIn("CoverageGlobalPlanner_navfn/allow_unknown", manager)
        self.assertIn("hybrid_minimum_turning_radius", manager)
        self.assertIn('"allow_init_with_backwards_motion": backwards > 0.0',
                      manager)

    def test_hybrid_transit_profile_and_launch_contract(self):
        profile = (
            PACKAGE_ROOT / "msg" / "TransitProfile.msg"
        ).read_text(encoding="utf-8").splitlines()
        self.assertEqual([
            "bool allow_reverse",
            "float64 max_forward_speed_mps",
            "float64 max_reverse_speed_mps",
            "float64 max_angular_speed_rps",
            "float64 linear_accel_mps2",
            "float64 angular_accel_rps2",
            "float64 direction_change_penalty_sec",
            "float64 goal_position_tolerance_m",
            "float64 goal_yaw_tolerance_rad",
            "float64 replan_period_sec",
        ], profile)
        navigation = (
            WORKSPACE_ROOT / "src" / "scripts" / "robot_bringup" /
            "launch" / "navigation_j6m.launch"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'name="CoverageGlobalPlanner/hybrid_minimum_turning_radius"',
            navigation,
        )
        self.assertIn(
            'name="CoverageGlobalPlanner/hybrid_planning_timeout"',
            navigation,
        )
        self.assertIn(
            'name="CoverageGlobalPlanner_navfn/allow_unknown"', navigation
        )

    def test_status_reports_route_and_verified_chassis_constraints(self):
        status = (PACKAGE_ROOT / "msg" / "CoverageStatus.msg").read_text(
            encoding="utf-8"
        )
        for field in (
            "operation_width_m",
            "lane_spacing_m",
            "minimum_turning_radius_m",
            "allow_reverse_transit",
            "max_forward_speed_mps",
            "max_reverse_speed_mps",
            "max_angular_speed_rps",
            "linear_accel_mps2",
            "angular_accel_rps2",
            "direction_change_penalty_sec",
            "segment_handoff_penalty_sec",
            "transit_replan_period_sec",
            "estimated_total_time_sec",
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
        self.assertIn("Use four safety", m2_driver)
        self.assertIn("status_query_rate_limit_hz", m2_driver)
        self.assertIn("m2_status_query_rate_hz", m2_driver)
        self.assertIn("srv.request.requests.push_back(next_req)", m2_driver)
        self.assertNotIn("for (std::size_t index = 0;", m2_driver)


if __name__ == "__main__":
    unittest.main()
