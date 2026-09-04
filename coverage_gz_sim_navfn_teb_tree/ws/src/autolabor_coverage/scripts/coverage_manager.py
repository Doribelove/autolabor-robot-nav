#!/usr/bin/env python3
"""Plan and execute static-map coverage tasks through the existing move_base chain."""

import copy
import hashlib
import json
import math
import os
import re
import tempfile
import threading
import time
import uuid

import actionlib
from actionlib_msgs.msg import GoalStatus
from autolabor_coverage.coverage_geometry import (
    CoveragePlanner,
    CoverageTimeParameters,
    GridMap,
    Point,
    estimate_transition_time,
    occupancy_grid_digest,
    order_swaths,
    rasterize_swept_cells,
    sample_path,
)
from autolabor_coverage.msg import (
    CoveragePlanningParameters,
    CoverageStatus,
    EnforcedPath,
    HybridTransitionRequest,
    HybridTransitionResult,
    TransitProfile,
)
from autolabor_coverage.srv import (
    CancelCoverageBatch,
    CancelCoverageBatchResponse,
    PlanCoverage,
    PlanCoverageResponse,
    PrecomputeTransitions,
    SetEnforcedPath,
    SetCoveragePlanningDefaults,
    SetCoveragePlanningDefaultsRequest,
    SetCoveragePlanningDefaultsResponse,
    SetNavigationProfile,
    SetNavigationProfileResponse,
    SetCoverageOwner,
    StartCoverage,
    StartCoverageBatch,
    StartCoverageBatchResponse,
    StartCoverageResponse,
)
from autolabor_canbus_driver.msg import ChassisMonitorInfo, ChassisStatusInfo
from autolabor_canbus_driver.srv import ChassisParameterServer
from geometry_msgs.msg import Point as RosPoint
from geometry_msgs.msg import PolygonStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
import yaml


TERMINAL_STATES = {"COMPLETED", "COMPLETED_PARTIAL", "CANCELED", "FAILED"}
REGION_TERMINAL_STATES = TERMINAL_STATES | {"SKIPPED"}
TRUSTED_MOVE_BASE_TERMINAL_STATES = {
    GoalStatus.PREEMPTED,
    GoalStatus.SUCCEEDED,
    GoalStatus.ABORTED,
    GoalStatus.REJECTED,
    GoalStatus.RECALLED,
}
_EXPECTED_HANDLE_UNSET = object()


PLANNING_DEFAULT_YAML_FIELDS = (
    ("operation_width_m", "operation_width_m", "{:.2f}"),
    ("overlap_ratio", "overlap_ratio", "{:.4f}"),
    ("allow_reverse", "allow_reverse_transit", None),
    ("max_forward_speed_mps", "default_max_speed_mps", "{:.2f}"),
    ("max_reverse_speed_mps", "reverse_transit_speed_mps", "{:.2f}"),
    ("max_angular_speed_rps", "default_max_angular_speed_rps", "{:.2f}"),
    ("linear_accel_mps2", "default_linear_accel_mps2", "{:.2f}"),
    ("angular_accel_rps2", "default_angular_accel_rps2", "{:.2f}"),
    ("direction_change_penalty_sec", "direction_change_penalty_sec", "{:.2f}"),
    ("segment_handoff_penalty_sec", "segment_handoff_penalty_sec", "{:.2f}"),
    (
        "transit_replan_period_sec",
        "default_transit_replan_period_sec",
        "{:.2f}",
    ),
)


def _planning_values_to_yaml(parameters):
    """Return exact coverage.yaml replacements for one validated parameter set."""
    replacements = {}
    for attribute, yaml_key, number_format in PLANNING_DEFAULT_YAML_FIELDS:
        value = getattr(parameters, attribute)
        if number_format is None:
            replacements[yaml_key] = "true" if bool(value) else "false"
        else:
            replacements[yaml_key] = number_format.format(float(value))
    return replacements


def _rewrite_flat_yaml_values(source, replacements):
    """Replace unique top-level scalar keys without losing comments or ordering."""
    loaded = yaml.safe_load(source)
    if not isinstance(loaded, dict):
        raise ValueError("coverage configuration must be a YAML mapping")
    missing = [key for key in replacements if key not in loaded]
    if missing:
        raise ValueError(
            "coverage configuration is missing keys: {}".format(
                ", ".join(sorted(missing))
            )
        )

    lines = source.splitlines(True)
    for key, replacement in replacements.items():
        pattern = re.compile(
            r"^(" + re.escape(key) + r"[ \t]*:[ \t]*)([^#\r\n]*?)"
            r"([ \t]*(?:#.*)?)(\r?\n)?$"
        )
        matches = []
        for index, line in enumerate(lines):
            match = pattern.match(line)
            if match:
                matches.append((index, match))
        if len(matches) != 1:
            raise ValueError(
                "coverage configuration key {} must occur exactly once at top level"
                .format(key)
            )
        index, match = matches[0]
        suffix = match.group(3) or ""
        newline = match.group(4) or ""
        lines[index] = "{}{}{}{}".format(
            match.group(1), replacement, suffix, newline
        )

    rewritten = "".join(lines)
    # Parse the result before staging it, so an invalid edit never reaches disk.
    yaml.safe_load(rewritten)
    return rewritten


def _stage_atomic_yaml_update(target_path, parameters):
    """Write and fsync a replacement beside target; caller decides commit/rollback."""
    target_path = os.path.abspath(target_path)
    with open(target_path, "r", encoding="utf-8") as stream:
        source = stream.read()
    stat_result = os.stat(target_path)
    rewritten = _rewrite_flat_yaml_values(
        source, _planning_values_to_yaml(parameters)
    )
    descriptor, staged_path = tempfile.mkstemp(
        prefix=".coverage.yaml.", dir=os.path.dirname(target_path), text=True
    )
    try:
        os.fchmod(descriptor, stat_result.st_mode & 0o7777)
        if os.geteuid() == 0:
            os.fchown(descriptor, stat_result.st_uid, stat_result.st_gid)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            descriptor = -1
            stream.write(rewritten)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        if descriptor >= 0:
            os.close(descriptor)
        try:
            os.unlink(staged_path)
        except OSError:
            pass
        raise
    return staged_path, target_path


def _commit_staged_yaml(staged_path, target_path):
    os.replace(staged_path, target_path)
    directory_descriptor = os.open(os.path.dirname(target_path), os.O_RDONLY)
    try:
        os.fsync(directory_descriptor)
    finally:
        os.close(directory_descriptor)


class CoverageManager:
    def __init__(self):
        self.lock = threading.RLock()
        self.config_file = os.path.abspath(str(rospy.get_param("~config_file", "")))
        self.factory_defaults_file = os.path.abspath(
            str(rospy.get_param("~factory_defaults_file", ""))
        )
        if not os.path.isfile(self.config_file):
            raise ValueError("coverage config_file is not a regular file")
        if not os.path.isfile(self.factory_defaults_file):
            raise ValueError("coverage factory_defaults_file is not a regular file")
        self.grid = None
        self.map_message = None
        self.map_digest = ""
        self.plan = None
        self.plan_id = ""
        self.hybrid_path_generation = 0
        self.plan_map_digest = ""
        self.region = PolygonStamped()
        self.operation_width = float(rospy.get_param("~operation_width_m", 1.00))
        self.overlap_ratio = float(rospy.get_param("~overlap_ratio", 0.15))
        self.allow_reverse_transit = self._strict_bool("~allow_reverse_transit", True)
        self.reverse_transit_speed = float(
            rospy.get_param("~reverse_transit_speed_mps", 0.30)
        )
        self.default_max_speed = float(
            rospy.get_param("~default_max_speed_mps", 0.80)
        )
        self.max_speed_limit = float(
            rospy.get_param("~max_speed_limit_mps", 1.60)
        )
        self.max_reverse_speed_limit = float(
            rospy.get_param("~max_reverse_speed_limit_mps", 0.80)
        )
        self.default_max_angular_speed = float(
            rospy.get_param("~default_max_angular_speed_rps", 0.60)
        )
        self.max_angular_speed_limit = float(
            rospy.get_param("~max_angular_speed_limit_rps", 1.00)
        )
        self.default_linear_accel = float(
            rospy.get_param("~default_linear_accel_mps2", 1.00)
        )
        self.linear_accel_limit = float(
            rospy.get_param("~linear_accel_limit_mps2", 2.00)
        )
        self.default_angular_accel = float(
            rospy.get_param("~default_angular_accel_rps2", 0.50)
        )
        self.angular_accel_limit = float(
            rospy.get_param("~angular_accel_limit_rps2", 1.00)
        )
        self.direction_change_penalty = float(
            rospy.get_param("~direction_change_penalty_sec", 0.50)
        )
        self.segment_handoff_penalty = float(
            rospy.get_param("~segment_handoff_penalty_sec", 0.50)
        )
        self.default_transit_replan_period = float(
            rospy.get_param("~default_transit_replan_period_sec", 1.00)
        )
        self.time_search_beam_width = int(
            rospy.get_param("~time_search_beam_width", 128)
        )
        self.route_first_entry_distance_slack = float(
            rospy.get_param("~route_first_entry_distance_slack_m", 0.30)
        )
        if not 0.10 <= self.default_max_speed <= self.max_speed_limit <= 1.70:
            raise ValueError(
                "coverage speed limits must satisfy 0.10 <= default <= limit <= 1.70"
            )
        if not 0.05 <= self.reverse_transit_speed <= self.max_reverse_speed_limit <= 1.70:
            raise ValueError(
                "coverage reverse limits must satisfy 0.05 <= default <= limit <= 1.70"
            )
        if not 0.10 <= self.default_max_angular_speed <= self.max_angular_speed_limit <= 1.50:
            raise ValueError(
                "coverage angular-speed limits must satisfy 0.10 <= default <= limit <= 1.50"
            )
        if not 0.10 <= self.default_linear_accel <= self.linear_accel_limit <= 3.00:
            raise ValueError(
                "coverage linear-acceleration limits must satisfy 0.10 <= default <= limit <= 3.00"
            )
        if not 0.10 <= self.default_angular_accel <= self.angular_accel_limit <= 2.00:
            raise ValueError(
                "coverage angular-acceleration limits must satisfy 0.10 <= default <= limit <= 2.00"
            )
        if not 8 <= self.time_search_beam_width <= 4096:
            raise ValueError("coverage time-search beam width must be in [8, 4096]")
        if not 0.0 <= self.route_first_entry_distance_slack <= 2.0:
            raise ValueError(
                "coverage first-entry distance slack must be in [0, 2] m"
            )
        self.default_time_parameters = CoverageTimeParameters(
            max_forward_speed_mps=self.default_max_speed,
            max_reverse_speed_mps=self.reverse_transit_speed,
            max_angular_speed_rps=self.default_max_angular_speed,
            linear_accel_mps2=self.default_linear_accel,
            angular_accel_rps2=self.default_angular_accel,
            allow_reverse=self.allow_reverse_transit,
            direction_change_penalty_sec=self.direction_change_penalty,
            segment_handoff_penalty_sec=self.segment_handoff_penalty,
            transit_replan_period_sec=self.default_transit_replan_period,
        )
        self._validate_time_parameters(self.default_time_parameters)
        self.plan_time_parameters = None
        self.task_time_parameters = self.default_time_parameters
        self._apply_time_parameters_locked(self.default_time_parameters)
        self.minimum_turning_radius = float(
            rospy.get_param("~minimum_turning_radius_m", 1.35)
        )
        self.expected_wheelbase = float(
            rospy.get_param("~expected_wheelbase_m", 0.65)
        )
        self.expected_max_steering_angle = float(
            rospy.get_param("~expected_max_steering_angle_rad", 0.488692)
        )
        self.steering_angle_margin = float(
            rospy.get_param("~steering_angle_margin_rad", math.radians(2.0))
        )
        self.chassis_wheelbase_tolerance = float(
            rospy.get_param("~chassis_wheelbase_tolerance_m", 0.02)
        )
        if not all(math.isfinite(value) and value > 0.0 for value in (
                self.minimum_turning_radius,
                self.expected_wheelbase,
                self.expected_max_steering_angle,
                self.steering_angle_margin,
                self.chassis_wheelbase_tolerance,
        )):
            raise ValueError("coverage Ackermann parameters must be finite and positive")
        configured_steering = math.atan(
            self.expected_wheelbase / self.minimum_turning_radius
        )
        if configured_steering + self.steering_angle_margin > (
                self.expected_max_steering_angle + 1.0e-6):
            raise ValueError(
                "coverage turning radius does not retain the steering margin"
            )
        self.path_sample_spacing = float(
            rospy.get_param("~path_sample_spacing_m", 0.10)
        )
        self.sweep_viapoint_separation = float(
            rospy.get_param("~sweep_viapoint_separation_m", 0.30)
        )
        self.sweep_weight_viapoint = float(
            rospy.get_param("~sweep_weight_viapoint", 50.0)
        )
        self.sweep_weight_viapoint_lateral = float(
            rospy.get_param("~sweep_weight_viapoint_lateral", 200.0)
        )
        self.sweep_weight_viapoint_heading = float(
            rospy.get_param("~sweep_weight_viapoint_heading", 100.0)
        )
        self.sweep_weight_kinematics_forward_drive = float(
            rospy.get_param("~sweep_weight_kinematics_forward_drive", 1000.0)
        )
        self.sweep_selection_viapoint_cost_scale = float(
            rospy.get_param("~sweep_selection_viapoint_cost_scale", 5.0)
        )
        self.sweep_viapoints_all_candidates = self._strict_bool(
            "~sweep_viapoints_all_candidates", True
        )
        sweep_weights = (
            self.sweep_weight_viapoint,
            self.sweep_weight_viapoint_lateral,
            self.sweep_weight_viapoint_heading,
            self.sweep_weight_kinematics_forward_drive,
        )
        if (
            not math.isfinite(self.sweep_viapoint_separation)
            or not 0.05 <= self.sweep_viapoint_separation <= 5.0
            or not all(math.isfinite(value) and 0.0 <= value <= 1000.0
                       for value in sweep_weights)
            or self.sweep_weight_kinematics_forward_drive <= 0.0
            or not math.isfinite(self.sweep_selection_viapoint_cost_scale)
            or not 0.0 <= self.sweep_selection_viapoint_cost_scale <= 100.0
            or (
                self.sweep_weight_viapoint_lateral == 0.0
                and self.sweep_weight_viapoint_heading == 0.0
            )
        ):
            raise ValueError("coverage TEB straight-sweep profile is invalid")
        self.hybrid_transit_viapoint_separation = float(rospy.get_param(
            "~hybrid_transit_viapoint_separation_m", 0.30
        ))
        self.hybrid_transit_lookahead_distance = float(rospy.get_param(
            "~hybrid_transit_lookahead_dist_m", 3.00
        ))
        self.hybrid_cusp_position_tolerance = float(rospy.get_param(
            "~hybrid_cusp_position_tolerance_m", 0.25
        ))
        self.hybrid_cusp_yaw_tolerance = float(rospy.get_param(
            "~hybrid_cusp_yaw_tolerance_rad", 0.20
        ))
        self.hybrid_cusp_max_forward_speed = float(rospy.get_param(
            "~hybrid_cusp_max_forward_speed_mps", 0.60
        ))
        self.hybrid_cusp_join_max_skip = float(rospy.get_param(
            "~hybrid_cusp_join_max_skip_m", 0.30
        ))
        self.hybrid_cusp_join_chord_tolerance = float(rospy.get_param(
            "~hybrid_cusp_join_chord_tolerance_rad", 0.10
        ))
        self.hybrid_transit_min_obstacle_dist = float(rospy.get_param(
            "~hybrid_transit_min_obstacle_dist_m", 0.20
        ))
        self.hybrid_transit_inflation_dist = float(rospy.get_param(
            "~hybrid_transit_inflation_dist_m", 0.45
        ))
        self.hybrid_transit_weight_viapoint = float(rospy.get_param(
            "~hybrid_transit_weight_viapoint", 8.0
        ))
        self.hybrid_transit_weight_viapoint_lateral = float(rospy.get_param(
            "~hybrid_transit_weight_viapoint_lateral", 100.0
        ))
        self.hybrid_transit_weight_viapoint_heading = float(rospy.get_param(
            "~hybrid_transit_weight_viapoint_heading", 50.0
        ))
        self.hybrid_transit_include_costmap_obstacles = self._strict_bool(
            "~hybrid_transit_include_costmap_obstacles", True
        )
        self.hybrid_transit_weight_kinematics_forward_drive = float(
            rospy.get_param(
                "~hybrid_transit_weight_kinematics_forward_drive", 5.0
            )
        )
        self.hybrid_transit_selection_viapoint_cost_scale = float(
            rospy.get_param(
                "~hybrid_transit_selection_viapoint_cost_scale", 1.0
            )
        )
        self.hybrid_transit_viapoints_all_candidates = self._strict_bool(
            "~hybrid_transit_viapoints_all_candidates", False
        )
        self.hybrid_transit_inner_iterations = int(rospy.get_param(
            "~hybrid_transit_inner_iterations", 5
        ))
        self.hybrid_transit_outer_iterations = int(rospy.get_param(
            "~hybrid_transit_outer_iterations", 3
        ))
        self.online_hybrid_without_precompute = self._strict_bool(
            "~online_hybrid_without_precompute", False
        )
        self.hierarchical_hybrid_on_demand = self._strict_bool(
            "~hierarchical_hybrid_on_demand", False
        )
        self.direct_hybrid_to_final_goal = self._strict_bool(
            "~direct_hybrid_to_final_goal", False
        )
        self.entry_navfn_single_topology = self._strict_bool(
            "~entry_navfn_single_topology", False
        )
        # Isolated comparison-only switch.  Keep route selection, entry
        # tolerances, speeds and sweep execution identical to the direct
        # Hybrid baseline, but represent every same-region connector as the
        # same Navfn + TEB point-to-point action used by the first swath.
        self.navfn_all_swath_transitions = self._strict_bool(
            "~navfn_all_swath_transitions", False
        )
        self.hybrid_execute_unsplit_cusps = self._strict_bool(
            "~hybrid_execute_unsplit_cusps", False
        )
        self.hybrid_transit_max_forward_speed = float(rospy.get_param(
            "~hybrid_transit_max_forward_speed_mps", 0.0
        ))
        self.hybrid_transit_max_reverse_speed = float(rospy.get_param(
            "~hybrid_transit_max_reverse_speed_mps", 0.0
        ))
        self.hybrid_transit_max_angular_speed = float(rospy.get_param(
            "~hybrid_transit_max_angular_speed_rps", 0.0
        ))
        self.hybrid_transit_linear_accel = float(rospy.get_param(
            "~hybrid_transit_linear_accel_mps2", 0.0
        ))
        self.hybrid_transit_angular_accel = float(rospy.get_param(
            "~hybrid_transit_angular_accel_rps2", 0.0
        ))
        self.hybrid_rolling_horizon = float(rospy.get_param(
            "~hybrid_rolling_horizon_m", 10.0
        ))
        self.hybrid_rolling_plan_timeout = float(rospy.get_param(
            "~hybrid_rolling_plan_timeout_sec", 3.0
        ))
        self.hybrid_rolling_max_chunks = int(rospy.get_param(
            "~hybrid_rolling_max_chunks", 64
        ))
        self.hybrid_no_progress_timeout = float(rospy.get_param(
            "~hybrid_no_progress_timeout_sec", 3.0
        ))
        self.hybrid_no_progress_distance = float(rospy.get_param(
            "~hybrid_no_progress_distance_m", 0.10
        ))
        if (
            not 0.05 <= self.hybrid_transit_viapoint_separation <= 5.0
            or not 1.0 <= self.hybrid_transit_lookahead_distance <= 8.0
            or not 0.02 <= self.hybrid_cusp_position_tolerance <= 0.30
            or not 0.05 <= self.hybrid_cusp_yaw_tolerance <= 0.40
            or not 0.10 <= self.hybrid_cusp_join_max_skip <= 0.60
            or not 0.02 <= self.hybrid_cusp_join_chord_tolerance <= 0.20
            or not 0.15 <= self.hybrid_transit_min_obstacle_dist <= 0.30
            or not (
                self.hybrid_transit_min_obstacle_dist
                <= self.hybrid_transit_inflation_dist <= 0.80
            )
            or not 0.0 <= self.hybrid_transit_weight_viapoint <= 1000.0
            or not 0.0 <= self.hybrid_transit_weight_viapoint_lateral <= 1000.0
            or not 0.0 <= self.hybrid_transit_weight_viapoint_heading <= 1000.0
            or not 0.0 <= self.hybrid_transit_weight_kinematics_forward_drive <= 1000.0
            or not 0.0 <= self.hybrid_transit_selection_viapoint_cost_scale <= 100.0
            or not 1 <= self.hybrid_transit_inner_iterations <= 20
            or not 1 <= self.hybrid_transit_outer_iterations <= 10
            or not all(math.isfinite(value) for value in (
                self.hybrid_transit_viapoint_separation,
                self.hybrid_transit_lookahead_distance,
                self.hybrid_cusp_position_tolerance,
                self.hybrid_cusp_yaw_tolerance,
                self.hybrid_cusp_max_forward_speed,
                self.hybrid_cusp_join_max_skip,
                self.hybrid_cusp_join_chord_tolerance,
                self.hybrid_transit_min_obstacle_dist,
                self.hybrid_transit_inflation_dist,
                self.hybrid_transit_weight_viapoint,
                self.hybrid_transit_weight_viapoint_lateral,
                self.hybrid_transit_weight_viapoint_heading,
                self.hybrid_transit_weight_kinematics_forward_drive,
                self.hybrid_transit_selection_viapoint_cost_scale,
                self.hybrid_no_progress_timeout,
                self.hybrid_no_progress_distance,
                self.hybrid_rolling_horizon,
                self.hybrid_rolling_plan_timeout,
            ))
            or not 2.0 <= self.hybrid_no_progress_timeout <= 60.0
            or not 0.02 <= self.hybrid_no_progress_distance <= 1.0
            or not 0.10 <= self.hybrid_cusp_max_forward_speed <= 1.70
            or not 3.0 <= self.hybrid_rolling_horizon <= 20.0
            or not 0.5 <= self.hybrid_rolling_plan_timeout <= 10.0
            or not 2 <= self.hybrid_rolling_max_chunks <= 256
            or (
                self.online_hybrid_without_precompute
                and self.hierarchical_hybrid_on_demand
            )
            or (
                self.direct_hybrid_to_final_goal
                and not self.hierarchical_hybrid_on_demand
            )
            # Hybrid motion is executed by TEB as constant-gear actions.  A
            # complete signed path cannot be passed as one action because
            # nav_msgs/Path carries no gear at each edge and TEB must stop
            # physically before every direction change.
            or self.hybrid_execute_unsplit_cusps
            or not all(
                math.isfinite(value) and value >= 0.0
                for value in (
                    self.hybrid_transit_max_forward_speed,
                    self.hybrid_transit_max_reverse_speed,
                    self.hybrid_transit_max_angular_speed,
                    self.hybrid_transit_linear_accel,
                    self.hybrid_transit_angular_accel,
                )
            )
            or self.hybrid_transit_max_forward_speed > self.max_speed_limit
            or self.hybrid_transit_max_reverse_speed > self.max_reverse_speed_limit
            or self.hybrid_transit_max_angular_speed > self.max_angular_speed_limit
            or self.hybrid_transit_linear_accel > self.linear_accel_limit
            or self.hybrid_transit_angular_accel > self.angular_accel_limit
        ):
            raise ValueError("coverage Hybrid A* TEB transit profile is invalid")
        self.obstacle_wait_sec = float(rospy.get_param("~obstacle_wait_sec", 10.0))
        self.segment_retry_count = int(rospy.get_param("~segment_retry_count", 3))
        self.final_retry_count = int(rospy.get_param("~final_retry_count", 1))
        self.localization_fresh_sec = float(
            rospy.get_param("~localization_fresh_sec", 0.75)
        )
        self.watchdog_fresh_sec = float(rospy.get_param("~watchdog_fresh_sec", 1.0))
        self.avoidance_scan_fresh_sec = float(
            rospy.get_param("~avoidance_scan_fresh_sec", 0.5)
        )
        self.dual_lidar_fresh_sec = float(
            rospy.get_param("~dual_lidar_fresh_sec", 1.0)
        )
        self.chassis_status_fresh_sec = float(
            rospy.get_param("~chassis_status_fresh_sec", 3.0)
        )
        self.chassis_odom_fresh_sec = float(
            rospy.get_param("~chassis_odom_fresh_sec", 1.0)
        )
        self.chassis_monitor_fault_latch_sec = float(
            rospy.get_param("~chassis_monitor_fault_latch_sec", 3.0)
        )
        self.avoidance_scan_future_tolerance_sec = float(
            rospy.get_param("~avoidance_scan_future_tolerance_sec", 0.2)
        )
        self.avoidance_scan_frame = str(
            rospy.get_param("~avoidance_scan_frame", "base_link")
        ).lstrip("/")
        self.require_dual_lidar = self._strict_bool(
            "~require_dual_lidar_for_coverage", True
        )
        if not all(math.isfinite(value) and value > 0.0 for value in (
                self.avoidance_scan_fresh_sec,
                self.dual_lidar_fresh_sec,
                self.chassis_status_fresh_sec,
                self.chassis_odom_fresh_sec,
                self.chassis_monitor_fault_latch_sec,
                self.avoidance_scan_future_tolerance_sec,
        )):
            raise ValueError(
                "coverage sensing and chassis-status timeouts must be finite and positive"
            )
        if not self.avoidance_scan_frame:
            raise ValueError("coverage obstacle scan frame must not be empty")
        self.goal_timeout_base_sec = float(
            rospy.get_param("~goal_timeout_base_sec", 20.0)
        )
        self.goal_timeout_per_meter_sec = float(
            rospy.get_param("~goal_timeout_per_meter_sec", 20.0)
        )
        self.move_base_terminal_timeout_sec = float(
            rospy.get_param("~move_base_terminal_timeout_sec", 2.0)
        )
        if (not math.isfinite(self.move_base_terminal_timeout_sec) or
                self.move_base_terminal_timeout_sec <= 0.0):
            raise ValueError(
                "move_base terminal confirmation timeout must be finite and positive"
            )
        self.enforced_path_service_name = str(rospy.get_param(
            "~enforced_path_service",
            "/move_base/CoverageGlobalPlanner/set_enforced_path",
        ))
        self.hybrid_precompute_service_name = str(rospy.get_param(
            "~hybrid_precompute_service",
            "/move_base/CoverageGlobalPlanner/precompute_transitions",
        ))
        self.hybrid_precompute_timeout_sec = float(rospy.get_param(
            "~hybrid_precompute_timeout_sec", 60.0
        ))
        self.hybrid_precompute_retry_sec = float(rospy.get_param(
            "~hybrid_precompute_retry_sec", 5.0
        ))
        self.navigation_owner_service_name = str(rospy.get_param(
            "~navigation_owner_service",
            "/navigation_pause/set_coverage_owner",
        ))
        self.entry_position_tolerance = float(rospy.get_param(
            "~entry_position_tolerance_m", 0.40
        ))
        self.entry_yaw_tolerance = float(rospy.get_param(
            "~entry_yaw_tolerance_rad", 0.436332
        ))
        # move_base point tolerances remain the normal success path, but an
        # Ackermann chassis can cross a cusp or swath entrance before its
        # controller reports goal success.  Track directed path progress and
        # cancel the exact action at the boundary, then require physical zero
        # speed before arming the next gear or the sweep.
        self.transition_completion_start_gate = float(rospy.get_param(
            "~transition_completion_start_gate_m", 0.60
        ))
        self.transition_completion_cross_track_tolerance = float(
            rospy.get_param(
                "~transition_completion_cross_track_tolerance_m", 0.30
            )
        )
        self.transition_completion_pass_margin = float(rospy.get_param(
            "~transition_completion_pass_margin_m", 0.02
        ))
        self.transition_completion_min_progress_ratio = float(rospy.get_param(
            "~transition_completion_min_progress_ratio", 0.85
        ))
        self.transition_completion_max_sample_step = float(rospy.get_param(
            "~transition_completion_max_sample_step_m", 0.50
        ))
        self.transition_completion_max_overshoot = float(rospy.get_param(
            "~transition_completion_max_overshoot_m", 0.60
        ))
        self.transition_tracking_deviation_tolerance = float(rospy.get_param(
            "~transition_tracking_deviation_tolerance_m", 0.35
        ))
        self.transition_tracking_heading_tolerance = float(rospy.get_param(
            "~transition_tracking_heading_tolerance_rad", 0.40
        ))
        self.transition_tracking_deviation_confirmation_samples = int(
            rospy.get_param(
                "~transition_tracking_deviation_confirmation_samples", 3
            )
        )
        self.transition_completion_confirmation_samples = int(rospy.get_param(
            "~transition_completion_confirmation_samples", 2
        ))
        self.transition_completion_poll_period = float(rospy.get_param(
            "~transition_completion_poll_period_sec", 0.05
        ))
        self.transition_completion_stop_speed = float(rospy.get_param(
            "~transition_completion_stop_speed_mps", 0.08
        ))
        self.transition_completion_stop_timeout = float(rospy.get_param(
            "~transition_completion_stop_timeout_sec", 3.00
        ))
        self.hybrid_recovery_timeout_sec = float(rospy.get_param(
            "~hybrid_recovery_timeout_sec", 3.00
        ))
        # A sweep may legitimately pass its endpoint before TEB can satisfy a
        # point-goal tolerance (the sweep itself never permits reverse).  Do
        # not solve that by merely enlarging the point tolerance: completion
        # is armed near the line entrance and then requires continuous,
        # on-line progress through the directed exit plane.
        self.sweep_completion_start_gate = float(rospy.get_param(
            "~sweep_completion_start_gate_m", 0.45
        ))
        self.sweep_completion_cross_track_tolerance = float(rospy.get_param(
            "~sweep_completion_cross_track_tolerance_m", 0.30
        ))
        self.sweep_completion_heading_tolerance = float(rospy.get_param(
            "~sweep_completion_heading_tolerance_rad", 0.35
        ))
        self.sweep_completion_pass_margin = float(rospy.get_param(
            "~sweep_completion_pass_margin_m", 0.02
        ))
        self.sweep_completion_min_progress_ratio = float(rospy.get_param(
            "~sweep_completion_min_progress_ratio", 0.90
        ))
        self.sweep_completion_max_sample_step = float(rospy.get_param(
            "~sweep_completion_max_sample_step_m", 0.50
        ))
        self.sweep_completion_confirmation_samples = int(rospy.get_param(
            "~sweep_completion_confirmation_samples", 2
        ))
        self.sweep_completion_poll_period = float(rospy.get_param(
            "~sweep_completion_poll_period_sec", 0.05
        ))
        self.sweep_completion_max_linear_speed = float(rospy.get_param(
            "~sweep_completion_max_linear_speed_mps", 0.08
        ))
        # A localization/control disturbance can occur in the small hand-off
        # window after the connector has been accepted but before the sweep
        # controller has acquired the line.  TEB is intentionally forbidden
        # to reverse on a sweep, so a large lateral/yaw error is not an
        # orientation-only point-goal problem it can safely repair.  Detect
        # that condition near the line entrance and hand the same entry pose
        # back to the rolling Hybrid/cusp pipeline instead of letting a local
        # optimizer circle indefinitely.
        self.sweep_entry_recovery_enabled = self._strict_bool(
            "~sweep_entry_recovery_enabled", True
        )
        self.sweep_entry_recovery_progress_limit = float(rospy.get_param(
            "~sweep_entry_recovery_progress_limit_m", 0.75
        ))
        self.sweep_entry_recovery_confirmation_samples = int(rospy.get_param(
            "~sweep_entry_recovery_confirmation_samples", 3
        ))
        # Entry alignment is not an ordinary long connector.  Its planning
        # cost treats reverse travel as comparable to forward travel so the
        # search does not prefer a large forward loop merely because the
        # chassis executes reverse more slowly.  These are cost-equivalent
        # values only; physical reverse commands retain reverse_transit_speed.
        self.sweep_entry_recovery_reverse_cost_speed = float(rospy.get_param(
            "~sweep_entry_recovery_reverse_cost_speed_mps", 0.80
        ))
        self.sweep_entry_recovery_forward_cost_speed = float(rospy.get_param(
            "~sweep_entry_recovery_forward_cost_speed_mps", 0.20
        ))
        self.sweep_entry_recovery_direction_change_penalty = float(
            rospy.get_param(
                "~sweep_entry_recovery_direction_change_penalty_sec", 0.15
            )
        )
        self.sweep_entry_recovery_max_path_length = float(rospy.get_param(
            "~sweep_entry_recovery_max_path_length_m", 4.00
        ))
        self.sweep_entry_recovery_goal_position_tolerance = float(
            rospy.get_param(
                "~sweep_entry_recovery_goal_position_tolerance_m", 0.30
            )
        )
        self.sweep_entry_recovery_goal_yaw_tolerance = float(rospy.get_param(
            "~sweep_entry_recovery_goal_yaw_tolerance_rad", 0.349066
        ))
        self.sweep_entry_alignment_radius = float(rospy.get_param(
            "~sweep_entry_alignment_radius_m", 2.00
        ))
        sweep_completion_values = (
            self.sweep_completion_start_gate,
            self.sweep_completion_cross_track_tolerance,
            self.sweep_completion_heading_tolerance,
            self.sweep_completion_pass_margin,
            self.sweep_completion_min_progress_ratio,
            self.sweep_completion_max_sample_step,
            self.sweep_completion_poll_period,
            self.sweep_completion_max_linear_speed,
            self.sweep_entry_recovery_progress_limit,
            self.sweep_entry_recovery_reverse_cost_speed,
            self.sweep_entry_recovery_forward_cost_speed,
            self.sweep_entry_recovery_direction_change_penalty,
            self.sweep_entry_recovery_max_path_length,
            self.sweep_entry_recovery_goal_position_tolerance,
            self.sweep_entry_recovery_goal_yaw_tolerance,
            self.sweep_entry_alignment_radius,
        )
        transition_completion_values = (
            self.transition_completion_start_gate,
            self.transition_completion_cross_track_tolerance,
            self.transition_completion_pass_margin,
            self.transition_completion_min_progress_ratio,
            self.transition_completion_max_sample_step,
            self.transition_completion_max_overshoot,
            self.transition_tracking_deviation_tolerance,
            self.transition_tracking_heading_tolerance,
            self.transition_completion_poll_period,
            self.transition_completion_stop_speed,
            self.transition_completion_stop_timeout,
            self.hybrid_recovery_timeout_sec,
        )
        if (
            not self.enforced_path_service_name
            or not self.hybrid_precompute_service_name
            or not self.navigation_owner_service_name
            or not math.isfinite(self.entry_position_tolerance)
            or not math.isfinite(self.entry_yaw_tolerance)
            or self.entry_position_tolerance <= 0.0
            or self.entry_yaw_tolerance <= 0.0
            or not math.isfinite(self.hybrid_precompute_timeout_sec)
            or self.hybrid_precompute_timeout_sec <= 0.0
            or not math.isfinite(self.hybrid_precompute_retry_sec)
            or self.hybrid_precompute_retry_sec <= 0.0
            or not all(math.isfinite(value)
                       for value in sweep_completion_values)
            or not all(math.isfinite(value)
                       for value in transition_completion_values)
            or not 0.10 <= self.transition_completion_start_gate <= 1.50
            or not 0.05 <= self.transition_completion_cross_track_tolerance <= 0.50
            or not 0.0 <= self.transition_completion_pass_margin <= 0.20
            or not 0.50 <= self.transition_completion_min_progress_ratio <= 1.00
            or not 0.20 <= self.transition_completion_max_sample_step <= 1.00
            or not 0.10 <= self.transition_completion_max_overshoot <= 1.00
            or not 0.10 <= self.transition_tracking_deviation_tolerance <= 1.00
            or not 0.10 <= self.transition_tracking_heading_tolerance <= 1.20
            or not 1 <= (
                self.transition_tracking_deviation_confirmation_samples
            ) <= 10
            or not 1 <= self.transition_completion_confirmation_samples <= 5
            or not 0.02 <= self.transition_completion_poll_period <= 0.20
            or not 0.01 <= self.transition_completion_stop_speed <= 0.20
            or not 0.5 <= self.transition_completion_stop_timeout <= 10.0
            or not 0.2 <= self.hybrid_recovery_timeout_sec <= 10.0
            or not 0.10 <= self.sweep_completion_start_gate <= 1.00
            or not 0.05 <= self.sweep_completion_cross_track_tolerance <= 0.50
            or not 0.05 <= self.sweep_completion_heading_tolerance <= 0.80
            or not 0.0 <= self.sweep_completion_pass_margin <= 0.20
            or not 0.75 <= self.sweep_completion_min_progress_ratio <= 1.00
            or not 0.20 <= self.sweep_completion_max_sample_step <= 1.00
            or not 1 <= self.sweep_completion_confirmation_samples <= 5
            or not 0.02 <= self.sweep_completion_poll_period <= 0.20
            or not 0.01 <= self.sweep_completion_max_linear_speed <= 0.20
            or not 0.20 <= self.sweep_entry_recovery_progress_limit <= 2.00
            or not 1 <= self.sweep_entry_recovery_confirmation_samples <= 10
            or not 0.10 <= self.sweep_entry_recovery_reverse_cost_speed <= 2.00
            or not 0.10 <= self.sweep_entry_recovery_forward_cost_speed <= 2.00
            or not 0.0 <= (
                self.sweep_entry_recovery_direction_change_penalty
            ) <= 5.00
            or not 1.00 <= self.sweep_entry_recovery_max_path_length <= 10.00
            or not 0.05 <= (
                self.sweep_entry_recovery_goal_position_tolerance
            ) <= self.entry_position_tolerance
            or not 0.05 <= (
                self.sweep_entry_recovery_goal_yaw_tolerance
            ) <= self.entry_yaw_tolerance
            or not 0.30 <= self.sweep_entry_alignment_radius <= 3.00
        ):
            raise ValueError(
                "coverage entry, sweep completion, and planner hand-off "
                "parameters are invalid"
            )
        self.state = "IDLE"
        self.detail = "waiting for a static map"
        self.localized = False
        self.localization_received_wall = 0.0
        self.watchdog_motion_enabled = False
        self.watchdog_max_linear_speed = 0.0
        self.watchdog_max_angular_speed = 0.0
        self.watchdog_received_wall = 0.0
        self.avoidance_scan_valid = False
        self.avoidance_scan_detail = "/scan obstacle data has not arrived"
        self.avoidance_scan_received_wall = 0.0
        # Keep the last *valid* sample separate from the latest callback.  The
        # fusion node can occasionally publish one malformed/stale sample while
        # the preceding sample is still inside the fail-closed freshness
        # window.  Treating that one callback as an immediate permanent pause
        # made coverage appear to accept a task and then do nothing.
        self.avoidance_scan_last_valid_wall = 0.0
        self.dual_lidar_active = False
        self.dual_lidar_received_wall = 0.0
        self.avoidance_loss_paused = False
        self.chassis_status = None
        self.chassis_status_received_wall = 0.0
        self.chassis_odom = None
        self.chassis_odom_received_wall = 0.0
        self.chassis_monitor = None
        self.chassis_monitor_received_wall = 0.0
        self.chassis_fault_paused = False
        self.mode_goals_allowed = False
        self.mode_state = ""
        self.mode_received_wall = 0.0
        self.odom = None
        self.odom_received_wall = 0.0
        self.active = False
        self.plan_pending = False
        self.plan_token = ""
        self.start_pending = False
        self.start_token = ""
        self.navigation_owner_token = ""
        self.navigation_owner_claimed = False
        self.navigation_owner_releasing = False
        # Serialize the complete cleanup transaction for an owner retained
        # after a failed start.  Without this operation-level guard, two
        # cancel callbacks can both act on global "current" goal/planner state
        # and the older callback can spill into a newly started mission.
        self.retained_cleanup_lock = threading.Lock()
        self.manual_pause = False
        self.manual_pause_reason = ""
        self.external_pause = False
        self.cancel_requested = False
        self.current_segment = 0
        self.total_segments = 0
        self.blocked_segments = []
        self.traversed_distance = 0.0
        self.covered_cells = set()
        self.last_tracked_point = None
        self.executed_path = Path()
        self.executed_path.header.frame_id = "map"
        self.worker = None
        # SimpleActionClient only tracks its latest GoalHandle.  Keep a local
        # monotonically increasing generation and the exact handle captured by
        # send_goal so cleanup can prove it is observing this manager's same
        # goal before opening any downstream ownership gate.
        self.move_base_goal_generation = 0
        self.move_base_goal_pending = False
        self.move_base_goal_handle = None
        self.move_base_goal_terminal_state = GoalStatus.LOST
        # Batch definitions are immutable snapshots owned by the J6M manager.
        # Saved/editable region persistence remains a Qt responsibility; this
        # state only exists for the lifetime of one accepted execution batch.
        self.batch_id = ""
        self.batch_token = ""
        self.batch_start_request_id = ""
        self.batch_request_records = {}
        self.batch_map_digest = ""
        self.batch_active = False
        self.batch_cancel_requested = False
        self.batch_skip_requested = False
        self.batch_abort_detail = ""
        self.batch_phase = "IDLE"
        self.batch_region_token = ""
        self.batch_region_outcome = ""
        self.batch_current_is_last = False
        self.batch_regions = []
        self.batch_current_index = 0
        self.batch_total_regions = 0
        self.batch_completed_regions = 0
        self.batch_partial_regions = 0
        self.batch_skipped_regions = 0
        self.current_region_id = ""
        self.current_region_name = ""
        self.last_region_id = ""
        self.last_region_name = ""
        self.last_region_state = ""
        self.batch_worker = None
        self.batch_wake_event = threading.Event()
        self.original_teb = None
        self.navigation_profile_update_pending = False
        self.kinematics_verified = False
        self.kinematics_detail = "waiting for task-start VCU and TEB verification"
        self.required_steering_angle = configured_steering
        self.chassis_wheelbase = 0.0
        self.chassis_max_steering_angle = 0.0
        self.chassis_max_speed = 0.0
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.status_pub = rospy.Publisher(
            "/coverage/status", CoverageStatus, queue_size=1, latch=True
        )
        self.active_pub = rospy.Publisher(
            "/coverage/active", Bool, queue_size=1, latch=True
        )
        self.region_pub = rospy.Publisher(
            "/coverage/region", PolygonStamped, queue_size=1, latch=True
        )
        self.path_pub = rospy.Publisher(
            "/coverage/planned_path", Path, queue_size=1, latch=True
        )
        self.hybrid_transition_path_pub = rospy.Publisher(
            "/coverage/hybrid_transition_path", Path, queue_size=1, latch=True
        )
        self.executed_path_pub = rospy.Publisher(
            "/coverage/executed_path", Path, queue_size=1, latch=True
        )
        self.marker_pub = rospy.Publisher(
            "/coverage/markers", MarkerArray, queue_size=1, latch=True
        )
        self.enforced_path_pub = rospy.Publisher(
            "/coverage/enforced_path", EnforcedPath, queue_size=1
        )

        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self._map_callback, queue_size=1)
        self.localization_sub = rospy.Subscriber(
            "/fast_lio/localization_status", String, self._localization_callback, queue_size=5
        )
        self.watchdog_sub = rospy.Subscriber(
            "/nvidia_cmd_vel_watchdog/status", String, self._watchdog_callback, queue_size=5
        )
        self.avoidance_scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self._avoidance_scan_callback, queue_size=5
        )
        self.dual_lidar_active_sub = rospy.Subscriber(
            "/avoidance/dual_lidar_active",
            Bool,
            self._dual_lidar_active_callback,
            queue_size=5,
        )
        self.chassis_status_sub = rospy.Subscriber(
            "/m2_driver/chassis_info",
            ChassisStatusInfo,
            self._chassis_status_callback,
            queue_size=5,
        )
        self.chassis_monitor_sub = rospy.Subscriber(
            "/m2_driver/chassis_monitor",
            ChassisMonitorInfo,
            self._chassis_monitor_callback,
            queue_size=5,
        )
        self.chassis_odom_sub = rospy.Subscriber(
            "/odom", Odometry, self._chassis_odom_callback, queue_size=10
        )
        self.mode_sub = rospy.Subscriber(
            "/fod_navigation_mode/status", String, self._mode_callback, queue_size=5
        )
        self.pause_sub = rospy.Subscriber(
            "/navigation_pause/paused", Bool, self._external_pause_callback, queue_size=5
        )
        self.odom_sub = rospy.Subscriber("/Odometry", Odometry, self._odom_callback, queue_size=10)

        self.plan_service = rospy.Service("/coverage/plan", PlanCoverage, self._plan_service)
        self.start_service = rospy.Service(
            "/coverage/start", StartCoverage, self._start_service
        )
        self.start_batch_service = rospy.Service(
            "/coverage/start_batch", StartCoverageBatch,
            self._start_batch_service,
        )
        self.cancel_batch_service = rospy.Service(
            "/coverage/cancel_batch", CancelCoverageBatch,
            self._cancel_batch_service,
        )
        self.pause_service = rospy.Service(
            "/coverage/set_paused", SetBool, self._pause_service
        )
        self.cancel_service = rospy.Service(
            "/coverage/cancel", Trigger, self._cancel_service
        )
        self.skip_current_service = rospy.Service(
            "/coverage/skip_current", Trigger, self._skip_current_service
        )
        self.navigation_profile_service = rospy.Service(
            "/coverage/set_navigation_profile",
            SetNavigationProfile,
            self._set_navigation_profile_service,
        )
        self.planning_defaults_service = rospy.Service(
            "/coverage/set_planning_defaults",
            SetCoveragePlanningDefaults,
            self._set_planning_defaults_service,
        )
        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.enforced_path_client = rospy.ServiceProxy(
            self.enforced_path_service_name, SetEnforcedPath
        )
        self.hybrid_precompute_client = rospy.ServiceProxy(
            self.hybrid_precompute_service_name, PrecomputeTransitions
        )
        self.navigation_owner_client = rospy.ServiceProxy(
            self.navigation_owner_service_name, SetCoverageOwner
        )
        self.status_timer = rospy.Timer(rospy.Duration(0.5), self._status_timer)
        self.tracking_timer = rospy.Timer(rospy.Duration(0.2), self._tracking_timer)
        self.active_pub.publish(Bool(data=False))
        self._publish_status()

    @staticmethod
    def _strict_bool(name, default):
        value = rospy.get_param(name, default)
        if type(value) is not bool:
            raise ValueError("{} must be a YAML boolean".format(name))
        return value

    def _validate_time_parameters(self, parameters):
        parameters.validate()
        max_speed_limit = getattr(self, "max_speed_limit", 1.60)
        max_reverse_speed_limit = getattr(
            self, "max_reverse_speed_limit", 0.80
        )
        max_angular_speed_limit = getattr(
            self, "max_angular_speed_limit", 1.00
        )
        linear_accel_limit = getattr(self, "linear_accel_limit", 2.00)
        angular_accel_limit = getattr(self, "angular_accel_limit", 1.00)
        if not 0.10 <= parameters.max_forward_speed_mps <= max_speed_limit:
            raise ValueError(
                "maximum forward speed must be in [0.10, {:.2f}] m/s".format(
                    max_speed_limit
                )
            )
        if not 0.05 <= parameters.max_reverse_speed_mps <= max_reverse_speed_limit:
            raise ValueError(
                "maximum reverse speed must be in [0.05, {:.2f}] m/s".format(
                    max_reverse_speed_limit
                )
            )
        if not 0.10 <= parameters.max_angular_speed_rps <= max_angular_speed_limit:
            raise ValueError(
                "maximum angular speed must be in [0.10, {:.2f}] rad/s".format(
                    max_angular_speed_limit
                )
            )
        if not 0.10 <= parameters.linear_accel_mps2 <= linear_accel_limit:
            raise ValueError(
                "linear acceleration must be in [0.10, {:.2f}] m/s^2".format(
                    linear_accel_limit
                )
            )
        if not 0.10 <= parameters.angular_accel_rps2 <= angular_accel_limit:
            raise ValueError(
                "angular acceleration must be in [0.10, {:.2f}] rad/s^2".format(
                    angular_accel_limit
                )
            )
        if parameters.direction_change_penalty_sec > 30.0:
            raise ValueError("direction-change penalty must be in [0, 30] s")
        if parameters.segment_handoff_penalty_sec > 30.0:
            raise ValueError("segment handoff penalty must be in [0, 30] s")
        return parameters

    def _time_parameters_from_request(self, request):
        defaults = getattr(
            self, "default_time_parameters", CoverageTimeParameters()
        )
        try:
            parameters = CoverageTimeParameters(
                max_forward_speed_mps=float(getattr(
                    request, "max_speed_mps", defaults.max_forward_speed_mps
                )),
                max_reverse_speed_mps=float(getattr(
                    request, "reverse_speed_mps", defaults.max_reverse_speed_mps
                )),
                max_angular_speed_rps=float(getattr(
                    request, "max_angular_speed_rps",
                    defaults.max_angular_speed_rps
                )),
                linear_accel_mps2=float(getattr(
                    request, "linear_accel_mps2", defaults.linear_accel_mps2
                )),
                angular_accel_rps2=float(getattr(
                    request, "angular_accel_rps2", defaults.angular_accel_rps2
                )),
                allow_reverse=bool(getattr(
                    request, "allow_reverse_transit", defaults.allow_reverse
                )),
                direction_change_penalty_sec=float(
                    getattr(
                        request,
                        "direction_change_penalty_sec",
                        defaults.direction_change_penalty_sec,
                    )
                ),
                segment_handoff_penalty_sec=float(
                    getattr(
                        request,
                        "segment_handoff_penalty_sec",
                        defaults.segment_handoff_penalty_sec,
                    )
                ),
                transit_replan_period_sec=float(
                    getattr(
                        request,
                        "transit_replan_period_sec",
                        defaults.transit_replan_period_sec,
                    )
                ),
            )
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            raise ValueError(
                "coverage time-model parameters must be finite numbers: {}".format(
                    error
                )
            )
        return self._validate_time_parameters(parameters)

    @staticmethod
    def _time_parameters_match(first, second):
        if first is None or second is None or first.allow_reverse != second.allow_reverse:
            return False
        fields = (
            "max_forward_speed_mps",
            "max_reverse_speed_mps",
            "max_angular_speed_rps",
            "linear_accel_mps2",
            "angular_accel_rps2",
            "direction_change_penalty_sec",
            "segment_handoff_penalty_sec",
            "transit_replan_period_sec",
        )
        return all(
            math.isclose(getattr(first, field), getattr(second, field),
                         rel_tol=1.0e-9, abs_tol=1.0e-9)
            for field in fields
        )

    def _apply_time_parameters_locked(self, parameters):
        self.task_time_parameters = parameters
        self.task_max_speed = parameters.max_forward_speed_mps
        self.allow_reverse_transit = parameters.allow_reverse
        self.reverse_transit_speed = parameters.max_reverse_speed_mps
        self.task_max_angular_speed = parameters.max_angular_speed_rps
        self.task_linear_accel = parameters.linear_accel_mps2
        self.task_angular_accel = parameters.angular_accel_rps2
        self.direction_change_penalty = parameters.direction_change_penalty_sec
        self.segment_handoff_penalty = parameters.segment_handoff_penalty_sec
        self.transit_replan_period = parameters.transit_replan_period_sec

    @staticmethod
    def _teb_navigation_profile(parameters):
        return {
            "max_vel_x": parameters.max_forward_speed_mps,
            "max_vel_x_backwards": (
                parameters.max_reverse_speed_mps
                if parameters.allow_reverse else 0.0
            ),
            "max_vel_theta": parameters.max_angular_speed_rps,
            "acc_lim_x": parameters.linear_accel_mps2,
            "acc_lim_theta": parameters.angular_accel_rps2,
            "allow_init_with_backwards_motion": parameters.allow_reverse,
        }

    def _validate_planning_defaults_message(self, message):
        try:
            operation_width = float(message.operation_width_m)
            overlap_ratio = float(message.overlap_ratio)
            parameters = CoverageTimeParameters(
                max_forward_speed_mps=float(message.max_forward_speed_mps),
                max_reverse_speed_mps=float(message.max_reverse_speed_mps),
                max_angular_speed_rps=float(message.max_angular_speed_rps),
                linear_accel_mps2=float(message.linear_accel_mps2),
                angular_accel_rps2=float(message.angular_accel_rps2),
                allow_reverse=bool(message.allow_reverse),
                direction_change_penalty_sec=float(
                    message.direction_change_penalty_sec
                ),
                segment_handoff_penalty_sec=float(
                    message.segment_handoff_penalty_sec
                ),
                transit_replan_period_sec=float(
                    message.transit_replan_period_sec
                ),
            )
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            raise ValueError(
                "planning defaults must contain finite numeric values: {}".format(
                    error
                )
            )
        if not math.isfinite(operation_width) or not 0.30 <= operation_width <= 3.0:
            raise ValueError("operation width must be in [0.30, 3.00] m")
        if not math.isfinite(overlap_ratio) or not 0.0 <= overlap_ratio <= 0.5:
            raise ValueError("overlap ratio must be in [0.00, 0.50]")
        self._validate_time_parameters(parameters)
        return operation_width, overlap_ratio, parameters

    @staticmethod
    def _planning_defaults_message(operation_width, overlap_ratio, parameters):
        message = CoveragePlanningParameters()
        message.operation_width_m = operation_width
        message.overlap_ratio = overlap_ratio
        message.allow_reverse = parameters.allow_reverse
        message.max_forward_speed_mps = parameters.max_forward_speed_mps
        message.max_reverse_speed_mps = parameters.max_reverse_speed_mps
        message.max_angular_speed_rps = parameters.max_angular_speed_rps
        message.linear_accel_mps2 = parameters.linear_accel_mps2
        message.angular_accel_rps2 = parameters.angular_accel_rps2
        message.direction_change_penalty_sec = (
            parameters.direction_change_penalty_sec
        )
        message.segment_handoff_penalty_sec = (
            parameters.segment_handoff_penalty_sec
        )
        message.transit_replan_period_sec = parameters.transit_replan_period_sec
        return message

    def _factory_planning_defaults(self):
        with open(self.factory_defaults_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict):
            raise ValueError("factory defaults must be a YAML mapping")
        required_keys = [item[1] for item in PLANNING_DEFAULT_YAML_FIELDS]
        missing = [key for key in required_keys if key not in data]
        if missing:
            raise ValueError(
                "factory defaults are missing keys: {}".format(
                    ", ".join(sorted(missing))
                )
            )
        if type(data["allow_reverse_transit"]) is not bool:
            raise ValueError("factory allow_reverse_transit must be a YAML boolean")
        message = CoveragePlanningParameters()
        for attribute, yaml_key, _number_format in PLANNING_DEFAULT_YAML_FIELDS:
            setattr(message, attribute, data[yaml_key])
        return self._validate_planning_defaults_message(message)

    @staticmethod
    def _verify_teb_navigation_profile(applied, expected):
        for key, value in expected.items():
            actual = applied.get(key)
            if isinstance(value, bool):
                if type(actual) is not bool or actual != value:
                    raise RuntimeError(
                        "TEB did not retain {}={}".format(key, value)
                    )
            elif actual is None or not math.isclose(
                    float(actual), float(value), rel_tol=1.0e-6,
                    abs_tol=1.0e-6):
                raise RuntimeError(
                    "TEB did not retain {}={:.3f}".format(key, value)
                )

    def _set_planning_defaults_service(self, request):
        """Atomically apply Qt defaults to TEB, manager state and coverage.yaml."""
        response = SetCoveragePlanningDefaultsResponse()
        try:
            if request.restore_factory_defaults:
                operation_width, overlap_ratio, parameters = (
                    self._factory_planning_defaults()
                )
            else:
                operation_width, overlap_ratio, parameters = (
                    self._validate_planning_defaults_message(request.parameters)
                )
        except (AttributeError, OSError, TypeError, ValueError, yaml.YAMLError) as error:
            response.message = "invalid planning defaults: {}".format(error)
            return response

        with self.lock:
            if (
                self.active
                or self.batch_active
                or self.plan_pending
                or self.start_pending
                or self.plan is not None
                or self.original_teb is not None
                or self.navigation_profile_update_pending
            ):
                response.message = (
                    "planning defaults cannot change while coverage planning, "
                    "preview, execution, or restoration is active"
                )
                return response
            self.navigation_profile_update_pending = True

        staged_path = ""
        teb_client = None
        previous_teb = None
        expected = self._teb_navigation_profile(parameters)
        try:
            staged_path, target_path = _stage_atomic_yaml_update(
                self.config_file,
                self._planning_defaults_message(
                    operation_width, overlap_ratio, parameters
                ),
            )
            import dynamic_reconfigure.client
            teb_client = dynamic_reconfigure.client.Client(
                "/move_base/TebLocalPlannerROS", timeout=2.0
            )
            current_teb = teb_client.get_configuration(timeout=2.0)
            previous_teb = {}
            for key in expected:
                if key not in current_teb:
                    raise RuntimeError(
                        "TEB configuration does not expose {}".format(key)
                    )
                previous_teb[key] = current_teb[key]
            applied = teb_client.update_configuration(expected)
            self._verify_teb_navigation_profile(applied, expected)
            _commit_staged_yaml(staged_path, target_path)
            staged_path = ""
        except Exception as error:
            rollback_error = None
            if teb_client is not None and previous_teb is not None:
                try:
                    restored = teb_client.update_configuration(previous_teb)
                    self._verify_teb_navigation_profile(restored, previous_teb)
                except Exception as caught:
                    rollback_error = caught
            if staged_path:
                try:
                    os.unlink(staged_path)
                except OSError:
                    pass
            with self.lock:
                self.navigation_profile_update_pending = False
            response.message = (
                "could not persist and apply planning defaults: {}".format(error)
            )
            if rollback_error is not None:
                response.message += "; TEB rollback failed: {}".format(
                    rollback_error
                )
            return response

        with self.lock:
            self.operation_width = operation_width
            self.overlap_ratio = overlap_ratio
            self.default_max_speed = parameters.max_forward_speed_mps
            self.reverse_transit_speed = parameters.max_reverse_speed_mps
            self.default_max_angular_speed = parameters.max_angular_speed_rps
            self.default_linear_accel = parameters.linear_accel_mps2
            self.default_angular_accel = parameters.angular_accel_rps2
            self.direction_change_penalty = parameters.direction_change_penalty_sec
            self.segment_handoff_penalty = parameters.segment_handoff_penalty_sec
            self.default_transit_replan_period = (
                parameters.transit_replan_period_sec
            )
            self.default_time_parameters = parameters
            self._apply_time_parameters_locked(parameters)
            self.navigation_profile_update_pending = False
            self.detail = (
                "planning defaults persisted: width {:.2f} m, overlap {:.0f}%, "
                "forward {:.2f} m/s, reverse {}, Hybrid retry {:.1f} s"
            ).format(
                operation_width,
                100.0 * overlap_ratio,
                parameters.max_forward_speed_mps,
                "{:.2f} m/s".format(parameters.max_reverse_speed_mps)
                if parameters.allow_reverse else "disabled",
                parameters.transit_replan_period_sec,
            )
            response.effective = self._planning_defaults_message(
                operation_width, overlap_ratio, parameters
            )
            response.message = self.detail
            response.success = True

        # Keep rosparam introspection consistent with the committed runtime and
        # file.  A transient master-side metadata failure does not invalidate
        # the already atomic TEB/manager/YAML transaction.
        try:
            for attribute, yaml_key, _number_format in PLANNING_DEFAULT_YAML_FIELDS:
                rospy.set_param("~" + yaml_key, getattr(response.effective, attribute))
        except Exception as error:
            rospy.logwarn("planning defaults applied but rosparam mirror failed: %s", error)
        self._publish_status()
        return response

    def _set_navigation_profile_service(self, request):
        """Compatibility adapter for clients built before full-default persistence."""
        response = SetNavigationProfileResponse()
        with self.lock:
            defaults = getattr(
                self, "default_time_parameters", CoverageTimeParameters()
            )
            operation_width = getattr(self, "operation_width", 1.00)
            overlap_ratio = getattr(self, "overlap_ratio", 0.15)
        full_request = SetCoveragePlanningDefaultsRequest()
        full_request.restore_factory_defaults = False
        full_request.parameters = self._planning_defaults_message(
            operation_width,
            overlap_ratio,
            CoverageTimeParameters(
                max_forward_speed_mps=request.max_forward_speed_mps,
                max_reverse_speed_mps=request.max_reverse_speed_mps,
                max_angular_speed_rps=request.max_angular_speed_rps,
                linear_accel_mps2=request.linear_accel_mps2,
                angular_accel_rps2=request.angular_accel_rps2,
                allow_reverse=request.allow_reverse,
                direction_change_penalty_sec=(
                    defaults.direction_change_penalty_sec
                ),
                segment_handoff_penalty_sec=(
                    defaults.segment_handoff_penalty_sec
                ),
                transit_replan_period_sec=request.transit_replan_period_sec,
            ),
        )
        full_response = self._set_planning_defaults_service(full_request)
        response.success = full_response.success
        response.message = full_response.message
        return response

    def _map_callback(self, message):
        try:
            grid = GridMap(
                message.info.width,
                message.info.height,
                message.info.resolution,
                message.info.origin.position.x,
                message.info.origin.position.y,
                message.data,
            )
            origin = message.info.origin
            digest = occupancy_grid_digest(
                message.header.frame_id,
                message.info.width,
                message.info.height,
                message.info.resolution,
                (
                    origin.position.x,
                    origin.position.y,
                    origin.position.z,
                ),
                (
                    origin.orientation.x,
                    origin.orientation.y,
                    origin.orientation.z,
                    origin.orientation.w,
                ),
                message.data,
            )
        except ValueError as error:
            rospy.logerr_throttle(2.0, "coverage rejected map: %s", error)
            return
        with self.lock:
            previous = self.map_digest
            self.grid = grid
            self.map_message = copy.deepcopy(message)
            self.map_digest = digest
            if previous and previous != digest:
                self.plan = None
                self.plan_id = ""
                self.plan_map_digest = ""
                self.plan_time_parameters = None
                changed_while_active = self.active or self.batch_active
                self.detail = "static map changed; coverage plan invalidated"
                if not changed_while_active:
                    self.state = "IDLE"
                # Invalidate the old preview while the same lifecycle
                # generation is locked.  Otherwise a concurrent /plan could
                # publish a new preview and then have it erased here.
                self._clear_visualizations()
                if changed_while_active:
                    # RLock makes this re-entrant.  Keeping the cancellation
                    # request in this critical section prevents an old map
                    # callback from canceling a newer mission generation.
                    self._request_cancel(
                        "static map changed during coverage"
                    )
            elif self.state == "IDLE":
                self.detail = "static map ready; select a coverage region"
        self._publish_status()

    def _localization_callback(self, message):
        with self.lock:
            self.localized = message.data.startswith("state=LOCALIZED;")
            self.localization_received_wall = time.monotonic()

    def _watchdog_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        max_linear_speed = payload.get("max_linear_speed")
        max_angular_speed = payload.get("max_angular_speed")
        if (
            isinstance(max_linear_speed, bool)
            or not isinstance(max_linear_speed, (int, float))
            or not math.isfinite(float(max_linear_speed))
            or float(max_linear_speed) <= 0.0
            or isinstance(max_angular_speed, bool)
            or not isinstance(max_angular_speed, (int, float))
            or not math.isfinite(float(max_angular_speed))
            or float(max_angular_speed) <= 0.0
        ):
            return
        with self.lock:
            self.watchdog_motion_enabled = payload.get("motion_enabled") is True
            self.watchdog_max_linear_speed = float(max_linear_speed)
            self.watchdog_max_angular_speed = float(max_angular_speed)
            self.watchdog_received_wall = time.monotonic()

    def _validate_avoidance_scan(self, message, ros_now=None):
        geometry_valid = (
            len(message.ranges) > 0
            and all(math.isfinite(value) for value in (
                message.angle_min,
                message.angle_max,
                message.angle_increment,
                message.range_min,
                message.range_max,
            ))
            and message.angle_max > message.angle_min
            and message.angle_increment > 0.0
            and message.range_max > message.range_min >= 0.0
        )
        if not geometry_valid:
            return False, "/scan geometry or ranges are invalid"
        frame = str(message.header.frame_id).lstrip("/")
        if frame != self.avoidance_scan_frame:
            return False, "/scan frame is {}, expected {}".format(
                frame or "<empty>", self.avoidance_scan_frame
            )
        try:
            stamp = float(message.header.stamp.to_sec())
            now = float(rospy.Time.now().to_sec()) if ros_now is None else float(ros_now)
            age = now - stamp
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False, "/scan timestamp is invalid"
        if not math.isfinite(stamp) or stamp <= 0.0 or not math.isfinite(age):
            return False, "/scan timestamp is invalid"
        if age < -self.avoidance_scan_future_tolerance_sec:
            return False, "/scan timestamp is too far in the future"
        if age > self.avoidance_scan_fresh_sec:
            return False, "/scan message timestamp is stale"
        return True, "/scan geometry, frame and timestamp are valid"

    def _avoidance_scan_callback(self, message):
        valid, detail = self._validate_avoidance_scan(message)
        received_wall = time.monotonic()
        with self.lock:
            self.avoidance_scan_valid = valid
            self.avoidance_scan_detail = detail
            self.avoidance_scan_received_wall = received_wall
            if valid:
                self.avoidance_scan_last_valid_wall = received_wall
        self._pause_for_avoidance_loss()

    def _dual_lidar_active_callback(self, message):
        with self.lock:
            self.dual_lidar_active = bool(message.data)
            self.dual_lidar_received_wall = time.monotonic()
        self._pause_for_avoidance_loss()

    def _avoidance_ready_locked(self, now=None):
        """Return fail-closed readiness for the move_base obstacle scan chain."""
        if now is None:
            now = time.monotonic()
        if (
            self.avoidance_scan_last_valid_wall <= 0.0
            or now - self.avoidance_scan_last_valid_wall
            > self.avoidance_scan_fresh_sec
        ):
            if (self.avoidance_scan_received_wall > 0.0 and
                    not self.avoidance_scan_valid):
                return False, "no recent valid /scan sample; latest rejected: {}".format(
                    self.avoidance_scan_detail
                )
            return False, "/scan obstacle data has no recent valid sample"
        if self.avoidance_scan_valid:
            scan_detail = "/scan is fresh"
        else:
            scan_detail = (
                "/scan last valid sample is still fresh; latest rejected: {}"
            ).format(self.avoidance_scan_detail)
        if self.require_dual_lidar:
            if (
                self.dual_lidar_received_wall <= 0.0
                or now - self.dual_lidar_received_wall
                > self.dual_lidar_fresh_sec
            ):
                return False, "front/rear LD19 fusion status is absent or stale"
            if not self.dual_lidar_active:
                return False, "front/rear LD19 are not contributing to /scan"
            return True, "{}; MID360 and front/rear LD19 are active".format(
                scan_detail
            )
        return True, "{}; dual LD19 is not required by configuration".format(
            scan_detail
        )

    def _pause_for_avoidance_loss(self):
        cancel_goal = False
        with self.lock:
            ready, reason = self._avoidance_ready_locked()
            if (self.active and not ready and not self.cancel_requested and
                    not self.avoidance_loss_paused):
                self.avoidance_loss_paused = True
                self.manual_pause = True
                self.state = "PAUSED"
                self.manual_pause_reason = (
                    "obstacle sensing lost: {}; manual resume is required"
                ).format(reason)
                self.detail = self.manual_pause_reason
                cancel_goal = True
        if cancel_goal:
            rospy.logwarn("coverage paused: %s", self.manual_pause_reason)
            self.move_base.cancel_goal()
        return ready

    def _chassis_status_callback(self, message):
        with self.lock:
            self.chassis_status = copy.deepcopy(message)
            self.chassis_status_received_wall = time.monotonic()
        self._pause_for_chassis_fault()

    def _chassis_monitor_callback(self, message):
        with self.lock:
            self.chassis_monitor = copy.deepcopy(message)
            self.chassis_monitor_received_wall = time.monotonic()
        self._pause_for_chassis_fault()

    def _chassis_odom_callback(self, message):
        with self.lock:
            self.chassis_odom = copy.deepcopy(message)
            self.chassis_odom_received_wall = time.monotonic()
        self._pause_for_chassis_fault()

    def _chassis_ready_locked(self, now=None):
        """Return fail-closed readiness for physical M2 command execution."""
        if now is None:
            now = time.monotonic()
        if (
            self.chassis_odom_received_wall <= 0.0
            or now - self.chassis_odom_received_wall
            > self.chassis_odom_fresh_sec
        ):
            return False, "M2 VCU feedback odometry is absent or stale"
        if (
            self.chassis_status is None
            or self.chassis_status_received_wall <= 0.0
            or now - self.chassis_status_received_wall
            > self.chassis_status_fresh_sec
        ):
            return False, "VCU chassis status is absent or stale"

        emergencies = []
        for field, label in (
            ("hard_emergency", "hard emergency"),
            ("soft_emergency", "software emergency"),
            ("gamepad_emergency", "gamepad/remote emergency"),
            ("robot_emergency", "robot emergency"),
        ):
            if bool(getattr(self.chassis_status, field, True)):
                emergencies.append(label)
        if emergencies:
            return False, "VCU emergency active: {}".format(
                ", ".join(emergencies)
            )

        faults = []
        # ControllerMonitor is an event/fault frame rather than a periodic
        # health heartbeat on the installed VCU.  A current fault repeats at
        # high rate, while a healthy controller may publish nothing.  Latch a
        # received fault for a bounded interval, but do not interpret silence
        # as a fault after the periodic chassis emergency status is healthy.
        monitor_recent = (
            self.chassis_monitor is not None
            and self.chassis_monitor_received_wall > 0.0
            and now - self.chassis_monitor_received_wall
            <= self.chassis_monitor_fault_latch_sec
        )
        if monitor_recent:
            for prefix, label in (
                ("tcu", "TCU"),
                ("lecu", "left ECU"),
                ("recu", "right ECU"),
            ):
                if int(getattr(self.chassis_monitor, prefix + "_state", 1)) != 0:
                    faults.append(label + " emergency")
                if int(getattr(self.chassis_monitor, prefix + "_timeout", 1)) != 0:
                    faults.append(label + " communication timeout")
                if int(getattr(self.chassis_monitor, prefix + "_stuck", 1)) != 0:
                    faults.append(label + " current over-limit")
            for prefix, label in (("lecu", "left ECU"), ("recu", "right ECU")):
                if int(getattr(self.chassis_monitor, prefix + "_brake", 1)) != 0:
                    faults.append(label + " brake engaged")
        if faults:
            return False, "VCU controller fault: {}".format(", ".join(faults))

        monitor_detail = (
            "latest controller monitor has no fault"
            if monitor_recent
            else "no recent controller fault frame"
        )
        return True, "VCU emergency status is fresh; {}".format(monitor_detail)

    def _pause_for_chassis_fault(self):
        cancel_goal = False
        with self.lock:
            ready, reason = self._chassis_ready_locked()
            if (
                self.active
                and not ready
                and not self.cancel_requested
                and not self.chassis_fault_paused
            ):
                self.chassis_fault_paused = True
                self.manual_pause = True
                self.state = "PAUSED"
                self.manual_pause_reason = (
                    "chassis execution lost: {}; manual resume is required"
                ).format(reason)
                self.detail = self.manual_pause_reason
                cancel_goal = True
        if cancel_goal:
            rospy.logwarn("coverage paused: %s", self.manual_pause_reason)
            self.move_base.cancel_goal()
        return ready

    def _mode_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            self.mode_state = str(payload.get("state", ""))
            self.mode_goals_allowed = payload.get("move_base_goals_allowed") is True
            self.mode_received_wall = time.monotonic()

    def _external_pause_callback(self, message):
        with self.lock:
            self.external_pause = bool(message.data)
            if self.active and self.external_pause:
                self.state = "PAUSED"
                self.detail = "navigation paused by FOD safety arbitration"
        if message.data and self.active:
            self.move_base.cancel_goal()

    def _odom_callback(self, message):
        with self.lock:
            self.odom = copy.deepcopy(message)
            self.odom_received_wall = time.monotonic()

    def _localization_is_fresh(self):
        with self.lock:
            return self.localized and (
                time.monotonic() - self.localization_received_wall
                <= self.localization_fresh_sec
            )

    def _current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rospy.Time(0), rospy.Duration(0.5)
            )
        except Exception:
            return None
        quaternion = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y +
                         quaternion.z * quaternion.z),
        )
        if not math.isfinite(yaw):
            return None
        return (
            Point(transform.transform.translation.x,
                  transform.transform.translation.y),
            yaw,
        )

    def _current_point(self):
        current = self._current_pose()
        return None if current is None else current[0]

    def _current_chassis_linear_speed(self):
        """Return fresh physical planar speed, or None when it is untrusted."""
        with self.lock:
            message = copy.deepcopy(getattr(self, "chassis_odom", None))
            received_wall = float(getattr(
                self, "chassis_odom_received_wall", 0.0
            ))
            fresh_sec = float(getattr(self, "chassis_odom_fresh_sec", 1.0))
        if (
            message is None
            or received_wall <= 0.0
            or time.monotonic() - received_wall > fresh_sec
        ):
            return None
        linear = message.twist.twist.linear
        values = (float(linear.x), float(linear.y))
        if not all(math.isfinite(value) for value in values):
            return None
        return math.hypot(values[0], values[1])

    @staticmethod
    def _points_from_region(region):
        return [Point(float(point.x), float(point.y)) for point in region.polygon.points]

    @staticmethod
    def _pose(point, yaw, stamp=None):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp if stamp is not None else rospy.Time.now()
        pose.pose.position.x = point.x
        pose.pose.position.y = point.y
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        return pose

    def _path_for_points(self, points, yaw):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        path.poses = [self._pose(point, yaw, path.header.stamp) for point in points]
        return path

    def _planner(self, grid=None):
        return CoveragePlanner(
            self.grid if grid is None else grid,
            footprint_front=float(rospy.get_param("~footprint_front_m", 0.62)),
            footprint_rear=float(rospy.get_param("~footprint_rear_m", 0.62)),
            footprint_half_width=float(
                rospy.get_param("~footprint_half_width_m", 0.45)
            ),
            minimum_swath_length=float(
                rospy.get_param("~minimum_swath_length_m", 1.20)
            ),
            angle_step_degrees=float(
                rospy.get_param("~candidate_angle_step_deg", 15.0)
            ),
            minimum_turning_radius=self.minimum_turning_radius,
        )

    def _plan_region_route(self, route_planner, points, operation_width,
                           overlap_ratio, reachable_seed, route_origin,
                           route_yaw, time_parameters):
        """Plan lanes without building a full static connector graph online.

        The legacy precompute architecture evaluates four sweep angles with a
        seconds-based permutation beam whose edge weights run static-map A*
        queries.  That duplicates global planning in either online mode and
        made the nine-row C region spend nearly three simulated minutes
        motionless.  Both on-demand modes instead select the sweep angle with
        the deterministic geometric planner, then run one bounded time beam
        over that angle's lines.  The first entry uses the static known-free
        distance because it is executed by Navfn + TEB; later edges use the
        obstacle-free curvature proxy because they are executed by live
        Hybrid A*.  The beam may skip physical neighbours when that reduces
        the complete route cost.  No route-level step constructs or caches an
        executable connector.
        """
        lightweight = bool(
            getattr(self, "online_hybrid_without_precompute", False)
            or getattr(self, "hierarchical_hybrid_on_demand", False)
        )
        plan = route_planner.plan(
            points,
            operation_width,
            overlap_ratio,
            reachable_seed=reachable_seed,
            route_origin=route_origin,
            route_yaw=route_yaw,
            time_parameters=None if lightweight else time_parameters,
            time_search_beam_width=getattr(
                self, "time_search_beam_width", 128
            ),
        )
        if lightweight:
            routing_time_parameters = time_parameters
            if bool(getattr(self, "direct_hybrid_to_final_goal", False)):
                routing_time_parameters = self._transit_time_parameters(
                    time_parameters
                )
            direct_to_final = bool(getattr(
                self, "direct_hybrid_to_final_goal", False
            ))
            plan.swaths, estimate = order_swaths(
                plan.swaths,
                route_origin,
                float(getattr(
                    plan,
                    "spacing",
                    operation_width * (1.0 - overlap_ratio),
                )),
                self.minimum_turning_radius,
                current_yaw=route_yaw,
                time_parameters=routing_time_parameters,
                # Long-range obstacle topology is needed only for the first
                # Navfn entry.  Later Hybrid edges stay path-free here and are
                # planned from live pose immediately before execution.
                connector_distance=None,
                first_entry_connector_distance=(
                    route_planner.connector_distance
                    if direct_to_final else None
                ),
                # Treat starts within one estimated second as equivalent, then
                # choose the lowest complete route time.  Before that, keep
                # the first endpoint within 0.30 m of the shortest known-free
                # entry path.  This preserves the operator-visible rule that
                # a nearby end of a long swath must beat its remote end while
                # still letting the beam choose among essentially tied rows.
                first_entry_slack_sec=1.0 if direct_to_final else None,
                first_entry_distance_slack_m=(
                    float(getattr(
                        self, "route_first_entry_distance_slack", 0.30
                    )) if direct_to_final else None
                ),
                return_estimate=True,
                time_search_beam_width=getattr(
                    self, "time_search_beam_width", 128
                ),
            )
            plan.score = estimate.total_time_sec
            plan.estimated_total_time_sec = estimate.total_time_sec
            plan.estimated_sweep_time_sec = estimate.sweep_time_sec
            plan.estimated_transit_time_sec = estimate.transit_time_sec
            plan.estimated_reverse_transitions = estimate.reverse_transitions
            plan.alternative_plans = []
        return plan

    def _plan_service(self, request):
        response = PlanCoverageResponse()
        try:
            time_parameters = self._time_parameters_from_request(request)
        except ValueError as error:
            response.message = str(error)
            with self.lock:
                response.map_digest = self.map_digest
            return response
        with self.lock:
            response.map_digest = self.map_digest
            if getattr(self, "navigation_profile_update_pending", False):
                response.message = "navigation profile update is still in progress"
                return response
            if self.active or self.batch_active:
                response.message = (
                    "cannot replace a plan while coverage or a coverage batch is active"
                )
                return response
            if self.start_pending:
                response.message = "cannot replace a plan while coverage start is preparing"
                return response
            if self.plan_pending:
                response.message = "another coverage plan is already being prepared"
                return response
            if self.grid is None:
                response.message = "static map is not ready"
                return response
            if not request.map_digest:
                response.message = "map digest is required"
                return response
            if request.map_digest != self.map_digest:
                response.message = "map digest does not match the current static map"
                return response
            watchdog_received_wall = getattr(
                self, "watchdog_received_wall", 0.0
            )
            watchdog_fresh = watchdog_received_wall > 0.0 and (
                time.monotonic() - watchdog_received_wall <=
                getattr(self, "watchdog_fresh_sec", 1.0)
            )
            transit_parameters = self._transit_time_parameters(
                time_parameters
            )
            requested_linear = max(
                time_parameters.max_forward_speed_mps,
                time_parameters.max_reverse_speed_mps
                if time_parameters.allow_reverse else 0.0,
                transit_parameters.max_forward_speed_mps,
                transit_parameters.max_reverse_speed_mps
                if transit_parameters.allow_reverse else 0.0,
            )
            if (watchdog_fresh and
                    requested_linear >
                    getattr(self, "watchdog_max_linear_speed", 0.0) + 1.0e-6):
                response.message = (
                    "planning speed {:.2f} m/s exceeds live watchdog cap {:.2f} m/s"
                ).format(
                    requested_linear,
                    getattr(self, "watchdog_max_linear_speed", 0.0),
                )
                return response
            if (watchdog_fresh and
                    max(
                        time_parameters.max_angular_speed_rps,
                        transit_parameters.max_angular_speed_rps,
                    ) >
                    getattr(self, "watchdog_max_angular_speed", 0.0) + 1.0e-6):
                response.message = (
                    "planning angular speed {:.2f} rad/s exceeds live watchdog "
                    "cap {:.2f} rad/s"
                ).format(
                    max(
                        time_parameters.max_angular_speed_rps,
                        transit_parameters.max_angular_speed_rps,
                    ),
                    getattr(self, "watchdog_max_angular_speed", 0.0),
                )
                return response
            token = uuid.uuid4().hex
            self.plan_pending = True
            self.plan_token = token
            grid = self.grid
            map_digest = self.map_digest
            self.state = "PLANNING"
            self.detail = "validating the selected region against the static map"
        self._publish_status()
        if request.region.header.frame_id not in ("", "map"):
            self._finish_plan_failure(
                token, "coverage region must use the map frame"
            )
            response.message = "coverage region must use the map frame"
            return response
        operation_width = float(request.operation_width_m)
        overlap_ratio = float(request.overlap_ratio)
        current_pose = self._current_pose() if self._localization_is_fresh() else None
        current = None if current_pose is None else current_pose[0]
        current_yaw = None if current_pose is None else current_pose[1]
        try:
            points = self._points_from_region(request.region)
            route_origin = current if current is not None else points[0]
            # GridMap instances are immutable after construction.  Planning on
            # this snapshot keeps /scan, localization and map callbacks free,
            # and the digest is checked again before the plan is committed.
            route_planner = self._planner(grid)
            plan = self._plan_region_route(
                route_planner,
                points,
                operation_width,
                overlap_ratio,
                current,
                route_origin,
                current_yaw,
                time_parameters,
            )
        except ValueError as error:
            response.message = str(error)
            self._finish_plan_failure(token, response.message)
            return response
        except Exception as error:
            rospy.logerr("coverage region planning raised an exception: %s", error)
            response.message = "coverage planning failed: {}".format(error)
            self._finish_plan_failure(token, response.message)
            return response
        if current is None:
            current = points[0]
        route = plan.swaths
        plan_id = uuid.uuid4().hex
        region = copy.deepcopy(request.region)
        region.header.frame_id = "map"
        region.header.stamp = rospy.Time.now()
        planned_path = self._planned_path(route, current)
        with self.lock:
            owns_planning_generation = (
                self.plan_pending and self.plan_token == token
            )
            if not owns_planning_generation:
                response.message = "coverage planning was canceled or superseded"
                commit_plan = False
            elif (
                self.active or self.batch_active or self.start_pending or
                self.map_digest != map_digest
            ):
                self.plan_pending = False
                self.plan_token = ""
                response.message = "static map or task state changed during planning"
                if self.plan is not None:
                    self.state = "READY"
                else:
                    self.state = "IDLE"
                self.detail = response.message
                commit_plan = False
            else:
                commit_plan = True
            if commit_plan:
                self.plan_pending = False
                self.plan_token = ""
                self._reset_execution_locked(clear_progress=True)
                self.plan = plan
                self.plan.swaths = route
                self.plan_id = plan_id
                self.plan_map_digest = map_digest
                self.plan_time_parameters = time_parameters
                self.region = region
                self.operation_width = operation_width
                self.overlap_ratio = overlap_ratio
                self._apply_time_parameters_locked(time_parameters)
                self.kinematics_verified = False
                self.kinematics_detail = (
                    "waiting for task-start VCU and TEB verification"
                )
                self.state = "READY"
                self.detail = (
                    "coverage path is ready; estimated {:.1f} s total"
                ).format(getattr(plan, "estimated_total_time_sec", 0.0))
                self.total_segments = len(route) * 2
                # Keep plan commit and all latched visualization publications
                # in the same lifecycle critical section.  A concurrent
                # cancel can therefore only happen before this generation is
                # committed or after its preview is fully published and can
                # be cleared in the correct order.
                self._clear_visualizations()
                self.region_pub.publish(region)
                self.path_pub.publish(planned_path)
                self._publish_markers(route, region)
        if not commit_plan:
            self._publish_status()
            return response
        self._publish_status()
        response.success = True
        response.message = "coverage plan ready"
        response.plan_id = plan_id
        response.planned_path = planned_path
        response.requested_area_m2 = plan.requested_area
        response.reachable_area_m2 = plan.reachable_area
        response.unreachable_area_m2 = plan.unreachable_area
        response.sweep_angle_rad = plan.angle
        response.swath_count = len(route)
        response.segment_count = len(route) * 2
        response.estimated_total_time_sec = getattr(
            plan, "estimated_total_time_sec", 0.0
        )
        response.estimated_sweep_time_sec = getattr(
            plan, "estimated_sweep_time_sec", 0.0
        )
        response.estimated_transit_time_sec = getattr(
            plan, "estimated_transit_time_sec", 0.0
        )
        response.estimated_reverse_transitions = (
            getattr(plan, "estimated_reverse_transitions", 0)
        )
        return response

    def _finish_plan_failure(self, token, reason):
        publish_status = False
        with self.lock:
            if not self.plan_pending or self.plan_token != token:
                return
            self.plan_pending = False
            self.plan_token = ""
            if self.plan is not None and self.plan_map_digest == self.map_digest:
                self.state = "READY"
                self.detail = "new region rejected; previous plan kept: {}".format(
                    reason
                )
            else:
                self.state = "IDLE"
                self.detail = reason
            publish_status = True
        if publish_status:
            self._publish_status()

    @staticmethod
    def _swath_yaw(swath):
        return math.atan2(
            swath.end.y - swath.start.y,
            swath.end.x - swath.start.x,
        )

    def _precompute_hybrid_candidates(self, plan, current, current_yaw,
                                      time_parameters, planner, plan_id):
        """Rescore up to four routes with real entry/inter-swath Hybrid paths.

        The vehicle-to-first-swath entry is subject to exactly the same
        Ackermann constraints as every later connector.  Planning it with
        Navfn creates a holonomic path whose terminal yaw can require an
        in-place turn, leaving TEB to invent an infeasible reverse maneuver.
        Include transition zero in the Hybrid batch and split it at cusps in
        the same way as all other connectors.

        Resource-limit outcomes are retried no faster than every five seconds
        for a bounded 60-second window.  A proven static no-path invalidates
        only that route candidate, never silently skips its following swath.
        """
        candidates = list(getattr(plan, "alternative_plans", None) or [plan])[:4]
        online_unsplit = getattr(
            self, "online_hybrid_without_precompute", False
        )
        hierarchical = getattr(
            self, "hierarchical_hybrid_on_demand", False
        )
        if online_unsplit or hierarchical:
            selected = min(candidates, key=lambda candidate: (
                round(getattr(candidate, "unreachable_area", 0.0), 6),
                getattr(candidate, "estimated_total_time_sec", 0.0),
                len(candidate.swaths),
                getattr(candidate, "angle", 0.0),
            ))
            selected.transition_paths = [None] * len(selected.swaths)
            selected.hybrid_precompute_expansions = 0
            selected.alternative_plans = []
            rospy.loginfo(
                "coverage selected proxy route without Hybrid precompute: "
                "%.1fs, %d swaths; connectors will use %s",
                selected.estimated_total_time_sec,
                len(selected.swaths),
                (("direct live-pose Hybrid plus cusp-split fixed gears"
                  if bool(getattr(
                      self, "direct_hybrid_to_final_goal", False
                  )) else
                  "rolling Navfn topology plus cusp-split Hybrid chunks")
                 if hierarchical else "unsplit online Hybrid at 1 Hz"),
            )
            return selected
        if all(not candidate.swaths for candidate in candidates):
            selected = min(candidates, key=lambda candidate: (
                round(getattr(candidate, "unreachable_area", 0.0), 6),
                getattr(candidate, "estimated_total_time_sec", 0.0),
                getattr(candidate, "angle", 0.0),
            ))
            selected.transition_paths = []
            selected.hybrid_precompute_expansions = 0
            selected.alternative_plans = []
            return selected
        try:
            rospy.wait_for_service(
                self.hybrid_precompute_service_name, timeout=0.5
            )
        except Exception as error:
            raise RuntimeError(
                "Hybrid transition precompute service is unavailable: {}".format(
                    error
                )
            )
        pending = {}
        paths = {}
        results = {}
        invalid_candidates = {}
        for candidate_index, candidate in enumerate(candidates):
            for transition_index in range(len(candidate.swaths)):
                following = candidate.swaths[transition_index]
                request = HybridTransitionRequest()
                request.candidate_index = candidate_index
                request.transition_index = transition_index
                if transition_index == 0:
                    request.start = self._pose(current, current_yaw)
                else:
                    previous = candidate.swaths[transition_index - 1]
                    request.start = self._pose(
                        previous.end, self._swath_yaw(previous)
                    )
                request.goal = self._pose(
                    following.start, self._swath_yaw(following)
                )
                pending[(candidate_index, transition_index)] = request

        deadline = time.monotonic() + self.hybrid_precompute_timeout_sec
        while pending and not rospy.is_shutdown():
            with self.lock:
                if self.cancel_requested:
                    raise RuntimeError("Hybrid transition precompute was canceled")
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            active_requests = [
                request for key, request in sorted(pending.items())
                if key[0] not in invalid_candidates
            ]
            if not active_requests:
                break
            try:
                response = self.hybrid_precompute_client(
                    plan_id=plan_id,
                    transitions=active_requests,
                    transit_profile=self._transit_profile(),
                    total_timeout_sec=max(0.1, remaining),
                )
            except Exception as error:
                raise RuntimeError(
                    "Hybrid transition precompute service failed: {}".format(error)
                )
            if not response.success:
                raise RuntimeError(response.message or
                                   "Hybrid transition precompute was rejected")
            for result in response.results:
                key = (int(result.candidate_index),
                       int(result.transition_index))
                if key not in pending:
                    continue
                if result.outcome == HybridTransitionResult.OUTCOME_SUCCESS:
                    paths[key] = copy.deepcopy(result.path)
                    results[key] = copy.deepcopy(result)
                    pending.pop(key, None)
                elif result.outcome in (
                        HybridTransitionResult.OUTCOME_NO_PATH,
                        HybridTransitionResult.OUTCOME_INVALID):
                    invalid_candidates[key[0]] = result.reason or (
                        "Hybrid A* proved that transition {} has no path".format(
                            key[1]
                        )
                    )
            for key in list(pending):
                if key[0] in invalid_candidates:
                    pending.pop(key, None)
            if pending:
                retry_deadline = min(
                    deadline,
                    time.monotonic() + self.hybrid_precompute_retry_sec,
                )
                while time.monotonic() < retry_deadline and not rospy.is_shutdown():
                    with self.lock:
                        if self.cancel_requested:
                            raise RuntimeError(
                                "Hybrid transition precompute was canceled"
                            )
                    self._lifecycle_wait(
                        min(0.1, retry_deadline - time.monotonic())
                    )

        for candidate_index, _ in enumerate(candidates):
            if any(key[0] == candidate_index for key in pending):
                invalid_candidates[candidate_index] = (
                    "Hybrid A* transition precompute timed out after {:.0f}s"
                ).format(self.hybrid_precompute_timeout_sec)

        feasible = []
        for candidate_index, candidate in enumerate(candidates):
            if candidate_index in invalid_candidates:
                rospy.logwarn(
                    "coverage route candidate %d rejected: %s",
                    candidate_index + 1,
                    invalid_candidates[candidate_index],
                )
                continue
            transition_paths = [None] * len(candidate.swaths)
            proxy_time = 0.0
            actual_time = 0.0
            reverse_transitions = 0
            expansions = 0
            for transition_index in range(len(candidate.swaths)):
                following = candidate.swaths[transition_index]
                if transition_index == 0:
                    transition_start = current
                    transition_start_yaw = current_yaw
                else:
                    previous = candidate.swaths[transition_index - 1]
                    transition_start = previous.end
                    transition_start_yaw = self._swath_yaw(previous)
                proxy, _ = estimate_transition_time(
                    transition_start,
                    transition_start_yaw,
                    following.start,
                    self._swath_yaw(following),
                    self.minimum_turning_radius,
                    time_parameters,
                    connector_distance=planner.connector_distance,
                )
                key = (candidate_index, transition_index)
                result = results[key]
                proxy_time += proxy
                actual_time += (
                    float(result.estimated_time_sec) +
                    time_parameters.segment_handoff_penalty_sec
                )
                reverse_transitions += int(result.reverse_distance_m > 1.0e-6)
                expansions += int(result.expansions)
                transition_paths[transition_index] = paths[key]
            candidate.estimated_total_time_sec += actual_time - proxy_time
            candidate.estimated_transit_time_sec += actual_time - proxy_time
            candidate.estimated_reverse_transitions = reverse_transitions
            candidate.score = candidate.estimated_total_time_sec
            candidate.transition_paths = transition_paths
            candidate.hybrid_precompute_expansions = expansions
            candidate.alternative_plans = []
            feasible.append(candidate)

        if not feasible:
            reasons = "; ".join(sorted(set(invalid_candidates.values())))
            raise ValueError(
                "no complete swath route has feasible Hybrid transitions: {}".format(
                    reasons or "no candidate completed"
                )
            )
        selected = min(feasible, key=lambda candidate: (
            round(candidate.unreachable_area, 6),
            candidate.estimated_total_time_sec,
            len(candidate.swaths),
            candidate.angle,
        ))
        rospy.loginfo(
            "coverage selected Hybrid-rescored route: %.1fs, %d swaths, "
            "%d precompute expansions",
            selected.estimated_total_time_sec,
            len(selected.swaths),
            selected.hybrid_precompute_expansions,
        )
        return selected

    def _replan_remaining_hybrid_transition(self, segment):
        """Replan from the live pose to the final swath entry with both gears.

        A constant-gear action may become blocked or overshoot its cusp.  Its
        old intermediate cusp is no longer the correct recovery goal.  Reuse
        the full Hybrid service to target the actual next swath entrance, then
        split the newly selected path at its new cusps before execution.
        """
        if segment.get("type") != "transit":
            return None, "only Hybrid transit segments can be replanned"
        transition_goal = segment.get("transition_goal")
        transition_goal_yaw = segment.get("transition_goal_yaw")
        if transition_goal is None or transition_goal_yaw is None:
            return None, "transition has no final swath-entry metadata"
        current_pose = self._current_pose()
        if current_pose is None:
            return None, "map to base_link transform is unavailable"
        current, current_yaw = current_pose
        with self.lock:
            plan_id = self.plan_id
            if self.cancel_requested or not self.active:
                return None, "coverage transition replan was canceled"
        try:
            rospy.wait_for_service(
                self.hybrid_precompute_service_name, timeout=0.5
            )
        except Exception as error:
            return None, "Hybrid recovery service is unavailable: {}".format(
                error
            )

        request = HybridTransitionRequest()
        request.candidate_index = 0
        request.transition_index = int(segment.get("swath_index", 0))
        request.start = self._pose(current, current_yaw)
        request.goal = self._pose(transition_goal, float(transition_goal_yaw))
        request.accept_goal_region = True
        try:
            response = self.hybrid_precompute_client(
                plan_id=plan_id,
                transitions=[request],
                # This is deliberately the task-level profile, not the old
                # constant-gear subsegment override.  The new complete path is
                # split again before any move_base goal is submitted.
                transit_profile=self._entry_goal_region_planning_profile(),
                total_timeout_sec=min(
                    float(self.hybrid_precompute_timeout_sec),
                    float(self.hybrid_recovery_timeout_sec),
                ),
            )
        except Exception as error:
            return None, "Hybrid recovery service failed: {}".format(error)
        with self.lock:
            if (
                self.cancel_requested
                or not self.active
                or self.plan_id != plan_id
            ):
                return None, "coverage transition replan was superseded"
        if not response.success:
            return None, response.message or "Hybrid recovery was rejected"
        if len(response.results) != 1:
            return None, "Hybrid recovery returned an unexpected result count"
        result = response.results[0]
        if result.outcome != HybridTransitionResult.OUTCOME_SUCCESS:
            return None, result.reason or (
                "Hybrid recovery did not find a complete path"
            )
        try:
            planned_goal = self._hybrid_path_point(result.planned_goal)
            planned_goal_yaw = self._hybrid_path_pose_yaw(
                result.planned_goal
            )
            path_generation = self._allocate_hybrid_path_generation(plan_id)
            replanned = self._hybrid_transition_segments(
                result.path,
                int(segment.get("swath_index", 0)),
                current,
                planned_goal,
                planned_goal_yaw,
                final_transition_goal=transition_goal,
                final_transition_goal_yaw=float(transition_goal_yaw),
                path_generation=path_generation,
            )
            replanned[-1]["entry_goal_region"] = True
            replanned[-1]["entry_handoff_goal"] = Point(
                float(transition_goal.x), float(transition_goal.y)
            )
            replanned[-1]["entry_handoff_yaw"] = float(
                transition_goal_yaw
            )
        except ValueError as error:
            return None, "Hybrid recovery path is invalid: {}".format(error)
        rospy.loginfo(
            "coverage replanned complete remaining Hybrid transition: "
            "parts=%d expansions=%d cost=%.2fs reverse=%.2fm switches=%d",
            len(replanned),
            int(result.expansions),
            float(result.estimated_time_sec),
            float(result.reverse_distance_m),
            int(result.direction_changes),
        )
        return replanned, "complete remaining Hybrid transition replanned"

    def _plan_next_rolling_hybrid_chunk(self, segment):
        """Plan from the live pose to the next swath entry.

        ``direct_hybrid_to_final_goal`` deliberately bypasses the historical
        Navfn rolling-waypoint layer.  Nearby line-to-line connectors are then
        owned end-to-end by one Hybrid A* search.  The legacy rolling mode is
        retained only as a reproducible control architecture.
        """
        transition_goal = segment.get("transition_goal")
        transition_goal_yaw = segment.get("transition_goal_yaw")
        if transition_goal is None or transition_goal_yaw is None:
            return None, False, "rolling transition has no final goal metadata"
        current_pose = self._current_pose()
        if current_pose is None:
            return None, False, "map to base_link transform is unavailable"
        current, current_yaw = current_pose
        with self.lock:
            plan_id = self.plan_id
            if self.cancel_requested or not self.active:
                return None, False, "rolling Hybrid transition was canceled"
        try:
            rospy.wait_for_service(
                self.hybrid_precompute_service_name, timeout=0.5
            )
        except Exception as error:
            return None, False, (
                "rolling Hybrid service is unavailable: {}"
            ).format(error)

        request = HybridTransitionRequest()
        request.candidate_index = 0
        request.transition_index = int(segment.get("swath_index", 0))
        request.start = self._pose(current, current_yaw)
        request.goal = self._pose(
            transition_goal, float(transition_goal_yaw)
        )
        direct_to_final = (
            bool(getattr(self, "direct_hybrid_to_final_goal", False))
            and not bool(segment.get("force_rolling_topology", False))
        )
        request.rolling = not direct_to_final
        request.rolling_horizon_m = float(self.hybrid_rolling_horizon)
        entry_alignment = bool(segment.get("entry_recovery", False))
        # For the direct line-to-line architecture the true target is the
        # sweep-entry pose region, not its mathematical centre. Intermediate
        # legacy rolling waypoints remain exact because they are not hand-off
        # targets for a sweep.
        request.accept_goal_region = direct_to_final
        transit_profile = (
            self._entry_recovery_planning_profile()
            if entry_alignment
            else self._entry_goal_region_planning_profile()
            if direct_to_final
            else self._transit_profile()
        )
        try:
            response = self.hybrid_precompute_client(
                plan_id=plan_id,
                transitions=[request],
                transit_profile=transit_profile,
                total_timeout_sec=float(self.hybrid_rolling_plan_timeout),
            )
        except Exception as error:
            return None, False, "live Hybrid service failed: {}".format(
                error
            )
        with self.lock:
            if (
                self.cancel_requested
                or not self.active
                or self.plan_id != plan_id
            ):
                return None, False, "rolling Hybrid plan was superseded"
        if not response.success:
            return None, False, response.message or (
                "rolling Hybrid request was rejected"
            )
        if len(response.results) != 1:
            return None, False, (
                "live Hybrid service returned an unexpected result count"
            )
        result = response.results[0]
        if result.outcome != HybridTransitionResult.OUTCOME_SUCCESS:
            return None, False, result.reason or (
                "live Hybrid planner did not find a path"
            )
        path_length = self._hybrid_path_length(result.path)
        reverse_distance = float(result.reverse_distance_m)
        forward_distance = max(0.0, path_length - reverse_distance)
        # The final Hybrid goal is already a 0.30 m / 20 degree region and the
        # sweep hand-off independently checks the live 0.40 m / 25 degree
        # pose.  Rejecting every recovery with more than 1.20 m of forward
        # travel therefore discarded valid straight and ordinary arc entries,
        # including the path observed on the real robot.  Keep only the local
        # recovery envelope here; collision, unknown-space, curvature and goal
        # region validity remain enforced by Hybrid A* and the hand-off gate.
        if entry_alignment and path_length > float(getattr(
                self, "sweep_entry_recovery_max_path_length", 4.00
        )) + 1.0e-6:
            return None, False, (
                "entry-alignment Hybrid path exceeds the local recovery "
                "envelope: total {:.2f}m, forward {:.2f}m, reverse {:.2f}m"
            ).format(path_length, forward_distance, reverse_distance)
        try:
            planned_goal = self._hybrid_path_point(result.planned_goal)
            planned_goal_yaw = self._hybrid_path_pose_yaw(
                result.planned_goal
            )
            path_generation = self._allocate_hybrid_path_generation(plan_id)
            chunk_segments = self._hybrid_transition_segments(
                result.path,
                int(segment.get("swath_index", 0)),
                current,
                planned_goal,
                planned_goal_yaw,
                final_transition_goal=transition_goal,
                final_transition_goal_yaw=float(transition_goal_yaw),
                path_generation=path_generation,
            )
            for part_index, part in enumerate(chunk_segments):
                final_entry_part = (
                    bool(result.reaches_final_goal)
                    and part_index + 1 == len(chunk_segments)
                )
                part["entry_goal_region"] = final_entry_part
                if final_entry_part:
                    # The lattice endpoint lies in a deliberately smaller
                    # inner region, but the authoritative hand-off remains the
                    # original sweep entrance.  Completion must stop as soon
                    # as that outer contract is physically satisfied instead
                    # of reversing past it to chase an internal sample pose.
                    part["entry_handoff_goal"] = Point(
                        float(transition_goal.x), float(transition_goal.y)
                    )
                    part["entry_handoff_yaw"] = float(transition_goal_yaw)
        except ValueError as error:
            return None, False, "rolling Hybrid path is invalid: {}".format(
                error
            )
        rospy.loginfo(
            "coverage planned %s Hybrid connector: %.2fm, parts=%d, "
            "final=%s, expansions=%d, cost=%.2fs, reverse=%.2fm, switches=%d",
            "direct" if direct_to_final else "rolling",
            path_length,
            len(chunk_segments),
            "yes" if result.reaches_final_goal else "no",
            int(result.expansions),
            float(result.estimated_time_sec),
            float(result.reverse_distance_m),
            int(result.direction_changes),
        )
        return chunk_segments, bool(result.reaches_final_goal), "planned"

    def _execute_rolling_hybrid_transition(self, segment, segment_index):
        """Execute one logical connector as freshly planned bounded chunks."""
        final_goal = segment.get("transition_goal")
        final_yaw = segment.get("transition_goal_yaw")
        if final_goal is None or final_yaw is None:
            return "failed"
        for chunk_index in range(int(self.hybrid_rolling_max_chunks)):
            current_pose = self._current_pose()
            if current_pose is None:
                with self.lock:
                    self.detail = "map to base_link transform is unavailable"
                return "failed"
            current, current_yaw = current_pose
            entry_ready, entry_geometry = self._transition_entry_ready(
                current, current_yaw, final_goal, final_yaw
            )
            if entry_ready:
                if self._wait_for_transition_stop():
                    return "succeeded"
                with self.lock:
                    self.detail = (
                        "vehicle entered the sweep-entry region but did not "
                        "physically stop before the hand-off"
                    )
                return "blocked"
            before = Point(float(current.x), float(current.y))
            with self.lock:
                self.state = "TRANSITING"
                architecture = (
                    "direct Hybrid path"
                    if (
                        bool(getattr(
                            self, "direct_hybrid_to_final_goal", False
                        ))
                        and not bool(segment.get(
                            "force_rolling_topology", False
                        ))
                    )
                    else "Navfn-topology rolling Hybrid chunk {}".format(
                        chunk_index + 1
                    )
                )
                self.detail = (
                    "planning {} toward swath {}"
                ).format(
                    architecture,
                    int(segment.get("swath_index", 0)) + 1,
                )
            parts, reaches_final, detail = (
                self._plan_next_rolling_hybrid_chunk(segment)
            )
            if not parts:
                with self.lock:
                    self.detail = detail
                rospy.logwarn("coverage rolling Hybrid planning failed: %s", detail)
                return "blocked"
            self._publish_hybrid_transition_path(
                parts[0].get("transition_path")
            )
            replan_from_measured_cusp = False
            for part_index, part in enumerate(parts):
                outcome = self._execute_segment(part, segment_index)
                if outcome != "succeeded":
                    late_pose = self._current_pose()
                    if late_pose is not None and math.hypot(
                            float(final_goal.x) - float(late_pose[0].x),
                            float(final_goal.y) - float(late_pose[0].y),
                    ) <= float(getattr(
                            self, "sweep_entry_alignment_radius", 2.00
                    )):
                        # A disturbance can arrive after the final fixed-gear
                        # action was armed but before its terminal callback.
                        # Promote the next retry to the bounded entry-acquisition
                        # profile instead of replanning a full forward loop.
                        segment["entry_recovery"] = True
                        rospy.logwarn(
                            "coverage rolling transition failed near its swath "
                            "entry; the retry will acquire the entry goal region"
                        )
                    return outcome
                if part_index + 1 < len(parts):
                    # Every completed fixed-gear action is already a measured
                    # stop.  It may have entered the final sweep hand-off
                    # basin even when the planner's internal goal lies after
                    # another cusp.  Accept the business-level final region
                    # before trying to join or regenerate that stale suffix;
                    # otherwise a harmless 0.30 m terminal allowance can
                    # create F-R micro-manoeuvres inside an already acceptable
                    # entrance pose.
                    live_pose = self._current_pose()
                    if live_pose is not None:
                        entry_ready, entry_geometry = (
                            self._transition_entry_ready(
                                live_pose[0], live_pose[1],
                                final_goal, final_yaw,
                            )
                        )
                        if entry_ready:
                            if not self._wait_for_transition_stop():
                                with self.lock:
                                    self.detail = (
                                        "vehicle entered the sweep-entry "
                                        "region after a fixed-gear part but "
                                        "did not physically stop"
                                    )
                                return "blocked"
                            rospy.loginfo(
                                "coverage accepted final sweep-entry region "
                                "after fixed-gear part %d/%d: position %.3fm, "
                                "cross %.3fm, yaw %.1fdeg; discarding %d "
                                "remaining cusp part(s)",
                                part_index + 1,
                                len(parts),
                                entry_geometry["position_error"],
                                entry_geometry["lateral_error"],
                                math.degrees(entry_geometry["yaw_error"]),
                                len(parts) - part_index - 1,
                            )
                            return "succeeded"
                    # A cusp is not itself an abnormal event. Continue the
                    # cached suffix when the measured stopped pose admits a
                    # same-gear tangent bridge at the configured radius. Only
                    # an unjoinable cusp invalidates the suffix and triggers a
                    # complete live-pose-to-final-entry replan.
                    joined = False
                    join_detail = "map to base_link transform is unavailable"
                    if live_pose is not None:
                        joined, join_detail = (
                            self._rebase_joinable_hybrid_part(
                                parts[part_index + 1],
                                live_pose[0], live_pose[1],
                            )
                        )
                    if joined:
                        rospy.loginfo(
                            "coverage measured cusp %d/%d kept its cached "
                            "suffix: %s",
                            part_index + 1,
                            len(parts) - 1,
                            join_detail,
                        )
                        continue
                    rospy.logwarn(
                        "coverage measured cusp %d/%d invalidated its cached "
                        "suffix (%s); replanning the complete remaining "
                        "transition to swath %d",
                        part_index + 1,
                        len(parts) - 1,
                        join_detail,
                        int(segment.get("swath_index", 0)) + 1,
                    )
                    replan_from_measured_cusp = True
                    break
            if replan_from_measured_cusp:
                continue
            if reaches_final:
                return "succeeded"
            after_pose = self._current_pose()
            if after_pose is None:
                return "failed"
            moved = math.hypot(
                after_pose[0].x - before.x, after_pose[0].y - before.y
            )
            if moved < self.hybrid_no_progress_distance:
                with self.lock:
                    self.detail = (
                        "rolling Hybrid chunk completed without measurable "
                        "forward or reverse progress"
                    )
                return "blocked"
        with self.lock:
            self.detail = (
                "rolling Hybrid connector exceeded its {}-chunk safety bound"
            ).format(self.hybrid_rolling_max_chunks)
        return "blocked"

    def _transition_entry_ready(self, point, yaw, final_goal, final_yaw):
        """Evaluate the authoritative final swath-entry hand-off region."""
        position_error = math.hypot(
            float(final_goal.x) - float(point.x),
            float(final_goal.y) - float(point.y),
        )
        yaw_error = abs(math.atan2(
            math.sin(float(final_yaw) - float(yaw)),
            math.cos(float(final_yaw) - float(yaw)),
        ))
        lateral_error = abs(
            -math.sin(float(final_yaw)) *
                (float(point.x) - float(final_goal.x))
            + math.cos(float(final_yaw)) *
                (float(point.y) - float(final_goal.y))
        )
        geometry = {
            "position_error": position_error,
            "lateral_error": lateral_error,
            "yaw_error": yaw_error,
        }
        return (
            position_error <= float(getattr(
                self, "entry_position_tolerance", 0.40
            ))
            and lateral_error <= float(getattr(
                self, "entry_position_tolerance", 0.40
            ))
            and yaw_error <= float(getattr(
                self, "entry_yaw_tolerance", 0.436332
            )),
            geometry,
        )

    def _execute_sweep_entry_recovery(self, sweep, segment_index):
        """Reacquire a disturbed swath entrance through the Hybrid pipeline."""
        if not bool(getattr(self, "hierarchical_hybrid_on_demand", False)):
            with self.lock:
                self.detail = (
                    "sweep entry is outside its acquisition basin and this "
                    "architecture has no rolling Hybrid recovery"
                )
            return "blocked"
        current_pose = self._current_pose()
        if current_pose is None:
            with self.lock:
                self.detail = (
                    "map to base_link transform is unavailable for sweep "
                    "entry recovery"
                )
            return "failed"
        current = current_pose[0]
        recovery = {
            "type": "rolling_transit",
            "swath_index": int(sweep.get("swath_index", 0)),
            "start": Point(float(current.x), float(current.y)),
            "end": Point(float(sweep["start"].x), float(sweep["start"].y)),
            "yaw": float(sweep["yaw"]),
            "length": math.hypot(
                float(sweep["start"].x) - float(current.x),
                float(sweep["start"].y) - float(current.y),
            ),
            "path": None,
            "gear": 0,
            "entry_recovery": True,
            "transition_goal": Point(
                float(sweep["start"].x), float(sweep["start"].y)
            ),
            "transition_goal_yaw": float(sweep["yaw"]),
        }
        with self.lock:
            self.state = "TRANSITING"
            self.detail = (
                "recovering disturbed entry for sweep {} with rolling Hybrid; "
                "the sweep itself has not started"
            ).format(segment_index + 1)
        rospy.logwarn(
            "coverage is replanning disturbed sweep %d entry from the live "
            "pose with rolling Hybrid A*",
            segment_index + 1,
        )
        return self._execute_rolling_hybrid_transition(
            recovery, segment_index
        )

    def _sweep_entry_handoff_geometry(self, sweep, point, yaw):
        """Evaluate the live pose before a sweep is allowed to start.

        A rolling connector and the following sweep share one explicit
        hand-off contract. Keeping this check outside `_execute_segment`
        prevents an out-of-tolerance pose from briefly acquiring SWEEPING
        state or arming the forward-only sweep controller.
        """
        geometry = self._sweep_completion_geometry(sweep, point, yaw)
        if geometry is None:
            return None
        start = sweep["start"]
        start_distance = math.hypot(
            float(point.x) - float(start.x),
            float(point.y) - float(start.y),
        )
        position_tolerance = float(getattr(
            self, "entry_position_tolerance", 0.40
        ))
        cross_tolerance = float(getattr(
            self, "entry_position_tolerance", 0.40
        ))
        yaw_tolerance = float(getattr(
            self, "entry_yaw_tolerance", 0.436332
        ))
        geometry.update({
            "start_distance": start_distance,
            "position_tolerance": position_tolerance,
            "cross_tolerance": cross_tolerance,
            "yaw_tolerance": yaw_tolerance,
            "ready": (
                start_distance <= position_tolerance
                and geometry["cross_track"] <= cross_tolerance
                and geometry["heading_error"] <= yaw_tolerance
            ),
        })
        return geometry

    def _prepare_sweep_entry(self, sweep, segment_index):
        """Keep TRANSITING until the swath entrance is physically acquired."""
        current_pose = self._current_pose()
        if current_pose is None:
            with self.lock:
                self.state = "TRANSITING"
                self.current_segment = segment_index + 1
                self.detail = (
                    "cannot validate the next sweep entrance because map to "
                    "base_link is unavailable"
                )
            return "failed"
        geometry = self._sweep_entry_handoff_geometry(
            sweep, current_pose[0], current_pose[1]
        )
        if geometry is not None and geometry["ready"]:
            return "ready"
        with self.lock:
            self.state = "TRANSITING"
            self.current_segment = segment_index + 1
            self.detail = (
                "sweep {} entrance is outside the hand-off tolerance; "
                "continuing rolling Hybrid alignment before sweep"
            ).format(segment_index + 1)
        if geometry is not None:
            rospy.logwarn(
                "coverage sweep %d has not acquired its entrance: position "
                "%.3f/%.3fm, cross-track %.3f/%.3fm, heading %.1f/%.1fdeg; "
                "remaining TRANSITING for rolling Hybrid alignment",
                segment_index + 1,
                geometry["start_distance"],
                geometry["position_tolerance"],
                geometry["cross_track"],
                geometry["cross_tolerance"],
                math.degrees(geometry["heading_error"]),
                math.degrees(geometry["yaw_tolerance"]),
            )
        return "entry-recovery"

    def _planned_path(self, route, current=None, transition_paths=None):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for swath_index, swath in enumerate(route):
            cached = (
                transition_paths[swath_index]
                if transition_paths and swath_index < len(transition_paths)
                else None
            )
            if cached is not None and cached.poses:
                for pose in cached.poses:
                    copied = copy.deepcopy(pose)
                    copied.header.stamp = path.header.stamp
                    path.poses.append(copied)
            else:
                if swath_index > 0:
                    previous = route[swath_index - 1]
                    transition_start = previous.end
                elif current is not None:
                    transition_start = current
                else:
                    transition_start = None
                if transition_start is not None:
                    transit_yaw = math.atan2(
                        swath.start.y - transition_start.y,
                        swath.start.x - transition_start.x,
                    )
                    for point in sample_path(
                            transition_start, swath.start,
                            self.path_sample_spacing):
                        path.poses.append(self._pose(
                            point, transit_yaw, path.header.stamp
                        ))
            sweep_yaw = math.atan2(swath.end.y - swath.start.y,
                                   swath.end.x - swath.start.x)
            for point in sample_path(swath.start, swath.end, self.path_sample_spacing):
                path.poses.append(self._pose(point, sweep_yaw, path.header.stamp))
        return path

    def _publish_markers(self, route, region):
        array = MarkerArray()
        outline = Marker()
        outline.header.frame_id = "map"
        outline.header.stamp = rospy.Time.now()
        outline.ns = "coverage_region"
        outline.id = 0
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.08
        outline.color.r = 1.0
        outline.color.g = 0.75
        outline.color.b = 0.1
        outline.color.a = 1.0
        for point in region.polygon.points:
            outline.points.append(RosPoint(x=point.x, y=point.y, z=0.05))
        if outline.points:
            outline.points.append(copy.deepcopy(outline.points[0]))
        array.markers.append(outline)
        lines = Marker()
        lines.header = outline.header
        lines.ns = "coverage_swaths"
        lines.id = 1
        lines.type = Marker.LINE_LIST
        lines.action = Marker.ADD
        lines.pose.orientation.w = 1.0
        lines.scale.x = 0.06
        lines.color.r = 0.1
        lines.color.g = 0.85
        lines.color.b = 1.0
        lines.color.a = 1.0
        for swath in route:
            lines.points.append(RosPoint(x=swath.start.x, y=swath.start.y, z=0.08))
            lines.points.append(RosPoint(x=swath.end.x, y=swath.end.y, z=0.08))
        array.markers.append(lines)
        self.marker_pub.publish(array)

    def _clear_visualizations(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        self.path_pub.publish(path)
        self._publish_hybrid_transition_path()
        self.executed_path_pub.publish(copy.deepcopy(path))
        region = PolygonStamped()
        region.header.frame_id = "map"
        region.header.stamp = path.header.stamp
        self.region_pub.publish(region)
        markers = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def _reset_execution_locked(self, clear_progress=True):
        if clear_progress:
            self.current_segment = 0
            self.total_segments = 0
            self.blocked_segments = []
        self.traversed_distance = 0.0
        self.covered_cells = set()
        self.last_tracked_point = None
        self.executed_path = Path()
        self.executed_path.header.frame_id = "map"

    def _discard_plan_locked(self, clear_progress=True):
        """Invalidate one mission generation while the lifecycle lock is held."""
        self.plan_pending = False
        self.plan_token = ""
        self.start_pending = False
        self.start_token = ""
        self.plan = None
        self.plan_id = ""
        self.plan_map_digest = ""
        self.plan_time_parameters = None
        self.region = PolygonStamped()
        self.region.header.frame_id = "map"
        self._reset_execution_locked(clear_progress=clear_progress)
        self.kinematics_verified = False
        self.kinematics_detail = (
            "waiting for task-start VCU and TEB verification"
        )
        self.worker = None

    def _clear_batch_display_identity_locked(self):
        """Detach a new standalone mission from the last terminal batch.

        Batch request records remain available for idempotent operation-ID
        replay/cancel handling.  Only the public status identity and counters
        are cleared, so a later standalone retained-owner failure cannot be
        mistaken for the previous batch by a recovering client.
        """
        self.batch_id = ""
        self.batch_token = ""
        self.batch_start_request_id = ""
        self.batch_map_digest = ""
        self.batch_cancel_requested = False
        self.batch_skip_requested = False
        self.batch_abort_detail = ""
        self.batch_phase = "IDLE"
        self.batch_region_token = ""
        self.batch_region_outcome = ""
        self.batch_current_is_last = False
        self.batch_regions = []
        self.batch_current_index = 0
        self.batch_total_regions = 0
        self.batch_completed_regions = 0
        self.batch_partial_regions = 0
        self.batch_skipped_regions = 0
        self.current_region_id = ""
        self.current_region_name = ""
        self.last_region_id = ""
        self.last_region_name = ""
        self.last_region_state = ""

    def _clear_never_started_batch_identity_locked(self, batch_id):
        """Clear a safely settled reservation without touching its replay record."""
        if (
            self.batch_id != batch_id
            or self.batch_active
            or self.batch_token
            or self.batch_start_request_id not in ("", batch_id)
        ):
            return False
        self._clear_batch_display_identity_locked()
        return True

    def _start_service(self, request):
        try:
            time_parameters = self._time_parameters_from_request(request)
        except ValueError as error:
            return StartCoverageResponse(
                False,
                str(error),
            )
        requested_speed = time_parameters.max_forward_speed_mps

        with self.lock:
            if getattr(self, "navigation_profile_update_pending", False):
                return StartCoverageResponse(
                    False, "navigation profile update is still in progress"
                )
            if self._retained_cleanup_in_progress_locked():
                return StartCoverageResponse(
                    False,
                    "a retained coverage cleanup is still in progress",
                )
            if self.active or self.batch_active:
                return StartCoverageResponse(
                    False, "coverage or a coverage batch is already active"
                )
            if self.start_pending:
                return StartCoverageResponse(False, "coverage start is already preparing")
            if self.plan_pending:
                return StartCoverageResponse(False, "coverage planning is still in progress")
            if self.plan is None or request.plan_id != self.plan_id:
                return StartCoverageResponse(False, "coverage plan id is missing or stale")
            if self.plan_map_digest != self.map_digest:
                return StartCoverageResponse(False, "static map changed after planning")
            planned_parameters = getattr(self, "plan_time_parameters", None)
            if (planned_parameters is not None and
                    not self._time_parameters_match(
                        time_parameters, planned_parameters)):
                return StartCoverageResponse(
                    False,
                    "planning parameters changed after preview; regenerate the coverage path",
                )
            token = uuid.uuid4().hex
            owner_token = self._new_navigation_owner_token()
            plan_id = self.plan_id
            map_digest = self.map_digest
            grid = self.grid
            region = copy.deepcopy(self.region)
            operation_width = self.operation_width
            overlap_ratio = self.overlap_ratio
            self._clear_batch_display_identity_locked()
            self.start_pending = True
            self.start_token = token
            self.cancel_requested = False
            self.state = "PREPARING"
            self.detail = "checking live safety gates and current-map coverage entry"
        self._publish_status()

        with self.lock:
            initial_checks_ok = self._start_prechecks_locked(
                requested_speed, require_kinematics=False
            )
            initial_failure = self.detail
        if not initial_checks_ok:
            return self._finish_start_failure(token, initial_failure)

        if not self._start_external_prechecks(token):
            with self.lock:
                external_failure = self.detail
            return self._finish_start_failure(token, external_failure)

        current_pose = self._current_pose()
        if current_pose is None:
            return self._finish_start_failure(
                token, "map to base_link transform is unavailable"
            )
        reachable_seed, route_yaw = current_pose
        try:
            # Planning is allowed before localization so the operator can
            # preview a region.  Starting is stricter: recompute against the
            # vehicle's current known-free connected component.  Crucially,
            # this expensive work runs without self.lock, so /scan and the
            # dual-lidar freshness callbacks cannot be starved by replanning.
            route_planner = self._planner(grid)
            replanned = self._plan_region_route(
                route_planner,
                self._points_from_region(region),
                operation_width,
                overlap_ratio,
                reachable_seed,
                reachable_seed,
                route_yaw,
                time_parameters,
            )
        except ValueError as error:
            return self._finish_start_failure(
                token, "coverage start replan failed: {}".format(error)
            )
        except Exception as error:
            rospy.logerr("coverage start replan raised an exception: %s", error)
            return self._finish_start_failure(
                token, "coverage start replan raised an exception: {}".format(error)
            )

        # Use the freshest pose as the advertised path/executor origin.  The
        # final state check below still requires the vehicle to be stopped;
        # route optimization used the pose captured immediately before this
        # unlocked planning call.
        current_pose = self._current_pose()
        if current_pose is None:
            return self._finish_start_failure(
                token, "map to base_link transform disappeared during replanning"
            )
        current, current_yaw = current_pose
        try:
            replanned = self._precompute_hybrid_candidates(
                replanned,
                current,
                current_yaw,
                time_parameters,
                route_planner,
                plan_id,
            )
        except (RuntimeError, ValueError) as error:
            return self._finish_start_failure(
                token, "coverage transition precompute failed: {}".format(error)
            )
        route = replanned.swaths
        transition_paths = getattr(replanned, "transition_paths", None)
        planned_path = self._planned_path(
            route, current, transition_paths=transition_paths
        )
        has_hybrid_paths = bool(
            transition_paths and any(
                path is not None for path in transition_paths
            )
        )
        worker_args = (
            (copy.deepcopy(route), current, False,
             copy.deepcopy(transition_paths))
            if has_hybrid_paths else (copy.deepcopy(route), current)
        )
        worker = threading.Thread(
            target=self._run_task,
            args=worker_args,
            daemon=True,
        )

        failure = ""
        with self.lock:
            if (not self.start_pending or self.start_token != token or
                    self.cancel_requested):
                failure = "coverage start was canceled or superseded"
            elif self.active or self.batch_active:
                failure = "coverage or a coverage batch became active during preparation"
        if failure:
            return self._finish_start_failure(token, failure)

        owner_state, owner_detail = self._resolve_navigation_owner_claim(
            owner_token, "coverage owner claim is pending"
        )
        if owner_state == "REJECTED":
            return self._finish_start_failure(
                token,
                "coverage could not claim navigation ownership: {}".format(
                    owner_detail
                ),
            )
        if owner_state != "READY":
            return self._retain_failed_navigation_owner_release(
                owner_token,
                "coverage navigation owner claim remained unknown: {}".format(
                    owner_detail
                ),
            )

        with self.lock:
            if (not self.start_pending or self.start_token != token or
                    self.cancel_requested):
                failure = "coverage start was canceled or superseded"
            elif (
                self.active
                or self.batch_active
                or self.plan is None
                or self.plan_id != plan_id
                or self.plan_map_digest != map_digest
                or self.map_digest != map_digest
            ):
                failure = "static map or coverage plan changed during start preparation"
            elif not self._start_prechecks_locked(
                    requested_speed, require_kinematics=True):
                failure = self.detail
            if not failure:
                self.start_pending = False
                self.start_token = ""
                self.cancel_requested = False
                self.plan = replanned
                self.plan_time_parameters = time_parameters
                self._apply_time_parameters_locked(time_parameters)
                self.cancel_requested = False
                self.manual_pause = False
                self.manual_pause_reason = ""
                self.avoidance_loss_paused = False
                self.chassis_fault_paused = False
                self.navigation_owner_token = owner_token
                self.navigation_owner_claimed = True
                self.navigation_owner_releasing = False
                self.active = True
                self._reset_execution_locked(clear_progress=True)
                self.total_segments = len(route) * 2
                self.state = "GOING_TO_START"
                self.detail = "coverage task accepted at {:.2f} m/s maximum".format(
                    self.task_max_speed
                )
                self.worker = worker
        if failure:
            released, release_detail = self._set_navigation_owner(
                False, owner_token
            )
            if not released:
                return self._retain_failed_navigation_owner_release(
                    owner_token,
                    "coverage start was not committed ({}) and owner release "
                    "failed: {}".format(failure, release_detail),
                )
            return self._finish_start_failure(token, failure)

        # Advertise mission ownership before the worker can submit its first
        # move_base goal.  Each segment also performs a synchronous planner
        # mode hand-off before the action goal is sent.
        self.active_pub.publish(Bool(data=True))
        self.path_pub.publish(planned_path)
        self._publish_markers(route, region)
        self._publish_status()
        try:
            worker.start()
        except RuntimeError as error:
            return self._abort_committed_start(
                owner_token,
                "coverage worker could not start: {}".format(error),
            )
        return StartCoverageResponse(
            True,
            "coverage task started at {:.2f} m/s maximum".format(requested_speed),
        )

    def _finish_start_failure(self, token, reason):
        with self.lock:
            if self.start_pending and self.start_token == token:
                self.start_pending = False
                self.start_token = ""
                self.cancel_requested = False
                if self.plan is not None and self.plan_map_digest == self.map_digest:
                    self.state = "READY"
                else:
                    self.state = "IDLE"
                self.detail = reason
            else:
                reason = "coverage start was canceled or superseded"
        rospy.logwarn("coverage start rejected: %s", reason)
        self._publish_status()
        return StartCoverageResponse(False, reason)

    def _retain_failed_navigation_owner_release(self, owner_token, reason):
        """Keep every local gate closed when a claimed token cannot release."""
        with self.lock:
            self.start_pending = False
            self.start_token = ""
            self.navigation_owner_token = owner_token
            self.navigation_owner_claimed = True
            self.navigation_owner_releasing = True
            self.active = True
            self.cancel_requested = True
            self.worker = None
            self.state = "FAILED"
            self.detail = reason
        self.active_pub.publish(Bool(data=True))
        self._publish_status()
        rospy.logerr(reason)
        return StartCoverageResponse(False, reason)

    def _abort_committed_start(self, owner_token, reason):
        with self.lock:
            if self.navigation_owner_token == owner_token:
                self.navigation_owner_releasing = True
            plan_id = self.plan_id
        goal_terminal_ok, goal_terminal_detail = (
            self._confirm_move_base_goal_terminal(cancel=True)
        )
        if not goal_terminal_ok:
            return self._retain_failed_navigation_owner_release(
                owner_token,
                "{}; move_base goal is not safely terminal: {}; navigation "
                "owner retained".format(reason, goal_terminal_detail),
            )
        off = EnforcedPath()
        off.header.frame_id = "map"
        off.plan_id = plan_id
        off.active = False
        handoff_ok = self._set_enforced_path(off, coverage_active=False)
        teb_restored = self._restore_teb()
        if not handoff_ok or not teb_restored:
            cleanup_detail = (
                "planner ownership could not be restored"
                if not handoff_ok else
                "TEB parameters could not be restored"
            )
            return self._retain_failed_navigation_owner_release(
                owner_token,
                "{}; {}; navigation owner retained".format(
                    reason, cleanup_detail
                ),
            )
        released, release_detail = self._set_navigation_owner(False, owner_token)
        if not released:
            return self._retain_failed_navigation_owner_release(
                owner_token,
                "{}; navigation owner release failed: {}".format(
                    reason, release_detail
                ),
            )
        with self.lock:
            if self.navigation_owner_token == owner_token:
                self.navigation_owner_token = ""
                self.navigation_owner_claimed = False
                self.navigation_owner_releasing = False
            self.active = False
            self.cancel_requested = False
            self.state = "FAILED"
            self.detail = reason
            self._discard_plan_locked(clear_progress=False)
            self._clear_visualizations()
            self.active_pub.publish(Bool(data=False))
            self._publish_status()
        rospy.logerr(reason)
        return StartCoverageResponse(False, reason)

    @staticmethod
    def _validate_batch_regions(regions):
        if not regions:
            return "coverage batch must contain at least one region"
        if len(regions) > 100:
            return "coverage batch must not contain more than 100 regions"
        seen_ids = set()
        for index, item in enumerate(regions):
            region_id = str(item.id)
            if not region_id.strip():
                return "coverage batch region {} has an empty id".format(index + 1)
            if len(region_id.encode("utf-8")) > 128:
                return "coverage batch region {} id is too long".format(index + 1)
            region_name = str(item.name)
            if not region_name.strip():
                return "coverage batch region {} has an empty name".format(index + 1)
            if len(region_name.encode("utf-8")) > 256:
                return "coverage batch region {} name is too long".format(index + 1)
            if region_id in seen_ids:
                return "coverage batch region id is duplicated: {}".format(region_id)
            seen_ids.add(region_id)
            if item.region.header.frame_id not in ("", "map"):
                return "coverage batch regions must use the map frame"
            points = item.region.polygon.points
            if len(points) < 3:
                return "coverage batch region {} needs at least three vertices".format(
                    index + 1
                )
            if len(points) > 4096:
                return "coverage batch region {} has too many vertices".format(
                    index + 1
                )
            for point in points:
                if not all(math.isfinite(float(value)) for value in (
                        point.x, point.y, point.z)):
                    return "coverage batch region {} has non-finite vertices".format(
                        index + 1
                    )
        return ""

    @staticmethod
    def _valid_batch_request_id(request_id):
        return re.fullmatch(
            r"coverage-batch-[0-9a-f]{32}", str(request_id)
        ) is not None

    @staticmethod
    def _batch_request_fingerprint(
            regions, operation_width, overlap_ratio, allow_reverse,
            requested_speed, map_digest, reverse_speed=0.30,
            max_angular_speed=0.60, linear_accel=1.00,
            angular_accel=0.50, direction_change_penalty=0.50,
            segment_handoff_penalty=0.50, transit_replan_period=1.00):
        """Hash every semantically relevant field of an immutable request."""
        payload = {
            "operation_width": float(operation_width).hex(),
            "overlap_ratio": float(overlap_ratio).hex(),
            "allow_reverse": bool(allow_reverse),
            "requested_speed": float(requested_speed).hex(),
            "reverse_speed": float(reverse_speed).hex(),
            "max_angular_speed": float(max_angular_speed).hex(),
            "linear_accel": float(linear_accel).hex(),
            "angular_accel": float(angular_accel).hex(),
            "direction_change_penalty": float(
                direction_change_penalty
            ).hex(),
            "segment_handoff_penalty": float(
                segment_handoff_penalty
            ).hex(),
            "transit_replan_period": float(transit_replan_period).hex(),
            "map_digest": str(map_digest),
            "regions": [
                {
                    "id": str(item.id),
                    "name": str(item.name),
                    "frame_id": str(item.region.header.frame_id),
                    "points": [
                        [
                            float(point.x).hex(),
                            float(point.y).hex(),
                            float(point.z).hex(),
                        ]
                        for point in item.region.polygon.points
                    ],
                }
                for item in regions
            ],
        }
        encoded = json.dumps(
            payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    def _batch_request_records_locked(self):
        records = getattr(self, "batch_request_records", None)
        if records is None:
            records = {}
            self.batch_request_records = records
        return records

    def _retained_cleanup_in_progress_locked(self):
        cleanup_lock = getattr(self, "retained_cleanup_lock", None)
        return bool(cleanup_lock is not None and cleanup_lock.locked())

    def _retained_cleanup_mutex(self):
        with self.lock:
            cleanup_lock = getattr(self, "retained_cleanup_lock", None)
            if cleanup_lock is None:
                # Compatibility for old serialized fixtures/process state;
                # production instances create this in __init__.
                cleanup_lock = threading.Lock()
                self.retained_cleanup_lock = cleanup_lock
            return cleanup_lock

    @staticmethod
    def _batch_replay_response(response, request_id, record, fingerprint):
        response.batch_id = request_id
        recorded_fingerprint = record.get("fingerprint", "")
        if recorded_fingerprint and recorded_fingerprint != fingerprint:
            response.message = (
                "client_request_id was already used with a different payload"
            )
            return response
        state = record.get("state", "REJECTED")
        if record.get("accepted", False):
            response.accepted = True
            response.message = (
                "coverage batch request was already accepted ({})"
            ).format(state.lower())
            return response
        if state == "STARTING":
            response.message = (
                "coverage batch request is still starting; retry the same id"
            )
        else:
            response.message = record.get(
                "message", "coverage batch request was not accepted"
            )
        return response

    def _start_batch_service(self, request):
        response = StartCoverageBatchResponse()
        batch_id = str(getattr(request, "client_request_id", ""))
        response.batch_id = batch_id
        if not self._valid_batch_request_id(batch_id):
            response.message = (
                "client_request_id must match coverage-batch-[0-9a-f]{32}"
            )
            return response
        try:
            operation_width = float(request.operation_width_m)
            overlap_ratio = float(request.overlap_ratio)
        except (TypeError, ValueError, OverflowError):
            response.message = "coverage batch parameters must be finite numbers"
            return response
        try:
            time_parameters = self._time_parameters_from_request(request)
        except ValueError as error:
            response.message = str(error)
            return response
        requested_speed = time_parameters.max_forward_speed_mps
        if not math.isfinite(operation_width) or not 0.30 <= operation_width <= 3.0:
            response.message = "operation width must be in [0.30, 3.00] m"
            return response
        if not math.isfinite(overlap_ratio) or not 0.0 <= overlap_ratio <= 0.5:
            response.message = "overlap ratio must be in [0.0, 0.5]"
            return response
        if not request.map_digest:
            response.message = "map digest is required"
            return response
        requested_regions = list(request.regions)
        validation_error = self._validate_batch_regions(requested_regions)
        if validation_error:
            response.message = validation_error
            return response
        regions = copy.deepcopy(requested_regions)
        fingerprint = self._batch_request_fingerprint(
            regions,
            operation_width,
            overlap_ratio,
            request.allow_reverse_transit,
            requested_speed,
            request.map_digest,
            reverse_speed=time_parameters.max_reverse_speed_mps,
            max_angular_speed=time_parameters.max_angular_speed_rps,
            linear_accel=time_parameters.linear_accel_mps2,
            angular_accel=time_parameters.angular_accel_rps2,
            direction_change_penalty=(
                time_parameters.direction_change_penalty_sec
            ),
            segment_handoff_penalty=(
                time_parameters.segment_handoff_penalty_sec
            ),
            transit_replan_period=(
                time_parameters.transit_replan_period_sec
            ),
        )

        token = uuid.uuid4().hex
        reservation_token = uuid.uuid4().hex
        owner_token = self._new_navigation_owner_token()
        with self.lock:
            records = self._batch_request_records_locked()
            existing = records.get(batch_id)
            if existing is not None:
                return self._batch_replay_response(
                    response, batch_id, existing, fingerprint
                )
            # Do not permanently reject a fresh operation ID merely because
            # an older retained cleanup is between external side effects and
            # its final local commit.  The client can safely retry this ID.
            if self._retained_cleanup_in_progress_locked():
                response.message = (
                    "a retained coverage cleanup is still in progress; "
                    "retry the same client_request_id"
                )
                return response
            if getattr(self, "navigation_profile_update_pending", False):
                response.message = (
                    "navigation profile update is still in progress; "
                    "retry the same client_request_id"
                )
                return response
            records[batch_id] = {
                "fingerprint": fingerprint,
                "state": "STARTING",
                "accepted": False,
                "message": "coverage batch request is starting",
            }
            if self.active or self.batch_active or self.batch_worker is not None:
                response.message = "coverage or a coverage batch is already active"
                records[batch_id].update(
                    state="REJECTED", message=response.message
                )
                return response
            if self.plan_pending or self.start_pending:
                response.message = "coverage planning or start preparation is in progress"
                records[batch_id].update(
                    state="REJECTED", message=response.message
                )
                return response
            if self.grid is None:
                response.message = "static map is not ready"
                records[batch_id].update(
                    state="REJECTED", message=response.message
                )
                return response
            if request.map_digest != self.map_digest:
                response.message = "map digest does not match the current static map"
                records[batch_id].update(
                    state="REJECTED", message=response.message
                )
                return response

            self.start_pending = True
            self.start_token = reservation_token
            self.cancel_requested = False
            self.batch_start_request_id = batch_id
            # Expose the exact operation while its owner claim is pending so
            # broad/UI cancellation cannot mistake active=false for quiescence.
            self.batch_id = batch_id
            self.state = "PREPARING"
            self.detail = "claiming navigation ownership for coverage batch"
        self._publish_status()

        owner_state, owner_detail = self._resolve_navigation_owner_claim(
            owner_token, "coverage batch owner claim is pending"
        )
        if owner_state != "READY":
            if owner_state == "UNKNOWN":
                reason = (
                    "coverage batch owner claim remained unknown: {}"
                ).format(owner_detail)
                with self.lock:
                    self.start_pending = False
                    self.start_token = ""
                    self.batch_start_request_id = ""
                    self.batch_id = batch_id
                    self.batch_phase = "FINALIZING"
                    self.navigation_owner_token = owner_token
                    self.navigation_owner_claimed = True
                    self.navigation_owner_releasing = True
                    self.active = True
                    self.cancel_requested = True
                    self.state = "FAILED"
                    self.detail = reason
                    record = self._batch_request_records_locked().get(
                        batch_id, {}
                    )
                    record.update(
                        state="FAILED_RETAINED",
                        message=reason,
                        owner_token=owner_token,
                    )
                self.active_pub.publish(Bool(data=True))
                self._publish_status()
                rospy.logerr(reason)
                response.message = reason
                return response
            with self.lock:
                record = self._batch_request_records_locked().get(batch_id, {})
                canceled = record.get("state") in (
                    "CANCEL_PENDING_BEFORE_START",
                    "CANCELED_BEFORE_START",
                )
                if (self.start_pending and
                        self.start_token == reservation_token):
                    self.start_pending = False
                    self.start_token = ""
                    self.batch_start_request_id = ""
                if canceled:
                    record.update(
                        state="CANCELED_BEFORE_START",
                        message="coverage batch was canceled before it started",
                    )
                else:
                    record.update(
                        state="REJECTED",
                        message=(
                            "coverage batch owner claim was rejected with no "
                            "ownership for this token: {}"
                        ).format(owner_detail),
                    )
                self.state = "READY" if self.plan is not None else "IDLE"
                self.detail = record["message"]
                response.message = record.get("message", self.detail)
                self._clear_never_started_batch_identity_locked(batch_id)
            self._publish_status()
            return response

        worker = threading.Thread(
            target=self._run_batch,
            args=(token,),
            daemon=True,
        )
        failure = ""
        with self.lock:
            record = self._batch_request_records_locked().get(batch_id, {})
            if (not self.start_pending or
                    self.start_token != reservation_token):
                failure = "coverage batch start was canceled or superseded"
            elif record.get("state") != "STARTING":
                failure = record.get(
                    "message", "coverage batch start was canceled"
                )
            elif self.active or self.batch_active or self.batch_worker is not None:
                failure = "coverage or a coverage batch became active during start"
            elif self.grid is None or request.map_digest != self.map_digest:
                failure = "static map changed during coverage batch start"

            if not failure:
                # Explicit batch start supersedes an idle single-region preview.
                self._discard_plan_locked(clear_progress=True)
                self._clear_visualizations()
                self.batch_id = batch_id
                self.batch_token = token
                self.batch_map_digest = self.map_digest
                self.batch_active = True
                self.batch_cancel_requested = False
                self.batch_skip_requested = False
                self.batch_abort_detail = ""
                self.batch_phase = "PLANNING"
                self.batch_region_token = uuid.uuid4().hex
                self.batch_region_outcome = ""
                self.batch_current_is_last = len(regions) == 1
                self.batch_regions = regions
                self.batch_current_index = 1
                self.batch_total_regions = len(regions)
                self.batch_completed_regions = 0
                self.batch_partial_regions = 0
                self.batch_skipped_regions = 0
                self.current_region_id = str(regions[0].id)
                self.current_region_name = str(regions[0].name)
                self.last_region_id = ""
                self.last_region_name = ""
                self.last_region_state = ""
                self.navigation_owner_token = owner_token
                self.navigation_owner_claimed = True
                self.navigation_owner_releasing = False
                self.active = False
                self.cancel_requested = False
                self.manual_pause = False
                self.manual_pause_reason = ""
                self.avoidance_loss_paused = False
                self.chassis_fault_paused = False
                self.operation_width = operation_width
                self.overlap_ratio = overlap_ratio
                self.plan_time_parameters = None
                self._apply_time_parameters_locked(time_parameters)
                self.state = "PLANNING"
                self.detail = "preparing coverage batch region 1 of {}".format(
                    len(regions)
                )
                self.batch_wake_event.clear()
                self.batch_worker = worker
                self.start_pending = False
                self.start_token = ""
                self.batch_start_request_id = ""
                record.update(
                    state="COMMITTED",
                    message="coverage batch start was committed",
                )

        if failure:
            released, release_detail = self._set_navigation_owner(
                False, owner_token
            )
            if not released:
                reason = "{}; navigation owner release failed: {}".format(
                    failure, release_detail
                )
                with self.lock:
                    self.start_pending = False
                    self.start_token = ""
                    self.batch_start_request_id = ""
                    self.batch_id = batch_id
                    self.batch_phase = "FINALIZING"
                    self.navigation_owner_token = owner_token
                    self.navigation_owner_claimed = True
                    self.navigation_owner_releasing = True
                    self.active = True
                    self.cancel_requested = True
                    self.state = "FAILED"
                    self.detail = reason
                    record = self._batch_request_records_locked().get(
                        batch_id, {}
                    )
                    record.update(
                        state="FAILED_RETAINED",
                        message=reason,
                        owner_token=owner_token,
                    )
                self.active_pub.publish(Bool(data=True))
                self._publish_status()
                rospy.logerr(reason)
                response.message = reason
                return response
            with self.lock:
                if (self.start_pending and
                        self.start_token == reservation_token):
                    self.start_pending = False
                    self.start_token = ""
                    self.batch_start_request_id = ""
                    self.state = (
                        "READY" if self.plan is not None else "IDLE"
                    )
                    self.detail = failure
                record = self._batch_request_records_locked().get(batch_id, {})
                if record.get("state") == "STARTING":
                    record.update(state="REJECTED", message=failure)
                elif (
                    record.get("state") in (
                        "CANCEL_PENDING_BEFORE_START",
                        "CANCELED_BEFORE_START",
                    )
                    and not self.active
                    and not self.batch_active
                    and self.batch_worker is None
                    and not self.start_pending
                ):
                    record.update(
                        state="CANCELED_BEFORE_START",
                        message="coverage batch was canceled before it started",
                    )
                    self.state = "READY" if self.plan is not None else "IDLE"
                    self.detail = record.get("message", failure)
                self._clear_never_started_batch_identity_locked(batch_id)
            self._publish_status()
            with self.lock:
                response.message = self._batch_request_records_locked().get(
                    batch_id, {}
                ).get("message", failure)
            return response

        # Atomic service ownership is already established before the public
        # compatibility latch and before the worker can submit any goal.
        self.active_pub.publish(Bool(data=True))
        self._publish_status()

        # Per-region self.active is intentionally false during JIT planning and
        # inter-region gaps, while the public ownership latch remains true.
        try:
            worker.start()
        except RuntimeError as error:
            self._finalize_batch(
                token,
                "FAILED",
                "coverage batch worker could not start: {}".format(error),
            )
            response.message = "coverage batch worker could not start"
            return response
        with self.lock:
            record = self._batch_request_records_locked().get(batch_id, {})
            record["accepted"] = True
            if record.get("state") == "COMMITTED":
                record["state"] = "ACTIVE"
            record["message"] = "coverage batch accepted"
        response.accepted = True
        response.message = "coverage batch accepted"
        response.batch_id = batch_id
        return response

    def _batch_interrupt_locked(self, token):
        if not self.batch_active or self.batch_token != token:
            return "CANCELED"
        if self.batch_cancel_requested:
            return "CANCELED"
        if self.batch_skip_requested:
            return "SKIPPED"
        return ""

    def _commit_batch_region_outcome_locked(self, token, outcome):
        """Linearize one region result against operator cancel/skip calls."""
        if not self.batch_active or self.batch_token != token:
            return "CANCELED"
        if (
            self.batch_phase in ("RESULT_COMMITTED", "FINALIZING")
            and self.batch_region_outcome
        ):
            return self.batch_region_outcome
        # A lifecycle cleanup/planning failure is fail-closed and cannot be
        # hidden by a concurrent operator request.
        if outcome == "FAILED":
            region_state = "FAILED"
        elif self.batch_cancel_requested:
            region_state = "CANCELED"
        elif self.batch_skip_requested or outcome == "SKIPPED":
            region_state = "SKIPPED"
        else:
            region_state = outcome
        if region_state not in REGION_TERMINAL_STATES:
            region_state = "FAILED"
            self.detail = "coverage batch produced an invalid region outcome"
        self.batch_region_outcome = region_state
        continues = region_state in (
            "COMPLETED", "COMPLETED_PARTIAL", "SKIPPED"
        )
        self.batch_phase = (
            "FINALIZING"
            if self.batch_current_is_last or not continues
            else "RESULT_COMMITTED"
        )
        return region_state

    def _batch_external_prechecks(self, token):
        if not self.move_base.wait_for_server(rospy.Duration(0.5)):
            with self.lock:
                if not self._batch_interrupt_locked(token):
                    self.detail = "move_base action server is unavailable"
            return False
        try:
            rospy.wait_for_service(self.enforced_path_service_name, timeout=0.5)
            rospy.wait_for_service(
                self.hybrid_precompute_service_name, timeout=0.5
            )
        except Exception as error:
            with self.lock:
                if not self._batch_interrupt_locked(token):
                    self.detail = (
                        "coverage global-planner hand-off service is unavailable: {}"
                    ).format(error)
            return False
        return self._verify_kinematics()

    def _prepare_batch_region(self, token, item):
        with self.lock:
            interrupted = self._batch_interrupt_locked(token)
            if interrupted:
                return interrupted, None, None
            if self.map_digest != self.batch_map_digest:
                self.batch_cancel_requested = True
                self.batch_abort_detail = "static map changed during coverage batch"
                self.cancel_requested = True
                return "CANCELED", None, None
            grid = self.grid
            map_digest = self.batch_map_digest
            operation_width = self.operation_width
            overlap_ratio = self.overlap_ratio
            requested_speed = self.task_max_speed
            time_parameters = self.task_time_parameters
            self.state = "PLANNING"
            self.detail = "planning coverage batch region {} of {}".format(
                self.batch_current_index, self.batch_total_regions
            )
        self._publish_status()

        if not self._wait_while_paused():
            with self.lock:
                return (self._batch_interrupt_locked(token) or "CANCELED",
                        None, None)
        current_pose = self._current_pose()
        if current_pose is None:
            with self.lock:
                self.detail = "map to base_link transform is unavailable"
            return "FAILED", None, None
        current, current_yaw = current_pose
        try:
            route_planner = self._planner(grid)
            plan = self._plan_region_route(
                route_planner,
                self._points_from_region(item.region),
                operation_width,
                overlap_ratio,
                current,
                current,
                current_yaw,
                time_parameters,
            )
        except ValueError as error:
            with self.lock:
                self.detail = "coverage batch region planning failed: {}".format(error)
            return "FAILED", None, None
        except Exception as error:
            rospy.logerr("coverage batch region planning raised an exception: %s", error)
            with self.lock:
                self.detail = (
                    "coverage batch region planning raised an exception: {}"
                ).format(error)
            return "FAILED", None, None

        current_pose = self._current_pose()
        if current_pose is None:
            with self.lock:
                self.detail = (
                    "map to base_link transform disappeared during batch planning"
                )
            return "FAILED", None, None
        current, current_yaw = current_pose
        plan_id = uuid.uuid4().hex
        try:
            plan = self._precompute_hybrid_candidates(
                plan,
                current,
                current_yaw,
                time_parameters,
                route_planner,
                plan_id,
            )
        except (RuntimeError, ValueError) as error:
            with self.lock:
                self.detail = (
                    "coverage batch transition precompute failed: {}"
                ).format(error)
            return "FAILED", None, None
        route = plan.swaths
        transition_paths = getattr(plan, "transition_paths", None)
        region = copy.deepcopy(item.region)
        region.header.frame_id = "map"
        region.header.stamp = rospy.Time.now()
        planned_path = self._planned_path(
            route, current, transition_paths=transition_paths
        )

        with self.lock:
            interrupted = self._batch_interrupt_locked(token)
            if interrupted:
                return interrupted, None, None
            if self.map_digest != map_digest:
                self.batch_cancel_requested = True
                self.batch_abort_detail = "static map changed during coverage batch"
                self.cancel_requested = True
                return "CANCELED", None, None
            self._reset_execution_locked(clear_progress=True)
            self.plan = plan
            self.plan_id = plan_id
            self.plan_map_digest = map_digest
            self.plan_time_parameters = time_parameters
            self.region = region
            self.kinematics_verified = False
            self.kinematics_detail = (
                "waiting for task-start VCU and TEB verification"
            )
            self.total_segments = len(route) * 2
            self.state = "PREPARING"
            self.detail = "checking live safety gates for coverage batch region"
            self._clear_visualizations()
            self.region_pub.publish(region)
            self.path_pub.publish(planned_path)
            self._publish_markers(route, region)
        self._publish_status()

        if not self._wait_while_paused():
            with self.lock:
                return (self._batch_interrupt_locked(token) or "CANCELED",
                        None, None)
        with self.lock:
            interrupted = self._batch_interrupt_locked(token)
            if interrupted:
                return interrupted, None, None
            if not self._start_prechecks_locked(
                    requested_speed, require_kinematics=False):
                return "FAILED", None, None
        if not self._batch_external_prechecks(token):
            with self.lock:
                interrupted = self._batch_interrupt_locked(token)
            return (interrupted or "FAILED"), None, None
        with self.lock:
            interrupted = self._batch_interrupt_locked(token)
            if interrupted:
                return interrupted, None, None
            if self.map_digest != map_digest:
                self.batch_cancel_requested = True
                self.batch_abort_detail = "static map changed during coverage batch"
                self.cancel_requested = True
                return "CANCELED", None, None
            if not self._start_prechecks_locked(
                    requested_speed, require_kinematics=True):
                return "FAILED", None, None
            self.active = True
            self.cancel_requested = False
            self.batch_phase = "EXECUTING"
            self.state = "GOING_TO_START"
            self.detail = (
                "coverage batch region {} of {} accepted at {:.2f} m/s maximum"
            ).format(
                self.batch_current_index,
                self.batch_total_regions,
                requested_speed,
            )
        self._publish_status()
        return "READY", route, current

    def _record_batch_region_result(self, token, item, outcome):
        with self.lock:
            if not self.batch_active or self.batch_token != token:
                return "CANCELED", False
            region_state = self._commit_batch_region_outcome_locked(
                token, outcome
            )
            self.last_region_id = str(item.id)
            self.last_region_name = str(item.name)
            self.last_region_state = region_state
            if region_state == "COMPLETED":
                self.batch_completed_regions += 1
            elif region_state == "COMPLETED_PARTIAL":
                self.batch_partial_regions += 1
            elif region_state == "SKIPPED":
                self.batch_skipped_regions += 1

            should_continue = region_state in (
                "COMPLETED", "COMPLETED_PARTIAL", "SKIPPED"
            )
            self.active = False
            self.batch_current_index = 0
            self.current_region_id = ""
            self.current_region_name = ""
            self._discard_plan_locked(clear_progress=False)
            self._clear_visualizations()
            if should_continue:
                self.batch_skip_requested = False
                if not self.batch_cancel_requested:
                    self.cancel_requested = False
                if not self.batch_current_is_last:
                    self.batch_phase = "BETWEEN_REGIONS"
                self.state = "PREPARING"
                self.detail = (
                    "coverage batch region {} finished as {}; preparing next region"
                ).format(item.id, region_state)
            self.batch_wake_event.clear()
        self._publish_status()
        return region_state, should_continue

    def _run_batch(self, token):
        terminal_state = "FAILED"
        terminal_detail = "coverage batch failed"
        try:
            with self.lock:
                regions = copy.deepcopy(self.batch_regions)
            for index, item in enumerate(regions):
                with self.lock:
                    interrupted = self._batch_interrupt_locked(token)
                    if interrupted == "CANCELED":
                        terminal_state = "CANCELED"
                        terminal_detail = (
                            self.batch_abort_detail or "coverage batch canceled"
                        )
                        break
                    self.batch_current_index = index + 1
                    self.batch_phase = "PLANNING"
                    self.batch_region_token = uuid.uuid4().hex
                    self.batch_region_outcome = ""
                    self.batch_current_is_last = index + 1 == len(regions)
                    self.current_region_id = str(item.id)
                    self.current_region_name = str(item.name)
                    self.state = "PLANNING"
                    self.detail = "preparing coverage batch region {} of {}".format(
                        index + 1, len(regions)
                    )
                self._publish_status()

                outcome, route, current = self._prepare_batch_region(token, item)
                if outcome == "READY":
                    with self.lock:
                        transition_paths = copy.deepcopy(getattr(
                            self.plan, "transition_paths", None
                        ))
                    if transition_paths is None:
                        outcome = self._run_task(
                            route, current, batch_context=True
                        )
                    else:
                        outcome = self._run_task(
                            route, current, batch_context=True,
                            transition_paths=transition_paths,
                        )
                with self.lock:
                    outcome = self._commit_batch_region_outcome_locked(
                        token, outcome
                    )
                region_state, should_continue = self._record_batch_region_result(
                    token, item, outcome
                )
                with self.lock:
                    cancel_after_result = self.batch_cancel_requested
                    cancel_detail = self.batch_abort_detail
                if cancel_after_result:
                    terminal_state = "CANCELED"
                    terminal_detail = cancel_detail or "coverage batch canceled"
                    break
                if not should_continue:
                    terminal_state = region_state
                    with self.lock:
                        terminal_detail = (
                            self.batch_abort_detail or self.detail or
                            "coverage batch stopped"
                        )
                    break
            else:
                with self.lock:
                    partial = (
                        self.batch_partial_regions > 0 or
                        self.batch_skipped_regions > 0
                    )
                terminal_state = "COMPLETED_PARTIAL" if partial else "COMPLETED"
                terminal_detail = (
                    "coverage batch completed with partial or skipped regions"
                    if partial else "coverage batch completed"
                )
        except Exception as error:
            rospy.logerr("coverage batch failed: %s", error)
            terminal_state = "FAILED"
            terminal_detail = "coverage batch exception: {}".format(error)
        finally:
            self._finalize_batch(token, terminal_state, terminal_detail)

    def _finalize_batch(self, token, terminal_state, terminal_detail):
        with self.lock:
            if self.batch_token != token:
                return
            finalizing_batch_id = self.batch_id
            record = self._batch_request_records_locked().get(
                finalizing_batch_id
            )
            if record is not None:
                record.update(
                    state="FINALIZING", message=terminal_detail
                )
            self.batch_phase = "FINALIZING"
            self.navigation_owner_releasing = True
            owner_token = self.navigation_owner_token
            if self.batch_cancel_requested and terminal_state != "FAILED":
                terminal_state = "CANCELED"
                terminal_detail = (
                    self.batch_abort_detail or "coverage batch canceled"
                )
        goal_terminal_ok, goal_terminal_detail = (
            self._wait_for_move_base_goal_terminal()
        )
        handoff_ok = False
        teb_restored = False
        if not goal_terminal_ok:
            terminal_state = "FAILED"
            terminal_detail = (
                "coverage batch retained navigation ownership because its "
                "move_base goal is not safely terminal: {}"
            ).format(goal_terminal_detail)
        else:
            off = EnforcedPath()
            off.header.frame_id = "map"
            with self.lock:
                off.plan_id = self.plan_id
            off.active = False
            handoff_ok = self._set_enforced_path(off, coverage_active=False)
            if not handoff_ok:
                terminal_state = "FAILED"
                terminal_detail = (
                    "coverage batch stopped but planner ownership could not be restored"
                )
            teb_restored = self._restore_teb()
            if not teb_restored:
                terminal_state = "FAILED"
                terminal_detail = (
                    "coverage batch stopped but TEB parameters could not be restored"
                )
        released = False
        if handoff_ok and teb_restored:
            released, release_detail = self._set_navigation_owner(
                False, owner_token
            )
            if not released:
                terminal_state = "FAILED"
                terminal_detail = (
                    "coverage batch stopped but navigation ownership could not be "
                    "released: {}"
                ).format(release_detail)
        with self.lock:
            if self.batch_token != token:
                return
            # A map update is safety-critical and may arrive while external
            # cleanup above is in progress.  Re-evaluate its cancellation flag
            # before committing the terminal batch state.
            if self.batch_cancel_requested and terminal_state != "FAILED":
                terminal_state = "CANCELED"
                terminal_detail = (
                    self.batch_abort_detail or "coverage batch canceled"
                )
            if not released:
                # The bridge may still hold this exact token.  Preserve both
                # the service token and the public latch so AI navigation can
                # never race a failed coverage finalizer.
                self.active = True
                self.batch_active = True
                self.cancel_requested = True
                self.batch_cancel_requested = True
                self.batch_phase = "FINALIZING"
                self.state = "FAILED"
                self.detail = terminal_detail
                record = self._batch_request_records_locked().get(
                    finalizing_batch_id
                )
                if record is not None:
                    record.update(
                        state="FINALIZING", message=terminal_detail
                    )
                self._publish_status()
                rospy.logerr(terminal_detail)
                return
            if self.navigation_owner_token == owner_token:
                self.navigation_owner_token = ""
                self.navigation_owner_claimed = False
                self.navigation_owner_releasing = False
            self.active = False
            self.batch_active = False
            self.batch_token = ""
            self.batch_start_request_id = ""
            self.batch_map_digest = ""
            self.batch_regions = []
            self.batch_current_index = 0
            self.current_region_id = ""
            self.current_region_name = ""
            self.batch_skip_requested = False
            self.batch_phase = "IDLE"
            self.batch_region_token = ""
            self.batch_region_outcome = ""
            self.batch_current_is_last = False
            self.cancel_requested = False
            self.manual_pause = False
            self.manual_pause_reason = ""
            self.avoidance_loss_paused = False
            self.chassis_fault_paused = False
            self.plan_pending = False
            self.plan_token = ""
            self.start_pending = False
            self.start_token = ""
            self.state = terminal_state
            self.detail = terminal_detail
            record = self._batch_request_records_locked().get(
                finalizing_batch_id
            )
            if record is not None:
                record.update(state="TERMINAL", message=terminal_detail)
            self.batch_cancel_requested = False
            self._discard_plan_locked(clear_progress=False)
            self._clear_visualizations()
            self.batch_worker = None
            self.batch_abort_detail = ""
            self.batch_wake_event.set()
            self.active_pub.publish(Bool(data=False))
            self._publish_status()

    def _start_external_prechecks(self, token):
        if not self.move_base.wait_for_server(rospy.Duration(0.5)):
            with self.lock:
                if self.start_pending and self.start_token == token:
                    self.detail = "move_base action server is unavailable"
            return False
        try:
            rospy.wait_for_service(self.enforced_path_service_name, timeout=0.5)
            rospy.wait_for_service(
                self.hybrid_precompute_service_name, timeout=0.5
            )
        except Exception as error:
            with self.lock:
                if self.start_pending and self.start_token == token:
                    self.detail = (
                        "coverage global-planner hand-off service is unavailable: {}"
                    ).format(error)
            return False
        return self._verify_kinematics()

    def _start_prechecks_locked(self, requested_speed, require_kinematics):
        now = time.monotonic()
        if self.original_teb is not None:
            self.detail = "TEB parameters from the previous task are not restored"
            return False
        if not self.localized or now - self.localization_received_wall > self.localization_fresh_sec:
            self.detail = "known-map localization is not LOCALIZED and fresh"
            return False
        if now - self.watchdog_received_wall > self.watchdog_fresh_sec:
            self.detail = "NVIDIA command watchdog status is stale"
            return False
        if not self.watchdog_motion_enabled:
            self.detail = "main motion gate is disabled"
            return False
        if now - self.mode_received_wall > 1.0 or not self.mode_goals_allowed:
            self.detail = "FOD navigation mode has not allowed move_base goals"
            return False
        if self.mode_state != "GPS_ACTIVE":
            self.detail = "navigation arbitration is not in GPS_ACTIVE"
            return False
        if self.external_pause:
            self.detail = "navigation safety pause is still active"
            return False
        avoidance_ready, avoidance_detail = self._avoidance_ready_locked(now)
        if not avoidance_ready:
            self.detail = "obstacle sensing is not ready: {}".format(
                avoidance_detail
            )
            return False
        chassis_ready, chassis_detail = self._chassis_ready_locked(now)
        if not chassis_ready:
            self.detail = "chassis execution is not ready: {}".format(
                chassis_detail
            )
            return False
        if self.odom is None or now - self.odom_received_wall > 0.5:
            self.detail = "odometry is not fresh"
            return False
        linear = self.odom.twist.twist.linear
        speed = math.sqrt(linear.x * linear.x + linear.y * linear.y)
        if speed > 0.02:
            self.detail = "vehicle must be stopped before starting coverage"
            return False
        transit_parameters = self._transit_time_parameters(
            getattr(self, "task_time_parameters", None)
        )
        requested_motion_speed = max(
            requested_speed,
            self.reverse_transit_speed if self.allow_reverse_transit else 0.0,
            transit_parameters.max_forward_speed_mps,
            transit_parameters.max_reverse_speed_mps
            if transit_parameters.allow_reverse else 0.0,
        )
        if requested_motion_speed > self.watchdog_max_linear_speed + 1.0e-6:
            self.detail = (
                "requested forward/reverse speed {:.2f} m/s exceeds NVIDIA watchdog "
                "cap {:.2f} m/s"
            ).format(requested_motion_speed, self.watchdog_max_linear_speed)
            return False
        requested_angular_speed = max(
            self.task_max_angular_speed,
            transit_parameters.max_angular_speed_rps,
        )
        if (requested_angular_speed >
                self.watchdog_max_angular_speed + 1.0e-6):
            self.detail = (
                "requested angular speed {:.2f} rad/s exceeds NVIDIA watchdog "
                "cap {:.2f} rad/s"
            ).format(
                requested_angular_speed,
                self.watchdog_max_angular_speed,
            )
            return False
        if require_kinematics:
            if not self.kinematics_verified:
                self.detail = self.kinematics_detail
                return False
            if requested_motion_speed > self.chassis_max_speed + 1.0e-6:
                self.detail = (
                    "requested forward/reverse speed {:.2f} m/s exceeds live VCU "
                    "cap {:.2f} m/s"
                ).format(requested_motion_speed, self.chassis_max_speed)
                return False
        return True

    def _kinematics_failure_locked(self, reason):
        with self.lock:
            self.kinematics_verified = False
            self.kinematics_detail = reason
            self.detail = reason
        return False

    def _verify_kinematics(self):
        """Check live VCU/TEB limits without starving subscriber callbacks."""
        try:
            rospy.wait_for_service("/m2_driver/chassis_parameter", timeout=0.5)
            response = rospy.ServiceProxy(
                "/m2_driver/chassis_parameter", ChassisParameterServer
            )()
        except Exception as error:
            return self._kinematics_failure_locked(
                "VCU chassis parameters are unavailable: {}".format(error)
            )
        if not response.success:
            return self._kinematics_failure_locked(
                "VCU chassis parameters are not ready: {}".format(response.message)
            )
        parameters = response.parameters
        wheelbase = float(parameters.robot_length)
        maximum_steering = float(parameters.max_steer)
        maximum_speed = float(parameters.max_speed)
        if not all(math.isfinite(value) and value > 0.0 for value in (
                wheelbase, maximum_steering, maximum_speed)):
            return self._kinematics_failure_locked(
                "VCU returned invalid wheelbase, steering, or speed limits"
            )
        with self.lock:
            self.chassis_wheelbase = wheelbase
            self.chassis_max_steering_angle = maximum_steering
            self.chassis_max_speed = maximum_speed
        if abs(wheelbase - self.expected_wheelbase) > self.chassis_wheelbase_tolerance:
            return self._kinematics_failure_locked(
                "VCU wheelbase {:.3f} m differs from coverage model {:.3f} m".format(
                    wheelbase, self.expected_wheelbase
                )
            )
        required_steering = math.atan(wheelbase / self.minimum_turning_radius)
        with self.lock:
            self.required_steering_angle = required_steering
        if required_steering + self.steering_angle_margin > maximum_steering:
            return self._kinematics_failure_locked(
                "coverage radius {:.2f} m needs {:.2f} deg steering and does not "
                "retain the {:.2f} deg VCU margin".format(
                    self.minimum_turning_radius,
                    math.degrees(required_steering),
                    math.degrees(self.steering_angle_margin),
                )
            )
        try:
            teb_radius = float(rospy.get_param(
                "/move_base/TebLocalPlannerROS/min_turning_radius"
            ))
            teb_wheelbase = float(rospy.get_param(
                "/move_base/TebLocalPlannerROS/wheelbase"
            ))
            proportional = rospy.get_param(
                "/move_base/TebLocalPlannerROS/use_proportional_saturation"
            )
            angle_command = rospy.get_param(
                "/move_base/TebLocalPlannerROS/cmd_angle_instead_rotvel"
            )
            unknown_is_obstacle = rospy.get_param(
                "/move_base/TebLocalPlannerROS/treat_unknown_as_obstacle"
            )
            teb_lookahead = float(rospy.get_param(
                "/move_base/TebLocalPlannerROS/max_global_plan_lookahead_dist"
            ))
            local_costmap_width = float(rospy.get_param(
                "/move_base/local_costmap/width"
            ))
            local_costmap_height = float(rospy.get_param(
                "/move_base/local_costmap/height"
            ))
            hybrid_radius = float(rospy.get_param(
                "/move_base/CoverageGlobalPlanner/"
                "hybrid_minimum_turning_radius"
            ))
            navfn_allow_unknown = rospy.get_param(
                "/move_base/CoverageGlobalPlanner_navfn/allow_unknown"
            )
        except Exception as error:
            return self._kinematics_failure_locked(
                "TEB Ackermann parameters are unavailable: {}".format(error)
            )
        if (not math.isfinite(teb_radius) or
                teb_radius + 1.0e-6 < self.minimum_turning_radius):
            return self._kinematics_failure_locked(
                "TEB minimum turning radius is below the coverage requirement"
            )
        if (not math.isfinite(teb_wheelbase) or
                abs(teb_wheelbase - wheelbase) > self.chassis_wheelbase_tolerance):
            return self._kinematics_failure_locked(
                "TEB wheelbase does not match the live VCU"
            )
        if (not math.isfinite(hybrid_radius) or
                abs(hybrid_radius - self.minimum_turning_radius) > 1.0e-6):
            return self._kinematics_failure_locked(
                "Hybrid A* minimum turning radius does not match the "
                "coverage model"
            )
        if type(proportional) is not bool or not proportional:
            return self._kinematics_failure_locked(
                "TEB proportional velocity saturation is not enabled"
            )
        if type(angle_command) is not bool or angle_command:
            return self._kinematics_failure_locked(
                "TEB must publish yaw rate for the M2 Twist steering adapter"
            )
        if type(unknown_is_obstacle) is not bool or not unknown_is_obstacle:
            return self._kinematics_failure_locked(
                "TEB must reject unknown-map trajectory footprints"
            )
        local_half_extent = 0.5 * min(local_costmap_width, local_costmap_height)
        if (not all(math.isfinite(value) and value > 0.0 for value in (
                teb_lookahead, local_costmap_width, local_costmap_height)) or
                teb_lookahead > 0.85 * local_half_extent + 1.0e-6):
            return self._kinematics_failure_locked(
                "TEB lookahead must retain margin inside the rolling local costmap"
            )
        if type(navfn_allow_unknown) is not bool or navfn_allow_unknown:
            return self._kinematics_failure_locked(
                "ordinary Navfn navigation must reject unknown map cells"
            )
        with self.lock:
            self.kinematics_verified = True
            self.kinematics_detail = (
                "VCU/Hybrid A*/TEB verified: L={:.3f} m, R={:.2f} m, "
                "steer {:.2f}/{:.2f} deg"
            ).format(
                wheelbase,
                self.minimum_turning_radius,
                math.degrees(required_steering),
                math.degrees(maximum_steering),
            )
        return True

    def _verify_kinematics_locked(self):
        """Compatibility entry point for existing state-machine tests."""
        return self._verify_kinematics()

    def _pause_service(self, request):
        with self.lock:
            if not (self.active or self.batch_active):
                return SetBoolResponse(False, "no active coverage task")
            if not request.data and not self._localization_is_fresh():
                return SetBoolResponse(False, "localization must be LOCALIZED before resume")
            if not request.data and self.external_pause:
                return SetBoolResponse(
                    False, "navigation safety pause must clear before resume"
                )
            if not request.data:
                avoidance_ready, avoidance_detail = self._avoidance_ready_locked()
                if not avoidance_ready:
                    return SetBoolResponse(
                        False,
                        "obstacle sensing must be ready before resume: {}".format(
                            avoidance_detail
                        ),
                    )
                chassis_ready, chassis_detail = self._chassis_ready_locked()
                if not chassis_ready:
                    return SetBoolResponse(
                        False,
                        "chassis execution must be ready before resume: {}".format(
                            chassis_detail
                        ),
                    )
            self.manual_pause = bool(request.data)
            if not request.data:
                self.avoidance_loss_paused = False
                self.chassis_fault_paused = False
                self.manual_pause_reason = ""
            else:
                self.manual_pause_reason = "coverage paused by operator"
            self.detail = self.manual_pause_reason if request.data else "coverage resuming"
            if request.data:
                self.state = "PAUSED"
        if request.data:
            self.move_base.cancel_goal()
        self._publish_status()
        return SetBoolResponse(True, self.detail)

    def _cancel_batch_service(self, request):
        response = CancelCoverageBatchResponse()
        batch_id = str(getattr(request, "batch_id", ""))
        response.batch_id = batch_id
        if not self._valid_batch_request_id(batch_id):
            response.message = (
                "batch_id must match coverage-batch-[0-9a-f]{32}"
            )
            return response

        cancel_committed = False
        cancel_goal_generation = None
        cancel_goal_handle = _EXPECTED_HANDLE_UNSET
        retained_owner_token = ""
        with self.lock:
            records = self._batch_request_records_locked()
            record = records.get(batch_id)
            if record is None:
                records[batch_id] = {
                    "fingerprint": "",
                    "state": "CANCELED_BEFORE_START",
                    "accepted": False,
                    "message": (
                        "coverage batch was canceled before it started"
                    ),
                }
                response.success = True
                response.not_started = True
                response.message = records[batch_id]["message"]
                return response

            state = record.get("state", "REJECTED")
            if state == "STARTING":
                record.update(
                    state="CANCEL_PENDING_BEFORE_START",
                    message=(
                        "coverage batch cancellation is waiting for the "
                        "in-flight owner claim to settle"
                    ),
                )
                # Keep the reservation live until the in-flight claim RPC
                # returns and its same-token compensation is confirmed.  This
                # prevents another operation from entering the uncertainty
                # window and being overwritten by the old callback.
                self.detail = record["message"]
                response.success = False
                response.not_started = False
                response.cancellation_requested = True
                response.message = record["message"]
            elif state == "CANCEL_PENDING_BEFORE_START":
                response.success = False
                response.not_started = False
                response.cancellation_requested = True
                response.message = record.get(
                    "message",
                    "coverage batch cancellation is still settling",
                )
            elif state in ("CANCELED_BEFORE_START", "REJECTED"):
                response.success = True
                response.not_started = True
                response.message = record.get(
                    "message", "coverage batch never started"
                )
            elif state == "TERMINAL":
                response.success = True
                response.message = (
                    "coverage batch is already terminal; no current task was changed"
                )
            elif (
                state == "FAILED_RETAINED"
                and self.batch_id == batch_id
                and self.navigation_owner_token == record.get("owner_token", "")
            ):
                retained_owner_token = self.navigation_owner_token
                response.cancellation_requested = True
                response.not_started = True
            elif self.batch_active and self.batch_id == batch_id:
                self.batch_cancel_requested = True
                self.batch_abort_detail = (
                    "coverage batch {} canceled by exact id"
                ).format(batch_id)
                self.batch_skip_requested = False
                self.cancel_requested = True
                self.plan_pending = False
                self.plan_token = ""
                self.start_pending = False
                self.start_token = ""
                self.batch_start_request_id = ""
                self.detail = self.batch_abort_detail
                self.batch_wake_event.set()
                record.update(
                    state="CANCEL_REQUESTED", message=self.detail
                )
                cancel_goal_generation = self.move_base_goal_generation
                cancel_goal_handle = self.move_base_goal_handle
                response.cancellation_requested = True
                cancel_committed = True
            else:
                response.message = (
                    "batch id is not the current coverage batch; no task was changed"
                )

        if retained_owner_token:
            success, detail = self._finalize_retained_batch_start(
                batch_id, retained_owner_token
            )
            response.success = success
            response.cancellation_requested = not success
            response.not_started = True
            response.message = detail
            self._publish_status()
            return response
        if not cancel_committed:
            self._publish_status()
            return response

        cancel_ok, cancel_detail = (
            self._request_exact_move_base_cancel_or_retain_owner(
                "exact coverage batch cancellation requested",
                expected_generation=cancel_goal_generation,
                expected_handle=cancel_goal_handle,
            )
        )
        response.success = cancel_ok
        response.message = (
            "coverage batch cancellation requested; {}"
        ).format(cancel_detail) if cancel_ok else cancel_detail
        self._publish_status()
        return response

    def _finalize_retained_batch_start(self, batch_id, owner_token):
        """Retry cleanup for a claimed owner whose batch never committed."""
        cleanup_lock = self._retained_cleanup_mutex()
        if not cleanup_lock.acquire(False):
            return False, (
                "retained coverage batch cleanup is already in progress"
            )
        try:
            with self.lock:
                record = self._batch_request_records_locked().get(batch_id)
                if (
                    record is None
                    or record.get("state") != "FAILED_RETAINED"
                    or self.batch_id != batch_id
                    or self.navigation_owner_token != owner_token
                    or not self.navigation_owner_releasing
                ):
                    return False, (
                        "retained coverage batch cleanup no longer owns this lifecycle"
                    )
                plan_id = self.plan_id
                goal_generation = self.move_base_goal_generation
                goal_handle = self.move_base_goal_handle

            goal_terminal_ok, goal_terminal_detail = (
                self._confirm_move_base_goal_terminal(
                    cancel=True,
                    expected_generation=goal_generation,
                    expected_handle=goal_handle,
                )
            )
            if not goal_terminal_ok:
                detail = (
                    "coverage batch never started, but its exact move_base goal "
                    "is not safely terminal: {}"
                ).format(goal_terminal_detail)
                with self.lock:
                    if (self.batch_id == batch_id and
                            self.navigation_owner_token == owner_token):
                        self.state = "FAILED"
                        self.detail = detail
                        record = self._batch_request_records_locked().get(batch_id)
                        if record is not None:
                            record["message"] = detail
                return False, detail

            off = EnforcedPath()
            off.header.frame_id = "map"
            off.plan_id = plan_id
            off.active = False
            if not self._set_enforced_path(off, coverage_active=False):
                detail = (
                    "coverage batch never started, but planner ownership "
                    "could not be restored"
                )
                with self.lock:
                    if (self.batch_id == batch_id and
                            self.navigation_owner_token == owner_token):
                        self.state = "FAILED"
                        self.detail = detail
                        record = self._batch_request_records_locked().get(batch_id)
                        if record is not None:
                            record["message"] = detail
                return False, detail
            if not self._restore_teb():
                detail = (
                    "coverage batch never started, but TEB parameters "
                    "could not be restored"
                )
                with self.lock:
                    if (self.batch_id == batch_id and
                            self.navigation_owner_token == owner_token):
                        self.state = "FAILED"
                        self.detail = detail
                        record = self._batch_request_records_locked().get(batch_id)
                        if record is not None:
                            record["message"] = detail
                return False, detail

            released, release_detail = self._set_navigation_owner(
                False, owner_token
            )
            if not released:
                detail = (
                    "coverage batch never started, but its navigation owner "
                    "is still retained: {}"
                ).format(release_detail)
                with self.lock:
                    if (self.batch_id == batch_id and
                            self.navigation_owner_token == owner_token):
                        self.state = "FAILED"
                        self.detail = detail
                        record = self._batch_request_records_locked().get(batch_id)
                        if record is not None:
                            record["message"] = detail
                return False, detail

            with self.lock:
                record = self._batch_request_records_locked().get(batch_id)
                if (
                    record is None
                    or record.get("state") != "FAILED_RETAINED"
                    or self.batch_id != batch_id
                    or self.navigation_owner_token != owner_token
                ):
                    # The exact token was released, but another callback changed
                    # local bookkeeping.  Never overwrite that newer lifecycle.
                    return True, (
                        "retained navigation owner was released; lifecycle changed"
                    )
                record.update(
                    state="TERMINAL",
                    message="coverage batch was canceled before it started",
                )
                self.navigation_owner_token = ""
                self.navigation_owner_claimed = False
                self.navigation_owner_releasing = False
                self.active = False
                self.cancel_requested = False
                self.batch_phase = "IDLE"
                self.state = "CANCELED"
                self.detail = record["message"]
                self._clear_never_started_batch_identity_locked(batch_id)
                self.active_pub.publish(Bool(data=False))
            return True, "coverage batch was canceled before it started"
        finally:
            cleanup_lock.release()

    def _finalize_retained_single_owner(self, owner_token):
        """Retry the fail-closed cleanup for an orphaned standalone owner.

        This path is used only after a failed standalone start/finalizer has no
        worker left to perform cleanup.  Every external operation stays outside
        ``self.lock`` and the public ownership latch remains asserted until the
        exact goal, planner mode, TEB parameters, and atomic owner are all
        confirmed safe in that order.
        """
        cleanup_lock = self._retained_cleanup_mutex()
        if not cleanup_lock.acquire(False):
            return False, (
                "retained standalone coverage cleanup is already in progress"
            )
        try:
            with self.lock:
                if (
                    self.batch_active
                    or self.worker is not None
                    or not self.navigation_owner_releasing
                    or self.navigation_owner_token != owner_token
                ):
                    return False, (
                        "standalone coverage cleanup is owned by another lifecycle"
                    )
                plan_id = self.plan_id
                goal_generation = self.move_base_goal_generation
                goal_handle = self.move_base_goal_handle

            goal_terminal_ok, goal_terminal_detail = (
                self._confirm_move_base_goal_terminal(
                    cancel=True,
                    expected_generation=goal_generation,
                    expected_handle=goal_handle,
                )
            )
            if not goal_terminal_ok:
                detail = (
                    "standalone coverage still retains navigation ownership because "
                    "its exact move_base goal is not safely terminal: {}"
                ).format(goal_terminal_detail)
                with self.lock:
                    if self.navigation_owner_token == owner_token:
                        self.state = "FAILED"
                        self.detail = detail
                return False, detail

            off = EnforcedPath()
            off.header.frame_id = "map"
            off.plan_id = plan_id
            off.active = False
            if not self._set_enforced_path(off, coverage_active=False):
                detail = (
                    "standalone coverage still retains navigation ownership because "
                    "planner ownership could not be restored"
                )
                with self.lock:
                    if self.navigation_owner_token == owner_token:
                        self.state = "FAILED"
                        self.detail = detail
                return False, detail
            if not self._restore_teb():
                detail = (
                    "standalone coverage still retains navigation ownership because "
                    "TEB parameters could not be restored"
                )
                with self.lock:
                    if self.navigation_owner_token == owner_token:
                        self.state = "FAILED"
                        self.detail = detail
                return False, detail

            released, release_detail = self._set_navigation_owner(
                False, owner_token
            )
            if not released:
                detail = (
                    "standalone coverage navigation ownership is still retained: {}"
                ).format(release_detail)
                with self.lock:
                    if self.navigation_owner_token == owner_token:
                        self.state = "FAILED"
                        self.detail = detail
                return False, detail

            with self.lock:
                if self.navigation_owner_token != owner_token:
                    # The exact token has already been released.  Do not mutate
                    # a newer lifecycle if bookkeeping changed during the RPC.
                    return True, (
                        "standalone navigation owner was released; lifecycle changed"
                    )
                self.navigation_owner_token = ""
                self.navigation_owner_claimed = False
                self.navigation_owner_releasing = False
                self.active = False
                self.cancel_requested = False
                self.manual_pause = False
                self.manual_pause_reason = ""
                self.avoidance_loss_paused = False
                self.chassis_fault_paused = False
                self.state = "CANCELED"
                self.detail = (
                    "failed standalone coverage ownership was safely released"
                )
                self._discard_plan_locked(clear_progress=False)
                self._clear_visualizations()
                self.active_pub.publish(Bool(data=False))
            return True, self.detail
        finally:
            cleanup_lock.release()

    def _cancel_service(self, _request):
        clear_inactive = False
        cancel_batch = False
        cancel_pending_start = False
        reassert_finalizing = False
        reassert_goal_generation = None
        reassert_goal_handle = _EXPECTED_HANDLE_UNSET
        retained_batch_cleanup = None
        retained_single_owner_token = ""
        cancel_goal_generation = None
        cancel_goal_handle = _EXPECTED_HANDLE_UNSET
        with self.lock:
            if self.navigation_owner_releasing:
                record = self._batch_request_records_locked().get(
                    self.batch_id, {}
                )
                retained_batch_start = (
                    record.get("state") == "FAILED_RETAINED"
                    and record.get("owner_token", "")
                    == self.navigation_owner_token
                )
                if retained_batch_start:
                    retained_batch_cleanup = (
                        self.batch_id, self.navigation_owner_token
                    )
                elif (
                    not self.batch_active
                    and self.worker is None
                    and self._valid_navigation_owner_token(
                        self.navigation_owner_token
                    )
                ):
                    retained_single_owner_token = self.navigation_owner_token
                else:
                    reassert_finalizing = True
                    reassert_goal_generation = self.move_base_goal_generation
                    reassert_goal_handle = self.move_base_goal_handle
            elif self.batch_active:
                self.batch_cancel_requested = True
                self.batch_abort_detail = "coverage batch canceled by operator"
                self.batch_skip_requested = False
                self.cancel_requested = True
                self.plan_pending = False
                self.plan_token = ""
                self.start_pending = False
                self.start_token = ""
                self.detail = self.batch_abort_detail
                self.batch_wake_event.set()
                cancel_goal_generation = self.move_base_goal_generation
                cancel_goal_handle = self.move_base_goal_handle
                cancel_batch = True
                detail = self.detail
            else:
                detail = ""
        if retained_batch_cleanup is not None:
            success, detail = self._finalize_retained_batch_start(
                retained_batch_cleanup[0], retained_batch_cleanup[1]
            )
            self._publish_status()
            return TriggerResponse(success, detail)
        if retained_single_owner_token:
            success, detail = self._finalize_retained_single_owner(
                retained_single_owner_token
            )
            self._publish_status()
            return TriggerResponse(success, detail)
        if reassert_finalizing:
            success, detail = self._reassert_move_base_goal_cancel(
                expected_generation=reassert_goal_generation,
                expected_handle=reassert_goal_handle,
            )
            self._publish_status()
            return TriggerResponse(success, detail)
        if cancel_batch:
            cancel_ok, cancel_detail = (
                self._request_exact_move_base_cancel_or_retain_owner(
                    "coverage batch cancellation requested",
                    expected_generation=cancel_goal_generation,
                    expected_handle=cancel_goal_handle,
                )
            )
            self._publish_status()
            if not cancel_ok:
                return TriggerResponse(False, cancel_detail)
            return TriggerResponse(True, (
                "coverage batch cancellation requested; {}"
            ).format(cancel_detail))

        with self.lock:
            if self.plan_pending:
                self.plan_pending = False
                self.plan_token = ""
            if self.start_pending:
                self.cancel_requested = True
                pending_batch_id = getattr(
                    self, "batch_start_request_id", ""
                )
                if self._valid_batch_request_id(pending_batch_id):
                    record = self._batch_request_records_locked().get(
                        pending_batch_id
                    )
                    if record is not None and record.get("state") == "STARTING":
                        record.update(
                            state="CANCEL_PENDING_BEFORE_START",
                            message=(
                                "coverage batch cancellation is waiting for the "
                                "in-flight owner claim to settle"
                            ),
                        )
                    cancel_pending_start = True
                    self.detail = record.get(
                        "message", "coverage batch cancellation is pending"
                    ) if record is not None else (
                        "coverage batch cancellation is pending"
                    )
                else:
                    cancel_pending_start = True
                    self.detail = (
                        "coverage start cancellation is waiting for the "
                        "in-flight owner claim to settle"
                    )
                canceled_preparation = True
            else:
                canceled_preparation = False
            active = self.active
            if not canceled_preparation and not active:
                had_plan = self.plan is not None or bool(self.plan_id)
                self._discard_plan_locked(clear_progress=True)
                self.state = "IDLE"
                self.detail = (
                    "coverage plan canceled; select a new region"
                    if had_plan else
                    "no coverage plan is active; stale displays were cleared"
                )
                clear_inactive = True
            detail = self.detail
            # Keep plan invalidation and the latched DELETEALL publications in
            # one lifecycle critical section.  Otherwise a newly accepted plan
            # could be published and then erased by an older cancel request.
            if clear_inactive:
                self._clear_visualizations()
                self.active_pub.publish(Bool(data=False))
                self._publish_status()
        if clear_inactive:
            return TriggerResponse(True, detail)
        if cancel_pending_start:
            self._publish_status()
            return TriggerResponse(False, detail)
        cancel_ok, cancel_detail = self._request_cancel(
            "coverage canceled by operator"
        )
        if not cancel_ok:
            return TriggerResponse(False, self.detail)
        return TriggerResponse(True, (
            "coverage cancellation requested; {}"
        ).format(cancel_detail))

    def _skip_current_service(self, _request):
        with self.lock:
            if not self.batch_active:
                return TriggerResponse(False, "no active coverage batch")
            if self.batch_cancel_requested:
                return TriggerResponse(False, "coverage batch cancellation is pending")
            if self.batch_phase in ("RESULT_COMMITTED", "FINALIZING"):
                return TriggerResponse(
                    False, "current coverage region result is already committed"
                )
            if self.batch_current_index == 0:
                return TriggerResponse(False, "coverage batch has no current region")
            if self.batch_skip_requested:
                return TriggerResponse(True, "current coverage region skip is already pending")
            self.batch_skip_requested = True
            self.cancel_requested = True
            self.plan_pending = False
            self.plan_token = ""
            self.start_pending = False
            self.start_token = ""
            self.detail = "current coverage batch region skip requested"
            self.batch_wake_event.set()
            goal_generation = self.move_base_goal_generation
            goal_handle = self.move_base_goal_handle
        cancel_ok, cancel_detail = (
            self._request_exact_move_base_cancel_or_retain_owner(
                "current coverage batch region skip requested",
                expected_generation=goal_generation,
                expected_handle=goal_handle,
            )
        )
        self._publish_status()
        if not cancel_ok:
            return TriggerResponse(False, cancel_detail)
        return TriggerResponse(True, (
            "current coverage region skip requested; {}"
        ).format(cancel_detail))

    def _request_cancel(self, reason):
        with self.lock:
            self.cancel_requested = True
            self.detail = reason
            self.plan_pending = False
            self.plan_token = ""
            self.start_pending = False
            self.start_token = ""
            if self.batch_active:
                self.batch_cancel_requested = True
                self.batch_abort_detail = reason
                self.batch_skip_requested = False
                self.batch_wake_event.set()
            goal_generation = self.move_base_goal_generation
            goal_handle = self.move_base_goal_handle
        cancel_ok, cancel_detail = (
            self._request_exact_move_base_cancel_or_retain_owner(
                reason,
                expected_generation=goal_generation,
                expected_handle=goal_handle,
            )
        )
        self._publish_status()
        return cancel_ok, cancel_detail

    def _set_teb(self, backwards, straight_tracking=False,
                 hybrid_tracking=None):
        if hybrid_tracking is None:
            hybrid_tracking = bool(getattr(
                self, "_teb_hybrid_tracking_requested", backwards > 0.0
            ))
        hybrid_expected_reverse = bool(getattr(
            self, "_teb_hybrid_expected_reverse", False
        ))
        hybrid_cusp_goal = bool(getattr(
            self, "_teb_hybrid_cusp_goal", False
        ))
        if not math.isfinite(backwards) or backwards < 0.0:
            rospy.logerr("coverage requested an invalid TEB reverse limit: %r", backwards)
            return False
        try:
            import dynamic_reconfigure.client
            client = dynamic_reconfigure.client.Client(
                "/move_base/TebLocalPlannerROS", timeout=2.0
            )
            if self.original_teb is None:
                configuration = client.get_configuration(timeout=2.0)
                self.original_teb = {
                    "motion_direction_mode": configuration.get(
                        "motion_direction_mode", 0
                    ),
                    "max_vel_x": configuration.get("max_vel_x", 0.8),
                    "max_vel_x_backwards": configuration.get("max_vel_x_backwards", 0.3),
                    "allow_init_with_backwards_motion": configuration.get(
                        "allow_init_with_backwards_motion", False
                    ),
                    "xy_goal_tolerance": configuration.get("xy_goal_tolerance", 0.5),
                    "yaw_goal_tolerance": configuration.get("yaw_goal_tolerance", 0.3),
                    "global_plan_viapoint_sep": configuration.get(
                        "global_plan_viapoint_sep", 0.8
                    ),
                    "weight_viapoint": configuration.get("weight_viapoint", 8.0),
                    "weight_viapoint_lateral": configuration.get(
                        "weight_viapoint_lateral", 0.0
                    ),
                    "weight_viapoint_heading": configuration.get(
                        "weight_viapoint_heading", 0.0
                    ),
                    "weight_kinematics_forward_drive": configuration.get(
                        "weight_kinematics_forward_drive", 100.0
                    ),
                    "selection_viapoint_cost_scale": configuration.get(
                        "selection_viapoint_cost_scale", 1.0
                    ),
                    "viapoints_all_candidates": configuration.get(
                        "viapoints_all_candidates", False
                    ),
                    "global_plan_overwrite_orientation": configuration.get(
                        "global_plan_overwrite_orientation", True
                    ),
                    "via_points_ordered": configuration.get(
                        "via_points_ordered", False
                    ),
                    "max_number_classes": configuration.get(
                        "max_number_classes", 3
                    ),
                    "max_global_plan_lookahead_dist": configuration.get(
                        "max_global_plan_lookahead_dist", 8.0
                    ),
                    "min_obstacle_dist": configuration.get(
                        "min_obstacle_dist", 0.30
                    ),
                    "inflation_dist": configuration.get(
                        "inflation_dist", 0.60
                    ),
                    "include_costmap_obstacles": configuration.get(
                        "include_costmap_obstacles", True
                    ),
                    "no_inner_iterations": configuration.get(
                        "no_inner_iterations", 5
                    ),
                    "no_outer_iterations": configuration.get(
                        "no_outer_iterations", 4
                    ),
                }
                for key in ("max_vel_theta", "acc_lim_x", "acc_lim_theta"):
                    if key in configuration:
                        self.original_teb[key] = configuration[key]
            target = copy.deepcopy(self.original_teb)
            # A point-to-point transit only has to deliver the Ackermann
            # chassis to a maneuverable swath entry.  Requiring the same
            # tight terminal yaw as an exact sweep makes TEB chase an
            # orientation-only correction after the position is already met,
            # which this chassis cannot realize by spinning in place.  Keep
            # exact sweep completion strict, but make transit completion match
            # the manager's explicit entry acceptance gate.
            goal_position_tolerance = (
                0.20 if straight_tracking else (
                    self.hybrid_cusp_position_tolerance
                    if hybrid_tracking and hybrid_cusp_goal else
                    self.entry_position_tolerance
                )
            )
            goal_yaw_tolerance = (
                0.20 if straight_tracking else (
                    self.hybrid_cusp_yaw_tolerance
                    if hybrid_tracking and hybrid_cusp_goal else
                    self.entry_yaw_tolerance
                )
            )
            maximum_forward_speed = self.task_max_speed
            if hybrid_tracking and hybrid_cusp_goal:
                # A cusp is a mandatory zero-speed direction change, not an
                # ordinary pass-through waypoint.  Bound the complete short
                # approach action so delayed controller braking cannot carry
                # the chassis far beyond the hand-off plane.
                maximum_forward_speed = min(
                    maximum_forward_speed,
                    float(getattr(
                        self, "hybrid_cusp_max_forward_speed", 0.60
                    )),
                )
            target.update({
                "motion_direction_mode": 1 if straight_tracking else 0,
                "max_vel_x": maximum_forward_speed,
                "max_vel_x_backwards": backwards,
                "max_vel_theta": getattr(
                    self, "task_max_angular_speed", 0.60
                ),
                "acc_lim_x": getattr(self, "task_linear_accel", 1.00),
                "acc_lim_theta": getattr(
                    self, "task_angular_accel", 0.50
                ),
                "allow_init_with_backwards_motion": backwards > 0.0,
                "xy_goal_tolerance": goal_position_tolerance,
                "yaw_goal_tolerance": goal_yaw_tolerance,
            })
            if straight_tracking:
                # Exact coverage sweeps need a different optimization objective
                # from point-to-point transit.  Dense positional via-points plus
                # the TEB-native tangent edge penalize cross-track and yaw error,
                # while every homotopy candidate is tied to the same reference.
                # A zero reverse bound is supported by this fork and clamped at
                # command output, so stationary commands remain valid without
                # permitting a coverage sweep to back up.
                target.update({
                    # The global Hybrid planner has already selected the
                    # connector topology. Limit the initialized homotopy
                    # planner to that one class instead of duplicating global
                    # planning authority in the local controller.
                    "max_number_classes": 1,
                    "include_costmap_obstacles": False,
                    "max_vel_x_backwards": 0.0,
                    "allow_init_with_backwards_motion": False,
                    "global_plan_viapoint_sep": self.sweep_viapoint_separation,
                    "weight_viapoint": self.sweep_weight_viapoint,
                    "weight_viapoint_lateral": self.sweep_weight_viapoint_lateral,
                    "weight_viapoint_heading": self.sweep_weight_viapoint_heading,
                    "weight_kinematics_forward_drive": (
                        self.sweep_weight_kinematics_forward_drive
                    ),
                    "selection_viapoint_cost_scale": (
                        self.sweep_selection_viapoint_cost_scale
                    ),
                    "viapoints_all_candidates": self.sweep_viapoints_all_candidates,
                })
            elif hybrid_tracking:
                # Hybrid A* has already selected a curvature-feasible vehicle
                # orientation and an explicit constant gear for this action.
                # Keep those orientations instead of replacing them with
                # Navfn-like tangents. TEB is the actual feedback controller;
                # the output mux only enforces freshness, safety and gear.
                target.update({
                    "motion_direction_mode": (
                        -1 if hybrid_expected_reverse else 1
                    ),
                    "max_number_classes": 1,
                    # Hybrid A* has already collision-checked the static map.
                    # Production retains live obstacle optimization for its
                    # background TEB trajectory.  The command mux does
                    # not weaken the existing path invalidation, FOD
                    # arbitration, localization gate, or final watchdog.
                    "include_costmap_obstacles": (
                        self.hybrid_transit_include_costmap_obstacles
                    ),
                    # Hybrid A* supplies one event-triggered connector.  The
                    # global plugin validates it at 1 Hz; a short TEB local
                    # refinement is sufficient and avoids stale commands from
                    # a redundant 10x6 optimization cycle.
                    "no_inner_iterations": self.hybrid_transit_inner_iterations,
                    "no_outer_iterations": self.hybrid_transit_outer_iterations,
                    "global_plan_overwrite_orientation": False,
                    "via_points_ordered": True,
                    "max_global_plan_lookahead_dist": (
                        self.hybrid_transit_lookahead_distance
                    ),
                    "min_obstacle_dist": (
                        self.hybrid_transit_min_obstacle_dist
                    ),
                    "inflation_dist": (
                        self.hybrid_transit_inflation_dist
                    ),
                    "global_plan_viapoint_sep": (
                        self.hybrid_transit_viapoint_separation
                    ),
                    "weight_viapoint": self.hybrid_transit_weight_viapoint,
                    "weight_viapoint_lateral": (
                        self.hybrid_transit_weight_viapoint_lateral
                    ),
                    "weight_viapoint_heading": (
                        self.hybrid_transit_weight_viapoint_heading
                    ),
                    "weight_kinematics_forward_drive": (
                        self.hybrid_transit_weight_kinematics_forward_drive
                        if backwards > 0.0 else
                        self.original_teb[
                            "weight_kinematics_forward_drive"
                        ]
                    ),
                    "selection_viapoint_cost_scale": (
                        self.hybrid_transit_selection_viapoint_cost_scale
                    ),
                    "viapoints_all_candidates": (
                        self.hybrid_transit_viapoints_all_candidates
                    ),
                })
            elif bool(getattr(self, "entry_navfn_single_topology", False)):
                # Navfn owns the inter-region topology. Running TEB's
                # homotopy search over the same static map duplicated that
                # authority and spent multiple seconds per control cycle
                # without producing motion. Follow only the Navfn class with
                # a bounded smoothing solve.  Static obstacles must remain in
                # that solve: merely relying on the final feasibility check
                # lets the optimizer cut an inflated-map corner, after which
                # it can only reject the same colliding trajectory and stop.
                # Dense, ordered via-points keep the local trajectory attached
                # to Navfn while still allowing obstacle-aware smoothing.
                target.update({
                    "max_number_classes": 1,
                    "include_costmap_obstacles": True,
                    "no_inner_iterations": (
                        self.hybrid_transit_inner_iterations
                    ),
                    "no_outer_iterations": (
                        self.hybrid_transit_outer_iterations
                    ),
                    "max_global_plan_lookahead_dist": (
                        self.hybrid_transit_lookahead_distance
                    ),
                    "global_plan_overwrite_orientation": True,
                    "via_points_ordered": True,
                    "global_plan_viapoint_sep": (
                        self.hybrid_transit_viapoint_separation
                    ),
                    "weight_viapoint": (
                        self.hybrid_transit_weight_viapoint
                    ),
                    "selection_viapoint_cost_scale": (
                        self.hybrid_transit_selection_viapoint_cost_scale
                    ),
                    "viapoints_all_candidates": False,
                })
            client.update_configuration(target)
            return True
        except Exception as error:
            rospy.logerr("coverage could not configure TEB: %s", error)
            return False

    def _restore_teb(self):
        if not self.original_teb:
            return True
        configuration = copy.deepcopy(self.original_teb)
        for attempt in range(3):
            try:
                import dynamic_reconfigure.client
                client = dynamic_reconfigure.client.Client(
                    "/move_base/TebLocalPlannerROS", timeout=2.0
                )
                client.update_configuration(configuration)
                self.original_teb = None
                return True
            except Exception as error:
                rospy.logerr(
                    "coverage could not restore TEB (attempt %d of 3): %s",
                    attempt + 1,
                    error,
                )
                if attempt < 2:
                    time.sleep(0.2)
        return False

    @staticmethod
    def _hybrid_path_pose_yaw(stamped_pose):
        quaternion = stamped_pose.pose.orientation
        norm = (
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if not math.isfinite(norm) or norm < 1.0e-12:
            raise ValueError("Hybrid transition contains an invalid orientation")
        inverse = 1.0 / math.sqrt(norm)
        x = quaternion.x * inverse
        y = quaternion.y * inverse
        z = quaternion.z * inverse
        w = quaternion.w * inverse
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        if not math.isfinite(yaw):
            raise ValueError("Hybrid transition contains a non-finite yaw")
        return yaw

    def _split_hybrid_transition(self, path):
        """Split one Reeds-Shepp path at every forward/reverse cusp.

        ``nav_msgs/Path`` has no signed gear field.  Recover the gear of each
        sampled edge by projecting its displacement onto the vehicle heading,
        then put each constant-gear run in its own move_base action.  Since TEB
        uses ``free_goal_vel: false``, every cusp is consequently reached at
        zero velocity before the next direction is armed.
        """
        if path is None or len(path.poses) < 2:
            return []
        cleaned = []
        for stamped_pose in path.poses:
            position = stamped_pose.pose.position
            if not (math.isfinite(position.x) and math.isfinite(position.y)):
                raise ValueError(
                    "Hybrid transition contains a non-finite position"
                )
            yaw = self._hybrid_path_pose_yaw(stamped_pose)
            if cleaned:
                previous = cleaned[-1]
                distance = math.hypot(
                    position.x - previous.pose.position.x,
                    position.y - previous.pose.position.y,
                )
                previous_yaw = self._hybrid_path_pose_yaw(previous)
                yaw_change = abs(math.atan2(
                    math.sin(yaw - previous_yaw),
                    math.cos(yaw - previous_yaw),
                ))
                if distance <= 1.0e-6:
                    if yaw_change > 1.0e-5:
                        raise ValueError(
                            "Hybrid transition contains an in-place rotation"
                        )
                    continue
            cleaned.append(copy.deepcopy(stamped_pose))
        if len(cleaned) < 2:
            raise ValueError("Hybrid transition has no finite motion edge")

        edge_gears = []
        for first, second in zip(cleaned, cleaned[1:]):
            first_yaw = self._hybrid_path_pose_yaw(first)
            second_yaw = self._hybrid_path_pose_yaw(second)
            yaw_change = math.atan2(
                math.sin(second_yaw - first_yaw),
                math.cos(second_yaw - first_yaw),
            )
            tangent_yaw = first_yaw + 0.5 * yaw_change
            delta_x = second.pose.position.x - first.pose.position.x
            delta_y = second.pose.position.y - first.pose.position.y
            distance = math.hypot(delta_x, delta_y)
            projection = (
                delta_x * math.cos(tangent_yaw)
                + delta_y * math.sin(tangent_yaw)
            )
            if abs(projection) < 0.5 * distance:
                raise ValueError(
                    "Hybrid transition edge is not tangent to vehicle heading"
                )
            edge_gears.append(1 if projection > 0.0 else -1)

        parts = []
        start_pose_index = 0
        current_gear = edge_gears[0]
        for edge_index in range(1, len(edge_gears)):
            if edge_gears[edge_index] == current_gear:
                continue
            part = Path()
            part.header = copy.deepcopy(path.header)
            part.poses = copy.deepcopy(
                cleaned[start_pose_index:edge_index + 1]
            )
            parts.append((part, current_gear))
            start_pose_index = edge_index
            current_gear = edge_gears[edge_index]
        part = Path()
        part.header = copy.deepcopy(path.header)
        part.poses = copy.deepcopy(cleaned[start_pose_index:])
        parts.append((part, current_gear))
        return parts

    @staticmethod
    def _hybrid_path_length(path):
        return sum(
            math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            )
            for first, second in zip(path.poses, path.poses[1:])
        )

    def _rebase_joinable_hybrid_part(self, segment, live_point, live_yaw):
        """Rebase a cached fixed-gear suffix when its cusp join is feasible.

        ``CoverageGlobalPlanner`` prepends the measured vehicle pose to an
        enforced path.  A small accepted cusp offset can consequently create
        one artificial first edge whose radius is much smaller than the
        vehicle limit.  Search only the first lattice step for a same-gear,
        tangent circular join.  If one exists, trim the obsolete cusp samples
        and make that checked edge explicit; otherwise the caller must run a
        complete live-pose Hybrid replan to the final swath entrance.
        """
        path = segment.get("path")
        expected_gear = int(segment.get("gear", 0))
        if (
            path is None or len(path.poses) < 2
            or expected_gear not in (-1, 1)
            or not all(math.isfinite(value) for value in (
                float(live_point.x), float(live_point.y), float(live_yaw),
            ))
        ):
            return False, "cached cusp suffix or live pose is invalid"

        maximum_skip = float(getattr(
            self, "hybrid_cusp_join_max_skip", 0.30
        ))
        chord_tolerance = float(getattr(
            self, "hybrid_cusp_join_chord_tolerance", 0.10
        ))
        minimum_radius = float(getattr(
            self, "minimum_turning_radius", 1.35
        ))
        accumulated = 0.0
        previous = path.poses[0].pose.position
        closest_reason = "no candidate lies inside the cusp join horizon"
        for index, stamped_pose in enumerate(path.poses):
            position = stamped_pose.pose.position
            if index > 0:
                accumulated += math.hypot(
                    float(position.x) - float(previous.x),
                    float(position.y) - float(previous.y),
                )
            previous = position
            if accumulated > maximum_skip + 1.0e-6:
                break
            candidate_yaw = self._hybrid_path_pose_yaw(stamped_pose)
            delta_x = float(position.x) - float(live_point.x)
            delta_y = float(position.y) - float(live_point.y)
            distance = math.hypot(delta_x, delta_y)
            if distance <= 0.02:
                closest_reason = "candidate is too close for a finite-radius join"
                continue
            yaw_change = math.atan2(
                math.sin(candidate_yaw - float(live_yaw)),
                math.cos(candidate_yaw - float(live_yaw)),
            )
            tangent_yaw = float(live_yaw) + 0.5 * yaw_change
            longitudinal = (
                delta_x * math.cos(tangent_yaw)
                + delta_y * math.sin(tangent_yaw)
            )
            lateral = abs(
                -delta_x * math.sin(tangent_yaw)
                + delta_y * math.cos(tangent_yaw)
            )
            chord_error = math.atan2(lateral, abs(longitudinal))
            if expected_gear * longitudinal <= 0.0:
                closest_reason = "candidate is behind the requested fixed gear"
                continue
            if chord_error > chord_tolerance:
                closest_reason = (
                    "join chord error {:.1f}deg exceeds {:.1f}deg"
                ).format(
                    math.degrees(chord_error),
                    math.degrees(chord_tolerance),
                )
                continue
            if abs(yaw_change) <= 1.0e-6:
                radius = float("inf")
            else:
                radius = distance / (
                    2.0 * abs(math.sin(0.5 * yaw_change))
                )
            if radius + 1.0e-3 < minimum_radius:
                closest_reason = (
                    "join radius {:.2f}m is below {:.2f}m"
                ).format(radius, minimum_radius)
                continue

            rebased = Path()
            rebased.header = copy.deepcopy(path.header)
            rebased.poses = [
                self._pose(
                    Point(float(live_point.x), float(live_point.y)),
                    float(live_yaw),
                    rebased.header.stamp,
                )
            ] + copy.deepcopy(path.poses[index:])
            try:
                split = self._split_hybrid_transition(rebased)
            except ValueError as error:
                closest_reason = "rebased suffix is invalid: {}".format(error)
                continue
            if len(split) != 1 or int(split[0][1]) != expected_gear:
                closest_reason = "rebased suffix changes its fixed gear"
                continue
            segment["path"] = rebased
            segment["start"] = Point(
                float(live_point.x), float(live_point.y)
            )
            segment["length"] = self._hybrid_path_length(rebased)
            return True, (
                "joined cached {} suffix after trimming {:.2f}m; "
                "bridge {:.2f}m, radius {}m, chord {:.1f}deg"
            ).format(
                "forward" if expected_gear > 0 else "reverse",
                accumulated,
                distance,
                "inf" if not math.isfinite(radius) else "{:.2f}".format(radius),
                math.degrees(chord_error),
            )
        return False, closest_reason

    @staticmethod
    def _hybrid_path_point(stamped_pose):
        return Point(
            float(stamped_pose.pose.position.x),
            float(stamped_pose.pose.position.y),
        )

    def _publish_hybrid_transition_path(self, path=None):
        """Publish one complete logical Hybrid connector, including all gears."""
        message = copy.deepcopy(path) if path is not None else Path()
        message.header.frame_id = "map"
        message.header.stamp = rospy.Time.now()
        for pose in message.poses:
            pose.header.frame_id = "map"
            pose.header.stamp = message.header.stamp
        publisher = getattr(self, "hybrid_transition_path_pub", None)
        if publisher is not None:
            publisher.publish(message)

    def _hybrid_transition_segments(
            self, path, swath_index, expected_start,
            transition_goal, transition_goal_yaw,
            final_transition_goal=None, final_transition_goal_yaw=None,
            path_generation=0):
        """Convert a Hybrid connector into one or more execution actions.

        Every constant-gear part is a separate move_base action. TEB therefore
        has one unambiguous hard gear, and ``free_goal_vel: false`` plus the
        manager's measured-stop check form the cusp hand-off contract.
        """
        transition_parts = self._split_hybrid_transition(path)
        if not transition_parts:
            raise ValueError("Hybrid transition has no constant-gear motion part")
        first_point = self._hybrid_path_point(
            transition_parts[0][0].poses[0]
        )
        last_point = self._hybrid_path_point(
            transition_parts[-1][0].poses[-1]
        )
        if (
            math.hypot(first_point.x - expected_start.x,
                       first_point.y - expected_start.y) > 0.05
            or math.hypot(last_point.x - transition_goal.x,
                          last_point.y - transition_goal.y) > 0.05
        ):
            raise ValueError("Hybrid transition endpoints do not match its swaths")

        complete_path = copy.deepcopy(path)
        authoritative_goal = (
            transition_goal
            if final_transition_goal is None
            else final_transition_goal
        )
        authoritative_yaw = (
            transition_goal_yaw
            if final_transition_goal_yaw is None
            else final_transition_goal_yaw
        )
        segments = []
        for part_index, (part, gear) in enumerate(transition_parts):
            part_start = self._hybrid_path_point(part.poses[0])
            part_end = self._hybrid_path_point(part.poses[-1])
            part_yaw = self._hybrid_path_pose_yaw(part.poses[-1])
            segments.append({
                "type": "transit",
                "swath_index": swath_index,
                "start": part_start,
                "end": part_end,
                "yaw": part_yaw,
                "length": self._hybrid_path_length(part),
                "path": part,
                "gear": gear,
                "path_generation": int(path_generation),
                "transition_part": part_index,
                "transition_part_count": len(transition_parts),
                # A measured cusp stop may differ from its lattice pose. Check
                # whether the next fixed-gear part can be rebased with a
                # finite-radius tangent edge before deciding to replan.
                "check_join_after_completion": (
                    part_index + 1 < len(transition_parts)
                ),
                "transition_goal": Point(
                    float(authoritative_goal.x),
                    float(authoritative_goal.y),
                ),
                "transition_goal_yaw": float(authoritative_yaw),
                "transition_path": complete_path,
            })
        return segments

    def _segments(self, route, current, transition_paths=None):
        segments = []
        cursor = current
        online_unsplit = bool(getattr(
            self, "online_hybrid_without_precompute", False
        ))
        hierarchical = bool(getattr(
            self, "hierarchical_hybrid_on_demand", False
        ))
        for swath_index, swath in enumerate(route):
            transit_distance = math.hypot(swath.start.x - cursor.x,
                                          swath.start.y - cursor.y)
            yaw = math.atan2(swath.end.y - swath.start.y,
                             swath.end.x - swath.start.x)
            transition_path = (
                transition_paths[swath_index]
                if transition_paths and swath_index < len(transition_paths)
                else None
            )
            # Entering the first swath is an ordinary point-to-point task,
            # both for the first batch region and after changing regions.
            # Give it directly to Navfn + TEB as one move_base action.  Do not
            # turn the Navfn topology into rolling Hybrid chunks or cusp
            # actions.  Only later same-region line-to-line connectors use
            # direct live-pose Hybrid A*.
            if hierarchical and (
                    swath_index == 0
                    or bool(getattr(
                        self, "navfn_all_swath_transitions", False
                    ))):
                segments.append({
                    "type": "entry",
                    "swath_index": swath_index,
                    "start": cursor,
                    "end": swath.start,
                    "yaw": yaw,
                    "length": transit_distance,
                    "path": None,
                    "gear": 0,
                })
            elif hierarchical:
                # Preserve only the logical swath-entry goal here.  The live
                # pose is used to request the direct connector at execution;
                # no mission-wide Hybrid trajectory is computed or cached.
                segments.append({
                    "type": "rolling_transit",
                    "swath_index": swath_index,
                    "start": cursor,
                    "end": swath.start,
                    "yaw": yaw,
                    "length": transit_distance,
                    "path": None,
                    "gear": 0,
                    "transition_goal": Point(
                        float(swath.start.x), float(swath.start.y)
                    ),
                    "transition_goal_yaw": float(yaw),
                })
            elif online_unsplit:
                # This comparison mode deliberately gives each complete
                # connector to one move_base action. CoverageGlobalPlanner
                # searches from the live pose at 1 Hz; no full connector is
                # cached and no cusp becomes a manager-level action boundary.
                segments.append({
                    "type": "transit",
                    "swath_index": swath_index,
                    "start": cursor,
                    "end": swath.start,
                    "yaw": yaw,
                    "length": transit_distance,
                    "path": None,
                    "gear": 0,
                    "transition_part": 0,
                    "transition_part_count": 1,
                    "transition_goal": Point(
                        float(swath.start.x), float(swath.start.y)
                    ),
                    "transition_goal_yaw": float(yaw),
                    "transition_path": None,
                })
            elif transition_path is not None and transition_path.poses:
                segments.extend(self._hybrid_transition_segments(
                    transition_path,
                    swath_index,
                    cursor,
                    swath.start,
                    yaw,
                ))
            elif swath_index == 0:
                # Retain a defensive legacy fallback for callers that do not
                # provide precomputed paths. Normal execution no longer takes
                # this branch.
                segments.append({
                    "type": "entry",
                    "swath_index": swath_index,
                    "start": cursor,
                    "end": swath.start,
                    "yaw": yaw,
                    "length": transit_distance,
                    "path": None,
                    "gear": 0,
                })
            else:
                segments.append({
                    "type": "transit",
                    "swath_index": swath_index,
                    "start": cursor,
                    "end": swath.start,
                    "yaw": yaw,
                    "length": transit_distance,
                    "path": None,
                    "gear": 0,
                })
            points = sample_path(swath.start, swath.end, self.path_sample_spacing)
            segments.append({
                "type": "sweep",
                "swath_index": swath_index,
                "start": swath.start,
                "end": swath.end,
                "yaw": yaw,
                "length": swath.length,
                "path": self._path_for_points(points, yaw),
            })
            cursor = swath.end
        return segments

    def _lifecycle_wait(self, timeout):
        """Sleep interruptibly when a batch cancel/skip changes generation."""
        with self.lock:
            event = (
                getattr(self, "batch_wake_event", None)
                if getattr(self, "batch_active", False) else None
            )
        if event is None:
            time.sleep(timeout)
            return
        event.wait(timeout)
        event.clear()

    def _wait_while_paused(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.cancel_requested:
                    return False
                paused = self.manual_pause or self.external_pause
                if paused:
                    self.state = "PAUSED"
                    if self.manual_pause:
                        self.detail = (
                            self.manual_pause_reason
                            or "coverage paused; manual resume is required"
                        )
                    else:
                        self.detail = "navigation paused by FOD safety arbitration"
            if not paused:
                return True
            self._lifecycle_wait(0.1)
        return False

    @staticmethod
    def _new_navigation_owner_token():
        return "coverage-{}".format(uuid.uuid4().hex)

    @staticmethod
    def _valid_navigation_owner_token(owner_token):
        prefix = "coverage-"
        suffix = str(owner_token)[len(prefix):]
        return (
            str(owner_token).startswith(prefix)
            and len(suffix) == 32
            and all(character in "0123456789abcdef" for character in suffix)
        )

    def _set_navigation_owner(self, claim, owner_token):
        """Synchronously claim or release the bridge's atomic ownership gate.

        This method performs a cross-node service call and must therefore never
        be invoked while ``self.lock`` is held.  Callers snapshot and later
        revalidate their lifecycle generation around it.
        """
        if not self._valid_navigation_owner_token(owner_token):
            detail = "coverage navigation owner token is invalid"
            rospy.logerr(detail)
            self._navigation_owner_last_outcome = (
                bool(claim), owner_token, "REJECTED"
            )
            return False, detail
        try:
            rospy.wait_for_service(
                self.navigation_owner_service_name, timeout=0.5
            )
            response = self.navigation_owner_client(
                claim=bool(claim), owner_token=owner_token
            )
        except Exception as error:
            detail = "coverage navigation owner service failed: {}".format(error)
            rospy.logerr(detail)
            self._navigation_owner_last_outcome = (
                bool(claim), owner_token, "UNKNOWN"
            )
            return False, detail

        message = str(getattr(response, "message", ""))
        current_owner = str(getattr(response, "current_owner_token", ""))
        claimed = getattr(response, "claimed", None) is True
        if not getattr(response, "success", False):
            detail = message or (
                "coverage navigation owner claim was rejected"
                if claim else "coverage navigation owner release was rejected"
            )
            rospy.logerr(detail)
            state = (
                "RETAINED_NOT_READY"
                if claim and claimed and current_owner == owner_token
                else "REJECTED"
            )
            self._navigation_owner_last_outcome = (
                bool(claim), owner_token, state
            )
            return False, detail
        if claim:
            if not claimed or current_owner != owner_token:
                detail = (
                    "coverage navigation owner claim returned an inconsistent state"
                )
                rospy.logerr(detail)
                self._navigation_owner_last_outcome = (
                    True, owner_token, "UNKNOWN"
                )
                return False, detail
        elif claimed or current_owner:
            detail = (
                "coverage navigation owner release returned an inconsistent state"
            )
            rospy.logerr(detail)
            self._navigation_owner_last_outcome = (
                False, owner_token, "UNKNOWN"
            )
            return False, detail
        self._navigation_owner_last_outcome = (
            bool(claim), owner_token, "READY" if claim else "RELEASED"
        )
        return True, message

    def _claim_navigation_owner_once(self, owner_token):
        """Return READY, RETAINED_NOT_READY, REJECTED, or UNKNOWN."""
        self._navigation_owner_last_outcome = None
        success, detail = self._set_navigation_owner(True, owner_token)
        outcome = self._navigation_owner_last_outcome
        if (
            isinstance(outcome, tuple)
            and len(outcome) == 3
            and outcome[0] is True
            and outcome[1] == owner_token
        ):
            return outcome[2], detail
        # Unit-test adapters and downstream overrides historically expose only
        # the two-value API.  A positive result is still READY; a negative
        # override is an explicit rejection unless it supplies tri-state data.
        return ("READY" if success else "REJECTED"), detail

    def _resolve_navigation_owner_claim(self, owner_token, context):
        """Reconcile an uncertain/retained same-token claim until definitive."""
        last_detail = "navigation owner claim has not started"
        while not rospy.is_shutdown():
            state, last_detail = self._claim_navigation_owner_once(owner_token)
            if state in ("READY", "REJECTED"):
                return state, last_detail
            with self.lock:
                self.detail = (
                    "{}; retaining the same navigation owner token while "
                    "waiting for prior goals to become terminal: {}"
                ).format(context, last_detail) if state == "RETAINED_NOT_READY" else (
                    "{}; reconciling an unknown navigation owner RPC outcome "
                    "with the same token: {}"
                ).format(context, last_detail)
            self._publish_status()
            self._lifecycle_wait(0.1)
        return "UNKNOWN", (
            "ROS shutdown before navigation owner claim was reconciled: {}"
        ).format(last_detail)

    def _transit_profile(self, allow_reverse=None):
        transit_profile = TransitProfile()
        transit_profile.allow_reverse = bool(
            getattr(self, "allow_reverse_transit", True)
            if allow_reverse is None else allow_reverse
        )
        transit_parameters = self._transit_time_parameters(
            getattr(self, "task_time_parameters", None)
        )
        transit_profile.max_forward_speed_mps = (
            transit_parameters.max_forward_speed_mps
        )
        transit_profile.max_reverse_speed_mps = (
            transit_parameters.max_reverse_speed_mps
        )
        transit_profile.max_angular_speed_rps = (
            transit_parameters.max_angular_speed_rps
        )
        transit_profile.linear_accel_mps2 = (
            transit_parameters.linear_accel_mps2
        )
        transit_profile.angular_accel_rps2 = (
            transit_parameters.angular_accel_rps2
        )
        transit_profile.direction_change_penalty_sec = float(getattr(
            self, "direction_change_penalty", 0.50
        ))
        transit_profile.goal_position_tolerance_m = float(getattr(
            self, "entry_position_tolerance", 0.40
        ))
        transit_profile.goal_yaw_tolerance_rad = float(getattr(
            self, "entry_yaw_tolerance", 0.436332
        ))
        transit_profile.replan_period_sec = float(getattr(
            self, "transit_replan_period", 1.00
        ))
        return transit_profile

    def _allocate_hybrid_path_generation(self, plan_id):
        """Return a new immutable-path generation for one complete replan."""
        with self.lock:
            if (
                self.plan_id != plan_id
                or self.cancel_requested
                or not self.active
            ):
                raise ValueError(
                    "Hybrid path generation was canceled or superseded"
                )
            self.hybrid_path_generation = (
                int(self.hybrid_path_generation) + 1
            ) & 0xFFFFFFFF
            if self.hybrid_path_generation == 0:
                self.hybrid_path_generation = 1
            return self.hybrid_path_generation

    def _entry_goal_region_planning_profile(self):
        """Use an inner planning basin for every final swath entrance.

        The outer 0.40 m / 25 degree hand-off remains the business contract.
        Hybrid A* stops at this smaller basin so TEB tracking error cannot
        consume the whole allowance, while retaining the operator's normal
        forward/reverse cost model.
        """
        profile = self._transit_profile()
        profile.goal_position_tolerance_m = float(getattr(
            self, "sweep_entry_recovery_goal_position_tolerance", 0.30
        ))
        profile.goal_yaw_tolerance_rad = float(getattr(
            self, "sweep_entry_recovery_goal_yaw_tolerance", 0.349066
        ))
        return profile

    def _transit_time_parameters(self, base=None):
        """Use a faster hardware-bounded profile only between cleaning lines."""
        if base is None:
            base = getattr(
                self, "task_time_parameters", CoverageTimeParameters()
            )

        def configured(name, fallback):
            value = float(getattr(self, name, 0.0))
            return value if value > 0.0 else float(fallback)

        return CoverageTimeParameters(
            max_forward_speed_mps=configured(
                "hybrid_transit_max_forward_speed",
                base.max_forward_speed_mps,
            ),
            max_reverse_speed_mps=configured(
                "hybrid_transit_max_reverse_speed",
                base.max_reverse_speed_mps,
            ),
            max_angular_speed_rps=configured(
                "hybrid_transit_max_angular_speed",
                base.max_angular_speed_rps,
            ),
            linear_accel_mps2=configured(
                "hybrid_transit_linear_accel", base.linear_accel_mps2
            ),
            angular_accel_rps2=configured(
                "hybrid_transit_angular_accel", base.angular_accel_rps2
            ),
            allow_reverse=bool(base.allow_reverse),
            direction_change_penalty_sec=float(
                base.direction_change_penalty_sec
            ),
            segment_handoff_penalty_sec=float(
                base.segment_handoff_penalty_sec
            ),
            transit_replan_period_sec=float(base.transit_replan_period_sec),
        )

    def _entry_recovery_planning_profile(self):
        """Return a cost profile that does not penalize useful reversing.

        The Hybrid service interprets linear-speed fields as time cost per
        metre. Raising this request's reverse cost-equivalent speed and
        lowering its forward equivalent makes a short reverse alignment
        preferable to a forward loop. Execution still uses the configured
        physical speeds in `_set_teb`; TEB remains the feedback controller.
        """
        profile = self._entry_goal_region_planning_profile()
        profile.allow_reverse = True
        profile.max_forward_speed_mps = float(getattr(
            self, "sweep_entry_recovery_forward_cost_speed", 0.20
        ))
        profile.max_reverse_speed_mps = float(getattr(
            self, "sweep_entry_recovery_reverse_cost_speed", 0.80
        ))
        profile.direction_change_penalty_sec = float(getattr(
            self, "sweep_entry_recovery_direction_change_penalty", 0.15
        ))
        return profile

    def _set_enforced_path(self, enforced, coverage_active=True,
                           allow_reverse=None):
        """Synchronously arm mission ownership and the segment planner mode."""
        transit_profile = self._transit_profile(allow_reverse=allow_reverse)
        try:
            response = self.enforced_path_client(
                coverage_active=coverage_active,
                enforced_path=enforced,
                transit_profile=transit_profile,
            )
        except Exception as error:
            rospy.logerr(
                "coverage planner mode hand-off service failed for segment %d: %s",
                enforced.segment_index,
                error,
            )
            return False
        if not response.success:
            rospy.logerr(
                "coverage planner rejected segment %d mode hand-off: %s",
                enforced.segment_index,
                response.message,
            )
            return False
        # Subsequent topic refreshes keep the plugin's fail-closed freshness
        # timer alive without changing the synchronously acknowledged mode.
        self.enforced_path_pub.publish(enforced)
        return True

    def _record_move_base_goal_terminal(self, generation, state):
        try:
            state = int(state)
        except (TypeError, ValueError, OverflowError):
            state = GoalStatus.LOST
        trusted = state in TRUSTED_MOVE_BASE_TERMINAL_STATES
        with self.lock:
            if self.move_base_goal_generation != generation:
                return False
            self.move_base_goal_terminal_state = state
            if trusted:
                self.move_base_goal_pending = False
        return trusted

    def _send_move_base_goal_locked(self, goal):
        """Send and track one goal while the caller holds the lifecycle lock."""
        generation = self.move_base_goal_generation + 1
        self.move_base_goal_generation = generation
        self.move_base_goal_pending = True
        self.move_base_goal_handle = None
        self.move_base_goal_terminal_state = GoalStatus.PENDING

        def done_callback(state, _result):
            self._record_move_base_goal_terminal(generation, state)

        try:
            self.move_base.send_goal(goal, done_cb=done_callback)
        finally:
            # SimpleActionClient.send_goal returns None; ``gh`` is its tracked
            # GoalHandle and is assigned synchronously by send_goal.  Capture
            # it even on an exception because an uncertain partial send must
            # remain fail-closed rather than be classified as "never sent".
            self.move_base_goal_handle = getattr(self.move_base, "gh", None)
        return generation

    def _confirm_move_base_goal_terminal(
            self, cancel=True, expected_generation=None,
            expected_handle=_EXPECTED_HANDLE_UNSET):
        """Confirm the exact tracked goal reached a trusted terminal state.

        SimpleActionClient maps PREEMPTING/RECALLING to ACTIVE/PENDING and
        reports LOST when it has no trustworthy tracked GoalHandle.  None of
        those states is sufficient to disarm the planner or release ownership.
        """
        with self.lock:
            generation = self.move_base_goal_generation
            if (expected_generation is not None and
                    generation != expected_generation):
                return False, "move_base goal generation changed"
            tracked_handle = self.move_base_goal_handle
            if (expected_handle is not _EXPECTED_HANDLE_UNSET and
                    tracked_handle is not expected_handle):
                return False, "move_base goal handle changed"
            if not self.move_base_goal_pending:
                return True, "move_base goal was never sent or is already terminal"
            timeout = self.move_base_terminal_timeout_sec

        if tracked_handle is None:
            return False, "tracked coverage move_base goal handle is unavailable"
        current_handle = getattr(self.move_base, "gh", None)
        if current_handle is not tracked_handle:
            return False, "move_base is tracking a different goal handle"
        cancel_exact = getattr(tracked_handle, "cancel", None)
        if cancel and not callable(cancel_exact):
            return False, "tracked coverage move_base GoalHandle cannot cancel exactly"
        deadline = time.monotonic() + timeout
        last_untrusted_state = None
        while not rospy.is_shutdown():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            try:
                if cancel:
                    # Cancel the captured ClientGoalHandle itself.  Calling
                    # SimpleActionClient.cancel_goal() here would act on its
                    # mutable latest handle and could cancel a newer mission
                    # if this callback resumed late.
                    cancel_exact()
                finished = self.move_base.wait_for_result(
                    rospy.Duration(min(0.2, remaining))
                )
                if not finished:
                    continue
                state = int(self.move_base.get_state())
            except Exception as error:
                return False, (
                    "move_base terminal confirmation failed: {}"
                ).format(error)

            current_handle = getattr(self.move_base, "gh", None)
            with self.lock:
                if self.move_base_goal_generation != generation:
                    return False, "move_base goal generation changed while waiting"
                if self.move_base_goal_handle is not tracked_handle:
                    return False, "move_base goal handle changed while waiting"
                if current_handle is not tracked_handle:
                    return False, "move_base goal handle changed while waiting"
            if self._record_move_base_goal_terminal(generation, state):
                return True, (
                    "move_base goal reached trusted terminal state {}"
                ).format(state)
            last_untrusted_state = state
            # A malformed/mock client may report DONE immediately forever.
            # Avoid a busy loop while retaining and repeatedly canceling the
            # exact handle until the bounded confirmation window expires.
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

        if last_untrusted_state == GoalStatus.LOST:
            return False, "move_base reported LOST for the tracked coverage goal"
        if last_untrusted_state is not None:
            return False, "move_base goal stopped in untrusted state {}".format(
                last_untrusted_state
            )
        return False, (
            "move_base goal did not reach a terminal state within {:.2f}s"
        ).format(timeout)

    def _wait_for_move_base_goal_terminal(self):
        """Persistently close the tracked action goal before cleanup.

        This runs only in the mission's existing worker/finalizer.  Transient
        timeouts, LOST, or actionlib exceptions therefore cannot strand the
        task with no process left to observe a later trustworthy terminal
        transition.  Ownership remains closed for every failed attempt.
        """
        last_detail = "move_base terminal confirmation has not started"
        while not rospy.is_shutdown():
            confirmed, last_detail = self._confirm_move_base_goal_terminal(
                cancel=True
            )
            if confirmed:
                return True, last_detail
            with self.lock:
                self.navigation_owner_releasing = True
                self.state = "FINALIZING"
                self.detail = (
                    "coverage is retaining navigation ownership while waiting "
                    "for the exact move_base goal to stop: {}"
                ).format(last_detail)
            self._publish_status()
            rospy.logerr_throttle(
                2.0,
                "coverage still waiting for exact move_base terminal state: %s",
                last_detail,
            )
            self._lifecycle_wait(0.1)
        return False, "ROS shutdown before move_base terminal confirmation"

    def _reassert_move_base_goal_cancel(
            self, expected_generation=None,
            expected_handle=_EXPECTED_HANDLE_UNSET):
        with self.lock:
            generation = self.move_base_goal_generation
            if (expected_generation is not None and
                    generation != expected_generation):
                return False, "move_base goal generation changed"
            tracked_handle = self.move_base_goal_handle
            if (expected_handle is not _EXPECTED_HANDLE_UNSET and
                    tracked_handle is not expected_handle):
                return False, "move_base goal handle changed"
            if not self.move_base_goal_pending:
                return True, "coverage move_base goal is already terminal"
        if tracked_handle is None:
            return False, "tracked coverage move_base goal handle is unavailable"
        current_handle = getattr(self.move_base, "gh", None)
        if current_handle is not tracked_handle:
            return False, "move_base is tracking a different goal handle"
        cancel_exact = getattr(tracked_handle, "cancel", None)
        if not callable(cancel_exact):
            return False, "tracked coverage move_base GoalHandle cannot cancel exactly"
        try:
            cancel_exact()
        except Exception as error:
            return False, "exact move_base cancellation failed: {}".format(error)
        return True, "exact coverage move_base cancellation reasserted"

    def _request_exact_move_base_cancel_or_retain_owner(
            self, reason, expected_generation=None,
            expected_handle=_EXPECTED_HANDLE_UNSET):
        """Cancel only this manager's proven goal, otherwise stay closed."""
        success, detail = self._reassert_move_base_goal_cancel(
            expected_generation=expected_generation,
            expected_handle=expected_handle,
        )
        if success:
            return True, detail
        with self.lock:
            if (expected_generation is not None and
                    self.move_base_goal_generation != expected_generation):
                # This callback belongs to an older operation.  The exact old
                # handle was not touched, and it must not put a newer mission
                # into FINALIZING or overwrite that mission's detail.
                return False, detail
            if (expected_handle is not _EXPECTED_HANDLE_UNSET and
                    self.move_base_goal_handle is not expected_handle):
                return False, detail
            self.navigation_owner_releasing = True
            self.state = "FINALIZING"
            self.detail = (
                "{}; coverage is retaining navigation ownership because the "
                "exact move_base goal cannot be canceled yet: {}"
            ).format(reason, detail)
        rospy.logerr("coverage exact move_base cancellation failed: %s", detail)
        return False, detail

    def _cancel_segment_goal(self, generation, outcome):
        confirmed, detail = self._confirm_move_base_goal_terminal(
            cancel=True, expected_generation=generation
        )
        if confirmed:
            return outcome
        with self.lock:
            if self.move_base_goal_generation == generation:
                self.detail = (
                    "coverage cannot leave the current move_base goal: {}"
                ).format(detail)
        rospy.logerr("coverage move_base terminal confirmation failed: %s", detail)
        return "failed"

    @staticmethod
    def _sweep_completion_geometry(segment, point, yaw):
        """Project a live pose onto one directed sweep and its exit plane."""
        start = segment["start"]
        end = segment["end"]
        delta_x = float(end.x) - float(start.x)
        delta_y = float(end.y) - float(start.y)
        length = math.hypot(delta_x, delta_y)
        values = (
            float(point.x), float(point.y), float(yaw),
            float(start.x), float(start.y), float(end.x), float(end.y),
            length,
        )
        if length <= 1.0e-6 or not all(math.isfinite(value)
                                       for value in values):
            return None
        relative_x = float(point.x) - float(start.x)
        relative_y = float(point.y) - float(start.y)
        along = (relative_x * delta_x + relative_y * delta_y) / length
        cross_track = abs(relative_x * delta_y - relative_y * delta_x) / length
        desired_yaw = math.atan2(delta_y, delta_x)
        heading_error = abs(math.atan2(
            math.sin(desired_yaw - float(yaw)),
            math.cos(desired_yaw - float(yaw)),
        ))
        # This is the dot product suggested by the operator, evaluated at E:
        # (P-E) dot (S-E) < 0 means angle P-E-S is greater than 90 degrees,
        # hence P lies beyond the directed exit plane.  Its magnitude is kept
        # in metre-squared units for diagnostics.
        exit_dot = (
            (float(point.x) - float(end.x)) *
            (float(start.x) - float(end.x))
            + (float(point.y) - float(end.y)) *
            (float(start.y) - float(end.y))
        )
        return {
            "length": length,
            "along": along,
            "cross_track": cross_track,
            "heading_error": heading_error,
            "exit_dot": exit_dot,
        }

    @staticmethod
    def _new_sweep_completion_tracker():
        return {
            "armed": False,
            "invalidated": False,
            "last_point": None,
            "last_along": None,
            "max_along": float("-inf"),
            "pass_samples": 0,
            "entry_deviation_samples": 0,
        }

    def _observe_sweep_entry_recovery(self, segment, tracker, point, yaw):
        """Detect a persistent, early sweep-entry acquisition failure.

        The normal connector acceptance tolerance remains the desired hand-off
        pose.  This guard uses the wider sweep acquisition gate as a hard
        boundary: small disturbances are left to straight-line tracking, while
        a pose outside that basin is replanned kinematically.  Restricting the
        test to the first part of the swath prevents normal accumulated
        cross-track error later in a sweep from being mistaken for an entry
        failure.
        """
        geometry = self._sweep_completion_geometry(segment, point, yaw)
        if geometry is None:
            tracker["entry_deviation_samples"] = 0
            return False, geometry
        start = segment["start"]
        start_distance = math.hypot(
            float(point.x) - float(start.x),
            float(point.y) - float(start.y),
        )
        early = geometry["along"] <= float(getattr(
            self, "sweep_entry_recovery_progress_limit", 0.75
        ))
        acquisition_distance = float(getattr(
            self, "sweep_completion_start_gate", 0.45
        ))
        # Distance to the start naturally grows as a valid sweep advances.
        # Acquisition failure is therefore lateral/heading/behind-start, not
        # Euclidean distance from the first point.
        outside_acquisition = (
            geometry["cross_track"] > acquisition_distance
            or geometry["along"] < -acquisition_distance
            or geometry["heading_error"] > float(getattr(
                self, "entry_yaw_tolerance", 0.436332
            ))
        )
        tracker["entry_deviation_samples"] = (
            int(tracker.get("entry_deviation_samples", 0)) + 1
            if early and outside_acquisition else 0
        )
        geometry.update({
            "start_distance": start_distance,
            "entry_deviation_samples": tracker["entry_deviation_samples"],
            "outside_entry_acquisition": outside_acquisition,
        })
        return (
            bool(getattr(self, "sweep_entry_recovery_enabled", True))
            and tracker["entry_deviation_samples"] >= int(getattr(
                self, "sweep_entry_recovery_confirmation_samples", 3
            )),
            geometry,
        )

    def _observe_sweep_completion(
            self, segment, tracker, point, yaw, linear_speed=None):
        """Return directed-line completion after physical speed reaches zero.

        The exit-plane dot product is deliberately only the final condition.
        A tracker first has to see this exact segment at its entrance, then see
        continuous map poses cover most of its length without a localization
        jump, and finally remain close and aligned to the same directed line.
        """
        geometry = self._sweep_completion_geometry(segment, point, yaw)
        if geometry is None or tracker.get("invalidated", False):
            return False, geometry

        start = segment["start"]
        start_distance = math.hypot(
            float(point.x) - float(start.x),
            float(point.y) - float(start.y),
        )
        cross_tolerance = float(getattr(
            self, "sweep_completion_cross_track_tolerance", 0.30
        ))
        heading_tolerance = float(getattr(
            self, "sweep_completion_heading_tolerance", 0.35
        ))
        if not tracker.get("armed", False):
            start_heading_tolerance = max(
                heading_tolerance,
                float(getattr(self, "entry_yaw_tolerance", 0.436332)),
            )
            if (
                start_distance <= float(getattr(
                    self, "sweep_completion_start_gate", 0.45
                ))
                and geometry["cross_track"] <= cross_tolerance
                and geometry["heading_error"] <= start_heading_tolerance
            ):
                tracker["armed"] = True
                tracker["last_point"] = (float(point.x), float(point.y))
                tracker["last_along"] = geometry["along"]
                tracker["max_along"] = geometry["along"]
            geometry["armed"] = tracker.get("armed", False)
            geometry["progress_ratio"] = max(
                0.0, geometry["along"] / geometry["length"]
            )
            geometry["pass_samples"] = 0
            return False, geometry

        previous_x, previous_y = tracker["last_point"]
        sample_step = math.hypot(
            float(point.x) - previous_x,
            float(point.y) - previous_y,
        )
        if sample_step > float(getattr(
                self, "sweep_completion_max_sample_step", 0.50)):
            # A single TF jump must never be allowed to manufacture the
            # entrance-to-exit history needed for completion.  The existing
            # tracking timer will independently latch the mission PAUSED.
            tracker["invalidated"] = True
            tracker["pass_samples"] = 0
            geometry["armed"] = True
            geometry["invalidated"] = True
            geometry["sample_step"] = sample_step
            return False, geometry

        previous_along = float(tracker["last_along"])
        tracker["last_point"] = (float(point.x), float(point.y))
        tracker["last_along"] = geometry["along"]
        tracker["max_along"] = max(
            float(tracker["max_along"]), geometry["along"]
        )
        progress_ratio = tracker["max_along"] / geometry["length"]
        pass_margin = float(getattr(
            self, "sweep_completion_pass_margin", 0.02
        ))
        progressed = progress_ratio >= float(getattr(
            self, "sweep_completion_min_progress_ratio", 0.90
        ))
        passed_exit = (
            geometry["along"] >= geometry["length"] + pass_margin
            and geometry["exit_dot"] <= -pass_margin * geometry["length"]
        )
        speed_ready = (
            linear_speed is not None
            and math.isfinite(float(linear_speed))
            and abs(float(linear_speed)) <= float(getattr(
                self, "sweep_completion_max_linear_speed", 0.08
            ))
        )
        # The sweep forbids reverse.  Requiring non-decreasing longitudinal
        # samples additionally rejects a pose sequence that approaches the
        # exit plane from its far side after an unrelated maneuver.
        forward_sample = geometry["along"] >= previous_along - 0.02
        candidate = (
            progressed
            and passed_exit
            and forward_sample
            and geometry["cross_track"] <= cross_tolerance
            and geometry["heading_error"] <= heading_tolerance
            and speed_ready
        )
        tracker["pass_samples"] = (
            int(tracker["pass_samples"]) + 1 if candidate else 0
        )
        geometry.update({
            "armed": True,
            "invalidated": False,
            "sample_step": sample_step,
            "progress_ratio": progress_ratio,
            "pass_samples": tracker["pass_samples"],
            "linear_speed": (
                float(linear_speed) if linear_speed is not None else None
            ),
        })
        return (
            tracker["pass_samples"] >= int(getattr(
                self, "sweep_completion_confirmation_samples", 2
            )),
            geometry,
        )

    def _transition_completion_geometry(self, segment, point, yaw):
        """Project a live pose onto a constant-gear path or swath entrance."""
        path = segment.get("path")
        if path is not None and len(path.poses) >= 2:
            poses = path.poses
            cumulative = 0.0
            best = None
            terminal_edge = None
            for first, second in zip(poses, poses[1:]):
                first_x = float(first.pose.position.x)
                first_y = float(first.pose.position.y)
                second_x = float(second.pose.position.x)
                second_y = float(second.pose.position.y)
                delta_x = second_x - first_x
                delta_y = second_y - first_y
                edge_length = math.hypot(delta_x, delta_y)
                if edge_length <= 1.0e-9:
                    continue
                relative_x = float(point.x) - first_x
                relative_y = float(point.y) - first_y
                fraction = max(0.0, min(
                    1.0,
                    (relative_x * delta_x + relative_y * delta_y) /
                    (edge_length * edge_length),
                ))
                projection_x = first_x + fraction * delta_x
                projection_y = first_y + fraction * delta_y
                distance = math.hypot(
                    float(point.x) - projection_x,
                    float(point.y) - projection_y,
                )
                progress = cumulative + fraction * edge_length
                first_yaw = self._hybrid_path_pose_yaw(first)
                second_yaw = self._hybrid_path_pose_yaw(second)
                nearest_yaw = first_yaw + fraction * math.atan2(
                    math.sin(second_yaw - first_yaw),
                    math.cos(second_yaw - first_yaw),
                )
                path_yaw_error = abs(math.atan2(
                    math.sin(nearest_yaw - float(yaw)),
                    math.cos(nearest_yaw - float(yaw)),
                ))
                # A Reeds-Shepp connector can pass close to itself around a
                # cusp. Pure distance plus a "furthest progress" tie-break
                # then jumped to a later edge with a different body yaw and
                # falsely declared a tracking failure. Body yaw is continuous
                # across a real gear change, so use it to disambiguate nearby
                # edges while retaining the metric path distance for limits.
                candidate = (
                    distance + 0.50 * path_yaw_error,
                    distance,
                    -progress,
                    progress,
                    nearest_yaw,
                )
                if best is None or candidate[:3] < best[:3]:
                    best = candidate
                cumulative += edge_length
                terminal_edge = (delta_x / edge_length,
                                 delta_y / edge_length)
            if best is None or terminal_edge is None or cumulative <= 1.0e-6:
                return None
            start = poses[0].pose.position
            end = poses[-1].pose.position
            desired_yaw = self._hybrid_path_pose_yaw(poses[-1])
            progress = best[3]
            distance_to_path = best[1]
            nearest_path_yaw = best[4]
            start_distance = math.hypot(
                float(point.x) - float(start.x),
                float(point.y) - float(start.y),
            )
            path_length = cumulative
            tangent_x, tangent_y = terminal_edge
        else:
            # Navfn supplies no stable global path to the manager.  Near the
            # first swath, use a short directed corridor aligned with the
            # swath itself; this accepts entering the line but cannot be armed
            # by merely passing beside the entrance from an unrelated route.
            end = segment["end"]
            desired_yaw = float(segment["yaw"])
            tangent_x = math.cos(desired_yaw)
            tangent_y = math.sin(desired_yaw)
            nearest_path_yaw = desired_yaw
            path_length = float(getattr(
                self, "transition_completion_start_gate", 0.60
            ))
            start_x = float(end.x) - path_length * tangent_x
            start_y = float(end.y) - path_length * tangent_y
            relative_start_x = float(point.x) - start_x
            relative_start_y = float(point.y) - start_y
            progress = (
                relative_start_x * tangent_x
                + relative_start_y * tangent_y
            )
            distance_to_path = abs(
                relative_start_x * tangent_y
                - relative_start_y * tangent_x
            )
            start_distance = math.hypot(
                float(point.x) - start_x,
                float(point.y) - start_y,
            )

        terminal_relative_x = float(point.x) - float(end.x)
        terminal_relative_y = float(point.y) - float(end.y)
        terminal_along = (
            terminal_relative_x * tangent_x
            + terminal_relative_y * tangent_y
        )
        terminal_cross = abs(
            terminal_relative_x * tangent_y
            - terminal_relative_y * tangent_x
        )
        heading_error = abs(math.atan2(
            math.sin(desired_yaw - float(yaw)),
            math.cos(desired_yaw - float(yaw)),
        ))
        path_heading_error = abs(math.atan2(
            math.sin(nearest_path_yaw - float(yaw)),
            math.cos(nearest_path_yaw - float(yaw)),
        ))
        values = (
            path_length, progress, distance_to_path, start_distance,
            terminal_along, terminal_cross, heading_error,
            path_heading_error,
        )
        if not all(math.isfinite(value) for value in values):
            return None
        return {
            "length": path_length,
            "progress": progress,
            "progress_ratio": progress / path_length,
            "distance_to_path": distance_to_path,
            "start_distance": start_distance,
            "terminal_along": terminal_along,
            "terminal_cross": terminal_cross,
            "heading_error": heading_error,
            "path_heading_error": path_heading_error,
        }

    @staticmethod
    def _new_transition_completion_tracker():
        return {
            "armed": False,
            "invalidated": False,
            "last_point": None,
            "last_progress": None,
            "max_progress": float("-inf"),
            "pass_samples": 0,
            "path_deviation_samples": 0,
        }

    def _observe_transition_completion(self, segment, tracker, point, yaw):
        """Recognize a continuously approached cusp or swath-entry plane."""
        geometry = self._transition_completion_geometry(
            segment, point, yaw
        )
        if geometry is None or tracker.get("invalidated", False):
            return False, geometry

        cross_tolerance = float(getattr(
            self, "transition_completion_cross_track_tolerance", 0.30
        ))
        start_gate = float(getattr(
            self, "transition_completion_start_gate", 0.60
        ))
        is_cusp = (
            segment.get("type") == "transit"
            and int(segment.get("transition_part", 0)) + 1
                < int(segment.get("transition_part_count", 1))
        )
        precise_hybrid_goal = (
            is_cusp or bool(segment.get("entry_goal_region", False))
        )
        monitored_hybrid_path = (
            segment.get("type") == "transit"
            and segment.get("path") is not None
            and len(segment["path"].poses) >= 2
        )
        outside_tracked_path = monitored_hybrid_path and (
            geometry["distance_to_path"] > float(getattr(
                self, "transition_tracking_deviation_tolerance", 0.35
            ))
            or geometry["path_heading_error"] > float(getattr(
                self, "transition_tracking_heading_tolerance", 0.40
            ))
        )
        tracker["path_deviation_samples"] = (
            int(tracker.get("path_deviation_samples", 0)) + 1
            if outside_tracked_path else 0
        )
        requires_path_replan = (
            tracker["path_deviation_samples"] >= int(getattr(
                self,
                "transition_tracking_deviation_confirmation_samples",
                3,
            ))
        )
        heading_tolerance = float(
            getattr(self, "hybrid_cusp_yaw_tolerance", 0.15)
            if precise_hybrid_goal else
            getattr(self, "entry_yaw_tolerance", 0.436332)
        )
        if not tracker.get("armed", False):
            near_path_start = geometry["start_distance"] <= start_gate
            early_on_path = (
                -0.25 <= geometry["progress_ratio"] <= 0.25
                and geometry["distance_to_path"] <= cross_tolerance
            )
            # A same-line Reeds-Shepp turn normally starts with the opposite
            # body yaw from its final swath-entry yaw.  Arm a monitored Hybrid
            # A* path against the local path tangent; the final yaw remains a
            # mandatory condition in the completion gate below.  Using the
            # terminal yaw here left U-turn trackers permanently unarmed, so
            # a valid stopped hand-off was ignored until move_base timed out.
            arming_heading_error = (
                geometry["path_heading_error"]
                if monitored_hybrid_path else geometry["heading_error"]
            )
            if (
                (near_path_start or early_on_path)
                and arming_heading_error <= max(
                    heading_tolerance,
                    float(getattr(self, "entry_yaw_tolerance", 0.436332)),
                )
            ):
                tracker["armed"] = True
                tracker["last_point"] = (float(point.x), float(point.y))
                tracker["last_progress"] = geometry["progress"]
                tracker["max_progress"] = geometry["progress"]
            geometry.update({
                "armed": tracker.get("armed", False),
                "invalidated": False,
                "pass_samples": 0,
                "requires_replan": False,
                "path_deviation_samples": tracker[
                    "path_deviation_samples"
                ],
                "requires_path_replan": requires_path_replan,
            })
            return False, geometry

        previous_x, previous_y = tracker["last_point"]
        sample_step = math.hypot(
            float(point.x) - previous_x,
            float(point.y) - previous_y,
        )
        if sample_step > float(getattr(
                self, "transition_completion_max_sample_step", 0.50)):
            tracker["invalidated"] = True
            tracker["pass_samples"] = 0
            geometry.update({
                "armed": True,
                "invalidated": True,
                "sample_step": sample_step,
                "pass_samples": 0,
                "requires_replan": False,
            })
            return False, geometry

        previous_progress = float(tracker["last_progress"])
        tracker["last_point"] = (float(point.x), float(point.y))
        tracker["last_progress"] = geometry["progress"]
        tracker["max_progress"] = max(
            float(tracker["max_progress"]), geometry["progress"]
        )
        progress_ratio = tracker["max_progress"] / geometry["length"]
        pass_margin = float(getattr(
            self, "transition_completion_pass_margin", 0.02
        ))
        progressed = progress_ratio >= float(getattr(
            self, "transition_completion_min_progress_ratio", 0.85
        ))
        passed_boundary = geometry["terminal_along"] >= pass_margin
        monotonic_sample = geometry["progress"] >= previous_progress - 0.03
        goal_position_tolerance = float(
            getattr(self, "hybrid_cusp_position_tolerance", 0.12)
            if precise_hybrid_goal else
            getattr(self, "entry_position_tolerance", 0.40)
        )
        entry_goal_region = (
            bool(segment.get("entry_goal_region", False)) and not is_cusp
        )
        handoff_goal = segment.get("entry_handoff_goal")
        handoff_yaw = segment.get("entry_handoff_yaw")
        handoff_position_error = None
        handoff_heading_error = None
        handoff_cross_error = None
        handoff_cross_tolerance = float(getattr(
            self, "entry_position_tolerance", 0.40
        ))
        if (
            entry_goal_region
            and handoff_goal is not None
            and handoff_yaw is not None
        ):
            handoff_position_error = math.hypot(
                float(point.x) - float(handoff_goal.x),
                float(point.y) - float(handoff_goal.y),
            )
            handoff_heading_error = abs(math.atan2(
                math.sin(float(handoff_yaw) - float(yaw)),
                math.cos(float(handoff_yaw) - float(yaw)),
            ))
            handoff_cross_error = abs(
                -(float(point.x) - float(handoff_goal.x)) *
                    math.sin(float(handoff_yaw))
                + (float(point.y) - float(handoff_goal.y)) *
                    math.cos(float(handoff_yaw))
            )
            within_goal_pose = (
                handoff_position_error <= float(getattr(
                    self, "entry_position_tolerance", 0.40
                ))
                and handoff_cross_error <= handoff_cross_tolerance
                and handoff_heading_error <= float(getattr(
                    self, "entry_yaw_tolerance", 0.436332
                ))
            )
            heading_ready = handoff_heading_error <= float(getattr(
                self, "entry_yaw_tolerance", 0.436332
            ))
        else:
            within_goal_pose = (
                math.hypot(
                    geometry["terminal_along"], geometry["terminal_cross"]
                ) <= goal_position_tolerance
                and geometry["heading_error"] <= heading_tolerance
            )
            heading_ready = geometry["heading_error"] <= heading_tolerance
        cross_ready = (
            handoff_cross_error <= handoff_cross_tolerance
            if entry_goal_region else
            geometry["terminal_cross"] <= cross_tolerance
        )
        overshoot_ready = (
            True if entry_goal_region else
            geometry["terminal_along"] <= float(getattr(
                self, "transition_completion_max_overshoot", 0.60
            ))
        )
        # A cusp position tolerance absorbs stopping error; it must not let a
        # 0.30 m fixed-gear action finish after moving only a few centimetres.
        # Intermediate cusps therefore also require directed path progress.
        # The final swath-entry region deliberately remains immediately
        # acceptable so the chassis never chases an internal lattice sample.
        pose_acceptance = within_goal_pose and (not is_cusp or progressed)
        candidate = (
            (pose_acceptance or
             (not entry_goal_region
              and progressed and passed_boundary and monotonic_sample))
            and cross_ready
            and heading_ready
            and overshoot_ready
        )
        tracker["pass_samples"] = (
            int(tracker["pass_samples"]) + 1 if candidate else 0
        )
        requires_replan = (
            progressed
            and geometry["terminal_along"] > float(getattr(
                self, "transition_completion_max_overshoot", 0.60
            ))
        )
        geometry.update({
            "armed": True,
            "invalidated": False,
            "sample_step": sample_step,
            "progress_ratio": progress_ratio,
            "pass_samples": tracker["pass_samples"],
            "requires_replan": requires_replan,
            "path_deviation_samples": tracker["path_deviation_samples"],
            "requires_path_replan": requires_path_replan,
            "is_cusp": is_cusp,
            "within_goal_pose": within_goal_pose,
            "entry_goal_region": entry_goal_region,
            "handoff_position_error": handoff_position_error,
            "handoff_heading_error": handoff_heading_error,
            "handoff_cross_error": handoff_cross_error,
        })
        confirmation_samples = (
            1 if entry_goal_region else int(getattr(
                self, "transition_completion_confirmation_samples", 2
            ))
        )
        return (
            tracker["pass_samples"] >= confirmation_samples,
            geometry,
        )

    def _wait_for_transition_stop(self):
        """Require fresh physical /odom to confirm a completed hand-off stop."""
        deadline = time.monotonic() + float(getattr(
            self, "transition_completion_stop_timeout", 3.0
        ))
        consecutive = 0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                if self.cancel_requested:
                    return False
            speed = self._current_chassis_linear_speed()
            if (
                speed is not None
                and math.isfinite(float(speed))
                and abs(float(speed)) <= float(getattr(
                    self, "transition_completion_stop_speed", 0.08
                ))
            ):
                consecutive += 1
                if consecutive >= 2:
                    return True
            else:
                consecutive = 0
            self._lifecycle_wait(float(getattr(
                self, "transition_completion_poll_period", 0.05
            )))
        return False

    def _complete_transition_boundary(self, goal_generation, geometry,
                                      segment_index):
        outcome = self._cancel_segment_goal(
            goal_generation, "boundary-crossed"
        )
        if outcome != "boundary-crossed":
            return outcome
        if self._wait_for_transition_stop():
            rospy.loginfo(
                "coverage connector %d accepted by directed boundary: "
                "progress %.1f%%, terminal %.3fm, cross %.3fm, yaw %.1fdeg",
                segment_index + 1,
                100.0 * geometry["progress_ratio"],
                geometry["terminal_along"],
                geometry["terminal_cross"],
                math.degrees(geometry["heading_error"]),
            )
            return "succeeded"
        with self.lock:
            if self.cancel_requested:
                return "canceled"
            self.manual_pause = True
            self.state = "PAUSED"
            self.manual_pause_reason = (
                "connector boundary crossed but physical /odom did not confirm "
                "zero speed; inspect the chassis before resuming"
            )
            self.detail = self.manual_pause_reason
        return "paused"

    def _execute_segment(self, segment, segment_index):
        if not self._wait_while_paused():
            return "canceled"
        with self.lock:
            if (self.cancel_requested or not self.active or
                    self.navigation_owner_releasing):
                return "canceled"
            if self.move_base_goal_pending:
                self.detail = (
                    "previous coverage move_base goal is not safely terminal"
                )
                return "failed"
            segment_plan_id = self.plan_id
            owner_token = self.navigation_owner_token
            if not self._valid_navigation_owner_token(owner_token):
                self.detail = "coverage navigation ownership is not established"
                return "failed"
        if (
            segment["type"] == "rolling_transit"
            or bool(segment.get("ordinary_topology_entry", False))
        ):
            return self._execute_rolling_hybrid_transition(
                segment, segment_index
            )
        hybrid_cusp_goal = (
            segment["type"] == "transit"
            and int(segment.get("transition_part", 0)) + 1
                < int(segment.get("transition_part_count", 1))
        )
        precise_hybrid_goal = (
            hybrid_cusp_goal or bool(segment.get("entry_goal_region", False))
        )
        if segment["type"] in ("entry", "transit"):
            live_pose = self._current_pose()
            if live_pose is not None:
                live_point, live_yaw = live_pose
                position_error = math.hypot(
                    segment["end"].x - live_point.x,
                    segment["end"].y - live_point.y,
                )
                yaw_error = abs(math.atan2(
                    math.sin(segment["yaw"] - live_yaw),
                    math.cos(segment["yaw"] - live_yaw),
                ))
                position_tolerance = (
                    self.hybrid_cusp_position_tolerance
                    if precise_hybrid_goal else self.entry_position_tolerance
                )
                yaw_tolerance = (
                    self.hybrid_cusp_yaw_tolerance
                    if precise_hybrid_goal else self.entry_yaw_tolerance
                )
                if position_error <= position_tolerance:
                    if yaw_error <= yaw_tolerance:
                        rospy.loginfo(
                            "coverage transit %d already satisfies entry pose: "
                            "position %.3f <= %.3f m, yaw %.1f <= %.1f deg",
                            segment_index + 1,
                            position_error,
                            position_tolerance,
                            math.degrees(yaw_error),
                            math.degrees(yaw_tolerance),
                        )
                        return "succeeded"
                    rospy.loginfo(
                        "coverage connector %d is at the entry position but yaw "
                        "error %.1f deg exceeds the %.1f deg entry tolerance; "
                        "requesting a %s orientation maneuver",
                        segment_index + 1,
                        math.degrees(yaw_error),
                        math.degrees(yaw_tolerance),
                        "reverse-enabled Navfn + TEB"
                        if (segment["type"] == "entry" and
                            self.allow_reverse_transit)
                        else ("Navfn + TEB" if segment["type"] == "entry"
                              else "Hybrid A*"),
                    )
        if segment["type"] == "sweep":
            # A transit may move several metres while tracking is disabled.
            # Never compare the next swath's first sample with the previous
            # swath endpoint and misclassify that legitimate transit as a TF
            # localization jump.
            with self.lock:
                self.last_tracked_point = None
        if (segment["type"] == "transit" and
                not getattr(self, "online_hybrid_without_precompute", False) and
                (segment.get("path") is None or
                 len(segment["path"].poses) < 2)):
            with self.lock:
                self.detail = (
                    "Hybrid transition cache is missing; refusing to skip its swath"
                )
            return "blocked"
        # Navfn continues to choose the geometric entry route, while TEB may
        # execute a short reverse motion whenever the operator has allowed it.
        # Previously the first entry forced this limit to zero until a second
        # orientation-only retry, which could leave an Ackermann chassis
        # stopped at the line entrance with no feasible forward command.
        expected_gear = int(segment.get("gear", 0))
        if expected_gear not in (-1, 0, 1):
            with self.lock:
                self.detail = "Hybrid transition contains an invalid signed gear"
            return "failed"
        if (
            segment["type"] == "transit"
            and expected_gear < 0
            and not self.allow_reverse_transit
        ):
            with self.lock:
                self.detail = (
                    "Hybrid transition requires reverse while reverse transit "
                    "is disabled"
                )
            return "failed"
        # Every Hybrid action contains exactly one signed gear.  Allowing a
        # forward action to invent a reverse escape recreates cusp oscillation;
        # TEB is therefore hardened to that action's gear and every cusp is a
        # separate action with a mandatory physical stop.
        hard_hierarchical_gear = (
            bool(getattr(self, "hierarchical_hybrid_on_demand", False))
            and segment["type"] == "transit"
            and expected_gear != 0
        )
        backwards = self.reverse_transit_speed if (
            self.allow_reverse_transit
            and segment["type"] in ("entry", "transit")
            and (not hard_hierarchical_gear or expected_gear < 0)
        ) else 0.0
        self._teb_hybrid_tracking_requested = segment["type"] == "transit"
        self._teb_hybrid_expected_reverse = (
            segment["type"] == "transit" and expected_gear < 0
        )
        self._teb_hybrid_cusp_goal = precise_hybrid_goal
        try:
            teb_ready = self._set_teb(
                backwards, segment["type"] == "sweep"
            )
        finally:
            self._teb_hybrid_tracking_requested = False
            self._teb_hybrid_expected_reverse = False
            self._teb_hybrid_cusp_goal = False
        if not teb_ready:
            return "failed"
        enforced = EnforcedPath()
        enforced.header.frame_id = "map"
        enforced.plan_id = segment_plan_id
        enforced.segment_index = segment_index
        enforced.path_generation = int(segment.get("path_generation", 0))
        enforced.expected_gear = EnforcedPath.GEAR_AUTO
        if segment["type"] == "entry":
            enforced.planner_mode = EnforcedPath.MODE_COVERAGE_NAVFN
        elif segment["type"] == "transit":
            enforced.planner_mode = EnforcedPath.MODE_HYBRID_TRANSIT
            enforced.expected_gear = expected_gear
        else:
            enforced.planner_mode = EnforcedPath.MODE_ENFORCED_SWEEP
        enforced.active = (
            enforced.planner_mode == EnforcedPath.MODE_ENFORCED_SWEEP
        )
        if segment.get("path") is not None:
            enforced.path = segment["path"]
        goal = MoveBaseGoal()
        goal.target_pose = self._pose(segment["end"], segment["yaw"])
        timeout = self.goal_timeout_base_sec + (
            self.goal_timeout_per_meter_sec * segment["length"]
        )
        started = time.monotonic()
        hybrid_no_progress_timeout = getattr(
            self, "hybrid_no_progress_timeout", 8.0
        )
        hybrid_no_progress_distance = getattr(
            self, "hybrid_no_progress_distance", 0.10
        )
        progress_anchor = None
        progress_anchor_value = None
        progress_anchor_time = started
        sweep_completion_tracker = None
        transition_completion_tracker = None
        if segment["type"] == "transit":
            live_pose = self._current_pose()
            if live_pose is not None:
                progress_anchor = live_pose[0]
            transition_completion_tracker = (
                self._new_transition_completion_tracker()
            )
            if live_pose is not None:
                _, initial_geometry = self._observe_transition_completion(
                    segment,
                    transition_completion_tracker,
                    live_pose[0],
                    live_pose[1],
                )
                if initial_geometry is not None:
                    progress_anchor_value = initial_geometry["progress"]
        elif segment["type"] == "entry":
            transition_completion_tracker = (
                self._new_transition_completion_tracker()
            )
            live_pose = self._current_pose()
            if live_pose is not None:
                self._observe_transition_completion(
                    segment,
                    transition_completion_tracker,
                    live_pose[0],
                    live_pose[1],
                )
        elif segment["type"] == "sweep":
            sweep_completion_tracker = self._new_sweep_completion_tracker()
            live_pose = self._current_pose()
            if live_pose is not None:
                self._observe_sweep_completion(
                    segment,
                    sweep_completion_tracker,
                    live_pose[0],
                    live_pose[1],
                )
        # Arm the exact sweep or Hybrid A* transit mode before move_base accepts
        # the goal.  Without this ordering its first planner cycle could still
        # observe the previous segment mode and execute the wrong path class.
        enforced.header.stamp = rospy.Time.now()
        if enforced.path.poses:
            enforced.path.header.stamp = enforced.header.stamp
        allow_reverse_override = None
        if segment["type"] == "transit" and expected_gear != 0:
            allow_reverse_override = expected_gear < 0
        planner_armed = (
            self._set_enforced_path(enforced)
            if allow_reverse_override is None else
            self._set_enforced_path(
                enforced, allow_reverse=allow_reverse_override
            )
        )
        if not planner_armed:
            return "failed"
        # Reassert the same mission token before every action goal.  This is
        # deliberately outside the lifecycle lock because it crosses ROS node
        # boundaries; the generation is revalidated below before send_goal.
        owner_state, owner_detail = self._resolve_navigation_owner_claim(
            owner_token, "coverage segment owner reassertion is pending"
        )
        if owner_state != "READY":
            with self.lock:
                if self.navigation_owner_token == owner_token:
                    self.detail = (
                        "coverage navigation ownership could not be reasserted: {}"
                    ).format(owner_detail)
            return "failed"
        # Linearize goal submission against cancel/skip/map invalidation.  The
        # planner hand-off above is synchronous but may yield long enough for a
        # cancellation request; never submit a stale goal after that request.
        stale_claim = False
        compensate_stale_claim = False
        result = ""
        goal_generation = None
        with self.lock:
            if (
                self.cancel_requested
                or not self.active
                or self.plan_id != segment_plan_id
                or self.navigation_owner_token != owner_token
                or self.navigation_owner_releasing
            ):
                stale_claim = True
                # Ordinary cancel/skip keeps the same batch owner until its
                # unified finalizer.  Compensation is only needed when a
                # finalizer may already have released this token while the
                # claim call was in flight, or the lifecycle token changed.
                compensate_stale_claim = (
                    self.navigation_owner_releasing
                    or self.navigation_owner_token != owner_token
                )
                result = "canceled"
            elif self.manual_pause or self.external_pause:
                result = "paused"
            else:
                self.navigation_owner_claimed = True
                goal_generation = self._send_move_base_goal_locked(goal)
        if stale_claim and compensate_stale_claim:
            # A finalizer can release while this service call is in flight; a
            # delayed idempotent claim could then reacquire the old token.  A
            # token-scoped compensation prevents that stale generation from
            # surviving, and never releases a newer owner's different token.
            released, release_detail = self._set_navigation_owner(
                False, owner_token
            )
            if not released:
                rospy.logerr(
                    "stale coverage owner compensation failed: %s",
                    release_detail,
                )
            return result
        if stale_claim:
            return result
        if result:
            return result
        while not rospy.is_shutdown():
            self._pause_for_avoidance_loss()
            enforced.header.stamp = rospy.Time.now()
            if enforced.path.poses:
                enforced.path.header.stamp = enforced.header.stamp
            self.enforced_path_pub.publish(enforced)
            wait_period = (
                float(getattr(
                    self, "sweep_completion_poll_period", 0.05
                )) if segment["type"] == "sweep" else (
                    float(getattr(
                        self, "transition_completion_poll_period", 0.05
                    )) if segment["type"] in ("entry", "transit") else 0.2
                )
            )
            if self.move_base.wait_for_result(rospy.Duration(wait_period)):
                state = int(self.move_base.get_state())
                terminal_trusted = self._record_move_base_goal_terminal(
                    goal_generation, state
                )
                if not terminal_trusted:
                    with self.lock:
                        self.detail = (
                            "move_base returned untrusted terminal state {}"
                        ).format(state)
                    return "failed"
                if state == GoalStatus.SUCCEEDED:
                    return "succeeded"
                with self.lock:
                    if self.cancel_requested:
                        return "canceled"
                    if self.manual_pause or self.external_pause:
                        return "paused"
                return "blocked" if state in (
                    GoalStatus.ABORTED, GoalStatus.REJECTED
                ) else "failed"
            with self.lock:
                if self.cancel_requested:
                    cancel_requested = True
                else:
                    cancel_requested = False
                paused = self.manual_pause or self.external_pause
            if cancel_requested:
                return self._cancel_segment_goal(
                    goal_generation, "canceled"
                )
            if paused:
                return self._cancel_segment_goal(goal_generation, "paused")
            if not self._localization_is_fresh():
                with self.lock:
                    self.manual_pause = True
                    self.state = "PAUSED"
                    self.manual_pause_reason = (
                        "localization lost; manual resume is required"
                    )
                    self.detail = self.manual_pause_reason
                return self._cancel_segment_goal(goal_generation, "paused")
            if segment["type"] == "sweep":
                live_pose = self._current_pose()
                sweep_complete = False
                completion_geometry = None
                if live_pose is not None:
                    needs_entry_recovery, entry_geometry = (
                        self._observe_sweep_entry_recovery(
                            segment,
                            sweep_completion_tracker,
                            live_pose[0],
                            live_pose[1],
                        )
                    )
                    if needs_entry_recovery:
                        rospy.logwarn(
                            "coverage sweep %d entry left its acquisition "
                            "basin: start distance %.3fm, along %.3fm, "
                            "cross-track %.3fm, heading error %.1fdeg; "
                            "canceling before rolling Hybrid recovery",
                            segment_index + 1,
                            entry_geometry["start_distance"],
                            entry_geometry["along"],
                            entry_geometry["cross_track"],
                            math.degrees(entry_geometry["heading_error"]),
                        )
                        outcome = self._cancel_segment_goal(
                            goal_generation, "entry-recovery"
                        )
                        if outcome != "entry-recovery":
                            return outcome
                        if not self._wait_for_transition_stop():
                            with self.lock:
                                if self.cancel_requested:
                                    return "canceled"
                                self.manual_pause = True
                                self.state = "PAUSED"
                                self.manual_pause_reason = (
                                    "sweep entry recovery canceled its goal but "
                                    "physical /odom did not confirm zero speed"
                                )
                                self.detail = self.manual_pause_reason
                            return "paused"
                        return "entry-recovery"
                    sweep_complete, completion_geometry = (
                        self._observe_sweep_completion(
                            segment,
                            sweep_completion_tracker,
                            live_pose[0],
                            live_pose[1],
                            self._current_chassis_linear_speed(),
                        )
                    )
                if sweep_complete:
                    rospy.loginfo(
                        "coverage sweep %d completed by directed exit-plane "
                        "crossing: along %.3f/%.3f m, cross-track %.3f m, "
                        "heading error %.1f deg, speed %.3f m/s, "
                        "exit dot %.4f m^2, "
                        "progress %.1f%% (%d consecutive samples)",
                        segment_index + 1,
                        completion_geometry["along"],
                        completion_geometry["length"],
                        completion_geometry["cross_track"],
                        math.degrees(completion_geometry["heading_error"]),
                        completion_geometry["linear_speed"],
                        completion_geometry["exit_dot"],
                        100.0 * completion_geometry["progress_ratio"],
                        completion_geometry["pass_samples"],
                    )
                    return self._cancel_segment_goal(
                        goal_generation, "succeeded"
                    )
            if segment["type"] in ("entry", "transit"):
                live_pose = self._current_pose()
                boundary_complete = False
                boundary_geometry = None
                if live_pose is not None:
                    boundary_complete, boundary_geometry = (
                        self._observe_transition_completion(
                            segment,
                            transition_completion_tracker,
                            live_pose[0],
                            live_pose[1],
                        )
                    )
                if boundary_geometry is not None and boundary_geometry.get(
                        "invalidated", False):
                    with self.lock:
                        self.manual_pause = True
                        self.state = "PAUSED"
                        self.manual_pause_reason = (
                            "localization jump detected while approaching a "
                            "connector boundary; manual resume is required"
                        )
                        self.detail = self.manual_pause_reason
                    return self._cancel_segment_goal(
                        goal_generation, "paused"
                    )
                if boundary_geometry is not None and boundary_geometry.get(
                        "requires_path_replan", False):
                    rospy.logwarn(
                        "coverage Hybrid connector %d left its fixed-gear "
                        "path for %d consecutive samples: lateral %.3fm, "
                        "heading %.1fdeg; canceling the stale action before "
                        "planning from the live pose",
                        segment_index + 1,
                        boundary_geometry["path_deviation_samples"],
                        boundary_geometry["distance_to_path"],
                        math.degrees(boundary_geometry[
                            "path_heading_error"
                        ]),
                    )
                    outcome = self._cancel_segment_goal(
                        goal_generation, "path-deviation"
                    )
                    if outcome != "path-deviation":
                        return outcome
                    if self._wait_for_transition_stop():
                        return "blocked"
                    with self.lock:
                        if self.cancel_requested:
                            return "canceled"
                        self.manual_pause = True
                        self.state = "PAUSED"
                        self.manual_pause_reason = (
                            "Hybrid path deviation canceled its action but "
                            "physical /odom did not confirm zero speed"
                        )
                        self.detail = self.manual_pause_reason
                    return "paused"
                if boundary_complete:
                    return self._complete_transition_boundary(
                        goal_generation,
                        boundary_geometry,
                        segment_index,
                    )
                if boundary_geometry is not None and boundary_geometry.get(
                        "requires_replan", False):
                    rospy.logwarn(
                        "coverage connector %d passed its boundary by %.2fm "
                        "outside the acceptance corridor; replanning the "
                        "complete remaining transition",
                        segment_index + 1,
                        boundary_geometry["terminal_along"],
                    )
                    return self._cancel_segment_goal(
                        goal_generation, "blocked"
                    )
            if segment["type"] == "transit":
                now = time.monotonic()
                live_pose = self._current_pose()
                if live_pose is not None:
                    live_point = live_pose[0]
                    path_progress = (
                        boundary_geometry.get("progress")
                        if boundary_geometry is not None else None
                    )
                    made_progress = (
                        path_progress is not None
                        and (
                            progress_anchor_value is None
                            or path_progress - progress_anchor_value >=
                                hybrid_no_progress_distance
                        )
                    )
                    if path_progress is None:
                        made_progress = (
                            progress_anchor is None or math.hypot(
                                live_point.x - progress_anchor.x,
                                live_point.y - progress_anchor.y,
                            ) >= hybrid_no_progress_distance
                        )
                    if made_progress:
                        progress_anchor = live_point
                        if path_progress is not None:
                            progress_anchor_value = path_progress
                        progress_anchor_time = now
                    elif now - progress_anchor_time >= (
                            hybrid_no_progress_timeout):
                        no_progress_detail = (
                            "Hybrid transition made less than {:.2f} m "
                            "progress for {:.1f} s; canceling this attempt "
                            "for a fresh obstacle replan"
                        ).format(
                            hybrid_no_progress_distance,
                            hybrid_no_progress_timeout,
                        )
                        with self.lock:
                            self.detail = no_progress_detail
                        rospy.logwarn(no_progress_detail)
                        return self._cancel_segment_goal(
                            goal_generation, "blocked"
                        )
            if time.monotonic() - started > timeout:
                return self._cancel_segment_goal(goal_generation, "blocked")
        return "canceled"

    def _run_task(self, route, current, batch_context=False,
                  transition_paths=None):
        """Execute one region and return its terminal state.

        A standalone task releases move_base ownership and clears its plan in
        this method.  A batch item deliberately does neither: the batch worker
        records the item result, prepares the next region, and releases
        `/coverage/active` only once for the whole immutable batch.
        """
        terminal_state = "FAILED"
        cleanup_error = ""
        try:
            segments = (
                self._segments(route, current)
                if transition_paths is None else
                self._segments(
                    route, current, transition_paths=transition_paths
                )
            )
            with self.lock:
                self.total_segments = len(segments)
            blocked = []
            index = 0
            main_loop_completed = True
            while index < len(segments):
                segment = segments[index]
                if rospy.is_shutdown():
                    terminal_state = "CANCELED"
                    main_loop_completed = False
                    break
                if segment["type"] == "transit":
                    self._publish_hybrid_transition_path(
                        segment.get("transition_path")
                    )
                else:
                    self._publish_hybrid_transition_path()
                result = "failed"
                attempts = 0
                entry_recovery_attempts = 0
                while not rospy.is_shutdown():
                    execute_segment = True
                    if segment["type"] == "sweep":
                        preparation = self._prepare_sweep_entry(segment, index)
                        if preparation != "ready":
                            execute_segment = False
                            result = preparation
                            if result == "entry-recovery":
                                entry_recovery_attempts += 1
                                if entry_recovery_attempts > max(
                                        1, self.segment_retry_count):
                                    with self.lock:
                                        self.detail = (
                                            "sweep entrance exceeded its {} "
                                            "pre-start Hybrid alignment attempts"
                                        ).format(max(
                                            1, self.segment_retry_count
                                        ))
                                    result = "blocked"
                                else:
                                    result = self._execute_sweep_entry_recovery(
                                        segment, index
                                    )
                                    if result == "succeeded":
                                        # Re-evaluate the live pose before ever
                                        # publishing SWEEPING or a sweep action.
                                        continue
                    if execute_segment:
                        with self.lock:
                            if not (self.manual_pause or self.external_pause):
                                self.state = (
                                    "SWEEPING" if segment["type"] == "sweep"
                                    else "TRANSITING"
                                )
                                if segment["type"] == "sweep":
                                    self.detail = (
                                        "executing enforced coverage sweep {} of {}"
                                    ).format(index + 1, len(segments))
                                elif segment["type"] == "entry":
                                    if segment.get(
                                            "ordinary_topology_entry", False):
                                        self.detail = (
                                            "navigating to the new region with "
                                            "Navfn topology + rolling Hybrid "
                                            "({} of {})"
                                        ).format(index + 1, len(segments))
                                    else:
                                        self.detail = (
                                            "navigating to the first swath with "
                                            "Navfn + TEB ({} of {})"
                                        ).format(index + 1, len(segments))
                                elif segment["type"] == "rolling_transit":
                                    self.detail = (
                                        "executing hierarchical rolling Hybrid "
                                        "transition {} of {}; every cusp is a stop"
                                    ).format(index + 1, len(segments))
                                else:
                                    self.detail = (
                                        ("executing unsplit 1 Hz online Hybrid A* "
                                         "transition {} of {}"
                                         if getattr(
                                             self,
                                             "online_hybrid_without_precompute",
                                             False,
                                         ) else
                                         "executing cached Hybrid A* swath transition "
                                         "{} of {}; TEB local "
                                         "avoidance remains active")
                                    ).format(index + 1, len(segments))
                            self.current_segment = index + 1
                        result = self._execute_segment(segment, index)
                    if result == "entry-recovery":
                        entry_recovery_attempts += 1
                        if entry_recovery_attempts > max(
                                1, self.segment_retry_count):
                            with self.lock:
                                self.detail = (
                                    "disturbed sweep entry exceeded its {} "
                                    "rolling Hybrid recovery attempts"
                                ).format(max(1, self.segment_retry_count))
                            result = "blocked"
                        else:
                            result = self._execute_sweep_entry_recovery(
                                segment, index
                            )
                            if result == "succeeded":
                                with self.lock:
                                    self.state = "TRANSITING"
                                    self.detail = (
                                        "disturbed entry for sweep {} was "
                                        "recovered; validating the unchanged "
                                        "coverage-line hand-off"
                                    ).format(index + 1)
                                continue
                    if (
                        result == "succeeded"
                        and segment["type"] == "transit"
                        and bool(segment.get(
                            "check_join_after_completion", False
                        ))
                    ):
                        next_segment = (
                            segments[index + 1]
                            if index + 1 < len(segments) else None
                        )
                        live_pose = self._current_pose()
                        joined = False
                        join_detail = "next cached cusp segment is unavailable"
                        if (
                            next_segment is not None
                            and next_segment.get("type") == "transit"
                            and next_segment.get("swath_index") ==
                                segment.get("swath_index")
                            and live_pose is not None
                        ):
                            joined, join_detail = (
                                self._rebase_joinable_hybrid_part(
                                    next_segment,
                                    live_pose[0], live_pose[1],
                                )
                            )
                        replacement = None
                        replan_detail = ""
                        if joined:
                            rospy.loginfo(
                                "coverage cusp retained its checked cached "
                                "suffix: %s", join_detail
                            )
                        else:
                            replacement, replan_detail = (
                                self._replan_remaining_hybrid_transition(
                                    segment
                                )
                            )
                        if not joined and replacement:
                            replace_end = index + 1
                            while (
                                replace_end < len(segments)
                                and segments[replace_end]["type"] == "transit"
                                and segments[replace_end].get("swath_index") ==
                                    segment.get("swath_index")
                            ):
                                replace_end += 1
                            segments[index + 1:replace_end] = replacement
                            with self.lock:
                                self.total_segments = len(segments)
                                self.detail = (
                                    "cusp accepted at measured stop; replanned "
                                    "the complete remaining transition from "
                                    "the live pose"
                                )
                            rospy.loginfo(
                                "coverage cusp hand-off triggered a complete "
                                "remaining-transition replan"
                            )
                        elif not joined:
                            rospy.logwarn(
                                "coverage cusp suffix was unjoinable (%s) and "
                                "the complete remaining transition could not "
                                "be replanned: %s",
                                join_detail,
                                replan_detail,
                            )
                            result = "blocked"
                    if result == "succeeded":
                        break
                    if result == "paused":
                        if not self._wait_while_paused():
                            result = "canceled"
                            break
                        continue
                    if result in ("canceled", "failed"):
                        break
                    attempts += 1
                    if (
                        segment["type"] == "transit"
                        and not getattr(
                            self, "online_hybrid_without_precompute", False
                        )
                        and segment.get("transition_goal") is not None
                        and attempts <= self.segment_retry_count
                    ):
                        replacement, replan_detail = (
                            self._replan_remaining_hybrid_transition(segment)
                        )
                        if replacement:
                            replace_end = index + 1
                            while (
                                replace_end < len(segments)
                                and segments[replace_end]["type"] == "transit"
                                and segments[replace_end].get("swath_index") ==
                                    segment.get("swath_index")
                            ):
                                replace_end += 1
                            segments[index:replace_end] = replacement
                            segment = segments[index]
                            with self.lock:
                                self.total_segments = len(segments)
                                self.detail = (
                                    "replanned the complete remaining Hybrid "
                                    "transition with forward/reverse enabled"
                                )
                            self._publish_hybrid_transition_path(
                                segment.get("transition_path")
                            )
                            continue
                        rospy.logwarn(
                            "coverage complete-transition recovery attempt %d "
                            "failed: %s",
                            attempts,
                            replan_detail,
                        )
                    if attempts <= self.segment_retry_count:
                        with self.lock:
                            self.state = "WAITING_OBSTACLE"
                            segment_label = (
                                "enforced coverage sweep"
                                if segment["type"] == "sweep"
                                else (
                                    "Navfn entry route"
                                    if segment["type"] == "entry"
                                    else (
                                        "rolling Hybrid swath transition"
                                        if segment["type"] == "rolling_transit"
                                        else "Hybrid swath transition"
                                    )
                                )
                            )
                            self.detail = (
                                "{} has no feasible detour yet; retry {} of {} "
                                "in {:.0f}s"
                            ).format(
                                segment_label,
                                attempts,
                                self.segment_retry_count,
                                self.obstacle_wait_sec,
                            )
                        deadline = time.monotonic() + self.obstacle_wait_sec
                        while time.monotonic() < deadline and not rospy.is_shutdown():
                            with self.lock:
                                if self.cancel_requested:
                                    result = "canceled"
                                    break
                            self._lifecycle_wait(0.1)
                        if result == "canceled":
                            break
                        continue

                    entry_alignment_blocked = (
                        segment["type"] == "sweep"
                        and entry_recovery_attempts > 0
                    )
                    if segment["type"] in (
                        "entry", "transit", "rolling_transit"
                    ) or entry_alignment_blocked:
                        # Never skip the dependent swath.  A connector that is
                        # still infeasible after bounded retries pauses the same
                        # segment until the operator cancels or explicitly
                        # resumes it after conditions change.
                        with self.lock:
                            if index not in self.blocked_segments:
                                self.blocked_segments.append(index)
                            self.manual_pause = True
                            self.state = "PAUSED"
                            self.manual_pause_reason = (
                                "{} cannot reach its required swath entry; "
                                "the swath was not skipped. Clear the route or "
                                "cancel, then resume manually."
                            ).format(
                                "sweep-entry Hybrid alignment"
                                if entry_alignment_blocked else (
                                    "Navfn entry"
                                    if segment["type"] == "entry" else (
                                        "rolling Hybrid A* transition"
                                        if segment["type"] == "rolling_transit"
                                        else "Hybrid A* transition"
                                    )
                                )
                            )
                            self.detail = self.manual_pause_reason
                        if not self._wait_while_paused():
                            result = "canceled"
                            break
                        attempts = 0
                        continue
                    break
                if result == "succeeded":
                    with self.lock:
                        if index in self.blocked_segments:
                            self.blocked_segments.remove(index)
                    index += 1
                    continue
                if result == "canceled":
                    terminal_state = "CANCELED"
                    main_loop_completed = False
                    break
                if result == "failed":
                    terminal_state = "FAILED"
                    main_loop_completed = False
                    break
                blocked.append((index, segment))
                with self.lock:
                    if index not in self.blocked_segments:
                        self.blocked_segments.append(index)
                index += 1
            if main_loop_completed:
                for index, segment in list(blocked):
                    result = "blocked"
                    attempt = 0
                    while attempt < self.final_retry_count:
                        preparation = self._prepare_sweep_entry(segment, index)
                        if preparation == "entry-recovery":
                            attempt += 1
                            result = self._execute_sweep_entry_recovery(
                                segment, index
                            )
                            if result == "succeeded":
                                # Recovery only restores the hand-off pose;
                                # validate it again before starting the sweep.
                                continue
                        elif preparation != "ready":
                            result = preparation
                        else:
                            with self.lock:
                                self.current_segment = index + 1
                                self.state = "SWEEPING"
                                self.detail = (
                                    "final retry for enforced coverage sweep {}"
                                ).format(index + 1)
                            result = self._execute_segment(segment, index)
                        if result == "paused":
                            if not self._wait_while_paused():
                                result = "canceled"
                                break
                            continue
                        if result == "succeeded":
                            blocked.remove((index, segment))
                            with self.lock:
                                if index in self.blocked_segments:
                                    self.blocked_segments.remove(index)
                            break
                        if result in ("canceled", "failed"):
                            break
                        attempt += 1
                    if result == "canceled":
                        terminal_state = "CANCELED"
                        break
                    if result == "failed":
                        terminal_state = "FAILED"
                        break
                else:
                    terminal_state = "COMPLETED_PARTIAL" if blocked else "COMPLETED"
        except Exception as error:
            rospy.logerr("coverage task failed: %s", error)
            with self.lock:
                self.detail = "coverage task exception: {}".format(error)
            terminal_state = "FAILED"
        finally:
            owner_token = ""
            owner_released = True
            owner_release_detail = ""
            if not batch_context:
                with self.lock:
                    self.navigation_owner_releasing = True
                    owner_token = self.navigation_owner_token
            goal_terminal_ok, goal_terminal_detail = (
                self._wait_for_move_base_goal_terminal()
            )
            handoff_ok = False
            teb_restored = False
            if not goal_terminal_ok:
                terminal_state = "FAILED"
                cleanup_error = (
                    "coverage retained navigation ownership because its move_base "
                    "goal is not safely terminal: {}"
                ).format(goal_terminal_detail)
            else:
                off = EnforcedPath()
                off.header.frame_id = "map"
                off.plan_id = self.plan_id
                off.active = False
                if batch_context:
                    off.planner_mode = EnforcedPath.MODE_COVERAGE_NAVFN
                    handoff_ok = self._set_enforced_path(
                        off, coverage_active=True
                    )
                else:
                    handoff_ok = self._set_enforced_path(
                        off, coverage_active=False
                    )
                if not handoff_ok:
                    rospy.logerr(
                        "coverage could not synchronously disarm enforced path"
                    )
                    terminal_state = "FAILED"
                    cleanup_error = (
                        "coverage stopped but planner ownership could not be restored"
                    )
                teb_restored = self._restore_teb()
                if not teb_restored:
                    terminal_state = "FAILED"
                    cleanup_error = (
                        "coverage stopped but TEB parameters could not be restored"
                    )
            if not batch_context:
                if handoff_ok and teb_restored:
                    owner_released, owner_release_detail = (
                        self._set_navigation_owner(False, owner_token)
                    )
                    if not owner_released:
                        terminal_state = "FAILED"
                        cleanup_error = (
                            "coverage stopped but navigation ownership could not be "
                            "released: {}"
                        ).format(owner_release_detail)
                else:
                    # Do not open the atomic owner while the planner may still
                    # be armed or temporary TEB settings may still be live.
                    owner_released = False
            with self.lock:
                self.manual_pause = False
                self.manual_pause_reason = ""
                self.avoidance_loss_paused = False
                self.chassis_fault_paused = False
                if batch_context:
                    terminal_state = self._commit_batch_region_outcome_locked(
                        self.batch_token, terminal_state
                    )
                    # Region execution is no longer active, but the batch still
                    # owns move_base through the latched /coverage/active topic.
                    # Do not expose a whole-task terminal state in this gap.
                    self.active = False
                    self.state = "PREPARING"
                    if cleanup_error:
                        self.detail = cleanup_error
                    elif terminal_state == "FAILED" and not self.detail.startswith(
                            "coverage task exception"):
                        self.detail = "coverage batch region execution failed"
                    elif terminal_state == "CANCELED":
                        self.detail = "coverage batch region execution canceled"
                    elif terminal_state == "SKIPPED":
                        self.detail = "coverage batch region skipped by operator"
                    elif terminal_state == "COMPLETED_PARTIAL":
                        self.detail = "coverage batch region completed partially"
                    else:
                        self.detail = "coverage batch region route completed"
                else:
                    self.state = terminal_state
                    if terminal_state == "COMPLETED":
                        self.detail = "coverage route completed"
                    elif terminal_state == "COMPLETED_PARTIAL":
                        self.detail = "coverage completed with blocked segments"
                    elif terminal_state == "CANCELED":
                        self.detail = "coverage task canceled"
                    elif cleanup_error:
                        self.detail = cleanup_error
                    elif not self.detail.startswith("coverage task exception"):
                        self.detail = "coverage task failed"
                    if not owner_released:
                        # Keep both the atomic bridge owner and compatibility
                        # latch closed until an operator can inspect/restart.
                        self.active = True
                        self.cancel_requested = True
                        self.worker = None
                        self._publish_status()
                        rospy.logerr(cleanup_error)
                    else:
                        if self.navigation_owner_token == owner_token:
                            self.navigation_owner_token = ""
                            self.navigation_owner_claimed = False
                            self.navigation_owner_releasing = False
                        self.active = False
                        self.cancel_requested = False
                        # Invalidate the old generation and clear all latched map
                        # overlays before releasing the lifecycle lock.  A second
                        # plan can therefore never be accepted and then erased by
                        # this old worker's finalizer.  Preserve only counters so
                        # terminal status still reports where/why it ended.
                        self._discard_plan_locked(clear_progress=False)
                        self._clear_visualizations()
                        self.active_pub.publish(Bool(data=False))
                        self._publish_status()
        return terminal_state

    def _tracking_timer(self, _event):
        with self.lock:
            should_track = self.active and self.state == "SWEEPING" and not (
                self.manual_pause or self.external_pause
            )
            tracked_plan_id = self.plan_id
        if not should_track or not self._localization_is_fresh():
            return
        current_pose = self._current_pose()
        if current_pose is None:
            return
        point, yaw = current_pose
        with self.lock:
            # The TF lookup above intentionally runs without the lifecycle
            # lock.  Recheck the mission generation before appending so a late
            # timer callback cannot resurrect a green path after terminal
            # cleanup or after a new plan is prepared.
            if (not self.active or self.state != "SWEEPING" or
                    self.plan_id != tracked_plan_id or
                    self.manual_pause or self.external_pause):
                return
            segment_start = point
            if self.last_tracked_point is not None:
                step = math.hypot(point.x - self.last_tracked_point.x,
                                  point.y - self.last_tracked_point.y)
                if step > 0.5:
                    self.manual_pause = True
                    self.state = "PAUSED"
                    self.manual_pause_reason = (
                        "localization jump detected; manual resume is required"
                    )
                    self.detail = self.manual_pause_reason
                    self.move_base.cancel_goal()
                    return
                if step < 0.02:
                    return
                self.traversed_distance += step
                segment_start = self.last_tracked_point
            if self.grid is not None and self.region.polygon.points:
                self.covered_cells.update(rasterize_swept_cells(
                    self.grid,
                    self._points_from_region(self.region),
                    segment_start,
                    point,
                    self.operation_width,
                ))
            self.last_tracked_point = point
            pose = self._pose(point, yaw)
            self.executed_path.header.stamp = pose.header.stamp
            self.executed_path.poses.append(pose)
            path = copy.deepcopy(self.executed_path)
        self.executed_path_pub.publish(path)

    def _status_timer(self, _event):
        self._pause_for_avoidance_loss()
        self._pause_for_chassis_fault()
        with self.lock:
            restore_pending = not self.active and self.original_teb is not None
            unresolved_goal = (
                self.navigation_owner_releasing
                and self.move_base_goal_pending
            )
            unresolved_generation = self.move_base_goal_generation
            unresolved_handle = self.move_base_goal_handle
        if unresolved_goal:
            cancel_ok, cancel_detail = self._reassert_move_base_goal_cancel(
                expected_generation=unresolved_generation,
                expected_handle=unresolved_handle,
            )
            if not cancel_ok:
                rospy.logerr_throttle(
                    2.0,
                    "coverage could not reassert exact goal cancellation: %s",
                    cancel_detail,
                )
        if restore_pending and self._restore_teb():
            with self.lock:
                if self.state == "FAILED" and "TEB parameters" in self.detail:
                    self.detail = (
                        "TEB parameters restored after task failure; inspect before retry"
                    )
        self._publish_status()

    def _publish_status(self):
        message = CoverageStatus()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        with self.lock:
            plan = self.plan
            message.state = self.state
            message.plan_id = self.plan_id
            message.map_ready = self.grid is not None
            message.localized = self._localization_is_fresh()
            message.active = self.active
            message.paused = self.manual_pause or self.external_pause
            message.current_segment = self.current_segment
            message.total_segments = self.total_segments
            message.blocked_segments = len(self.blocked_segments)
            message.operation_width_m = self.operation_width
            message.lane_spacing_m = self.operation_width * (1.0 - self.overlap_ratio)
            message.minimum_turning_radius_m = self.minimum_turning_radius
            parameters = getattr(
                self, "task_time_parameters", CoverageTimeParameters()
            )
            message.allow_reverse_transit = parameters.allow_reverse
            message.max_forward_speed_mps = parameters.max_forward_speed_mps
            message.max_reverse_speed_mps = parameters.max_reverse_speed_mps
            message.max_angular_speed_rps = parameters.max_angular_speed_rps
            message.watchdog_motion_enabled = self.watchdog_motion_enabled
            message.watchdog_max_linear_speed_mps = (
                self.watchdog_max_linear_speed
            )
            message.watchdog_max_angular_speed_rps = (
                self.watchdog_max_angular_speed
            )
            message.linear_accel_mps2 = parameters.linear_accel_mps2
            message.angular_accel_rps2 = parameters.angular_accel_rps2
            message.direction_change_penalty_sec = (
                parameters.direction_change_penalty_sec
            )
            message.segment_handoff_penalty_sec = (
                parameters.segment_handoff_penalty_sec
            )
            message.transit_replan_period_sec = (
                parameters.transit_replan_period_sec
            )
            transit_parameters = self._transit_time_parameters(parameters)
            message.transition_architecture = (
                "navfn_teb_all_swath_transitions_8m_lookahead"
                if bool(getattr(
                    self, "navfn_all_swath_transitions", False
                )) else
                "first_entry_navfn_teb_then_direct_hybrid_teb_fixed_gear_"
                "cusp_join_check_event_replan"
                if (
                    self.hierarchical_hybrid_on_demand
                    and self.direct_hybrid_to_final_goal
                    and self.entry_navfn_single_topology
                    and not self.hybrid_execute_unsplit_cusps
                ) else "legacy_coverage_transition"
            )
            message.transition_max_forward_speed_mps = (
                transit_parameters.max_forward_speed_mps
            )
            message.transition_max_reverse_speed_mps = (
                transit_parameters.max_reverse_speed_mps
                if transit_parameters.allow_reverse else 0.0
            )
            message.transition_max_angular_speed_rps = (
                transit_parameters.max_angular_speed_rps
            )
            message.transition_linear_accel_mps2 = (
                transit_parameters.linear_accel_mps2
            )
            message.transition_angular_accel_rps2 = (
                transit_parameters.angular_accel_rps2
            )
            message.transition_lookahead_dist_m = (
                self.hybrid_transit_lookahead_distance
            )
            message.required_steering_angle_rad = self.required_steering_angle
            message.chassis_wheelbase_m = self.chassis_wheelbase
            message.chassis_max_steering_angle_rad = (
                self.chassis_max_steering_angle
            )
            message.chassis_max_speed_mps = self.chassis_max_speed
            message.kinematics_verified = self.kinematics_verified
            message.kinematics_detail = self.kinematics_detail
            chassis_ready, chassis_detail = self._chassis_ready_locked()
            message.chassis_ready = chassis_ready
            message.chassis_detail = chassis_detail
            avoidance_ready, avoidance_detail = self._avoidance_ready_locked()
            message.avoidance_ready = avoidance_ready
            message.avoidance_detail = avoidance_detail
            message.map_digest = self.map_digest
            message.batch_id = self.batch_id
            message.batch_active = self.batch_active
            message.batch_cancel_requested = self.batch_cancel_requested
            message.batch_current_index = self.batch_current_index
            message.batch_total_regions = self.batch_total_regions
            message.batch_completed_regions = self.batch_completed_regions
            message.batch_partial_regions = self.batch_partial_regions
            message.batch_skipped_regions = self.batch_skipped_regions
            message.current_region_id = self.current_region_id
            message.current_region_name = self.current_region_name
            message.last_region_id = self.last_region_id
            message.last_region_name = self.last_region_name
            message.last_region_state = self.last_region_state
            if plan is not None:
                message.estimated_total_time_sec = getattr(
                    plan, "estimated_total_time_sec", 0.0
                )
                message.estimated_sweep_time_sec = getattr(
                    plan, "estimated_sweep_time_sec", 0.0
                )
                message.estimated_transit_time_sec = (
                    getattr(plan, "estimated_transit_time_sec", 0.0)
                )
                message.estimated_reverse_transitions = (
                    getattr(plan, "estimated_reverse_transitions", 0)
                )
                message.requested_area_m2 = plan.requested_area
                message.reachable_area_m2 = plan.reachable_area
                message.unreachable_area_m2 = plan.unreachable_area
                covered_area = 0.0
                if self.grid is not None:
                    covered_area = (
                        len(self.covered_cells) * self.grid.resolution ** 2
                    )
                message.traversed_area_m2 = min(plan.reachable_area, covered_area)
                if plan.reachable_area > 1.0e-6:
                    message.coverage_ratio = (
                        message.traversed_area_m2 / plan.reachable_area
                    )
            message.detail = self.detail
        self.status_pub.publish(message)


def main():
    rospy.init_node("coverage_manager", anonymous=False)
    CoverageManager()
    rospy.spin()


if __name__ == "__main__":
    main()
