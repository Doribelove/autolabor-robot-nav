#!/usr/bin/env python3
"""Plan and execute static-map coverage tasks through the existing move_base chain."""

import copy
import hashlib
import json
import math
import re
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
    occupancy_grid_digest,
    rasterize_swept_cells,
    sample_path,
)
from autolabor_coverage.msg import CoverageStatus, EnforcedPath
from autolabor_coverage.srv import (
    CancelCoverageBatch,
    CancelCoverageBatchResponse,
    PlanCoverage,
    PlanCoverageResponse,
    SetEnforcedPath,
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


class CoverageManager:
    def __init__(self):
        self.lock = threading.RLock()
        self.grid = None
        self.map_message = None
        self.map_digest = ""
        self.plan = None
        self.plan_id = ""
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
            rospy.get_param("~default_linear_accel_mps2", 2.00)
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
            rospy.get_param("~direction_change_penalty_sec", 1.00)
        )
        self.segment_handoff_penalty = float(
            rospy.get_param("~segment_handoff_penalty_sec", 0.50)
        )
        self.time_search_beam_width = int(
            rospy.get_param("~time_search_beam_width", 128)
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
        self.default_time_parameters = CoverageTimeParameters(
            max_forward_speed_mps=self.default_max_speed,
            max_reverse_speed_mps=self.reverse_transit_speed,
            max_angular_speed_rps=self.default_max_angular_speed,
            linear_accel_mps2=self.default_linear_accel,
            angular_accel_rps2=self.default_angular_accel,
            allow_reverse=self.allow_reverse_transit,
            direction_change_penalty_sec=self.direction_change_penalty,
            segment_handoff_penalty_sec=self.segment_handoff_penalty,
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
        self.navigation_owner_service_name = str(rospy.get_param(
            "~navigation_owner_service",
            "/navigation_pause/set_coverage_owner",
        ))
        self.entry_position_tolerance = float(rospy.get_param(
            "~entry_position_tolerance_m", 0.30
        ))
        self.entry_yaw_tolerance = float(rospy.get_param(
            "~entry_yaw_tolerance_rad", 0.40
        ))
        if (
            not self.enforced_path_service_name
            or not self.navigation_owner_service_name
            or not math.isfinite(self.entry_position_tolerance)
            or not math.isfinite(self.entry_yaw_tolerance)
            or self.entry_position_tolerance <= 0.0
            or self.entry_yaw_tolerance <= 0.0
        ):
            raise ValueError("coverage entry and planner hand-off parameters are invalid")
        self.state = "IDLE"
        self.detail = "waiting for a static map"
        self.localized = False
        self.localization_received_wall = 0.0
        self.watchdog_motion_enabled = False
        self.watchdog_max_linear_speed = 0.0
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
        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.enforced_path_client = rospy.ServiceProxy(
            self.enforced_path_service_name, SetEnforcedPath
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
        if (
            isinstance(max_linear_speed, bool)
            or not isinstance(max_linear_speed, (int, float))
            or not math.isfinite(float(max_linear_speed))
            or float(max_linear_speed) <= 0.0
        ):
            return
        with self.lock:
            self.watchdog_motion_enabled = payload.get("motion_enabled") is True
            self.watchdog_max_linear_speed = float(max_linear_speed)
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

    def _chassis_odom_callback(self, _message):
        with self.lock:
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
            plan = self._planner(grid).plan(
                points,
                operation_width,
                overlap_ratio,
                reachable_seed=current,
                route_origin=route_origin,
                route_yaw=current_yaw,
                time_parameters=time_parameters,
                time_search_beam_width=getattr(
                    self, "time_search_beam_width", 128
                ),
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

    def _planned_path(self, route, current):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        cursor = current
        for swath in route:
            transit_yaw = math.atan2(swath.start.y - cursor.y, swath.start.x - cursor.x)
            for point in sample_path(cursor, swath.start, self.path_sample_spacing):
                path.poses.append(self._pose(point, transit_yaw, path.header.stamp))
            sweep_yaw = math.atan2(swath.end.y - swath.start.y,
                                   swath.end.x - swath.start.x)
            for point in sample_path(swath.start, swath.end, self.path_sample_spacing):
                path.poses.append(self._pose(point, sweep_yaw, path.header.stamp))
            cursor = swath.end
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
            replanned = self._planner(grid).plan(
                self._points_from_region(region),
                operation_width,
                overlap_ratio,
                reachable_seed=reachable_seed,
                route_origin=reachable_seed,
                route_yaw=route_yaw,
                time_parameters=time_parameters,
                time_search_beam_width=getattr(
                    self, "time_search_beam_width", 128
                ),
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
        current, _ = current_pose
        route = replanned.swaths
        planned_path = self._planned_path(route, current)
        worker = threading.Thread(
            target=self._run_task,
            args=(copy.deepcopy(route), current),
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
            max_angular_speed=0.60, linear_accel=2.00,
            angular_accel=0.50, direction_change_penalty=1.00,
            segment_handoff_penalty=0.50):
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
            plan = self._planner(grid).plan(
                self._points_from_region(item.region),
                operation_width,
                overlap_ratio,
                reachable_seed=current,
                route_origin=current,
                route_yaw=current_yaw,
                time_parameters=time_parameters,
                time_search_beam_width=getattr(
                    self, "time_search_beam_width", 128
                ),
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
        current, _ = current_pose
        route = plan.swaths
        plan_id = uuid.uuid4().hex
        region = copy.deepcopy(item.region)
        region.header.frame_id = "map"
        region.header.stamp = rospy.Time.now()
        planned_path = self._planned_path(route, current)

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
                    outcome = self._run_task(
                        route, current, batch_context=True
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
        requested_motion_speed = max(
            requested_speed,
            self.reverse_transit_speed if self.allow_reverse_transit else 0.0,
        )
        if requested_motion_speed > self.watchdog_max_linear_speed + 1.0e-6:
            self.detail = (
                "requested forward/reverse speed {:.2f} m/s exceeds NVIDIA watchdog "
                "cap {:.2f} m/s"
            ).format(requested_motion_speed, self.watchdog_max_linear_speed)
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
                "coverage Navfn fallback must reject unknown map cells"
            )
        with self.lock:
            self.kinematics_verified = True
            self.kinematics_detail = (
                "VCU/TEB verified: L={:.3f} m, R>={:.2f} m, "
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

    def _set_teb(self, backwards, straight_tracking=False):
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
                0.20 if straight_tracking else self.entry_position_tolerance
            )
            goal_yaw_tolerance = (
                0.20 if straight_tracking else self.entry_yaw_tolerance
            )
            target.update({
                "max_vel_x": self.task_max_speed,
                "max_vel_x_backwards": backwards,
                "max_vel_theta": getattr(
                    self, "task_max_angular_speed", 0.60
                ),
                "acc_lim_x": getattr(self, "task_linear_accel", 2.00),
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

    def _segments(self, route, current):
        segments = []
        cursor = current
        for swath_index, swath in enumerate(route):
            transit_distance = math.hypot(swath.start.x - cursor.x,
                                          swath.start.y - cursor.y)
            yaw = math.atan2(swath.end.y - swath.start.y,
                             swath.end.x - swath.start.x)
            # Keep an explicit transit in front of every swath, including a
            # near-zero one.  Besides matching the advertised 2*N segment
            # count, this gives the executor an unambiguous dependency: a
            # sweep must never run unless move_base has reached its start.
            segments.append({
                "type": "transit",
                "swath_index": swath_index,
                "start": cursor,
                "end": swath.start,
                "yaw": yaw,
                "length": transit_distance,
                "path": None,
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

    def _set_enforced_path(self, enforced, coverage_active=True):
        """Synchronously arm mission ownership and the segment planner mode."""
        try:
            response = self.enforced_path_client(
                coverage_active=coverage_active,
                enforced_path=enforced,
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
        if segment["type"] == "transit":
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
                if position_error <= self.entry_position_tolerance:
                    if yaw_error <= self.entry_yaw_tolerance:
                        rospy.loginfo(
                            "coverage transit %d already satisfies entry pose: "
                            "position %.3f <= %.3f m, yaw %.1f <= %.1f deg",
                            segment_index + 1,
                            position_error,
                            self.entry_position_tolerance,
                            math.degrees(yaw_error),
                            math.degrees(self.entry_yaw_tolerance),
                        )
                        return "succeeded"
                    # TEB cannot spin an Ackermann chassis at a fixed point.
                    # Reject this explicit dependency instead of submitting an
                    # orientation-only goal that appears to do nothing.
                    rospy.logwarn(
                        "coverage transit %d is at the entry position but yaw "
                        "error %.1f deg exceeds the %.1f deg Ackermann "
                        "entry tolerance",
                        segment_index + 1,
                        math.degrees(yaw_error),
                        math.degrees(self.entry_yaw_tolerance),
                    )
                    return "blocked"
        if segment["type"] == "sweep":
            # A transit may move several metres while tracking is disabled.
            # Never compare the next swath's first sample with the previous
            # swath endpoint and misclassify that legitimate transit as a TF
            # localization jump.
            with self.lock:
                self.last_tracked_point = None
        backwards = self.reverse_transit_speed if (
            segment["type"] == "transit" and self.allow_reverse_transit
        ) else 0.0
        if not self._set_teb(
                backwards, straight_tracking=segment["type"] == "sweep"):
            return "failed"
        enforced = EnforcedPath()
        enforced.header.frame_id = "map"
        enforced.plan_id = segment_plan_id
        enforced.segment_index = segment_index
        enforced.active = segment["type"] == "sweep"
        if enforced.active:
            enforced.path = segment["path"]
        goal = MoveBaseGoal()
        goal.target_pose = self._pose(segment["end"], segment["yaw"])
        timeout = self.goal_timeout_base_sec + (
            self.goal_timeout_per_meter_sec * segment["length"]
        )
        started = time.monotonic()
        # Publish the exact sweep before move_base accepts the goal.  Without
        # this ordering its first planner cycle could observe an inactive path
        # and use the Navfn fallback to shortcut a coverage swath.
        enforced.header.stamp = rospy.Time.now()
        if enforced.active:
            enforced.path.header.stamp = enforced.header.stamp
        if not self._set_enforced_path(enforced):
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
            if enforced.active:
                enforced.path.header.stamp = enforced.header.stamp
            self.enforced_path_pub.publish(enforced)
            if self.move_base.wait_for_result(rospy.Duration(0.2)):
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
            if time.monotonic() - started > timeout:
                return self._cancel_segment_goal(goal_generation, "blocked")
        return "canceled"

    def _run_task(self, route, current, batch_context=False):
        """Execute one region and return its terminal state.

        A standalone task releases move_base ownership and clears its plan in
        this method.  A batch item deliberately does neither: the batch worker
        records the item result, prepares the next region, and releases
        `/coverage/active` only once for the whole immutable batch.
        """
        terminal_state = "FAILED"
        cleanup_error = ""
        try:
            segments = self._segments(route, current)
            with self.lock:
                self.total_segments = len(segments)
            blocked = []
            unreached_swaths = set()
            for index, segment in enumerate(segments):
                if rospy.is_shutdown():
                    terminal_state = "CANCELED"
                    break
                if (
                    segment["type"] == "sweep"
                    and segment["swath_index"] in unreached_swaths
                ):
                    blocked.append((index, segment))
                    with self.lock:
                        self.current_segment = index + 1
                        self.detail = (
                            "skipping sweep segment {} because its transit failed"
                        ).format(index + 1)
                        if index not in self.blocked_segments:
                            self.blocked_segments.append(index)
                    continue
                with self.lock:
                    self.current_segment = index + 1
                    if not (self.manual_pause or self.external_pause):
                        self.state = (
                            "SWEEPING" if segment["type"] == "sweep"
                            else "TRANSITING"
                        )
                        self.detail = (
                            "executing enforced coverage sweep {} of {}".format(
                                index + 1, len(segments)
                            ) if segment["type"] == "sweep" else
                            "executing point-to-point Navfn transit {} of {}; "
                            "dynamic replanning and TEB obstacle avoidance are active".format(
                                index + 1, len(segments)
                            )
                        )
                result = "failed"
                attempts = 0
                while attempts <= self.segment_retry_count:
                    with self.lock:
                        if not (self.manual_pause or self.external_pause):
                            self.state = (
                                "SWEEPING" if segment["type"] == "sweep"
                                else "TRANSITING"
                            )
                            self.detail = (
                                "executing enforced coverage sweep {} of {}".format(
                                    index + 1, len(segments)
                                ) if segment["type"] == "sweep" else
                                "executing point-to-point Navfn transit {} of {}; "
                                "dynamic replanning and TEB obstacle avoidance are active".format(
                                    index + 1, len(segments)
                                )
                            )
                    result = self._execute_segment(segment, index)
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
                    if attempts <= self.segment_retry_count:
                        with self.lock:
                            self.state = "WAITING_OBSTACLE"
                            segment_label = (
                                "enforced coverage sweep"
                                if segment["type"] == "sweep"
                                else "point-to-point transit route"
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
                if result == "succeeded":
                    if segment["type"] == "transit":
                        unreached_swaths.discard(segment["swath_index"])
                    continue
                if result == "canceled":
                    terminal_state = "CANCELED"
                    break
                if result == "failed":
                    terminal_state = "FAILED"
                    break
                blocked.append((index, segment))
                if segment["type"] == "transit":
                    unreached_swaths.add(segment["swath_index"])
                with self.lock:
                    if index not in self.blocked_segments:
                        self.blocked_segments.append(index)
            else:
                for index, segment in list(blocked):
                    if (
                        segment["type"] == "sweep"
                        and segment["swath_index"] in unreached_swaths
                    ):
                        continue
                    result = "blocked"
                    attempt = 0
                    while attempt < self.final_retry_count:
                        with self.lock:
                            self.current_segment = index + 1
                            self.state = (
                                "SWEEPING" if segment["type"] == "sweep"
                                else "TRANSITING"
                            )
                            self.detail = (
                                "final retry for enforced coverage sweep {}".format(
                                    index + 1
                                ) if segment["type"] == "sweep" else
                                "final Navfn replan for point-to-point transit {}".format(
                                    index + 1
                                )
                            )
                        result = self._execute_segment(segment, index)
                        if result == "paused":
                            if not self._wait_while_paused():
                                result = "canceled"
                                break
                            continue
                        if result == "succeeded":
                            blocked.remove((index, segment))
                            if segment["type"] == "transit":
                                unreached_swaths.discard(segment["swath_index"])
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
            message.linear_accel_mps2 = parameters.linear_accel_mps2
            message.angular_accel_rps2 = parameters.angular_accel_rps2
            message.direction_change_penalty_sec = (
                parameters.direction_change_penalty_sec
            )
            message.segment_handoff_penalty_sec = (
                parameters.segment_handoff_penalty_sec
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
