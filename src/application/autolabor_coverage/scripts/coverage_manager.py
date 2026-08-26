#!/usr/bin/env python3
"""Plan and execute static-map coverage tasks through the existing move_base chain."""

import copy
import json
import math
import threading
import time
import uuid

import actionlib
from actionlib_msgs.msg import GoalStatus
from autolabor_coverage.coverage_geometry import (
    CoveragePlanner,
    GridMap,
    Point,
    order_swaths,
    rasterize_swept_cells,
    sample_path,
)
from autolabor_coverage.msg import CoverageStatus, EnforcedPath
from autolabor_coverage.srv import (
    PlanCoverage,
    PlanCoverageResponse,
    SetEnforcedPath,
    StartCoverage,
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
        if not 0.10 <= self.default_max_speed <= self.max_speed_limit <= 1.70:
            raise ValueError(
                "coverage speed limits must satisfy 0.10 <= default <= limit <= 1.70"
            )
        self.task_max_speed = self.default_max_speed
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
        self.enforced_path_service_name = str(rospy.get_param(
            "~enforced_path_service",
            "/move_base/CoverageGlobalPlanner/set_enforced_path",
        ))
        self.entry_position_tolerance = float(rospy.get_param(
            "~entry_position_tolerance_m", 0.20
        ))
        self.entry_yaw_tolerance = float(rospy.get_param(
            "~entry_yaw_tolerance_rad", 0.20
        ))
        if (
            not self.enforced_path_service_name
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
        self.pause_service = rospy.Service(
            "/coverage/set_paused", SetBool, self._pause_service
        )
        self.cancel_service = rospy.Service(
            "/coverage/cancel", Trigger, self._cancel_service
        )
        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.enforced_path_client = rospy.ServiceProxy(
            self.enforced_path_service_name, SetEnforcedPath
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
            digest = grid.digest()
        except ValueError as error:
            rospy.logerr_throttle(2.0, "coverage rejected map: %s", error)
            return
        changed_while_active = False
        with self.lock:
            previous = self.map_digest
            self.grid = grid
            self.map_message = copy.deepcopy(message)
            self.map_digest = digest
            if previous and previous != digest:
                self.plan = None
                self.plan_id = ""
                self.plan_map_digest = ""
                changed_while_active = self.active
                self.detail = "static map changed; coverage plan invalidated"
                if not self.active:
                    self.state = "IDLE"
            elif self.state == "IDLE":
                self.detail = "static map ready; select a coverage region"
        if changed_while_active:
            self._request_cancel("static map changed during coverage")
        if previous and previous != digest:
            self._clear_visualizations()
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
        with self.lock:
            if self.active:
                response.message = "cannot replace a plan while coverage is active"
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
            # GridMap instances are immutable after construction.  Planning on
            # this snapshot keeps /scan, localization and map callbacks free,
            # and the digest is checked again before the plan is committed.
            plan = self._planner(grid).plan(
                points, operation_width, overlap_ratio, reachable_seed=current
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
        route = order_swaths(
            plan.swaths,
            current,
            plan.spacing,
            self.minimum_turning_radius,
            current_yaw,
        )
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
                self.active or self.start_pending or self.map_digest != map_digest
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
                self.region = region
                self.operation_width = operation_width
                self.overlap_ratio = overlap_ratio
                self.allow_reverse_transit = bool(request.allow_reverse_transit)
                self.kinematics_verified = False
                self.kinematics_detail = (
                    "waiting for task-start VCU and TEB verification"
                )
                self.state = "READY"
                self.detail = "coverage path is ready"
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
        self.region = PolygonStamped()
        self.region.header.frame_id = "map"
        self._reset_execution_locked(clear_progress=clear_progress)
        self.kinematics_verified = False
        self.kinematics_detail = (
            "waiting for task-start VCU and TEB verification"
        )
        self.worker = None

    def _start_service(self, request):
        try:
            requested_speed = float(request.max_speed_mps)
        except (TypeError, ValueError, OverflowError):
            requested_speed = float("nan")
        if (
            not math.isfinite(requested_speed)
            or requested_speed < 0.10
            or requested_speed > self.max_speed_limit
        ):
            return StartCoverageResponse(
                False,
                "maximum coverage speed must be in [0.10, {:.2f}] m/s".format(
                    self.max_speed_limit
                ),
            )

        with self.lock:
            if self.active:
                return StartCoverageResponse(False, "coverage is already active")
            if self.start_pending:
                return StartCoverageResponse(False, "coverage start is already preparing")
            if self.plan_pending:
                return StartCoverageResponse(False, "coverage planning is still in progress")
            if self.plan is None or request.plan_id != self.plan_id:
                return StartCoverageResponse(False, "coverage plan id is missing or stale")
            if self.plan_map_digest != self.map_digest:
                return StartCoverageResponse(False, "static map changed after planning")
            token = uuid.uuid4().hex
            plan_id = self.plan_id
            map_digest = self.map_digest
            grid = self.grid
            region = copy.deepcopy(self.region)
            operation_width = self.operation_width
            overlap_ratio = self.overlap_ratio
            self.start_pending = True
            self.start_token = token
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
        reachable_seed = current_pose[0]
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

        # Use the freshest pose for route entry ordering.  The second state
        # check below still requires the vehicle to be stopped.
        current_pose = self._current_pose()
        if current_pose is None:
            return self._finish_start_failure(
                token, "map to base_link transform disappeared during replanning"
            )
        current, current_yaw = current_pose
        route = order_swaths(
            replanned.swaths,
            current,
            replanned.spacing,
            self.minimum_turning_radius,
            current_yaw,
        )
        replanned.swaths = route
        planned_path = self._planned_path(route, current)
        worker = threading.Thread(
            target=self._run_task,
            args=(copy.deepcopy(route), current),
            daemon=True,
        )

        failure = ""
        with self.lock:
            if not self.start_pending or self.start_token != token:
                failure = "coverage start was canceled or superseded"
            elif (
                self.active
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
                self.plan = replanned
                self.task_max_speed = requested_speed
                self.cancel_requested = False
                self.manual_pause = False
                self.manual_pause_reason = ""
                self.avoidance_loss_paused = False
                self.chassis_fault_paused = False
                self.active = True
                self._reset_execution_locked(clear_progress=True)
                self.total_segments = len(route) * 2
                self.state = "GOING_TO_START"
                self.detail = "coverage task accepted at {:.2f} m/s maximum".format(
                    self.task_max_speed
                )
                self.worker = worker
        if failure:
            return self._finish_start_failure(token, failure)

        # Advertise mission ownership before the worker can submit its first
        # move_base goal.  Each segment also performs a synchronous planner
        # mode hand-off before the action goal is sent.
        self.active_pub.publish(Bool(data=True))
        self.path_pub.publish(planned_path)
        self._publish_markers(route, region)
        self._publish_status()
        worker.start()
        return StartCoverageResponse(
            True,
            "coverage task started at {:.2f} m/s maximum".format(requested_speed),
        )

    def _finish_start_failure(self, token, reason):
        with self.lock:
            if self.start_pending and self.start_token == token:
                self.start_pending = False
                self.start_token = ""
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
        if requested_speed > self.watchdog_max_linear_speed + 1.0e-6:
            self.detail = (
                "requested coverage speed {:.2f} m/s exceeds NVIDIA watchdog "
                "cap {:.2f} m/s"
            ).format(requested_speed, self.watchdog_max_linear_speed)
            return False
        if require_kinematics:
            if not self.kinematics_verified:
                self.detail = self.kinematics_detail
                return False
            if requested_speed > self.chassis_max_speed + 1.0e-6:
                self.detail = (
                    "requested coverage speed {:.2f} m/s exceeds live VCU "
                    "cap {:.2f} m/s"
                ).format(requested_speed, self.chassis_max_speed)
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
            if not self.active:
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

    def _cancel_service(self, _request):
        clear_inactive = False
        with self.lock:
            if self.plan_pending:
                self.plan_pending = False
            if self.start_pending:
                self.start_pending = False
                self.start_token = ""
                self._discard_plan_locked(clear_progress=True)
                self.state = "CANCELED"
                self.detail = "coverage start canceled before any goal was submitted"
                canceled_preparation = True
                clear_inactive = True
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
        if clear_inactive:
            self.active_pub.publish(Bool(data=False))
            self._publish_status()
            return TriggerResponse(True, detail)
        self._request_cancel("coverage canceled by operator")
        return TriggerResponse(True, "coverage cancellation requested")

    def _request_cancel(self, reason):
        with self.lock:
            self.cancel_requested = True
            self.detail = reason
        self.move_base.cancel_all_goals()
        self._publish_status()

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
            target = copy.deepcopy(self.original_teb)
            target.update({
                "max_vel_x": self.task_max_speed,
                "max_vel_x_backwards": backwards,
                "allow_init_with_backwards_motion": backwards > 0.0,
                "xy_goal_tolerance": 0.20,
                "yaw_goal_tolerance": 0.20,
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
            time.sleep(0.1)
        return False

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

    def _execute_segment(self, segment, segment_index):
        if not self._wait_while_paused():
            return "canceled"
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
                            "coverage transit %d already satisfies entry pose",
                            segment_index + 1,
                        )
                        return "succeeded"
                    # TEB cannot spin an Ackermann chassis at a fixed point.
                    # Reject this explicit dependency instead of submitting an
                    # orientation-only goal that appears to do nothing.
                    rospy.logwarn(
                        "coverage transit %d is at the entry position but yaw "
                        "error %.1f deg exceeds the Ackermann tolerance",
                        segment_index + 1,
                        math.degrees(yaw_error),
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
        enforced.plan_id = self.plan_id
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
        self.move_base.send_goal(goal)
        while not rospy.is_shutdown():
            self._pause_for_avoidance_loss()
            enforced.header.stamp = rospy.Time.now()
            if enforced.active:
                enforced.path.header.stamp = enforced.header.stamp
            self.enforced_path_pub.publish(enforced)
            if self.move_base.wait_for_result(rospy.Duration(0.2)):
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    return "succeeded"
                with self.lock:
                    if self.cancel_requested:
                        return "canceled"
                    if self.manual_pause or self.external_pause:
                        return "paused"
                return "blocked" if state in (
                    GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST
                ) else "failed"
            with self.lock:
                if self.cancel_requested:
                    self.move_base.cancel_goal()
                    return "canceled"
                paused = self.manual_pause or self.external_pause
            if paused:
                self.move_base.cancel_goal()
                return "paused"
            if not self._localization_is_fresh():
                with self.lock:
                    self.manual_pause = True
                    self.state = "PAUSED"
                    self.manual_pause_reason = (
                        "localization lost; manual resume is required"
                    )
                    self.detail = self.manual_pause_reason
                self.move_base.cancel_goal()
                return "paused"
            if time.monotonic() - started > timeout:
                self.move_base.cancel_goal()
                return "blocked"
        return "canceled"

    def _run_task(self, route, current):
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
                            time.sleep(0.1)
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
            self.move_base.cancel_goal()
            off = EnforcedPath()
            off.header.frame_id = "map"
            off.plan_id = self.plan_id
            off.active = False
            if not self._set_enforced_path(off, coverage_active=False):
                rospy.logerr("coverage could not synchronously disarm enforced path")
            if not self._restore_teb():
                terminal_state = "FAILED"
                cleanup_error = (
                    "coverage stopped but TEB parameters could not be restored"
                )
            with self.lock:
                self.active = False
                self.manual_pause = False
                self.manual_pause_reason = ""
                self.avoidance_loss_paused = False
                self.chassis_fault_paused = False
                self.cancel_requested = False
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
                # Invalidate the old generation and clear all latched map
                # overlays before releasing the lifecycle lock.  A second plan
                # can therefore never be accepted and then erased by this old
                # worker's finalizer.  Preserve only segment counters so the
                # terminal status still reports where/why it ended.
                self._discard_plan_locked(clear_progress=False)
                self._clear_visualizations()
            self.active_pub.publish(Bool(data=False))
            self._publish_status()

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
            if plan is not None:
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
