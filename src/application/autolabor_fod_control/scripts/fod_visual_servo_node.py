#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fail-closed pixel visual servo for recovering one detected FOD.

This node is intentionally not auto-started by the perception launch.  It owns
``/cmd_vel`` only after a dedicated launch is started, and non-zero motion also
requires both ``~allow_motion:=true`` and an explicit SetBool(true) call.
"""

from collections import deque
from dataclasses import dataclass
import json
import math
import threading
import time
from typing import Optional, Tuple

import rosgraph
import rospy
from autolabor_canbus_driver.msg import CanBusMessage, ChassisStatusInfo
from autolabor_canbus_driver.srv import CanBusService, ChassisParameterServer
from autolabor_fod_msgs.msg import FodDetectionArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Empty, Float64, String
from std_srvs.srv import SetBool, SetBoolResponse

from autolabor_fod_control.visual_servo import (
    ACQUIRE,
    APPROACH,
    EDGE_ARMED,
    LOSS_CONFIRM,
    REACQUIRE,
    STEER_SETTLE,
    AssociationConfig,
    BlindDistanceTracker,
    MotionLease,
    PixelDetection,
    TerminalSensorFence,
    TargetMachineConfig,
    TargetPhaseMachine,
    advance_confirmation_window,
    approach_speed,
    blind_goal_reached,
    curvature_from_pixel_error,
    find_forbidden_publishers,
    horizontal_error,
    interpolate_planar_pose,
    renew_motion_lease_now,
    terminal_feedback_is_fresh,
    terminal_sensor_fence_unchanged,
    validate_detection,
)


DISABLED = "DISABLED"
PRECHECK = "PRECHECK"
BLIND_ADVANCE = "BLIND_ADVANCE"
FINAL_STOP = "FINAL_STOP"
COMPLETE = "COMPLETE"
ABORT = "ABORT"

MAX_COMMAND_SPEED_MPS = 0.20
MAX_BLIND_DISTANCE_M = 0.50
MAX_BLIND_HARD_DISTANCE_M = 0.55
MAX_STEERING_ANGLE_DEG = 12.0

VCU_NODE_TYPE = 0x10
VCU_NODE_ID = 0x00
VCU_HARD_EMERGENCY = 0x17
VCU_SOFT_EMERGENCY = 0x18
VCU_GAMEPAD_EMERGENCY = 0x19
VCU_CONTROLLER_MONITOR = 0x23
VCU_COMMON_STATE = 0x80
VCU_RUNNING_STATE = 0x10
RAW_REQUIRED_TYPES = {
    VCU_HARD_EMERGENCY: "hardware emergency stop",
    VCU_SOFT_EMERGENCY: "software emergency stop",
    VCU_GAMEPAD_EMERGENCY: "gamepad emergency stop",
    VCU_COMMON_STATE: "vehicle running state",
}
RAW_OBSERVED_TYPES = dict(RAW_REQUIRED_TYPES)
RAW_OBSERVED_TYPES[VCU_CONTROLLER_MONITOR] = "controller monitor"


class ControllerAbort(RuntimeError):
    """Expected fail-closed termination of an enabled recovery run."""


@dataclass(frozen=True)
class DetectionFrame:
    sequence: int
    receipt_monotonic: float
    stamp_sec: float
    width: int
    height: int
    observations: Tuple[PixelDetection, ...]
    candidates: Tuple[PixelDetection, ...]
    error: str = ""


@dataclass(frozen=True)
class CameraState:
    receipt_monotonic: float
    stamp_sec: float
    width: int
    height: int
    cx: float
    frame_id: str
    error: str = ""


@dataclass(frozen=True)
class OdomState:
    sequence: int
    receipt_monotonic: float
    stamp_sec: float
    x: float
    y: float
    yaw: float
    linear_x: float
    angular_z: float
    frame_id: str
    child_frame_id: str


def strict_bool_param(name, default):
    value = rospy.get_param(name, default)
    if type(value) is not bool:
        raise ValueError("%s must be a YAML boolean true/false" % name)
    return value


def quaternion_to_yaw(x, y, z, w):
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("odometry quaternion contains a non-finite value")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-6 or abs(norm - 1.0) > 0.2:
        raise ValueError("odometry quaternion norm is invalid: %.6f" % norm)
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def parse_class_names(value):
    if isinstance(value, str):
        names = [item.strip() for item in value.split(",") if item.strip()]
    elif isinstance(value, list):
        names = [str(item).strip() for item in value if str(item).strip()]
    else:
        raise ValueError("allowed_class_names must be a YAML list or comma string")
    if not names or len(set(names)) != len(names):
        raise ValueError("allowed_class_names must be non-empty and unique")
    return tuple(names)


class FodVisualServoNode:
    def __init__(self):
        self.detections_topic = rospy.get_param("~detections_topic", "/fod/detections")
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/fod_camera/camera_info"
        )
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.ackermann_topic = rospy.get_param("~ackermann_topic", "/ackerman_vel")
        self.wheel_angle_topic = rospy.get_param(
            "~wheel_angle_topic", "/m2_driver/wheel_angle"
        )
        self.chassis_status_topic = rospy.get_param(
            "~chassis_status_topic", "/m2_driver/chassis_info"
        )
        self.control_timeout_topic = rospy.get_param(
            "~control_timeout_topic", "/m2_driver/control_timeout"
        )
        self.canbus_topic = rospy.get_param("~canbus_topic", "/canbus_msg")
        self.canbus_service = rospy.get_param("~canbus_service", "/canbus_server")
        self.chassis_parameter_service = rospy.get_param(
            "~chassis_parameter_service", "/m2_driver/chassis_parameter"
        )
        self.steer_center_bias_topic = rospy.get_param(
            "~steer_center_bias_topic", "/m2_driver/steer_center_bias"
        )
        self.reset_odom_topic = rospy.get_param(
            "~reset_odom_topic", "/m2_driver/reset_odom"
        )
        self.brake_set_topic = rospy.get_param(
            "~brake_set_topic", "/m2_driver/brake_set"
        )
        self.emergency_stop_topic = rospy.get_param(
            "~emergency_stop_topic", "/m2_driver/emergency_stop"
        )
        self.m2_bypass_topics = (
            self.steer_center_bias_topic,
            self.reset_odom_topic,
            self.brake_set_topic,
            self.emergency_stop_topic,
        )

        self.expected_detector_node = rospy.get_param(
            "~expected_detector_node", "/fod_detector"
        )
        self.expected_camera_node = rospy.get_param(
            "~expected_camera_node", "/fod_camera/driver"
        )
        self.expected_driver_node = rospy.get_param(
            "~expected_driver_node", "/m2_driver"
        )
        self.expected_canbus_node = rospy.get_param(
            "~expected_canbus_node", "/canbus_driver"
        )
        self.expected_image_width = int(rospy.get_param("~expected_image_width", 1280))
        self.expected_image_height = int(
            rospy.get_param("~expected_image_height", 1024)
        )
        self.expected_camera_frame = rospy.get_param(
            "~expected_camera_frame", "fod_camera_optical_frame"
        )
        self.expected_odom_frame = rospy.get_param("~expected_odom_frame", "odom")
        self.expected_base_frame = rospy.get_param("~expected_base_frame", "base_link")
        self.expected_model_sha256 = str(
            rospy.get_param("~expected_model_sha256", "")
        ).strip().lower()

        self.allowed_class_names = parse_class_names(
            rospy.get_param(
                "~allowed_class_names",
                ["Metal", "Soft", "Plastic", "Wire", "Tool", "w"],
            )
        )
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.45))
        self.allow_motion = strict_bool_param("~allow_motion", False)
        self.use_camera_principal_point = strict_bool_param(
            "~use_camera_principal_point", True
        )
        self.target_u_offset_px = float(rospy.get_param("~target_u_offset_px", 0.0))

        self.precheck_timeout_sec = float(rospy.get_param("~precheck_timeout_sec", 15.0))
        self.mode_timeout_sec = float(rospy.get_param("~mode_timeout_sec", 75.0))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.control_rate_hz = float(rospy.get_param("~control_rate_hz", 20.0))
        self.status_rate_hz = float(rospy.get_param("~status_rate_hz", 2.0))
        self.command_lease_sec = float(rospy.get_param("~command_lease_sec", 0.25))
        self.stop_publish_sec = float(rospy.get_param("~stop_publish_sec", 1.0))

        self.detection_timeout_sec = float(
            rospy.get_param("~detection_timeout_sec", 0.35)
        )
        self.camera_info_timeout_sec = float(
            rospy.get_param("~camera_info_timeout_sec", 0.60)
        )
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 0.60))
        self.source_stamp_timeout_sec = float(
            rospy.get_param("~source_stamp_timeout_sec", 0.80)
        )
        self.detection_odom_sync_tolerance_sec = float(
            rospy.get_param("~detection_odom_sync_tolerance_sec", 0.20)
        )
        self.wheel_angle_timeout_sec = float(
            rospy.get_param("~wheel_angle_timeout_sec", 0.60)
        )
        self.chassis_status_timeout_sec = float(
            rospy.get_param("~chassis_status_timeout_sec", 3.0)
        )
        self.raw_can_timeout_sec = float(rospy.get_param("~raw_can_timeout_sec", 1.25))
        self.raw_can_query_interval_sec = float(
            rospy.get_param("~raw_can_query_interval_sec", 0.40)
        )
        self.graph_check_interval_sec = float(
            rospy.get_param("~graph_check_interval_sec", 0.50)
        )

        self.min_acquire_anchor_v_fraction = float(
            rospy.get_param("~min_acquire_anchor_v_fraction", 0.20)
        )
        self.max_acquire_anchor_v_fraction = float(
            rospy.get_param("~max_acquire_anchor_v_fraction", 0.80)
        )
        self.acquire_max_abs_horizontal_error = float(
            rospy.get_param("~acquire_max_abs_horizontal_error", 0.40)
        )
        association_config = AssociationConfig(
            min_iou=float(rospy.get_param("~association_min_iou", 0.05)),
            max_anchor_distance_ratio=float(
                rospy.get_param("~association_max_anchor_distance_ratio", 0.10)
            ),
            min_area_ratio=float(rospy.get_param("~association_min_area_ratio", 0.35)),
            max_area_ratio=float(rospy.get_param("~association_max_area_ratio", 2.80)),
        )
        machine_config = TargetMachineConfig(
            acquire_frames=int(rospy.get_param("~acquire_frames", 6)),
            bottom_fraction=float(rospy.get_param("~bottom_fraction", 0.88)),
            bottom_center_tolerance_fraction=float(
                rospy.get_param("~bottom_center_tolerance_fraction", 0.05)
            ),
            bottom_confirm_frames=int(rospy.get_param("~bottom_confirm_frames", 6)),
            min_approach_distance_m=float(
                rospy.get_param("~min_approach_distance_m", 0.10)
            ),
            min_vertical_progress_fraction=float(
                rospy.get_param("~min_vertical_progress_fraction", 0.06)
            ),
            loss_confirm_frames=int(rospy.get_param("~loss_confirm_frames", 5)),
            loss_confirm_min_sec=float(
                rospy.get_param("~loss_confirm_min_sec", 0.20)
            ),
            early_loss_max_frames=int(
                rospy.get_param("~early_loss_max_frames", 10)
            ),
            filter_alpha=float(rospy.get_param("~pixel_filter_alpha", 0.35)),
        )
        self.machine_config = machine_config
        self.association_config = association_config

        self.steering_sign = float(rospy.get_param("~steering_sign", -1.0))
        self.curvature_gain = float(rospy.get_param("~curvature_gain", 0.65))
        self.horizontal_deadband = float(
            rospy.get_param("~horizontal_deadband", 0.025)
        )
        self.max_steering_angle_deg = float(
            rospy.get_param("~max_steering_angle_deg", 12.0)
        )
        self.max_runtime_horizontal_error = float(
            rospy.get_param("~max_runtime_horizontal_error", 0.70)
        )
        self.far_speed_mps = float(rospy.get_param("~far_speed_mps", 0.15))
        self.near_speed_mps = float(rospy.get_param("~near_speed_mps", 0.06))
        self.slow_start_fraction = float(
            rospy.get_param("~slow_start_fraction", 0.65)
        )
        self.near_start_fraction = float(
            rospy.get_param("~near_start_fraction", 0.82)
        )
        self.lateral_slowdown_error = float(
            rospy.get_param("~lateral_slowdown_error", 0.45)
        )
        self.minimum_lateral_speed_scale = float(
            rospy.get_param("~minimum_lateral_speed_scale", 0.35)
        )
        self.max_linear_acceleration_mps2 = float(
            rospy.get_param("~max_linear_acceleration_mps2", 0.20)
        )
        self.max_curvature_rate_per_sec = float(
            rospy.get_param("~max_curvature_rate_per_sec", 0.80)
        )
        self.max_approach_distance_m = float(
            rospy.get_param("~max_approach_distance_m", 6.0)
        )
        self.approach_no_motion_timeout_sec = float(
            rospy.get_param("~approach_no_motion_timeout_sec", 3.0)
        )
        self.approach_no_progress_timeout_sec = float(
            rospy.get_param("~approach_no_progress_timeout_sec", 2.0)
        )
        self.approach_progress_step_m = float(
            rospy.get_param("~approach_progress_step_m", 0.02)
        )
        self.max_measured_speed_mps = float(
            rospy.get_param("~max_measured_speed_mps", 0.30)
        )
        self.max_pose_step_m = float(rospy.get_param("~max_pose_step_m", 0.05))

        self.settle_wheel_angle_deg = float(
            rospy.get_param("~settle_wheel_angle_deg", 2.0)
        )
        self.settle_speed_mps = float(rospy.get_param("~settle_speed_mps", 0.03))
        self.settle_confirm_sec = float(rospy.get_param("~settle_confirm_sec", 0.30))
        self.settle_timeout_sec = float(rospy.get_param("~settle_timeout_sec", 3.0))
        self.preblind_max_displacement_m = float(
            rospy.get_param("~preblind_max_displacement_m", 0.05)
        )
        self.blind_speed_mps = float(rospy.get_param("~blind_speed_mps", 0.08))
        self.blind_distance_m = float(rospy.get_param("~blind_distance_m", 0.50))
        self.blind_hard_distance_m = float(
            rospy.get_param("~blind_hard_distance_m", 0.55)
        )
        self.blind_timeout_sec = float(rospy.get_param("~blind_timeout_sec", 12.0))
        self.blind_no_motion_timeout_sec = float(
            rospy.get_param("~blind_no_motion_timeout_sec", 2.0)
        )
        self.blind_no_progress_timeout_sec = float(
            rospy.get_param("~blind_no_progress_timeout_sec", 1.50)
        )
        self.blind_progress_step_m = float(
            rospy.get_param("~blind_progress_step_m", 0.01)
        )
        self.blind_max_heading_change_deg = float(
            rospy.get_param("~blind_max_heading_change_deg", 4.0)
        )
        self.blind_max_lateral_deviation_m = float(
            rospy.get_param("~blind_max_lateral_deviation_m", 0.08)
        )
        self.final_stop_confirm_sec = float(
            rospy.get_param("~final_stop_confirm_sec", 0.50)
        )
        self.final_stop_speed_mps = float(
            rospy.get_param("~final_stop_speed_mps", 0.01)
        )
        self.final_stop_max_drift_m = float(
            rospy.get_param("~final_stop_max_drift_m", 0.005)
        )
        self.final_stop_timeout_sec = float(
            rospy.get_param("~final_stop_timeout_sec", 3.0)
        )

        self._validate_parameters()

        self.sensor_lock = threading.Lock()
        self.run_lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.operator_disable_requested = threading.Event()
        self.operator_disable_requested.set()
        self.latest_detection = None
        self.detection_queue = deque(maxlen=200)
        self.detection_sequence = 0
        self.last_detection_stamp = None
        self.detection_queue_overflow = False
        self.latest_camera = None
        self.invalid_camera_generation = 0
        self.session_invalid_camera_generation = 0
        self.latest_odom = None
        self.odom_queue = deque(maxlen=300)
        self.odom_history = deque(maxlen=300)
        self.odom_sequence = 0
        self.odom_queue_overflow = False
        self.last_unstopped_odom_sequence = 0
        self.last_final_unstopped_odom_sequence = 0
        self.invalid_odom_generation = 0
        self.session_invalid_odom_generation = 0
        self.last_odom_stamp = None
        self.invalid_odom_reason = "waiting for odometry"
        self.invalid_odom_monotonic = time.monotonic()
        self.latest_wheel_angle = None
        self.latest_wheel_angle_monotonic = None
        self.wheel_sequence = 0
        self.last_uncentered_wheel_sequence = 0
        self.invalid_wheel_generation = 0
        self.session_invalid_wheel_generation = 0
        self.invalid_wheel_reason = "waiting for wheel angle"
        self.invalid_wheel_monotonic = time.monotonic()
        self.latest_chassis_status = None
        self.latest_chassis_status_monotonic = None
        self.chassis_fault_generation = 0
        self.session_chassis_fault_generation = 0
        self.last_chassis_fault_reason = ""
        self.control_timeout_seen = False
        self.raw_can_status = {}
        self.raw_can_fault_generation = 0
        self.session_raw_can_fault_generation = 0
        self.last_raw_can_fault_reason = ""
        self.last_raw_query_monotonic = 0.0
        self.m2_bypass_event_generation = 0
        self.session_m2_bypass_event_generation = 0
        self.last_m2_bypass_event_topic = ""

        self.phase = DISABLED
        self.reason = "motion disabled; call set_enabled only after clearing the area"
        self.machine = TargetPhaseMachine(machine_config, association_config)
        self.mode_started_monotonic = None
        self.mode_deadline_monotonic = None
        self.precheck_deadline_monotonic = None
        self.last_processed_detection_sequence = 0
        self.session_detection_floor = 0
        self.last_processed_odom_sequence = 0
        self.target_u_px = None
        self.horizontal_error_value = None
        self.vertical_fraction_value = None
        self.target_visible = False
        self.approach_path_m = 0.0
        self.approach_last_odom = None
        self.approach_motion_started_monotonic = None
        self.approach_motion_seen = False
        self.approach_last_progress_monotonic = None
        self.approach_next_progress_m = self.approach_progress_step_m
        self.blind_tracker = None
        self.blind_progress = None
        self.blind_last_odom_state = None
        self.pending_loss_stamp_sec = None
        self.pending_loss_monotonic = None
        self.blind_started_monotonic = None
        self.blind_drive_started_monotonic = None
        self.blind_motion_seen = False
        self.blind_last_progress_monotonic = None
        self.blind_next_progress_m = self.blind_progress_step_m
        self.blind_max_forward_m = 0.0
        self.blind_seen_uncentered_wheel_sequence = 0
        self.final_stop_started_monotonic = None
        self.final_stop_last_odom_sequence = 0
        self.final_stop_last_wheel_sequence = 0
        self.final_stop_seen_unstopped_odom_sequence = 0
        self.final_stop_seen_uncentered_wheel_sequence = 0
        self.final_stop_good_start_odom_stamp = None
        self.final_stop_good_start_wheel_receipt = None
        self.final_stop_good_start_path_m = None
        self.settle_started_monotonic = None
        self.settle_last_odom_sequence = 0
        self.settle_last_wheel_sequence = 0
        self.settle_seen_unstopped_odom_sequence = 0
        self.settle_seen_uncentered_wheel_sequence = 0
        self.settle_good_start_odom_stamp = None
        self.settle_good_start_wheel_receipt = None
        self.wheelbase_m = None
        self.max_curvature = None
        self.last_command_update_monotonic = time.monotonic()
        self.last_graph_check_monotonic = 0.0
        self.last_status_publish_monotonic = 0.0
        self.last_logged_phase = ""
        self.shutdown_started = False

        self.motion_lease = MotionLease(self.command_lease_sec)
        self.master = rosgraph.Master(rospy.get_name())
        self.master_pid = self.master.getPid()

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.completed_pub = rospy.Publisher("~completed", Bool, queue_size=1, latch=True)
        self.diagnostics_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=5
        )

        self.detection_sub = rospy.Subscriber(
            self.detections_topic,
            FodDetectionArray,
            self._detection_cb,
            queue_size=50,
        )
        self.camera_sub = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._camera_cb, queue_size=20
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_cb, queue_size=100
        )
        self.wheel_sub = rospy.Subscriber(
            self.wheel_angle_topic, Float64, self._wheel_angle_cb, queue_size=50
        )
        self.chassis_sub = rospy.Subscriber(
            self.chassis_status_topic,
            ChassisStatusInfo,
            self._chassis_status_cb,
            queue_size=20,
        )
        self.control_timeout_sub = rospy.Subscriber(
            self.control_timeout_topic, Bool, self._control_timeout_cb, queue_size=20
        )
        self.canbus_sub = rospy.Subscriber(
            self.canbus_topic, CanBusMessage, self._raw_canbus_cb, queue_size=100
        )
        self.m2_bypass_subscribers = (
            rospy.Subscriber(
                self.steer_center_bias_topic,
                Float64,
                self._m2_bypass_control_cb,
                callback_args=self.steer_center_bias_topic,
                queue_size=20,
            ),
            rospy.Subscriber(
                self.reset_odom_topic,
                Empty,
                self._m2_bypass_control_cb,
                callback_args=self.reset_odom_topic,
                queue_size=20,
            ),
            rospy.Subscriber(
                self.brake_set_topic,
                Bool,
                self._m2_bypass_control_cb,
                callback_args=self.brake_set_topic,
                queue_size=20,
            ),
            rospy.Subscriber(
                self.emergency_stop_topic,
                Bool,
                self._m2_bypass_control_cb,
                callback_args=self.emergency_stop_topic,
                queue_size=20,
            ),
        )
        self.canbus_proxy = rospy.ServiceProxy(self.canbus_service, CanBusService)
        self.chassis_parameter_proxy = rospy.ServiceProxy(
            self.chassis_parameter_service, ChassisParameterServer
        )
        self.enable_service = rospy.Service(
            "~set_enabled", SetBool, self._set_enabled_cb
        )

        self.command_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self._publish_command
        )
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate_hz), self._control_tick
        )
        rospy.on_shutdown(self._shutdown)

        self._publish_status(force=True)
        rospy.logwarn(
            "FOD visual servo loaded with allow_motion=%s. It publishes zero on %s "
            "while disabled; non-zero motion also requires %s/set_enabled true.",
            self.allow_motion,
            self.cmd_vel_topic,
            rospy.get_name(),
        )

    def _validate_parameters(self):
        numeric = {
            "min_confidence": self.min_confidence,
            "target_u_offset_px": self.target_u_offset_px,
            "precheck_timeout_sec": self.precheck_timeout_sec,
            "mode_timeout_sec": self.mode_timeout_sec,
            "publish_rate_hz": self.publish_rate_hz,
            "control_rate_hz": self.control_rate_hz,
            "status_rate_hz": self.status_rate_hz,
            "command_lease_sec": self.command_lease_sec,
            "stop_publish_sec": self.stop_publish_sec,
            "detection_timeout_sec": self.detection_timeout_sec,
            "camera_info_timeout_sec": self.camera_info_timeout_sec,
            "odom_timeout_sec": self.odom_timeout_sec,
            "source_stamp_timeout_sec": self.source_stamp_timeout_sec,
            "detection_odom_sync_tolerance_sec": (
                self.detection_odom_sync_tolerance_sec
            ),
            "wheel_angle_timeout_sec": self.wheel_angle_timeout_sec,
            "chassis_status_timeout_sec": self.chassis_status_timeout_sec,
            "raw_can_timeout_sec": self.raw_can_timeout_sec,
            "raw_can_query_interval_sec": self.raw_can_query_interval_sec,
            "graph_check_interval_sec": self.graph_check_interval_sec,
            "min_acquire_anchor_v_fraction": self.min_acquire_anchor_v_fraction,
            "max_acquire_anchor_v_fraction": self.max_acquire_anchor_v_fraction,
            "acquire_max_abs_horizontal_error": self.acquire_max_abs_horizontal_error,
            "association_min_iou": self.association_config.min_iou,
            "association_max_anchor_distance_ratio": (
                self.association_config.max_anchor_distance_ratio
            ),
            "association_min_area_ratio": self.association_config.min_area_ratio,
            "association_max_area_ratio": self.association_config.max_area_ratio,
            "bottom_fraction": self.machine_config.bottom_fraction,
            "bottom_center_tolerance_fraction": (
                self.machine_config.bottom_center_tolerance_fraction
            ),
            "min_approach_distance_m": self.machine_config.min_approach_distance_m,
            "min_vertical_progress_fraction": (
                self.machine_config.min_vertical_progress_fraction
            ),
            "loss_confirm_min_sec": self.machine_config.loss_confirm_min_sec,
            "pixel_filter_alpha": self.machine_config.filter_alpha,
            "steering_sign": self.steering_sign,
            "curvature_gain": self.curvature_gain,
            "horizontal_deadband": self.horizontal_deadband,
            "max_steering_angle_deg": self.max_steering_angle_deg,
            "max_runtime_horizontal_error": self.max_runtime_horizontal_error,
            "far_speed_mps": self.far_speed_mps,
            "near_speed_mps": self.near_speed_mps,
            "slow_start_fraction": self.slow_start_fraction,
            "near_start_fraction": self.near_start_fraction,
            "lateral_slowdown_error": self.lateral_slowdown_error,
            "minimum_lateral_speed_scale": self.minimum_lateral_speed_scale,
            "max_linear_acceleration_mps2": self.max_linear_acceleration_mps2,
            "max_curvature_rate_per_sec": self.max_curvature_rate_per_sec,
            "max_approach_distance_m": self.max_approach_distance_m,
            "approach_no_motion_timeout_sec": self.approach_no_motion_timeout_sec,
            "approach_no_progress_timeout_sec": self.approach_no_progress_timeout_sec,
            "approach_progress_step_m": self.approach_progress_step_m,
            "max_measured_speed_mps": self.max_measured_speed_mps,
            "max_pose_step_m": self.max_pose_step_m,
            "settle_wheel_angle_deg": self.settle_wheel_angle_deg,
            "settle_speed_mps": self.settle_speed_mps,
            "settle_confirm_sec": self.settle_confirm_sec,
            "settle_timeout_sec": self.settle_timeout_sec,
            "preblind_max_displacement_m": self.preblind_max_displacement_m,
            "blind_speed_mps": self.blind_speed_mps,
            "blind_distance_m": self.blind_distance_m,
            "blind_hard_distance_m": self.blind_hard_distance_m,
            "blind_timeout_sec": self.blind_timeout_sec,
            "blind_no_motion_timeout_sec": self.blind_no_motion_timeout_sec,
            "blind_no_progress_timeout_sec": self.blind_no_progress_timeout_sec,
            "blind_progress_step_m": self.blind_progress_step_m,
            "blind_max_heading_change_deg": self.blind_max_heading_change_deg,
            "blind_max_lateral_deviation_m": self.blind_max_lateral_deviation_m,
            "final_stop_confirm_sec": self.final_stop_confirm_sec,
            "final_stop_speed_mps": self.final_stop_speed_mps,
            "final_stop_max_drift_m": self.final_stop_max_drift_m,
            "final_stop_timeout_sec": self.final_stop_timeout_sec,
        }
        for name, value in numeric.items():
            if not math.isfinite(value):
                raise ValueError("%s must be finite" % name)

        required_absolute_topics = {
            "detections_topic": self.detections_topic,
            "camera_info_topic": self.camera_info_topic,
            "odom_topic": self.odom_topic,
            "cmd_vel_topic": self.cmd_vel_topic,
            "ackermann_topic": self.ackermann_topic,
            "wheel_angle_topic": self.wheel_angle_topic,
            "chassis_status_topic": self.chassis_status_topic,
            "control_timeout_topic": self.control_timeout_topic,
            "canbus_topic": self.canbus_topic,
            "canbus_service": self.canbus_service,
            "chassis_parameter_service": self.chassis_parameter_service,
            "steer_center_bias_topic": self.steer_center_bias_topic,
            "reset_odom_topic": self.reset_odom_topic,
            "brake_set_topic": self.brake_set_topic,
            "emergency_stop_topic": self.emergency_stop_topic,
        }
        for name, value in required_absolute_topics.items():
            if not isinstance(value, str) or not value.startswith("/"):
                raise ValueError("%s must be an absolute ROS name" % name)
        if len(set(self.m2_bypass_topics)) != len(self.m2_bypass_topics):
            raise ValueError("M2 bypass-control topics must be unique")
        for name, value in {
            "expected_detector_node": self.expected_detector_node,
            "expected_camera_node": self.expected_camera_node,
            "expected_driver_node": self.expected_driver_node,
            "expected_canbus_node": self.expected_canbus_node,
        }.items():
            if not isinstance(value, str) or not value.startswith("/"):
                raise ValueError("%s must be an absolute ROS node name" % name)

        if self.expected_image_width < 320 or self.expected_image_height < 240:
            raise ValueError("expected image dimensions are implausibly small")
        if len(self.expected_model_sha256) != 64 or any(
            char not in "0123456789abcdef" for char in self.expected_model_sha256
        ):
            raise ValueError("expected_model_sha256 must be exactly 64 hex characters")
        if not 0.25 <= self.min_confidence <= 0.95:
            raise ValueError("min_confidence must be between 0.25 and 0.95")
        if abs(self.target_u_offset_px) > 0.15 * self.expected_image_width:
            raise ValueError("target_u_offset_px exceeds the conservative calibration range")
        if not 5.0 <= self.precheck_timeout_sec <= 60.0:
            raise ValueError("precheck_timeout_sec must be between 5 and 60")
        if not 10.0 <= self.mode_timeout_sec <= 120.0:
            raise ValueError("mode_timeout_sec must be between 10 and 120")
        if not 10.0 <= self.publish_rate_hz <= 50.0:
            raise ValueError("publish_rate_hz must be between 10 and 50")
        if not 10.0 <= self.control_rate_hz <= 50.0:
            raise ValueError("control_rate_hz must be between 10 and 50")
        if not 0.5 <= self.status_rate_hz <= 10.0:
            raise ValueError("status_rate_hz must be between 0.5 and 10")
        if not 0.15 <= self.command_lease_sec <= 0.35:
            raise ValueError("command_lease_sec must be between 0.15 and 0.35")
        if self.command_lease_sec <= 2.0 / self.control_rate_hz:
            raise ValueError("command lease must exceed two control periods")
        if not 0.5 <= self.stop_publish_sec <= 3.0:
            raise ValueError("stop_publish_sec must be between 0.5 and 3")

        for name, value, low, high in (
            ("detection_timeout_sec", self.detection_timeout_sec, 0.15, 0.60),
            ("camera_info_timeout_sec", self.camera_info_timeout_sec, 0.20, 1.50),
            ("odom_timeout_sec", self.odom_timeout_sec, 0.20, 1.00),
            ("source_stamp_timeout_sec", self.source_stamp_timeout_sec, 0.30, 2.00),
            (
                "detection_odom_sync_tolerance_sec",
                self.detection_odom_sync_tolerance_sec,
                0.08,
                0.30,
            ),
            ("wheel_angle_timeout_sec", self.wheel_angle_timeout_sec, 0.20, 1.00),
            ("chassis_status_timeout_sec", self.chassis_status_timeout_sec, 1.00, 5.00),
            ("raw_can_timeout_sec", self.raw_can_timeout_sec, 0.60, 2.00),
            ("raw_can_query_interval_sec", self.raw_can_query_interval_sec, 0.20, 0.80),
            ("graph_check_interval_sec", self.graph_check_interval_sec, 0.20, 1.00),
        ):
            if not low <= value <= high:
                raise ValueError("%s must be between %.2f and %.2f" % (name, low, high))
        if self.raw_can_timeout_sec <= self.raw_can_query_interval_sec:
            raise ValueError("raw CAN timeout must exceed its query interval")

        if not 0.05 <= self.min_acquire_anchor_v_fraction < 0.70:
            raise ValueError("min acquisition vertical fraction is invalid")
        if not 0.50 <= self.max_acquire_anchor_v_fraction <= 0.85:
            raise ValueError("max acquisition vertical fraction is invalid")
        if self.min_acquire_anchor_v_fraction >= self.max_acquire_anchor_v_fraction:
            raise ValueError("acquisition vertical range is empty")
        if not 0.10 <= self.acquire_max_abs_horizontal_error <= 0.60:
            raise ValueError("acquisition horizontal gate is invalid")
        if not 3 <= self.machine_config.acquire_frames <= 20:
            raise ValueError("acquire_frames must be between 3 and 20")
        if not 0.0 <= self.association_config.min_iou <= 0.5:
            raise ValueError("association_min_iou is invalid")
        if not 0.02 <= self.association_config.max_anchor_distance_ratio <= 0.20:
            raise ValueError("association anchor distance is invalid")
        if not 0.10 <= self.association_config.min_area_ratio < 1.0:
            raise ValueError("association minimum area ratio is invalid")
        if not 1.0 < self.association_config.max_area_ratio <= 5.0:
            raise ValueError("association maximum area ratio is invalid")
        if not 0.80 <= self.machine_config.bottom_fraction <= 0.96:
            raise ValueError("bottom_fraction must be between 0.80 and 0.96")
        if not 0.02 <= self.machine_config.bottom_center_tolerance_fraction <= 0.10:
            raise ValueError("bottom center tolerance is invalid")
        if not 3 <= self.machine_config.bottom_confirm_frames <= 20:
            raise ValueError("bottom_confirm_frames must be between 3 and 20")
        if not 0.05 <= self.machine_config.min_approach_distance_m <= 0.50:
            raise ValueError("minimum approach distance is invalid")
        if not 0.02 <= self.machine_config.min_vertical_progress_fraction <= 0.25:
            raise ValueError("minimum vertical progress is invalid")
        if not 3 <= self.machine_config.loss_confirm_frames <= 15:
            raise ValueError("loss_confirm_frames must be between 3 and 15")
        if not 0.10 <= self.machine_config.loss_confirm_min_sec <= 1.0:
            raise ValueError("loss_confirm_min_sec is invalid")
        if not 3 <= self.machine_config.early_loss_max_frames <= 30:
            raise ValueError("early_loss_max_frames must be between 3 and 30")
        if not 0.05 <= self.machine_config.filter_alpha <= 1.0:
            raise ValueError("pixel_filter_alpha is invalid")

        if self.steering_sign not in (-1.0, 1.0):
            raise ValueError("steering_sign must be exactly -1.0 or +1.0")
        if not 0.05 <= self.curvature_gain <= 2.0:
            raise ValueError("curvature_gain is invalid")
        if not 0.0 <= self.horizontal_deadband <= 0.10:
            raise ValueError("horizontal_deadband is invalid")
        if not 2.0 <= self.max_steering_angle_deg <= MAX_STEERING_ANGLE_DEG:
            raise ValueError("max steering angle exceeds the 12 degree safety limit")
        if not 0.30 <= self.max_runtime_horizontal_error <= 1.0:
            raise ValueError("runtime horizontal error gate is invalid")
        if not 0.02 <= self.near_speed_mps <= self.far_speed_mps:
            raise ValueError("near_speed_mps must be positive and no greater than far speed")
        if self.far_speed_mps > MAX_COMMAND_SPEED_MPS:
            raise ValueError("far_speed_mps exceeds the 0.20 m/s hard limit")
        if not 0.40 <= self.slow_start_fraction < self.near_start_fraction <= 0.90:
            raise ValueError("approach vertical speed thresholds are invalid")
        if not 0.15 <= self.lateral_slowdown_error <= 1.0:
            raise ValueError("lateral_slowdown_error is invalid")
        if not 0.20 <= self.minimum_lateral_speed_scale <= 1.0:
            raise ValueError("minimum lateral speed scale is invalid")
        if not 0.05 <= self.max_linear_acceleration_mps2 <= 0.50:
            raise ValueError("linear acceleration limit is invalid")
        if not 0.10 <= self.max_curvature_rate_per_sec <= 2.0:
            raise ValueError("curvature rate limit is invalid")
        if not 0.5 <= self.max_approach_distance_m <= 10.0:
            raise ValueError("max approach distance is invalid")
        if not 1.0 <= self.approach_no_motion_timeout_sec <= 6.0:
            raise ValueError("approach no-motion timeout is invalid")
        if not 0.5 <= self.approach_no_progress_timeout_sec <= 5.0:
            raise ValueError("approach no-progress timeout is invalid")
        if not 0.005 <= self.approach_progress_step_m <= 0.10:
            raise ValueError("approach progress step is invalid")
        if not self.far_speed_mps <= self.max_measured_speed_mps <= 0.50:
            raise ValueError("max measured speed is invalid")
        if not 0.02 <= self.max_pose_step_m <= 0.10:
            raise ValueError("max pose step is invalid")

        if not 0.5 <= self.settle_wheel_angle_deg <= 3.0:
            raise ValueError("settle wheel angle is invalid")
        if not 0.01 <= self.settle_speed_mps <= 0.05:
            raise ValueError("settle speed is invalid")
        if not 0.20 <= self.settle_confirm_sec <= 1.0:
            raise ValueError("settle confirmation duration is invalid")
        if not 1.0 <= self.settle_timeout_sec <= 8.0:
            raise ValueError("settle timeout is invalid")
        if not 0.02 <= self.preblind_max_displacement_m <= 0.15:
            raise ValueError("preblind displacement limit is invalid")
        if not 0.03 <= self.blind_speed_mps <= 0.10:
            raise ValueError("blind_speed_mps must be between 0.03 and 0.10")
        if not 0.05 <= self.blind_distance_m <= MAX_BLIND_DISTANCE_M:
            raise ValueError("blind_distance_m exceeds the 0.50 m hard target limit")
        if not self.blind_distance_m <= self.blind_hard_distance_m:
            raise ValueError("blind hard distance must not be below the target distance")
        if self.blind_hard_distance_m > MAX_BLIND_HARD_DISTANCE_M:
            raise ValueError("blind hard distance exceeds the 0.55 m absolute limit")
        if not 3.0 <= self.blind_timeout_sec <= 20.0:
            raise ValueError("blind timeout is invalid")
        if not 0.5 <= self.blind_no_motion_timeout_sec <= 5.0:
            raise ValueError("blind no-motion timeout is invalid")
        if not 0.5 <= self.blind_no_progress_timeout_sec <= 5.0:
            raise ValueError("blind no-progress timeout is invalid")
        if not 0.005 <= self.blind_progress_step_m <= 0.05:
            raise ValueError("blind progress step is invalid")
        if not 1.0 <= self.blind_max_heading_change_deg <= 8.0:
            raise ValueError("blind heading limit is invalid")
        if not 0.03 <= self.blind_max_lateral_deviation_m <= 0.20:
            raise ValueError("blind lateral limit is invalid")
        if not 0.20 <= self.final_stop_confirm_sec <= 1.50:
            raise ValueError("final stop confirmation duration is invalid")
        if not 0.005 <= self.final_stop_speed_mps <= 0.02:
            raise ValueError("final stop speed threshold is invalid")
        if self.final_stop_speed_mps >= self.settle_speed_mps:
            raise ValueError("final stop speed threshold must be below settle speed")
        if not 0.002 <= self.final_stop_max_drift_m <= 0.01:
            raise ValueError("final stop drift limit is invalid")
        if not 1.0 <= self.final_stop_timeout_sec <= 8.0:
            raise ValueError("final stop timeout is invalid")

    def _detection_cb(self, msg):
        receipt = time.monotonic()
        errors = []
        stamp_sec = float(msg.header.stamp.to_sec())
        width = int(msg.image_width)
        height = int(msg.image_height)
        if not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
            errors.append("detection source stamp is missing or invalid")
        if width != self.expected_image_width or height != self.expected_image_height:
            errors.append(
                "detection image is %dx%d, expected %dx%d"
                % (
                    width,
                    height,
                    self.expected_image_width,
                    self.expected_image_height,
                )
            )
        if msg.header.frame_id != self.expected_camera_frame:
            errors.append(
                "detection frame is %r, expected %r"
                % (msg.header.frame_id, self.expected_camera_frame)
            )
        if str(msg.model_sha256).strip().lower() != self.expected_model_sha256:
            errors.append("detector model SHA256 does not match the approved weights")
        if str(msg.model_task).strip().lower() not in ("detect", "segment"):
            errors.append("unsupported detector model task %r" % msg.model_task)

        observations = []
        candidates = []
        for index, item in enumerate(msg.detections):
            detection = PixelDetection(
                class_id=int(item.class_id),
                class_name=str(item.class_name),
                confidence=float(item.confidence),
                x=float(item.bbox.x_offset),
                y=float(item.bbox.y_offset),
                width=float(item.bbox.width),
                height=float(item.bbox.height),
                anchor_u=float(item.anchor_px.x),
                anchor_v=float(item.anchor_px.y),
            )
            try:
                validate_detection(detection, width, height)
            except ValueError as exc:
                errors.append("detection[%d] is invalid: %s" % (index, exc))
                continue
            if detection.class_name in self.allowed_class_names:
                observations.append(detection)
                if detection.confidence >= self.min_confidence:
                    candidates.append(detection)

        with self.sensor_lock:
            if (
                math.isfinite(stamp_sec)
                and stamp_sec > 0.0
                and self.last_detection_stamp is not None
                and stamp_sec <= self.last_detection_stamp
            ):
                errors.append(
                    "detection source stamp did not increase (%.9f <= %.9f)"
                    % (stamp_sec, self.last_detection_stamp)
                )
            if math.isfinite(stamp_sec) and stamp_sec > 0.0:
                self.last_detection_stamp = max(
                    stamp_sec,
                    self.last_detection_stamp
                    if self.last_detection_stamp is not None
                    else stamp_sec,
                )
            self.detection_sequence += 1
            frame = DetectionFrame(
                sequence=self.detection_sequence,
                receipt_monotonic=receipt,
                stamp_sec=stamp_sec,
                width=width,
                height=height,
                observations=tuple(observations),
                candidates=tuple(candidates),
                error="; ".join(errors),
            )
            if len(self.detection_queue) == self.detection_queue.maxlen:
                self.detection_queue_overflow = True
            self.detection_queue.append(frame)
            self.latest_detection = frame

    def _camera_cb(self, msg):
        receipt = time.monotonic()
        stamp_sec = float(msg.header.stamp.to_sec())
        width = int(msg.width)
        height = int(msg.height)
        cx = float(msg.K[2]) if len(msg.K) >= 3 else float("nan")
        errors = []
        if not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
            errors.append("CameraInfo source stamp is missing or invalid")
        if width != self.expected_image_width or height != self.expected_image_height:
            errors.append(
                "CameraInfo is %dx%d, expected %dx%d"
                % (
                    width,
                    height,
                    self.expected_image_width,
                    self.expected_image_height,
                )
            )
        if msg.header.frame_id != self.expected_camera_frame:
            errors.append(
                "CameraInfo frame is %r, expected %r"
                % (msg.header.frame_id, self.expected_camera_frame)
            )
        if not math.isfinite(cx) or not 0.0 < cx < width:
            errors.append("CameraInfo principal point cx is invalid")
        state = CameraState(
            receipt_monotonic=receipt,
            stamp_sec=stamp_sec,
            width=width,
            height=height,
            cx=cx,
            frame_id=msg.header.frame_id,
            error="; ".join(errors),
        )
        with self.sensor_lock:
            if state.error:
                self.invalid_camera_generation += 1
            self.latest_camera = state

    def _odom_cb(self, msg):
        receipt = time.monotonic()
        try:
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            stamp_sec = float(msg.header.stamp.to_sec())
            values = (
                position.x,
                position.y,
                msg.twist.twist.linear.x,
                msg.twist.twist.angular.z,
                stamp_sec,
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError("odometry pose, twist, or stamp is non-finite")
            if stamp_sec <= 0.0:
                raise ValueError("odometry source stamp is missing")
            if msg.header.frame_id != self.expected_odom_frame:
                raise ValueError(
                    "odometry frame is %r, expected %r"
                    % (msg.header.frame_id, self.expected_odom_frame)
                )
            if msg.child_frame_id != self.expected_base_frame:
                raise ValueError(
                    "odometry child frame is %r, expected %r"
                    % (msg.child_frame_id, self.expected_base_frame)
                )
            yaw = quaternion_to_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
        except ValueError as exc:
            with self.sensor_lock:
                self.invalid_odom_generation += 1
                self.invalid_odom_reason = str(exc)
                self.invalid_odom_monotonic = receipt
            return

        with self.sensor_lock:
            if self.last_odom_stamp is not None and stamp_sec < self.last_odom_stamp:
                self.invalid_odom_generation += 1
                self.invalid_odom_reason = (
                    "odometry source stamp moved backwards %.9f < %.9f"
                    % (stamp_sec, self.last_odom_stamp)
                )
                self.invalid_odom_monotonic = receipt
                return
            self.last_odom_stamp = stamp_sec
            self.odom_sequence += 1
            state = OdomState(
                sequence=self.odom_sequence,
                receipt_monotonic=receipt,
                stamp_sec=stamp_sec,
                x=float(position.x),
                y=float(position.y),
                yaw=yaw,
                linear_x=float(msg.twist.twist.linear.x),
                angular_z=float(msg.twist.twist.angular.z),
                frame_id=msg.header.frame_id,
                child_frame_id=msg.child_frame_id,
            )
            if abs(state.linear_x) > self.settle_speed_mps:
                self.last_unstopped_odom_sequence = state.sequence
            if abs(state.linear_x) > self.final_stop_speed_mps:
                self.last_final_unstopped_odom_sequence = state.sequence
            self.latest_odom = state
            if len(self.odom_queue) == self.odom_queue.maxlen:
                self.odom_queue_overflow = True
            self.odom_queue.append(state)
            self.odom_history.append(state)
            self.invalid_odom_reason = ""

    def _wheel_angle_cb(self, msg):
        receipt = time.monotonic()
        value = float(msg.data)
        with self.sensor_lock:
            if math.isfinite(value):
                self.latest_wheel_angle = value
                self.latest_wheel_angle_monotonic = receipt
                self.wheel_sequence += 1
                if abs(math.degrees(value)) > self.settle_wheel_angle_deg:
                    self.last_uncentered_wheel_sequence = self.wheel_sequence
                self.invalid_wheel_reason = ""
            else:
                self.invalid_wheel_generation += 1
                self.invalid_wheel_reason = "wheel angle is non-finite"
                self.invalid_wheel_monotonic = receipt

    def _chassis_status_cb(self, msg):
        with self.sensor_lock:
            self.latest_chassis_status = msg
            self.latest_chassis_status_monotonic = time.monotonic()
            if (
                msg.hard_emergency
                or msg.soft_emergency
                or msg.gamepad_emergency
                or msg.robot_emergency
            ):
                self.chassis_fault_generation += 1
                self.last_chassis_fault_reason = (
                    "hard=%s soft=%s gamepad=%s robot=%s"
                    % (
                        msg.hard_emergency,
                        msg.soft_emergency,
                        msg.gamepad_emergency,
                        msg.robot_emergency,
                    )
                )

    def _control_timeout_cb(self, msg):
        if msg.data:
            with self.sensor_lock:
                self.control_timeout_seen = True

    def _m2_bypass_control_cb(self, _msg, topic):
        # A one-shot rostopic publisher may register, send, and disappear
        # entirely between two ROS-master graph polls.  Latch the message itself
        # so no active-session M2 side-channel command can be hidden that way.
        with self.sensor_lock:
            self.m2_bypass_event_generation += 1
            self.last_m2_bypass_event_topic = str(topic)

    def _raw_canbus_cb(self, msg):
        if msg.node_type != VCU_NODE_TYPE or msg.node_seq != VCU_NODE_ID:
            return
        if msg.msg_type not in RAW_OBSERVED_TYPES:
            return
        receipt = time.monotonic()
        error = ""
        value = None
        safe = False
        try:
            if msg.msg_type == VCU_CONTROLLER_MONITOR:
                if len(msg.payload) < 3:
                    raise ValueError("controller monitor payload has fewer than 3 bytes")
                values = tuple(int(item) for item in msg.payload[:3])
                # Bits 0..2 mean emergency/status, data timeout, and current
                # over-limit.  Bit 3 is the motor-brake state and may be set
                # normally while stopped, so progress watchdogs handle it.
                safe = all((item & 0x07) == 0 for item in values)
                value = values
            else:
                if not msg.payload:
                    raise ValueError("raw CAN response has no payload")
                value = int(msg.payload[0])
                safe = (
                    value == VCU_RUNNING_STATE
                    if msg.msg_type == VCU_COMMON_STATE
                    else value == 0
                )
        except ValueError as exc:
            error = str(exc)
        with self.sensor_lock:
            self.raw_can_status[int(msg.msg_type)] = (
                receipt,
                safe,
                value,
                error,
            )
            if error or not safe:
                self.raw_can_fault_generation += 1
                self.last_raw_can_fault_reason = (
                    "%s unsafe=%s value=%r error=%s"
                    % (
                        RAW_OBSERVED_TYPES.get(msg.msg_type, "0x%02X" % msg.msg_type),
                        not safe,
                        value,
                        error or "none",
                    )
                )

    @staticmethod
    def _topic_nodes(entries, topic):
        for candidate, nodes in entries:
            if candidate == topic:
                return set(nodes)
        return set()

    @staticmethod
    def _format_nodes(nodes):
        return ", ".join(sorted(nodes)) if nodes else "none"

    def _check_graph(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_graph_check_monotonic < self.graph_check_interval_sec:
            return
        if self.master.getPid() != self.master_pid:
            raise ControllerAbort("ROS master was replaced during visual-servo mode")
        if rospy.get_param("/use_sim_time", False) is not False:
            raise ControllerAbort("real-vehicle visual servo requires /use_sim_time=false")

        publishers, subscribers, services = self.master.getSystemState()
        expected_publishers = (
            (self.detections_topic, self.expected_detector_node),
            (self.camera_info_topic, self.expected_camera_node),
            (self.odom_topic, self.expected_driver_node),
            (self.wheel_angle_topic, self.expected_driver_node),
            (self.chassis_status_topic, self.expected_driver_node),
            (self.control_timeout_topic, self.expected_driver_node),
            (self.canbus_topic, self.expected_canbus_node),
        )
        for topic, expected_node in expected_publishers:
            actual = self._topic_nodes(publishers, topic)
            if actual != {expected_node}:
                raise ControllerAbort(
                    "%s publisher must be exactly %s; current: %s"
                    % (topic, expected_node, self._format_nodes(actual))
                )

        command_publishers = self._topic_nodes(publishers, self.cmd_vel_topic)
        other_command_publishers = command_publishers - {rospy.get_name()}
        if other_command_publishers:
            raise ControllerAbort(
                "%s has another publisher; refusing control conflict: %s"
                % (
                    self.cmd_vel_topic,
                    self._format_nodes(other_command_publishers),
                )
            )
        ackermann_publishers = self._topic_nodes(publishers, self.ackermann_topic)
        if ackermann_publishers:
            raise ControllerAbort(
                "%s has publishers and can bypass cmd_vel control: %s"
                % (self.ackermann_topic, self._format_nodes(ackermann_publishers))
            )
        bypass_publishers = find_forbidden_publishers(
            publishers, self.m2_bypass_topics
        )
        if bypass_publishers:
            details = "; ".join(
                "%s: %s" % (topic, self._format_nodes(nodes))
                for topic, nodes in bypass_publishers
            )
            raise ControllerAbort(
                "M2 bypass-control publisher is active (%s). Complete any "
                "steering-center calibration before enabling with rostopic pub -1, "
                "then wait for that one-shot publisher (and all other listed "
                "publishers) to exit" % details
            )

        canbus_services = self._topic_nodes(services, self.canbus_service)
        if canbus_services != {self.expected_canbus_node}:
            raise ControllerAbort(
                "%s provider must be exactly %s; current: %s"
                % (
                    self.canbus_service,
                    self.expected_canbus_node,
                    self._format_nodes(canbus_services),
                )
            )
        parameter_services = self._topic_nodes(
            services, self.chassis_parameter_service
        )
        if parameter_services != {self.expected_driver_node}:
            raise ControllerAbort(
                "%s provider must be exactly %s; current: %s"
                % (
                    self.chassis_parameter_service,
                    self.expected_driver_node,
                    self._format_nodes(parameter_services),
                )
            )

        command_subscribers = self._topic_nodes(subscribers, self.cmd_vel_topic)
        if self.expected_driver_node not in command_subscribers:
            raise ControllerAbort(
                "%s is not connected to %s; subscribers: %s"
                % (
                    self.cmd_vel_topic,
                    self.expected_driver_node,
                    self._format_nodes(command_subscribers),
                )
            )
        if self.cmd_pub.get_num_connections() < 1:
            raise ControllerAbort("cmd_vel publisher is not connected to the M2 driver")
        if rospy.get_param(self.expected_driver_node + "/is_pub_control_timeout", None) is not True:
            raise ControllerAbort(
                self.expected_driver_node + "/is_pub_control_timeout must be true"
            )
        self.last_graph_check_monotonic = now

    def _query_and_check_raw_can(self):
        now = time.monotonic()
        if now - self.last_raw_query_monotonic >= self.raw_can_query_interval_sec:
            requests = []
            for msg_type in RAW_REQUIRED_TYPES:
                request = CanBusMessage()
                request.node_type = VCU_NODE_TYPE
                request.node_seq = VCU_NODE_ID
                request.msg_type = msg_type
                request.payload = []
                requests.append(request)
            try:
                self.canbus_proxy(requests)
            except (rospy.ROSException, rospy.ServiceException) as exc:
                raise ControllerAbort("raw CAN emergency query failed: %s" % exc)
            self.last_raw_query_monotonic = now

        with self.sensor_lock:
            statuses = dict(self.raw_can_status)
            fault_generation = self.raw_can_fault_generation
            fault_reason = self.last_raw_can_fault_reason
        now = time.monotonic()
        if (
            self.phase != PRECHECK
            and fault_generation > self.session_raw_can_fault_generation
        ):
            raise ControllerAbort(
                "raw CAN fault was observed during the active session: %s"
                % (fault_reason or "unknown")
            )
        for msg_type, label in RAW_REQUIRED_TYPES.items():
            status = statuses.get(msg_type)
            if status is None:
                raise ControllerAbort("waiting for raw CAN response: %s" % label)
            receipt, safe, value, error = status
            age = now - receipt
            if age > self.raw_can_timeout_sec:
                raise ControllerAbort("raw CAN %s response is stale %.3fs" % (label, age))
            if error:
                raise ControllerAbort("raw CAN %s response is invalid: %s" % (label, error))
            if not safe:
                raise ControllerAbort("raw CAN %s is unsafe, value=%r" % (label, value))

        # ControllerMonitor is not guaranteed to be queryable on every VCU
        # firmware, but if it is broadcast, never ignore a fresh fault.
        monitor = statuses.get(VCU_CONTROLLER_MONITOR)
        if monitor is not None:
            receipt, safe, value, error = monitor
            if now - receipt <= self.raw_can_timeout_sec:
                if error:
                    raise ControllerAbort("controller monitor response is invalid: %s" % error)
                if not safe:
                    raise ControllerAbort("controller monitor reports a fault: %r" % (value,))

    def _source_age_check(self, label, stamp_sec):
        age = rospy.Time.now().to_sec() - stamp_sec
        if age < -0.20 or age > self.source_stamp_timeout_sec:
            raise ControllerAbort("%s source stamp age is invalid: %.3fs" % (label, age))

    def _sensor_snapshot(self):
        with self.sensor_lock:
            return {
                "detection": self.latest_detection,
                "camera": self.latest_camera,
                "invalid_camera_generation": self.invalid_camera_generation,
                "odom": self.latest_odom,
                "invalid_odom_reason": self.invalid_odom_reason,
                "invalid_odom_monotonic": self.invalid_odom_monotonic,
                "invalid_odom_generation": self.invalid_odom_generation,
                "wheel_angle": self.latest_wheel_angle,
                "wheel_receipt": self.latest_wheel_angle_monotonic,
                "invalid_wheel_reason": self.invalid_wheel_reason,
                "invalid_wheel_monotonic": self.invalid_wheel_monotonic,
                "invalid_wheel_generation": self.invalid_wheel_generation,
                "chassis": self.latest_chassis_status,
                "chassis_receipt": self.latest_chassis_status_monotonic,
                "chassis_fault_generation": self.chassis_fault_generation,
                "last_chassis_fault_reason": self.last_chassis_fault_reason,
                "m2_bypass_event_generation": self.m2_bypass_event_generation,
                "last_m2_bypass_event_topic": self.last_m2_bypass_event_topic,
                "control_timeout_seen": self.control_timeout_seen,
                "detection_overflow": self.detection_queue_overflow,
                "odom_overflow": self.odom_queue_overflow,
            }

    def _terminal_sensor_fence_locked(self):
        """Capture terminal feedback state while ``sensor_lock`` is held."""

        return TerminalSensorFence(
            odom_sequence=self.odom_sequence,
            wheel_sequence=self.wheel_sequence,
            detection_sequence=self.detection_sequence,
            invalid_camera_generation=self.invalid_camera_generation,
            invalid_odom_generation=self.invalid_odom_generation,
            invalid_wheel_generation=self.invalid_wheel_generation,
            chassis_fault_generation=self.chassis_fault_generation,
            raw_can_fault_generation=self.raw_can_fault_generation,
            m2_bypass_event_generation=self.m2_bypass_event_generation,
            control_timeout_seen=self.control_timeout_seen,
            detection_queue_size=len(self.detection_queue),
            odom_queue_size=len(self.odom_queue),
            detection_queue_overflow=self.detection_queue_overflow,
            odom_queue_overflow=self.odom_queue_overflow,
        )

    def _check_terminal_commit_health_locked(self):
        """Revalidate freshness/deadlines immediately before COMPLETE.

        The regular health check ran earlier in this control tick.  A
        non-real-time scheduler can pause the thread after that check without
        generating any callback, so sequence/generation fencing alone cannot
        reveal that every feedback stream became stale while the tick slept.
        This method is called under ``sensor_lock`` at the terminal commit
        boundary and rejects that no-callback time-of-check/time-of-use gap.
        """

        if rospy.is_shutdown():
            raise ControllerAbort("ROS is shutting down before COMPLETE")
        if self.operator_disable_requested.is_set():
            raise ControllerAbort("operator disable request inhibits COMPLETE")
        if self.mode_deadline_monotonic is None:
            raise ControllerAbort("terminal commit has no mode deadline")
        if self.final_stop_started_monotonic is None:
            raise ControllerAbort("terminal commit has no final-stop deadline")

        detection = self.latest_detection
        camera = self.latest_camera
        odom = self.latest_odom
        wheel_angle = self.latest_wheel_angle
        wheel_receipt = self.latest_wheel_angle_monotonic
        chassis = self.latest_chassis_status
        chassis_receipt = self.latest_chassis_status_monotonic
        if detection is None or detection.error:
            raise ControllerAbort("terminal commit lacks valid detections")
        if camera is None or camera.error:
            raise ControllerAbort("terminal commit lacks valid CameraInfo")
        if odom is None:
            raise ControllerAbort("terminal commit lacks valid odometry")
        if wheel_angle is None or wheel_receipt is None or not math.isfinite(
            wheel_angle
        ):
            raise ControllerAbort("terminal commit lacks valid wheel feedback")
        if chassis is None or chassis_receipt is None:
            raise ControllerAbort("terminal commit lacks valid chassis status")
        if (
            chassis.hard_emergency
            or chassis.soft_emergency
            or chassis.gamepad_emergency
            or chassis.robot_emergency
        ):
            raise ControllerAbort("terminal commit observed an active chassis emergency")

        receipt_limits = [
            (detection.receipt_monotonic, self.detection_timeout_sec),
            (camera.receipt_monotonic, self.camera_info_timeout_sec),
            (odom.receipt_monotonic, self.odom_timeout_sec),
            (wheel_receipt, self.wheel_angle_timeout_sec),
            (chassis_receipt, self.chassis_status_timeout_sec),
        ]
        for msg_type, label in RAW_REQUIRED_TYPES.items():
            status = self.raw_can_status.get(msg_type)
            if status is None:
                raise ControllerAbort(
                    "terminal commit lacks raw CAN status: %s" % label
                )
            receipt, safe, value, error = status
            if error or not safe:
                raise ControllerAbort(
                    "terminal commit raw CAN %s is unsafe, value=%r error=%s"
                    % (label, value, error or "none")
                )
            receipt_limits.append((receipt, self.raw_can_timeout_sec))

        # Sample both clocks at the last practical point before phase commit.
        # terminal_feedback_is_fresh also rechecks the absolute mode and
        # FINAL_STOP deadlines, which the earlier tick-level checks may no
        # longer satisfy after a scheduler pause.
        now_source_time = rospy.Time.now().to_sec()
        now_monotonic = time.monotonic()
        if not terminal_feedback_is_fresh(
            now_monotonic=now_monotonic,
            now_source_time=now_source_time,
            receipt_limits=receipt_limits,
            source_stamps=(
                detection.stamp_sec,
                camera.stamp_sec,
                odom.stamp_sec,
            ),
            source_timeout=self.source_stamp_timeout_sec,
            absolute_deadlines=(
                self.mode_deadline_monotonic,
                self.final_stop_started_monotonic + self.final_stop_timeout_sec,
            ),
        ):
            raise ControllerAbort(
                "feedback or an absolute deadline became stale before COMPLETE"
            )

    def _arm_session_fault_latches(self):
        """Atomically verify current safety samples and capture generation floors."""

        with self.sensor_lock:
            camera = self.latest_camera
            odom = self.latest_odom
            wheel_receipt = self.latest_wheel_angle_monotonic
            chassis = self.latest_chassis_status
            if camera is None or camera.error:
                raise ControllerAbort("cannot arm with invalid CameraInfo")
            if odom is None or (
                self.invalid_odom_reason
                and self.invalid_odom_monotonic >= odom.receipt_monotonic
            ):
                raise ControllerAbort("cannot arm with invalid odometry")
            if self.latest_wheel_angle is None or wheel_receipt is None or (
                self.invalid_wheel_reason
                and self.invalid_wheel_monotonic >= wheel_receipt
            ):
                raise ControllerAbort("cannot arm with invalid wheel-angle feedback")
            if chassis is None or (
                chassis.hard_emergency
                or chassis.soft_emergency
                or chassis.gamepad_emergency
                or chassis.robot_emergency
            ):
                raise ControllerAbort("cannot arm while a chassis emergency is active")
            for msg_type, label in RAW_REQUIRED_TYPES.items():
                status = self.raw_can_status.get(msg_type)
                if status is None or status[3] or not status[1]:
                    raise ControllerAbort(
                        "cannot arm with unsafe raw CAN status: %s" % label
                    )

            # Any callback after this atomic capture advances its generation
            # above the floor and will be visible once ACQUIRE becomes active.
            self.session_invalid_camera_generation = self.invalid_camera_generation
            self.session_invalid_odom_generation = self.invalid_odom_generation
            self.session_invalid_wheel_generation = self.invalid_wheel_generation
            self.session_chassis_fault_generation = self.chassis_fault_generation
            self.session_raw_can_fault_generation = self.raw_can_fault_generation
            self.session_m2_bypass_event_generation = (
                self.m2_bypass_event_generation
            )

    def _check_sensor_health(self, require_new_detection=False, ignore_control_timeout=False):
        now = time.monotonic()
        snapshot = self._sensor_snapshot()
        detection = snapshot["detection"]
        if detection is None:
            raise ControllerAbort("no detection messages have been received")
        if detection.error:
            raise ControllerAbort("invalid detection stream: %s" % detection.error)
        detection_age = now - detection.receipt_monotonic
        if detection_age > self.detection_timeout_sec:
            raise ControllerAbort(
                "%s receipt timeout %.3fs" % (self.detections_topic, detection_age)
            )
        self._source_age_check("detections", detection.stamp_sec)
        if require_new_detection and detection.sequence <= self.session_detection_floor:
            raise ControllerAbort("waiting for a new detection frame after enable")
        if snapshot["detection_overflow"]:
            raise ControllerAbort("detection processing queue overflowed")
        if snapshot["odom_overflow"]:
            raise ControllerAbort("odometry processing queue overflowed")

        camera = snapshot["camera"]
        if camera is None:
            raise ControllerAbort("no CameraInfo has been received")
        if camera.error:
            raise ControllerAbort("invalid CameraInfo: %s" % camera.error)
        if (
            self.phase != PRECHECK
            and snapshot["invalid_camera_generation"]
            > self.session_invalid_camera_generation
        ):
            raise ControllerAbort(
                "an invalid CameraInfo sample was observed during the active session"
            )
        camera_age = now - camera.receipt_monotonic
        if camera_age > self.camera_info_timeout_sec:
            raise ControllerAbort(
                "%s receipt timeout %.3fs" % (self.camera_info_topic, camera_age)
            )
        self._source_age_check("CameraInfo", camera.stamp_sec)

        odom = snapshot["odom"]
        if odom is None:
            raise ControllerAbort(
                "no valid odometry: %s" % snapshot["invalid_odom_reason"]
            )
        if (
            snapshot["invalid_odom_reason"]
            and snapshot["invalid_odom_monotonic"] >= odom.receipt_monotonic
        ):
            raise ControllerAbort(
                "invalid odometry: %s" % snapshot["invalid_odom_reason"]
            )
        if (
            self.phase != PRECHECK
            and snapshot["invalid_odom_generation"]
            > self.session_invalid_odom_generation
        ):
            raise ControllerAbort(
                "an invalid odometry sample was observed during the active session"
            )
        odom_age = now - odom.receipt_monotonic
        if odom_age > self.odom_timeout_sec:
            raise ControllerAbort("%s receipt timeout %.3fs" % (self.odom_topic, odom_age))
        self._source_age_check("odometry", odom.stamp_sec)
        if abs(odom.linear_x) > self.max_measured_speed_mps:
            raise ControllerAbort(
                "measured speed %.3f m/s exceeds %.3f m/s"
                % (odom.linear_x, self.max_measured_speed_mps)
            )
        if odom.linear_x < -0.03:
            raise ControllerAbort("unexpected reverse motion %.3f m/s" % odom.linear_x)

        wheel_angle = snapshot["wheel_angle"]
        wheel_receipt = snapshot["wheel_receipt"]
        if wheel_angle is None or wheel_receipt is None:
            raise ControllerAbort(
                "no valid wheel angle: %s" % snapshot["invalid_wheel_reason"]
            )
        if (
            snapshot["invalid_wheel_reason"]
            and snapshot["invalid_wheel_monotonic"] >= wheel_receipt
        ):
            raise ControllerAbort(
                "invalid wheel angle: %s" % snapshot["invalid_wheel_reason"]
            )
        if (
            self.phase != PRECHECK
            and snapshot["invalid_wheel_generation"]
            > self.session_invalid_wheel_generation
        ):
            raise ControllerAbort(
                "an invalid wheel-angle sample was observed during the active session"
            )
        wheel_age = now - wheel_receipt
        if wheel_age > self.wheel_angle_timeout_sec:
            raise ControllerAbort(
                "%s receipt timeout %.3fs" % (self.wheel_angle_topic, wheel_age)
            )
        if abs(math.degrees(wheel_angle)) > self.max_steering_angle_deg + 3.0:
            raise ControllerAbort(
                "wheel angle feedback is outside the controller envelope: %+.2fdeg"
                % math.degrees(wheel_angle)
            )

        chassis = snapshot["chassis"]
        chassis_receipt = snapshot["chassis_receipt"]
        if chassis is None or chassis_receipt is None:
            raise ControllerAbort("no chassis status has been received")
        if now - chassis_receipt > self.chassis_status_timeout_sec:
            raise ControllerAbort("chassis status is stale")
        if (
            chassis.hard_emergency
            or chassis.soft_emergency
            or chassis.gamepad_emergency
            or chassis.robot_emergency
        ):
            raise ControllerAbort(
                "chassis emergency is active: hard=%s soft=%s gamepad=%s robot=%s"
                % (
                    chassis.hard_emergency,
                    chassis.soft_emergency,
                    chassis.gamepad_emergency,
                    chassis.robot_emergency,
                )
            )
        if (
            self.phase != PRECHECK
            and snapshot["chassis_fault_generation"]
            > self.session_chassis_fault_generation
        ):
            raise ControllerAbort(
                "a chassis emergency was observed during the active session: %s"
                % (snapshot["last_chassis_fault_reason"] or "unknown")
            )
        if (
            self.phase != PRECHECK
            and snapshot["m2_bypass_event_generation"]
            > self.session_m2_bypass_event_generation
        ):
            raise ControllerAbort(
                "an M2 bypass-control message was observed during the active "
                "session on %s"
                % (snapshot["last_m2_bypass_event_topic"] or "an unknown topic")
            )
        if snapshot["control_timeout_seen"] and not ignore_control_timeout:
            raise ControllerAbort("VCU reported a 200ms command timeout")

    def _check_health(self, force_graph=False, require_new_detection=False, ignore_control_timeout=False):
        if rospy.is_shutdown():
            raise ControllerAbort("ROS is shutting down")
        self._check_graph(force=force_graph)
        self._query_and_check_raw_can()
        self._check_sensor_health(
            require_new_detection=require_new_detection,
            ignore_control_timeout=ignore_control_timeout,
        )
        with self.command_lock:
            expired_reason = self.motion_lease.expired_reason
        if expired_reason:
            raise ControllerAbort(expired_reason)

    def _read_chassis_parameters(self):
        try:
            response = self.chassis_parameter_proxy()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise ControllerAbort("failed to read M2 chassis parameters: %s" % exc)
        parameters = response.parameters
        values = (
            float(parameters.max_speed),
            float(parameters.max_steer),
            float(parameters.robot_width),
            float(parameters.robot_length),
            float(parameters.wheel_radius),
        )
        if not response.success or not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ControllerAbort(
                "M2 chassis parameters are invalid: success=%s values=%r message=%s"
                % (response.success, values, response.message)
            )
        wheelbase = float(parameters.robot_length)
        if not 0.20 <= wheelbase <= 3.0:
            raise ControllerAbort("M2 wheelbase is implausible: %.3fm" % wheelbase)
        chassis_limit_deg = math.degrees(float(parameters.max_steer))
        safe_angle_deg = min(self.max_steering_angle_deg, 0.90 * chassis_limit_deg)
        if safe_angle_deg < 2.0:
            raise ControllerAbort(
                "M2 steering limit is too small for controlled approach: %.2fdeg"
                % chassis_limit_deg
            )
        self.wheelbase_m = wheelbase
        self.max_curvature = math.tan(math.radians(safe_angle_deg)) / wheelbase

    def _hard_stop(self):
        with self.command_lock:
            self.motion_lease.stop()
            self.last_command_update_monotonic = time.monotonic()

    def _set_motion(self, target_speed, target_curvature):
        if self.operator_disable_requested.is_set():
            raise ControllerAbort("operator disable request inhibits motion")
        if self.mode_deadline_monotonic is None:
            raise ControllerAbort("non-zero command has no mode deadline")
        if not all(math.isfinite(value) for value in (target_speed, target_curvature)):
            raise ControllerAbort("computed motion command is non-finite")
        if not 0.0 < target_speed <= MAX_COMMAND_SPEED_MPS:
            raise ControllerAbort("computed speed is outside the hard limit")
        if self.max_curvature is None or abs(target_curvature) > self.max_curvature + 1e-9:
            raise ControllerAbort("computed curvature is outside the steering envelope")

        with self.command_lock:
            # Take time only after acquiring the publisher's lock.  A timestamp
            # sampled before a lock wait could predate the old lease deadline
            # and incorrectly revive a command that has already expired.
            now = time.monotonic()
            if self.operator_disable_requested.is_set():
                self.motion_lease.stop()
                raise ControllerAbort("operator disable request inhibits motion")
            if self.motion_lease.expired_reason:
                raise ControllerAbort(self.motion_lease.expired_reason)
            dt = min(0.20, max(1.0 / self.control_rate_hz, now - self.last_command_update_monotonic))
            previous_speed = self.motion_lease.linear_x
            previous_curvature = self.motion_lease.curvature
            speed_step = self.max_linear_acceleration_mps2 * dt
            curvature_step = self.max_curvature_rate_per_sec * dt
            speed = max(
                previous_speed - speed_step,
                min(previous_speed + speed_step, target_speed),
            )
            curvature = max(
                previous_curvature - curvature_step,
                min(previous_curvature + curvature_step, target_curvature),
            )
            speed = min(speed, MAX_COMMAND_SPEED_MPS)
            curvature = max(-self.max_curvature, min(self.max_curvature, curvature))
            try:
                renewed_at = renew_motion_lease_now(
                    self.motion_lease,
                    speed,
                    curvature,
                    self.mode_deadline_monotonic,
                    time.monotonic,
                )
            except (RuntimeError, ValueError) as exc:
                raise ControllerAbort(str(exc))
            self.last_command_update_monotonic = renewed_at

    def _publish_command(self, _event=None):
        expired_reason = ""
        command = Twist()
        try:
            with self.command_lock:
                if self.operator_disable_requested.is_set():
                    self.motion_lease.stop()
                linear_x, curvature, expired_reason = self.motion_lease.sample(
                    time.monotonic()
                )
                if not all(math.isfinite(value) for value in (linear_x, curvature)):
                    self.motion_lease.stop()
                    self.motion_lease.expired_reason = "non-finite command reached publisher"
                    expired_reason = self.motion_lease.expired_reason
                    linear_x = 0.0
                    curvature = 0.0
                if self.operator_disable_requested.is_set():
                    self.motion_lease.stop()
                    linear_x = 0.0
                    curvature = 0.0
                command.linear.x = linear_x
                command.angular.z = linear_x * curvature
                # Publication is serialized with command transitions so a
                # previously sampled positive command cannot overtake a stop.
                self.cmd_pub.publish(command)
        except Exception as exc:
            rospy.logerr_throttle(1.0, "cmd_vel publisher failed: %s", exc)
        if expired_reason:
            rospy.logerr_throttle(1.0, "%s", expired_reason)

    def _set_enabled_cb(self, request):
        if not request.data:
            # Set this before taking either lock.  Both the control path and
            # command publisher consult it, so an in-flight control tick can
            # never renew or publish a positive command after STOP is asked.
            self.operator_disable_requested.set()
            self._hard_stop()
            with self.run_lock:
                self._reset_session(DISABLED, "motion disabled by operator")
                self._publish_status(force=True)
            return SetBoolResponse(success=True, message="visual servo disabled; cmd_vel is zero")

        if not self.allow_motion:
            return SetBoolResponse(
                success=False,
                message=(
                    "motion authorization is absent; relaunch with allow_motion:=true "
                    "after clearing the test area"
                ),
            )

        with self.run_lock:
            if self.phase != DISABLED:
                return SetBoolResponse(
                    success=False,
                    message=(
                        "controller state is %s; call set_enabled false before rearming"
                        % self.phase
                    ),
                )
            now = time.monotonic()
            with self.sensor_lock:
                self.session_detection_floor = self.detection_sequence
                self.detection_queue.clear()
                self.odom_queue.clear()
                self.detection_queue_overflow = False
                self.odom_queue_overflow = False
                self.control_timeout_seen = False
                self.session_invalid_odom_generation = self.invalid_odom_generation
                self.session_invalid_wheel_generation = self.invalid_wheel_generation
                self.session_invalid_camera_generation = self.invalid_camera_generation
                self.session_chassis_fault_generation = self.chassis_fault_generation
                self.session_raw_can_fault_generation = self.raw_can_fault_generation
                self.session_m2_bypass_event_generation = (
                    self.m2_bypass_event_generation
                )
            with self.command_lock:
                self.motion_lease = MotionLease(self.command_lease_sec)
                self.last_command_update_monotonic = now
            self.operator_disable_requested.clear()
            self.machine = TargetPhaseMachine(
                self.machine_config, self.association_config
            )
            self.phase = PRECHECK
            self.reason = "running graph, sensor, emergency-stop, and chassis prechecks"
            self.mode_started_monotonic = now
            self.mode_deadline_monotonic = now + self.mode_timeout_sec
            self.precheck_deadline_monotonic = now + self.precheck_timeout_sec
            self.last_processed_detection_sequence = self.session_detection_floor
            with self.sensor_lock:
                self.last_processed_odom_sequence = self.odom_sequence
            self.wheelbase_m = None
            self.max_curvature = None
            self._clear_run_metrics()
            self._publish_status(force=True)
        rospy.logwarn(
            "FOD visual-servo enable requested. Vehicle remains stopped until all prechecks "
            "pass and exactly one target is stable. Keep the physical emergency stop ready."
        )
        return SetBoolResponse(
            success=True,
            message="enable accepted; watch /fod_visual_servo/state for ACQUIRE",
        )

    def _clear_run_metrics(self):
        self.target_u_px = None
        self.horizontal_error_value = None
        self.vertical_fraction_value = None
        self.target_visible = False
        self.approach_path_m = 0.0
        self.approach_last_odom = None
        self.approach_motion_started_monotonic = None
        self.approach_motion_seen = False
        self.approach_last_progress_monotonic = None
        self.approach_next_progress_m = self.approach_progress_step_m
        self.blind_tracker = None
        self.blind_progress = None
        self.blind_last_odom_state = None
        self.pending_loss_stamp_sec = None
        self.pending_loss_monotonic = None
        self.blind_started_monotonic = None
        self.blind_drive_started_monotonic = None
        self.blind_motion_seen = False
        self.blind_last_progress_monotonic = None
        self.blind_next_progress_m = self.blind_progress_step_m
        self.blind_max_forward_m = 0.0
        self.blind_seen_uncentered_wheel_sequence = 0
        self.final_stop_started_monotonic = None
        self.final_stop_last_odom_sequence = 0
        self.final_stop_last_wheel_sequence = 0
        self.final_stop_seen_unstopped_odom_sequence = 0
        self.final_stop_seen_uncentered_wheel_sequence = 0
        self.final_stop_good_start_odom_stamp = None
        self.final_stop_good_start_wheel_receipt = None
        self.final_stop_good_start_path_m = None
        self.settle_started_monotonic = None
        self.settle_last_odom_sequence = 0
        self.settle_last_wheel_sequence = 0
        self.settle_seen_unstopped_odom_sequence = 0
        self.settle_seen_uncentered_wheel_sequence = 0
        self.settle_good_start_odom_stamp = None
        self.settle_good_start_wheel_receipt = None

    def _reset_session(self, phase, reason):
        self.phase = phase
        self.reason = reason
        self.machine = TargetPhaseMachine(
            self.machine_config, self.association_config
        )
        self.mode_started_monotonic = None
        self.mode_deadline_monotonic = None
        self.precheck_deadline_monotonic = None
        self.wheelbase_m = None
        self.max_curvature = None
        self._clear_run_metrics()
        with self.command_lock:
            self.motion_lease = MotionLease(self.command_lease_sec)
            self.last_command_update_monotonic = time.monotonic()

    def _transition(self, phase, reason):
        changed = phase != self.phase
        self.phase = phase
        self.reason = reason
        if changed:
            rospy.logwarn("FOD visual servo state -> %s: %s", phase, reason)

    def _abort_locked(self, reason):
        self._hard_stop()
        if self.phase != ABORT:
            rospy.logerr("FOD visual servo ABORT: %s", reason)
        self.phase = ABORT
        self.reason = str(reason)
        self.target_visible = False

    def _control_tick(self, _event=None):
        try:
            with self.run_lock:
                self._control_tick_locked()
                self._publish_status()
        except ControllerAbort as exc:
            with self.run_lock:
                if self.phase not in (DISABLED, COMPLETE, ABORT):
                    self._abort_locked(str(exc))
                self._publish_status(force=True)
        except Exception as exc:
            rospy.logerr_throttle(1.0, "unexpected visual-servo exception: %s", exc)
            with self.run_lock:
                if self.phase not in (DISABLED, COMPLETE, ABORT):
                    self._abort_locked("unexpected controller exception: %s" % exc)
                self._publish_status(force=True)

    def _control_tick_locked(self):
        if self.phase in (DISABLED, COMPLETE, ABORT):
            self._hard_stop()
            return

        now = time.monotonic()
        if self.mode_deadline_monotonic is None or now >= self.mode_deadline_monotonic:
            raise ControllerAbort("visual-servo mode reached its absolute time limit")

        if self.phase == PRECHECK:
            self._hard_stop()
            try:
                self._check_health(
                    force_graph=True,
                    require_new_detection=True,
                    ignore_control_timeout=True,
                )
                if self.wheelbase_m is None:
                    self._read_chassis_parameters()
            except ControllerAbort as exc:
                self.reason = "precheck waiting: %s" % exc
                # PRECHECK intentionally does not process historical frames.
                # Drop them so a fixable 15-second precheck wait cannot fill
                # the bounded queue and masquerade as a runtime overrun.
                with self.sensor_lock:
                    self.detection_queue.clear()
                    self.odom_queue.clear()
                    self.detection_queue_overflow = False
                    self.odom_queue_overflow = False
                if now >= self.precheck_deadline_monotonic:
                    raise ControllerAbort("precheck timed out: %s" % exc)
                return

            with self.sensor_lock:
                detection = self.latest_detection
                camera = self.latest_camera
                odom = self.latest_odom
                self.control_timeout_seen = False
                self.detection_queue.clear()
                self.odom_queue.clear()
                self.detection_queue_overflow = False
                self.odom_queue_overflow = False
            self._arm_session_fault_latches()
            target_u = (
                camera.cx
                if self.use_camera_principal_point
                else 0.5 * float(camera.width)
            ) + self.target_u_offset_px
            if not 0.10 * camera.width <= target_u <= 0.90 * camera.width:
                raise ControllerAbort("calibrated target_u lies outside the safe image region")
            self.target_u_px = target_u
            self.machine.reset()
            self.last_processed_detection_sequence = detection.sequence
            self.last_processed_odom_sequence = odom.sequence
            self._transition(
                ACQUIRE,
                "prechecks passed; waiting for exactly one stable central target",
            )
            return

        self._check_health()

        if self.phase in (APPROACH, REACQUIRE, EDGE_ARMED):
            self._update_approach_odometry()
        elif self.phase == ACQUIRE:
            self._discard_odom_events()

        self._process_detection_events()

        if self.phase in (APPROACH, EDGE_ARMED):
            self._command_visual_approach()
        elif self.phase in (ACQUIRE, REACQUIRE, LOSS_CONFIRM):
            self._hard_stop()
            if self.phase == LOSS_CONFIRM:
                self._update_preblind_odometry()
        elif self.phase == STEER_SETTLE:
            self._settle_tick()
        elif self.phase == BLIND_ADVANCE:
            self._blind_advance_tick()
        elif self.phase == FINAL_STOP:
            self._final_stop_tick()
        else:
            raise ControllerAbort("controller entered unknown active state %r" % self.phase)

    def _discard_odom_events(self):
        with self.sensor_lock:
            if self.latest_odom is not None:
                self.last_processed_odom_sequence = self.latest_odom.sequence
            self.odom_queue.clear()

    def _take_odom_events(self):
        with self.sensor_lock:
            events = [
                state
                for state in self.odom_queue
                if state.sequence > self.last_processed_odom_sequence
            ]
            self.odom_queue.clear()
        if events:
            self.last_processed_odom_sequence = events[-1].sequence
        return events

    def _take_detection_events(self):
        with self.sensor_lock:
            events = [
                frame
                for frame in self.detection_queue
                if frame.sequence > self.last_processed_detection_sequence
            ]
            self.detection_queue.clear()
        if events:
            self.last_processed_detection_sequence = events[-1].sequence
        return events

    def _start_approach_odometry(self):
        with self.sensor_lock:
            odom = self.latest_odom
            self.odom_queue.clear()
        if odom is None:
            raise ControllerAbort("cannot start approach distance without odometry")
        self.approach_path_m = 0.0
        self.approach_last_odom = odom
        self.approach_motion_started_monotonic = None
        self.approach_motion_seen = False
        self.approach_last_progress_monotonic = None
        self.approach_next_progress_m = self.approach_progress_step_m
        self.last_processed_odom_sequence = odom.sequence

    def _pause_approach_watchdog(self):
        self.approach_motion_started_monotonic = None
        self.approach_motion_seen = False
        self.approach_last_progress_monotonic = None
        self.approach_next_progress_m = (
            self.approach_path_m + self.approach_progress_step_m
        )

    def _update_approach_odometry(self):
        events = self._take_odom_events()
        if self.approach_last_odom is None:
            if self.machine.locked is not None:
                self._start_approach_odometry()
            return
        previous = self.approach_last_odom
        for state in events:
            if abs(state.linear_x) > self.max_measured_speed_mps:
                raise ControllerAbort(
                    "approach measured speed %.3fm/s exceeds %.3fm/s"
                    % (state.linear_x, self.max_measured_speed_mps)
                )
            if state.linear_x < -0.03:
                raise ControllerAbort(
                    "approach odometry indicates reverse motion %.3fm/s"
                    % state.linear_x
                )
            step = math.hypot(state.x - previous.x, state.y - previous.y)
            if step > self.max_pose_step_m:
                raise ControllerAbort(
                    "approach odometry jump %.3fm exceeds %.3fm"
                    % (step, self.max_pose_step_m)
                )
            self.approach_path_m += step
            previous = state
            if self.approach_path_m >= self.approach_next_progress_m:
                self.approach_motion_seen = True
                self.approach_last_progress_monotonic = time.monotonic()
                while self.approach_path_m >= self.approach_next_progress_m:
                    self.approach_next_progress_m += self.approach_progress_step_m
        self.approach_last_odom = previous
        if self.approach_path_m > self.max_approach_distance_m:
            raise ControllerAbort(
                "target did not reach the bottom gate within %.2fm"
                % self.max_approach_distance_m
            )

    def _acquisition_candidates(self, frame):
        if len(frame.candidates) != 1:
            return frame.candidates
        candidate = frame.candidates[0]
        q = candidate.anchor_v / float(frame.height - 1)
        error = horizontal_error(candidate.anchor_u, self.target_u_px, frame.width)
        if not (
            self.min_acquire_anchor_v_fraction
            <= q
            <= self.max_acquire_anchor_v_fraction
        ):
            return tuple()
        if abs(error) > self.acquire_max_abs_horizontal_error:
            return tuple()
        return frame.candidates

    def _process_detection_events(self):
        frames = self._take_detection_events()
        for frame in frames:
            if frame.error:
                raise ControllerAbort("invalid detection frame: %s" % frame.error)

            if self.phase in (STEER_SETTLE, BLIND_ADVANCE, FINAL_STOP):
                if frame.observations:
                    raise ControllerAbort(
                        "FOD remained/reappeared after blind-zone disappearance was confirmed"
                    )
                continue

            if self.phase not in (
                ACQUIRE,
                APPROACH,
                REACQUIRE,
                EDGE_ARMED,
                LOSS_CONFIRM,
            ):
                continue
            candidates = (
                self._acquisition_candidates(frame)
                if self.phase == ACQUIRE
                else frame.candidates
            )
            if (
                self.phase in (EDGE_ARMED, LOSS_CONFIRM)
                and not candidates
                and frame.observations
            ):
                raise ControllerAbort(
                    "FOD is still visible below the motion-confidence threshold; "
                    "blind advance is inhibited"
                )
            previous_state = self.machine.state
            decision = self.machine.process_frame(
                candidates=candidates,
                image_width=frame.width,
                image_height=frame.height,
                target_u=self.target_u_px,
                approach_distance_m=self.approach_path_m,
                frame_stamp=frame.stamp_sec,
            )
            if decision.fault:
                raise ControllerAbort(decision.fault)
            if decision.acquired:
                self._start_approach_odometry()
            self.phase = decision.state
            self.reason = decision.reason
            self.target_visible = self.phase in (APPROACH, EDGE_ARMED)
            if decision.filtered_u is not None and decision.filtered_v is not None:
                self.horizontal_error_value = horizontal_error(
                    decision.filtered_u, self.target_u_px, frame.width
                )
                self.vertical_fraction_value = decision.filtered_v / float(
                    frame.height - 1
                )

            if self.phase in (REACQUIRE, LOSS_CONFIRM, STEER_SETTLE):
                self._hard_stop()
                self._pause_approach_watchdog()
            if self.phase == LOSS_CONFIRM and previous_state == EDGE_ARMED:
                self._start_blind_measurement(frame.stamp_sec)
            if self.phase == EDGE_ARMED and previous_state == LOSS_CONFIRM:
                self._cancel_blind_measurement()
            if self.phase == STEER_SETTLE and previous_state != STEER_SETTLE:
                # The camera runs faster than odometry, so the first sample
                # after the loss timestamp may still be pending.  Keep zero
                # command and let STEER_SETTLE wait for the interpolation.
                self._ensure_blind_measurement_baseline()
                self.settle_started_monotonic = time.monotonic()
                with self.sensor_lock:
                    self.settle_last_odom_sequence = self.odom_sequence
                    self.settle_last_wheel_sequence = self.wheel_sequence
                    self.settle_seen_unstopped_odom_sequence = (
                        self.last_unstopped_odom_sequence
                    )
                    self.settle_seen_uncentered_wheel_sequence = (
                        self.last_uncentered_wheel_sequence
                    )
                self.settle_good_start_odom_stamp = None
                self.settle_good_start_wheel_receipt = None
            if self.phase != previous_state:
                rospy.logwarn(
                    "FOD visual servo state -> %s: %s", self.phase, self.reason
                )

    def _command_visual_approach(self):
        if (
            self.machine.filtered_u is None
            or self.machine.filtered_v is None
            or self.target_u_px is None
            or self.max_curvature is None
        ):
            raise ControllerAbort("visual approach lacks a valid locked target")
        error = horizontal_error(
            self.machine.filtered_u,
            self.target_u_px,
            self.expected_image_width,
        )
        q = self.machine.filtered_v / float(self.expected_image_height - 1)
        if abs(error) > self.max_runtime_horizontal_error:
            raise ControllerAbort(
                "locked target horizontal error %.3f exceeds %.3f"
                % (error, self.max_runtime_horizontal_error)
            )
        speed = approach_speed(
            vertical_fraction=q,
            horizontal_error_abs=abs(error),
            far_speed=self.far_speed_mps,
            near_speed=self.near_speed_mps,
            slow_start_fraction=self.slow_start_fraction,
            near_start_fraction=self.near_start_fraction,
            lateral_slowdown_error=self.lateral_slowdown_error,
            minimum_lateral_scale=self.minimum_lateral_speed_scale,
        )
        curvature = curvature_from_pixel_error(
            error=error,
            gain=self.curvature_gain,
            steering_sign=self.steering_sign,
            deadband=self.horizontal_deadband,
            max_curvature=self.max_curvature,
        )
        self.horizontal_error_value = error
        self.vertical_fraction_value = q
        now = time.monotonic()
        if self.approach_motion_started_monotonic is None:
            self.approach_motion_started_monotonic = now
        if (
            not self.approach_motion_seen
            and now - self.approach_motion_started_monotonic
            > self.approach_no_motion_timeout_sec
        ):
            raise ControllerAbort("vehicle made no odometry progress during visual approach")
        if (
            self.approach_motion_seen
            and self.approach_last_progress_monotonic is not None
            and now - self.approach_last_progress_monotonic
            > self.approach_no_progress_timeout_sec
        ):
            raise ControllerAbort("visual-approach odometry progress watchdog expired")
        self._set_motion(speed, curvature)

    def _start_blind_measurement(self, loss_stamp_sec):
        if not math.isfinite(loss_stamp_sec) or loss_stamp_sec <= 0.0:
            raise ControllerAbort("first-loss detection stamp is invalid")
        self.pending_loss_stamp_sec = float(loss_stamp_sec)
        self.pending_loss_monotonic = time.monotonic()
        self.blind_tracker = None
        self.blind_progress = None
        self.blind_last_odom_state = None
        self.blind_started_monotonic = self.pending_loss_monotonic
        self.blind_drive_started_monotonic = None
        self.blind_last_progress_monotonic = None
        self.blind_motion_seen = False
        self.blind_next_progress_m = self.blind_progress_step_m
        self.blind_max_forward_m = 0.0
        self.blind_seen_uncentered_wheel_sequence = 0
        self._ensure_blind_measurement_baseline()

    def _ensure_blind_measurement_baseline(self):
        if self.blind_tracker is not None:
            return True
        if self.pending_loss_stamp_sec is None or self.pending_loss_monotonic is None:
            raise ControllerAbort("blind baseline has no first-loss timestamp")
        target_stamp = self.pending_loss_stamp_sec
        with self.sensor_lock:
            history = list(self.odom_history)
        before = None
        after = None
        for state in history:
            if state.stamp_sec <= target_stamp:
                before = state
            if state.stamp_sec >= target_stamp:
                after = state
                break
        if before is None:
            raise ControllerAbort(
                "odometry history does not bracket the first-loss timestamp"
            )
        if after is None:
            waited = time.monotonic() - self.pending_loss_monotonic
            if waited <= self.detection_odom_sync_tolerance_sec:
                return False
            raise ControllerAbort(
                "no odometry sample arrived after first loss within %.3fs"
                % self.detection_odom_sync_tolerance_sec
            )
        before_gap = target_stamp - before.stamp_sec
        after_gap = after.stamp_sec - target_stamp
        if (
            before_gap > self.detection_odom_sync_tolerance_sec
            or after_gap > self.detection_odom_sync_tolerance_sec
        ):
            raise ControllerAbort(
                "detection/odometry first-loss sync gap is too large: -%.3fs/+%.3fs"
                % (before_gap, after_gap)
            )

        try:
            ratio, baseline_x, baseline_y, baseline_yaw = interpolate_planar_pose(
                before.stamp_sec,
                before.x,
                before.y,
                before.yaw,
                after.stamp_sec,
                after.x,
                after.y,
                after.yaw,
                target_stamp,
            )
        except ValueError as exc:
            raise ControllerAbort("first-loss odometry interpolation failed: %s" % exc)
        baseline = OdomState(
            sequence=before.sequence,
            receipt_monotonic=time.monotonic(),
            stamp_sec=target_stamp,
            x=baseline_x,
            y=baseline_y,
            yaw=baseline_yaw,
            linear_x=before.linear_x + ratio * (after.linear_x - before.linear_x),
            angular_z=before.angular_z + ratio * (after.angular_z - before.angular_z),
            frame_id=before.frame_id,
            child_frame_id=before.child_frame_id,
        )
        self.blind_tracker = BlindDistanceTracker(
            baseline.x, baseline.y, baseline.yaw
        )
        self.blind_progress = self.blind_tracker.update(
            baseline.x, baseline.y, baseline.yaw, self.max_pose_step_m
        )
        self.blind_last_odom_state = baseline
        self.last_processed_odom_sequence = before.sequence
        with self.sensor_lock:
            self.odom_queue.clear()
            for state in self.odom_history:
                if state.sequence > before.sequence:
                    self.odom_queue.append(state)
        self.pending_loss_stamp_sec = None
        self.pending_loss_monotonic = None
        return True

    def _cancel_blind_measurement(self):
        with self.sensor_lock:
            odom = self.latest_odom
            self.odom_queue.clear()
        self.blind_tracker = None
        self.blind_progress = None
        self.blind_last_odom_state = None
        self.pending_loss_stamp_sec = None
        self.pending_loss_monotonic = None
        self.blind_started_monotonic = None
        self.blind_drive_started_monotonic = None
        self.blind_last_progress_monotonic = None
        self.blind_motion_seen = False
        self.blind_next_progress_m = self.blind_progress_step_m
        self.blind_max_forward_m = 0.0
        self.blind_seen_uncentered_wheel_sequence = 0
        if odom is not None:
            self.approach_last_odom = odom
            self.last_processed_odom_sequence = odom.sequence

    def _update_blind_odometry(self, speed_limit=None, track_progress=False):
        if not self._ensure_blind_measurement_baseline():
            return None
        now = time.monotonic()
        for state in self._take_odom_events():
            if abs(state.linear_x) > self.max_measured_speed_mps:
                raise ControllerAbort(
                    "post-loss measured speed %.3fm/s exceeds %.3fm/s"
                    % (state.linear_x, self.max_measured_speed_mps)
                )
            if state.linear_x < -0.03:
                raise ControllerAbort(
                    "post-loss odometry indicates reverse motion %.3fm/s"
                    % state.linear_x
                )
            if speed_limit is not None and state.linear_x > speed_limit:
                raise ControllerAbort(
                    "blind measured speed %.3fm/s exceeds %.3fm/s"
                    % (state.linear_x, speed_limit)
                )
            if self.blind_last_odom_state is None:
                raise ControllerAbort("blind odometry lost its previous source sample")
            source_dt = state.stamp_sec - self.blind_last_odom_state.stamp_sec
            raw_step = math.hypot(
                state.x - self.blind_last_odom_state.x,
                state.y - self.blind_last_odom_state.y,
            )
            if source_dt < -1e-9:
                raise ControllerAbort("blind odometry source time moved backwards")
            envelope_speed = (
                speed_limit
                if speed_limit is not None
                else min(MAX_COMMAND_SPEED_MPS + 0.03, self.max_measured_speed_mps)
            )
            dynamic_step_limit = min(
                self.max_pose_step_m,
                max(0.015, envelope_speed * max(0.0, source_dt) + 0.010),
            )
            if source_dt <= 1e-9 and raw_step > 0.002:
                raise ControllerAbort(
                    "blind odometry pose changed %.3fm without a newer source stamp"
                    % raw_step
                )
            try:
                progress = self.blind_tracker.update(
                    state.x, state.y, state.yaw, dynamic_step_limit
                )
            except ValueError as exc:
                raise ControllerAbort("blind odometry is invalid: %s" % exc)
            self.blind_last_odom_state = state
            self.blind_progress = progress
            if progress.path_m > self.blind_hard_distance_m:
                raise ControllerAbort(
                    "post-disappearance path exceeded the %.2fm hard limit"
                    % self.blind_hard_distance_m
                )
            if progress.forward_m < -0.02:
                raise ControllerAbort("post-disappearance odometry indicates reverse motion")
            if progress.forward_m + 0.02 < self.blind_max_forward_m:
                raise ControllerAbort(
                    "post-disappearance odometry moved backward from %.3fm to %.3fm"
                    % (self.blind_max_forward_m, progress.forward_m)
                )
            self.blind_max_forward_m = max(
                self.blind_max_forward_m, progress.forward_m
            )
            if abs(progress.lateral_m) > self.blind_max_lateral_deviation_m:
                raise ControllerAbort(
                    "post-disappearance lateral deviation %.3fm exceeds %.3fm"
                    % (
                        progress.lateral_m,
                        self.blind_max_lateral_deviation_m,
                    )
                )
            heading_change_deg = abs(math.degrees(progress.yaw_change_rad))
            if heading_change_deg > self.blind_max_heading_change_deg:
                raise ControllerAbort(
                    "post-disappearance heading changed %.2fdeg, limit %.2fdeg"
                    % (heading_change_deg, self.blind_max_heading_change_deg)
                )
            if track_progress and progress.forward_m >= self.blind_next_progress_m:
                self.blind_motion_seen = True
                self.blind_last_progress_monotonic = now
                while progress.forward_m >= self.blind_next_progress_m:
                    self.blind_next_progress_m += self.blind_progress_step_m
        return self.blind_progress

    def _update_preblind_odometry(self):
        progress = self._update_blind_odometry()
        if progress is None:
            return None
        if progress.path_m > self.preblind_max_displacement_m:
            raise ControllerAbort(
                "vehicle moved %.3fm while confirming loss/settling steering; limit %.3fm"
                % (progress.path_m, self.preblind_max_displacement_m)
            )
        return progress

    def _settle_tick(self):
        self._hard_stop()
        progress = self._update_preblind_odometry()
        now = time.monotonic()
        if self.settle_started_monotonic is None:
            self.settle_started_monotonic = now
        if now - self.settle_started_monotonic > self.settle_timeout_sec:
            raise ControllerAbort("front steering did not settle before blind advance")
        if progress is None:
            # A detection frame can arrive just ahead of the odometry sample
            # that brackets its source timestamp.  The vehicle is already
            # commanded to zero; remain fail-closed during the bounded sync
            # window instead of treating that normal publisher skew as a
            # missing-feedback fault.
            waiting_reason = "waiting for synchronized odometry after first target loss"
            if self.reason != waiting_reason:
                self.reason = waiting_reason
                self._publish_status(force=True)
            else:
                self.reason = waiting_reason
            return
        with self.sensor_lock:
            wheel_angle = self.latest_wheel_angle
            wheel_receipt = self.latest_wheel_angle_monotonic
            wheel_sequence = self.wheel_sequence
            latest_odom_sequence = self.odom_sequence
            last_unstopped_odom_sequence = self.last_unstopped_odom_sequence
            last_uncentered_wheel_sequence = self.last_uncentered_wheel_sequence
        # Use the last odometry sample that was actually consumed by the
        # post-loss distance tracker.  Reading latest_odom directly here could
        # certify a sample whose displacement has not yet passed the hard-limit
        # checks above.
        odom = self.blind_last_odom_state
        if wheel_angle is None or odom is None:
            raise ControllerAbort("steering settle lacks wheel-angle or odometry feedback")
        if wheel_receipt is None:
            raise ControllerAbort("steering settle lacks timestamped wheel feedback")

        centered = abs(math.degrees(wheel_angle)) <= self.settle_wheel_angle_deg
        stopped = abs(odom.linear_x) <= self.settle_speed_mps
        odom_window = advance_confirmation_window(
            self.settle_last_odom_sequence,
            self.settle_seen_unstopped_odom_sequence,
            self.settle_good_start_odom_stamp,
            odom.sequence,
            odom.stamp_sec,
            stopped,
            last_unstopped_odom_sequence,
        )
        wheel_window = advance_confirmation_window(
            self.settle_last_wheel_sequence,
            self.settle_seen_uncentered_wheel_sequence,
            self.settle_good_start_wheel_receipt,
            wheel_sequence,
            wheel_receipt,
            centered,
            last_uncentered_wheel_sequence,
        )
        self.settle_last_odom_sequence = odom_window.last_sequence
        self.settle_seen_unstopped_odom_sequence = (
            odom_window.seen_unsafe_sequence
        )
        self.settle_good_start_odom_stamp = odom_window.start_time
        self.settle_last_wheel_sequence = wheel_window.last_sequence
        self.settle_seen_uncentered_wheel_sequence = (
            wheel_window.seen_unsafe_sequence
        )
        self.settle_good_start_wheel_receipt = wheel_window.start_time
        new_odom = odom_window.new_sample
        new_wheel = wheel_window.new_sample

        odom_confirmed = (
            self.settle_good_start_odom_stamp is not None
            and odom.stamp_sec - self.settle_good_start_odom_stamp
            >= self.settle_confirm_sec
        )
        wheel_confirmed = (
            self.settle_good_start_wheel_receipt is not None
            and wheel_receipt - self.settle_good_start_wheel_receipt
            >= self.settle_confirm_sec
        )
        # At least one newly received sample must trigger the transition.  This
        # prevents a control timer from repeatedly reusing a cached stopped
        # sample until the wall-clock confirmation interval happens to elapse.
        odom_queue_drained = odom.sequence == latest_odom_sequence
        if (
            (new_odom or new_wheel)
            and odom_confirmed
            and wheel_confirmed
            and odom_queue_drained
        ):
            self.blind_drive_started_monotonic = now
            self.blind_last_progress_monotonic = now
            completed_steps = int(
                max(0.0, progress.forward_m) / self.blind_progress_step_m
            )
            self.blind_next_progress_m = (
                completed_steps + 1
            ) * self.blind_progress_step_m
            self.blind_motion_seen = progress.forward_m >= self.blind_progress_step_m
            self.blind_seen_uncentered_wheel_sequence = (
                last_uncentered_wheel_sequence
            )
            self._transition(
                BLIND_ADVANCE,
                "steering centered; completing 0.50m from first target loss",
            )
            return

        drift = progress.forward_m if progress is not None else 0.0
        self.reason = (
            "settling steering: wheel=%+.2fdeg speed=%+.3fm/s drift=%.3fm "
            "fresh_odom=%s fresh_wheel=%s odom_queue_drained=%s"
            % (
                math.degrees(wheel_angle),
                odom.linear_x,
                drift,
                new_odom,
                new_wheel,
                odom_queue_drained,
            )
        )

    def _blind_advance_tick(self):
        if (
            self.blind_tracker is None
            or self.blind_started_monotonic is None
            or self.blind_drive_started_monotonic is None
            or self.blind_last_progress_monotonic is None
        ):
            raise ControllerAbort("blind advance was entered without a live baseline")
        now = time.monotonic()
        if now - self.blind_started_monotonic > self.blind_timeout_sec:
            raise ControllerAbort("post-disappearance advance exceeded its time limit")
        with self.sensor_lock:
            last_uncentered_wheel_sequence = self.last_uncentered_wheel_sequence
        if (
            last_uncentered_wheel_sequence
            > self.blind_seen_uncentered_wheel_sequence
        ):
            raise ControllerAbort(
                "front wheel left the %.2fdeg straight-ahead gate during blind advance"
                % self.settle_wheel_angle_deg
            )

        progress = self._update_blind_odometry(
            speed_limit=self.blind_speed_mps + 0.08,
            track_progress=True,
        )
        if progress is None:
            raise ControllerAbort("blind advance lost its synchronized odometry baseline")
        if blind_goal_reached(progress, self.blind_distance_m):
            self._hard_stop()
            self.phase = FINAL_STOP
            self.reason = (
                "0.50m reached; confirming full stop while enforcing 0.55m hard limit"
            )
            self.final_stop_started_monotonic = now
            with self.sensor_lock:
                self.final_stop_last_odom_sequence = self.odom_sequence
                self.final_stop_last_wheel_sequence = self.wheel_sequence
                self.final_stop_seen_unstopped_odom_sequence = (
                    self.last_final_unstopped_odom_sequence
                )
                self.final_stop_seen_uncentered_wheel_sequence = (
                    self.last_uncentered_wheel_sequence
                )
            self.final_stop_good_start_odom_stamp = None
            self.final_stop_good_start_wheel_receipt = None
            self.final_stop_good_start_path_m = None
            rospy.logwarn("FOD visual servo state -> FINAL_STOP: %s", self.reason)
            return

        drive_elapsed = now - self.blind_drive_started_monotonic
        if not self.blind_motion_seen and drive_elapsed > self.blind_no_motion_timeout_sec:
            raise ControllerAbort("vehicle made no odometry progress during blind advance")
        if (
            self.blind_motion_seen
            and now - self.blind_last_progress_monotonic
            > self.blind_no_progress_timeout_sec
        ):
            raise ControllerAbort("blind odometry progress watchdog expired")

        self.reason = "post-loss net forward %.3f/%.3fm (path %.3fm)" % (
            progress.forward_m,
            self.blind_distance_m,
            progress.path_m,
        )
        self._set_motion(self.blind_speed_mps, 0.0)

    def _final_stop_tick(self):
        self._hard_stop()
        progress = self._update_blind_odometry()
        if progress is None:
            raise ControllerAbort("final stop lost its synchronized odometry baseline")
        now = time.monotonic()
        if self.final_stop_started_monotonic is None:
            self.final_stop_started_monotonic = now
        if now - self.final_stop_started_monotonic > self.final_stop_timeout_sec:
            raise ControllerAbort("vehicle did not stop after reaching the blind distance")
        if progress.forward_m < self.blind_distance_m - 0.005:
            raise ControllerAbort("vehicle rolled backward after reaching blind distance")
        with self.sensor_lock:
            wheel_angle = self.latest_wheel_angle
            wheel_receipt = self.latest_wheel_angle_monotonic
            wheel_sequence = self.wheel_sequence
            latest_odom_sequence = self.odom_sequence
            last_unstopped_odom_sequence = self.last_final_unstopped_odom_sequence
            last_uncentered_wheel_sequence = self.last_uncentered_wheel_sequence
        odom = self.blind_last_odom_state
        if wheel_angle is None or odom is None:
            raise ControllerAbort("final stop lacks wheel-angle or odometry feedback")
        if wheel_receipt is None:
            raise ControllerAbort("final stop lacks timestamped wheel feedback")

        centered = abs(math.degrees(wheel_angle)) <= self.settle_wheel_angle_deg
        stopped = abs(odom.linear_x) <= self.final_stop_speed_mps
        previous_odom_start = self.final_stop_good_start_odom_stamp
        odom_window = advance_confirmation_window(
            self.final_stop_last_odom_sequence,
            self.final_stop_seen_unstopped_odom_sequence,
            self.final_stop_good_start_odom_stamp,
            odom.sequence,
            odom.stamp_sec,
            stopped,
            last_unstopped_odom_sequence,
        )
        wheel_window = advance_confirmation_window(
            self.final_stop_last_wheel_sequence,
            self.final_stop_seen_uncentered_wheel_sequence,
            self.final_stop_good_start_wheel_receipt,
            wheel_sequence,
            wheel_receipt,
            centered,
            last_uncentered_wheel_sequence,
        )
        self.final_stop_last_odom_sequence = odom_window.last_sequence
        self.final_stop_seen_unstopped_odom_sequence = (
            odom_window.seen_unsafe_sequence
        )
        self.final_stop_good_start_odom_stamp = odom_window.start_time
        if self.final_stop_good_start_odom_stamp is None:
            self.final_stop_good_start_path_m = None
        elif self.final_stop_good_start_odom_stamp != previous_odom_start:
            self.final_stop_good_start_path_m = progress.path_m
        self.final_stop_last_wheel_sequence = wheel_window.last_sequence
        self.final_stop_seen_uncentered_wheel_sequence = (
            wheel_window.seen_unsafe_sequence
        )
        self.final_stop_good_start_wheel_receipt = wheel_window.start_time
        new_odom = odom_window.new_sample
        new_wheel = wheel_window.new_sample

        final_stop_drift_m = (
            progress.path_m - self.final_stop_good_start_path_m
            if self.final_stop_good_start_path_m is not None
            else 0.0
        )
        if final_stop_drift_m > self.final_stop_max_drift_m:
            # Even a velocity estimate just below the threshold must not be
            # called stopped while pose continues to creep.  Restart the
            # confirmation window at the newest fully checked odom sample.
            self.final_stop_good_start_odom_stamp = odom.stamp_sec
            self.final_stop_good_start_path_m = progress.path_m
            final_stop_drift_m = 0.0

        odom_confirmed = (
            self.final_stop_good_start_odom_stamp is not None
            and odom.stamp_sec - self.final_stop_good_start_odom_stamp
            >= self.final_stop_confirm_sec
        )
        wheel_confirmed = (
            self.final_stop_good_start_wheel_receipt is not None
            and wheel_receipt - self.final_stop_good_start_wheel_receipt
            >= self.final_stop_confirm_sec
        )
        odom_queue_drained = odom.sequence == latest_odom_sequence
        if (
            (new_odom or new_wheel)
            and odom_confirmed
            and wheel_confirmed
            and odom_queue_drained
        ):
            expected_fence = TerminalSensorFence(
                odom_sequence=odom.sequence,
                wheel_sequence=wheel_sequence,
                detection_sequence=self.last_processed_detection_sequence,
                invalid_camera_generation=self.session_invalid_camera_generation,
                invalid_odom_generation=self.session_invalid_odom_generation,
                invalid_wheel_generation=self.session_invalid_wheel_generation,
                chassis_fault_generation=self.session_chassis_fault_generation,
                raw_can_fault_generation=self.session_raw_can_fault_generation,
                m2_bypass_event_generation=(
                    self.session_m2_bypass_event_generation
                ),
                control_timeout_seen=False,
                detection_queue_size=0,
                odom_queue_size=0,
                detection_queue_overflow=False,
                odom_queue_overflow=False,
            )
            # COMPLETE is terminal and subsequent ticks intentionally stop
            # processing feedback.  Recheck every relevant callback sequence,
            # sticky fault generation, and queue under the callback lock, then
            # commit the phase under that same lock.  A callback either lands
            # before this boundary and forces another FINAL_STOP tick, or lands
            # after the terminal outcome; it can never be silently skipped in
            # the TOCTOU gap between a stale snapshot and COMPLETE.
            with self.sensor_lock:
                current_fence = self._terminal_sensor_fence_locked()
                if terminal_sensor_fence_unchanged(
                    expected_fence, current_fence
                ):
                    self._check_terminal_commit_health_locked()
                    self.phase = COMPLETE
                    self.reason = (
                        "FOD recovery complete and stopped: net forward %.3fm, "
                        "path %.3fm" % (progress.forward_m, progress.path_m)
                    )
                    self.target_visible = False
                    completed = True
                else:
                    completed = False
            if completed:
                rospy.logwarn("FOD visual servo COMPLETE: %s", self.reason)
                return
            self.reason = (
                "final stop received new feedback at the completion boundary; "
                "verifying it before COMPLETE"
            )
            return
        self.reason = (
            "final stop: speed=%+.3fm/s wheel=%+.2fdeg forward=%.3fm path=%.3fm "
            "stop_drift=%.4fm fresh_odom=%s fresh_wheel=%s odom_queue_drained=%s"
            % (
                odom.linear_x,
                math.degrees(wheel_angle),
                progress.forward_m,
                progress.path_m,
                final_stop_drift_m,
                new_odom,
                new_wheel,
                odom_queue_drained,
            )
        )

    def _status_dictionary(self):
        now = time.monotonic()
        with self.command_lock:
            command_speed = self.motion_lease.linear_x
            command_curvature = self.motion_lease.curvature
            command_expired_reason = self.motion_lease.expired_reason
        with self.sensor_lock:
            detection = self.latest_detection
            camera = self.latest_camera
            odom = self.latest_odom
            wheel = self.latest_wheel_angle
            wheel_receipt = self.latest_wheel_angle_monotonic
            chassis_receipt = self.latest_chassis_status_monotonic
            raw_status = dict(self.raw_can_status)
        target = self.machine.locked
        blind_distance = self.blind_progress.path_m if self.blind_progress else 0.0
        blind_forward = self.blind_progress.forward_m if self.blind_progress else 0.0
        raw_can_ages = {
            RAW_OBSERVED_TYPES.get(msg_type, "0x%02X" % msg_type): round(
                max(0.0, now - value[0]), 3
            )
            for msg_type, value in raw_status.items()
        }
        return {
            "state": self.phase,
            "reason": self.reason,
            "allow_motion": self.allow_motion,
            "active": self.phase
            not in (DISABLED, PRECHECK, COMPLETE, ABORT),
            "completed": self.phase == COMPLETE,
            "target_visible": self.target_visible,
            "target_locked": target is not None,
            "target_class": target.class_name if target is not None else "",
            "target_confidence": round(target.confidence, 4) if target is not None else None,
            "target_anchor_u_px": (
                round(self.machine.filtered_u, 3)
                if self.machine.filtered_u is not None
                else None
            ),
            "target_anchor_v_px": (
                round(self.machine.filtered_v, 3)
                if self.machine.filtered_v is not None
                else None
            ),
            "target_u_px": round(self.target_u_px, 3) if self.target_u_px is not None else None,
            "horizontal_error": (
                round(self.horizontal_error_value, 5)
                if self.horizontal_error_value is not None
                else None
            ),
            "vertical_fraction": (
                round(self.vertical_fraction_value, 5)
                if self.vertical_fraction_value is not None
                else None
            ),
            "approach_path_m": round(self.approach_path_m, 4),
            "blind_distance_m": round(blind_distance, 4),
            "blind_forward_m": round(blind_forward, 4),
            "blind_target_m": self.blind_distance_m,
            "command_linear_x_mps": round(command_speed, 4),
            "command_curvature_per_m": round(command_curvature, 5),
            "command_angular_z_radps": round(command_speed * command_curvature, 5),
            "command_expired_reason": command_expired_reason,
            "wheelbase_m": round(self.wheelbase_m, 4) if self.wheelbase_m else None,
            "detection_age_sec": (
                round(max(0.0, now - detection.receipt_monotonic), 3)
                if detection is not None
                else None
            ),
            "camera_info_age_sec": (
                round(max(0.0, now - camera.receipt_monotonic), 3)
                if camera is not None
                else None
            ),
            "odom_age_sec": (
                round(max(0.0, now - odom.receipt_monotonic), 3)
                if odom is not None
                else None
            ),
            "wheel_angle_age_sec": (
                round(max(0.0, now - wheel_receipt), 3)
                if wheel_receipt is not None
                else None
            ),
            "wheel_angle_deg": round(math.degrees(wheel), 3) if wheel is not None else None,
            "chassis_status_age_sec": (
                round(max(0.0, now - chassis_receipt), 3)
                if chassis_receipt is not None
                else None
            ),
            "raw_can_age_sec": raw_can_ages,
        }

    def _publish_status(self, force=False):
        now = time.monotonic()
        if (
            not force
            and now - self.last_status_publish_monotonic < 1.0 / self.status_rate_hz
        ):
            return
        status_dict = self._status_dictionary()
        self.state_pub.publish(String(data=self.phase))
        self.status_pub.publish(
            String(data=json.dumps(status_dict, ensure_ascii=False, sort_keys=True))
        )
        self.completed_pub.publish(Bool(data=self.phase == COMPLETE))

        diagnostic = DiagnosticStatus()
        diagnostic.name = "fod_visual_servo/control"
        diagnostic.hardware_id = "autolabor_m2"
        if self.phase == ABORT:
            diagnostic.level = DiagnosticStatus.ERROR
        elif self.phase in (
            PRECHECK,
            ACQUIRE,
            REACQUIRE,
            LOSS_CONFIRM,
            STEER_SETTLE,
            FINAL_STOP,
        ):
            diagnostic.level = DiagnosticStatus.WARN
        else:
            diagnostic.level = DiagnosticStatus.OK
        diagnostic.message = "%s: %s" % (self.phase, self.reason)
        diagnostic.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in status_dict.items()
            if key != "raw_can_age_sec"
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [diagnostic]
        self.diagnostics_pub.publish(array)
        self.last_status_publish_monotonic = now

    def _stop_burst(self):
        self._hard_stop()
        deadline = time.monotonic() + self.stop_publish_sec
        period = 1.0 / self.publish_rate_hz
        while time.monotonic() < deadline:
            try:
                with self.command_lock:
                    self.cmd_pub.publish(Twist())
            except Exception:
                pass
            time.sleep(period)

    def _shutdown(self):
        if self.shutdown_started:
            return
        self.shutdown_started = True
        self.operator_disable_requested.set()
        self._stop_burst()


def main():
    rospy.init_node("fod_visual_servo", anonymous=False)
    try:
        FodVisualServoNode()
    except Exception as exc:
        rospy.logfatal("failed to initialize FOD visual servo: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
