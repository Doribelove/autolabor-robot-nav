#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pixel visual servo for recovering one detected FOD.

This node is intentionally not auto-started by the perception launch. In
standalone mode it publishes ``/cmd_vel``; the integrated GPS/FOD launch routes
its configured output through the sole chassis-command arbiter. Non-zero motion
also requires both ``~allow_motion:=true`` and an explicit SetBool(true) call.
Its normal mode is fail-closed on CAN/VCU safety feedback; an explicit
``~external_estop_override:=true`` delegates those checks to an attended
external emergency-stop operator while retaining the visual, odometry, wheel,
command-ownership, lease, distance, and absolute-time guards.
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
import tf2_ros
from autolabor_canbus_driver.msg import CanBusMessage, ChassisStatusInfo
from autolabor_canbus_driver.srv import ChassisParameterServer
from autolabor_fod_control.cfg import VisualServoConfig
from autolabor_fod_msgs.msg import FodDetectionArray, FodVisionDetectionArray
from autolabor_fod_msgs.srv import (
    SetFodCleaningTarget,
    SetFodCleaningTargetResponse,
)
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Empty, Float64, String
from std_srvs.srv import SetBool, SetBoolResponse

from autolabor_fod_control.pending_litter import map_target_to_wheel_odom
from autolabor_fod_control.visual_session import VisualSessionReceipt

from autolabor_fod_control.visual_servo import (
    ACQUIRE,
    APPROACH,
    EDGE_ARMED,
    LOSS_CONFIRM,
    REACQUIRE,
    STEER_SETTLE,
    AssociationConfig,
    BlindDistanceTracker,
    ForwardCoverageTracker,
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
    depth_rejection_reason,
    matching_detections,
    nearest_depth_target,
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
MAX_CLEANING_COVERAGE_TOLERANCE_M = 0.10
MAX_STEERING_ANGLE_DEG = 12.0
MIN_LOCK_CONFIDENCE = 0.25
MAX_LOCK_CONFIDENCE = 0.95

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
    depth_rejected: Tuple[PixelDetection, ...] = tuple()
    depth_rejections: Tuple[str, ...] = tuple()
    metric_targets: Tuple["MetricTarget", ...] = tuple()
    metric_error: str = ""
    error: str = ""


@dataclass(frozen=True)
class MetricTarget:
    object_id: int
    track_id: int
    class_name: str
    confidence: float
    x: float
    y: float
    z: float
    world_frame: str
    bbox_x: float
    bbox_y: float
    bbox_width: float
    bbox_height: float
    state: str


@dataclass(frozen=True)
class MetricFrame:
    sequence: int
    receipt_monotonic: float
    stamp_sec: float
    width: int
    height: int
    targets: Tuple[MetricTarget, ...]
    error: str = ""


@dataclass(frozen=True)
class StagedTarget:
    object_id: int
    x: float
    y: float
    frame: str
    observed_at_sec: float
    label: str
    expires_monotonic: float
    completion_policy: str = ""
    request_id: str = ""


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
        self.metric_results_topic = rospy.get_param(
            "~metric_results_topic", "/fod/vision/results"
        )
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
        self.expected_detector_node = rospy.get_param(
            "~expected_detector_node", "/fod_detector"
        )
        self.expected_camera_node = rospy.get_param(
            "~expected_camera_node", "/fod_camera/driver"
        )
        self.expected_driver_node = rospy.get_param(
            "~expected_driver_node", "/m2_driver"
        )
        self.expected_cmd_vel_subscriber_node = rospy.get_param(
            "~expected_cmd_vel_subscriber_node", self.expected_driver_node
        )
        self.expected_canbus_node = rospy.get_param(
            "~expected_canbus_node", "/canbus_driver"
        )
        self.expected_image_width = int(rospy.get_param("~expected_image_width", 640))
        self.expected_image_height = int(
            rospy.get_param("~expected_image_height", 360)
        )
        self.expected_camera_frame = rospy.get_param(
            "~expected_camera_frame", "zed2_left_camera_optical_frame"
        )
        self.expected_odom_frame = rospy.get_param("~expected_odom_frame", "odom")
        self.expected_base_frame = rospy.get_param("~expected_base_frame", "base_link")
        self.expected_model_sha256 = str(
            rospy.get_param("~expected_model_sha256", "")
        ).strip().lower()
        self.expected_metric_backend = str(
            rospy.get_param("~expected_metric_backend", "detect_and_classify")
        ).strip()
        self.expected_metric_frame = str(
            rospy.get_param("~expected_metric_frame", "map")
        ).strip()

        self.allowed_class_names = parse_class_names(
            rospy.get_param(
                "~allowed_class_names",
                ["Metal", "Soft", "Plastic", "Wire", "Tool", "w"],
            )
        )
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.30))
        self.require_depth_for_acquisition = strict_bool_param(
            "~require_depth_for_acquisition", True
        )
        self.min_target_depth_m = float(
            rospy.get_param("~min_target_depth_m", 0.35)
        )
        self.max_target_depth_m = float(
            rospy.get_param("~max_target_depth_m", 8.0)
        )
        self.min_depth_sample_count = int(
            rospy.get_param("~min_depth_sample_count", 20)
        )
        self.min_depth_valid_fraction = float(
            rospy.get_param("~min_depth_valid_fraction", 0.20)
        )
        self.max_depth_mad_m = float(
            rospy.get_param("~max_depth_mad_m", 0.35)
        )
        self.max_depth_sync_delta_sec = float(
            rospy.get_param("~max_depth_sync_delta_sec", 0.08)
        )
        self.nearest_depth_hysteresis_m = float(
            rospy.get_param("~nearest_depth_hysteresis_m", 0.10)
        )
        self.locked_depth_invalid_abort_frames = int(
            rospy.get_param("~locked_depth_invalid_abort_frames", 5)
        )
        self.allow_motion = strict_bool_param("~allow_motion", False)
        self.external_estop_override = strict_bool_param(
            "~external_estop_override", False
        )
        # Steering-center changes, odometry resets, and brake commands can
        # invalidate the controller's geometry/progress accounting, so they
        # remain forbidden in both modes.  The VCU emergency command topic is
        # part of the explicitly delegated CAN/VCU safety chain and is omitted
        # only when the external-estop override is requested.
        self.m2_bypass_topics = (
            self.steer_center_bias_topic,
            self.reset_odom_topic,
            self.brake_set_topic,
        )
        if not self.external_estop_override:
            self.m2_bypass_topics += (self.emergency_stop_topic,)
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
            rospy.get_param("~detection_timeout_sec", 0.50)
        )
        self.metric_results_timeout_sec = float(
            rospy.get_param("~metric_results_timeout_sec", 0.60)
        )
        self.metric_sync_tolerance_sec = float(
            rospy.get_param("~metric_sync_tolerance_sec", 0.002)
        )
        self.metric_bbox_min_iou = float(
            rospy.get_param("~metric_bbox_min_iou", 0.90)
        )
        self.metric_max_position_jump_m = float(
            rospy.get_param("~metric_max_position_jump_m", 0.10)
        )
        self.metric_max_selected_position_delta_m = float(
            rospy.get_param("~metric_max_selected_position_delta_m", 0.25)
        )
        self.map_tf_timeout_sec = float(
            rospy.get_param("~map_tf_timeout_sec", 0.60)
        )
        self.staged_target_timeout_sec = float(
            rospy.get_param("~staged_target_timeout_sec", 30.0)
        )
        self.camera_info_timeout_sec = float(
            rospy.get_param("~camera_info_timeout_sec", 0.60)
        )
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 0.60))
        self.source_stamp_timeout_sec = float(
            rospy.get_param("~source_stamp_timeout_sec", 0.50)
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
        self.raw_can_timeout_sec = float(rospy.get_param("~raw_can_timeout_sec", 8.0))
        self.graph_check_interval_sec = float(
            rospy.get_param("~graph_check_interval_sec", 0.50)
        )

        self.min_acquire_anchor_v_fraction = float(
            rospy.get_param("~min_acquire_anchor_v_fraction", 0.03)
        )
        self.max_acquire_anchor_v_fraction = float(
            rospy.get_param("~max_acquire_anchor_v_fraction", 0.97)
        )
        self.acquire_max_abs_horizontal_error = float(
            rospy.get_param("~acquire_max_abs_horizontal_error", 0.65)
        )
        association_config = AssociationConfig(
            min_iou=float(rospy.get_param("~association_min_iou", 0.05)),
            max_anchor_distance_ratio=float(
                rospy.get_param("~association_max_anchor_distance_ratio", 0.18)
            ),
            duplicate_min_iou=float(
                rospy.get_param("~duplicate_detection_min_iou", 0.80)
            ),
            duplicate_max_anchor_distance_ratio=float(
                rospy.get_param(
                    "~duplicate_detection_max_anchor_distance_ratio", 0.02
                )
            ),
            duplicate_max_depth_delta_m=float(
                rospy.get_param("~duplicate_detection_max_depth_delta_m", 0.15)
            ),
        )
        machine_config = TargetMachineConfig(
            acquire_frames=int(rospy.get_param("~acquire_frames", 6)),
            loss_confirm_frames=int(rospy.get_param("~loss_confirm_frames", 10)),
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
        self.far_speed_mps = float(rospy.get_param("~far_speed_mps", 0.20))
        self.near_speed_mps = float(rospy.get_param("~near_speed_mps", 0.20))
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
        self.approach_min_command_speed_mps = float(
            rospy.get_param("~approach_min_command_speed_mps", 0.20)
        )
        self.max_linear_acceleration_mps2 = float(
            rospy.get_param("~max_linear_acceleration_mps2", 4.00)
        )
        self.max_curvature_rate_per_sec = float(
            rospy.get_param("~max_curvature_rate_per_sec", 0.80)
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
        self.cleaning_reference_offset_m = float(
            rospy.get_param("~cleaning_reference_offset_m", 0.52)
        )
        self.cleaning_coverage_tolerance_m = float(
            rospy.get_param("~cleaning_coverage_tolerance_m", 0.10)
        )
        self.cleaning_min_forward_step_m = float(
            rospy.get_param("~cleaning_min_forward_step_m", 0.001)
        )
        self.cleaning_min_forward_speed_mps = float(
            rospy.get_param("~cleaning_min_forward_speed_mps", 0.02)
        )

        self.settle_wheel_angle_deg = float(
            rospy.get_param("~settle_wheel_angle_deg", 2.0)
        )
        self.settle_speed_mps = float(rospy.get_param("~settle_speed_mps", 0.03))
        self.blind_speed_mps = float(rospy.get_param("~blind_speed_mps", 0.20))
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
        self.final_stop_speed_mps = float(
            rospy.get_param("~final_stop_speed_mps", 0.01)
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
        self.latest_metric = None
        self.metric_frames = deque(maxlen=100)
        self.metric_sequence = 0
        self.last_metric_stamp = None
        self.invalid_metric_generation = 0
        self.session_invalid_metric_generation = 0
        self.session_metric_floor = 0
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
        self.locked_depth_invalid_frames = 0
        self.pending_metric_object_id = None
        self.locked_metric_target = None
        self.metric_target_distance_m = None
        self.metric_target_bearing_deg = None
        self.staged_target = None
        self.coverage_tracker = None
        self.coverage_progress = None
        self.coverage_last_odom_sequence = 0
        self.last_cleaning_completion = {}
        self.visual_session = VisualSessionReceipt()
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
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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
        self.metric_sub = rospy.Subscriber(
            self.metric_results_topic,
            FodVisionDetectionArray,
            self._metric_results_cb,
            queue_size=20,
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
        self.chassis_sub = None
        self.control_timeout_sub = None
        self.canbus_sub = None
        if not self.external_estop_override:
            self.chassis_sub = rospy.Subscriber(
                self.chassis_status_topic,
                ChassisStatusInfo,
                self._chassis_status_cb,
                queue_size=20,
            )
            self.control_timeout_sub = rospy.Subscriber(
                self.control_timeout_topic,
                Bool,
                self._control_timeout_cb,
                queue_size=20,
            )
            self.canbus_sub = rospy.Subscriber(
                self.canbus_topic,
                CanBusMessage,
                self._raw_canbus_cb,
                queue_size=100,
            )

        m2_bypass_subscribers = [
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
        ]
        if not self.external_estop_override:
            m2_bypass_subscribers.append(
                rospy.Subscriber(
                    self.emergency_stop_topic,
                    Bool,
                    self._m2_bypass_control_cb,
                    callback_args=self.emergency_stop_topic,
                    queue_size=20,
                )
            )
        self.m2_bypass_subscribers = tuple(m2_bypass_subscribers)
        self.chassis_parameter_proxy = rospy.ServiceProxy(
            self.chassis_parameter_service, ChassisParameterServer
        )
        self.enable_service = rospy.Service(
            "~set_enabled", SetBool, self._set_enabled_cb
        )
        self.cleaning_target_service = rospy.Service(
            "~set_cleaning_target",
            SetFodCleaningTarget,
            self._set_cleaning_target_cb,
        )
        self.dynamic_reconfigure_server = DynamicReconfigureServer(
            VisualServoConfig, self._dynamic_reconfigure_cb
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
            "FOD visual servo loaded with allow_motion=%s, "
            "external_estop_override=%s. It publishes zero on %s while disabled; "
            "non-zero motion also requires %s/set_enabled true.",
            self.allow_motion,
            self.external_estop_override,
            self.cmd_vel_topic,
            rospy.get_name(),
        )
        if self.external_estop_override:
            rospy.logwarn(
                "EXTERNAL ESTOP OVERRIDE ACTIVE: raw CAN observation, aggregated "
                "chassis emergency, VCU command-timeout, CAN publisher graph, and ROS "
                "emergency-stop-topic gates are disabled. Keep the attended "
                "remote emergency stop immediately operable throughout the run."
            )

    def _dynamic_reconfigure_cb(self, config, _level):
        requested = float(config["min_confidence"])
        if not math.isfinite(requested):
            config["min_confidence"] = self.min_confidence
            rospy.logwarn("Rejected non-finite visual lock confidence")
            return config
        requested = max(MIN_LOCK_CONFIDENCE, min(MAX_LOCK_CONFIDENCE, requested))
        with self.run_lock:
            if (
                self.phase not in (DISABLED, COMPLETE, ABORT)
                and not math.isclose(requested, self.min_confidence, abs_tol=1e-9)
            ):
                config["min_confidence"] = self.min_confidence
                rospy.logwarn(
                    "Visual lock confidence can only change while stopped; "
                    "controller state is %s",
                    self.phase,
                )
                return config
            previous = self.min_confidence
            self.min_confidence = requested
            config["min_confidence"] = requested
            if not math.isclose(previous, requested, abs_tol=1e-9):
                rospy.loginfo(
                    "Visual target lock confidence changed from %.3f to %.3f",
                    previous,
                    requested,
                )
                self._publish_status(force=True)
        return config

    def _validate_parameters(self):
        numeric = {
            "min_confidence": self.min_confidence,
            "min_target_depth_m": self.min_target_depth_m,
            "max_target_depth_m": self.max_target_depth_m,
            "min_depth_valid_fraction": self.min_depth_valid_fraction,
            "max_depth_mad_m": self.max_depth_mad_m,
            "max_depth_sync_delta_sec": self.max_depth_sync_delta_sec,
            "nearest_depth_hysteresis_m": self.nearest_depth_hysteresis_m,
            "target_u_offset_px": self.target_u_offset_px,
            "precheck_timeout_sec": self.precheck_timeout_sec,
            "mode_timeout_sec": self.mode_timeout_sec,
            "publish_rate_hz": self.publish_rate_hz,
            "control_rate_hz": self.control_rate_hz,
            "status_rate_hz": self.status_rate_hz,
            "command_lease_sec": self.command_lease_sec,
            "stop_publish_sec": self.stop_publish_sec,
            "detection_timeout_sec": self.detection_timeout_sec,
            "metric_results_timeout_sec": self.metric_results_timeout_sec,
            "metric_sync_tolerance_sec": self.metric_sync_tolerance_sec,
            "metric_bbox_min_iou": self.metric_bbox_min_iou,
            "metric_max_position_jump_m": self.metric_max_position_jump_m,
            "metric_max_selected_position_delta_m": (
                self.metric_max_selected_position_delta_m
            ),
            "map_tf_timeout_sec": self.map_tf_timeout_sec,
            "staged_target_timeout_sec": self.staged_target_timeout_sec,
            "camera_info_timeout_sec": self.camera_info_timeout_sec,
            "odom_timeout_sec": self.odom_timeout_sec,
            "source_stamp_timeout_sec": self.source_stamp_timeout_sec,
            "detection_odom_sync_tolerance_sec": (
                self.detection_odom_sync_tolerance_sec
            ),
            "wheel_angle_timeout_sec": self.wheel_angle_timeout_sec,
            "chassis_status_timeout_sec": self.chassis_status_timeout_sec,
            "raw_can_timeout_sec": self.raw_can_timeout_sec,
            "graph_check_interval_sec": self.graph_check_interval_sec,
            "min_acquire_anchor_v_fraction": self.min_acquire_anchor_v_fraction,
            "max_acquire_anchor_v_fraction": self.max_acquire_anchor_v_fraction,
            "acquire_max_abs_horizontal_error": self.acquire_max_abs_horizontal_error,
            "association_min_iou": self.association_config.min_iou,
            "association_max_anchor_distance_ratio": (
                self.association_config.max_anchor_distance_ratio
            ),
            "duplicate_detection_min_iou": (
                self.association_config.duplicate_min_iou
            ),
            "duplicate_detection_max_anchor_distance_ratio": (
                self.association_config.duplicate_max_anchor_distance_ratio
            ),
            "duplicate_detection_max_depth_delta_m": (
                self.association_config.duplicate_max_depth_delta_m
            ),
            "pixel_filter_alpha": self.machine_config.filter_alpha,
            "steering_sign": self.steering_sign,
            "curvature_gain": self.curvature_gain,
            "horizontal_deadband": self.horizontal_deadband,
            "max_steering_angle_deg": self.max_steering_angle_deg,
            "far_speed_mps": self.far_speed_mps,
            "near_speed_mps": self.near_speed_mps,
            "slow_start_fraction": self.slow_start_fraction,
            "near_start_fraction": self.near_start_fraction,
            "lateral_slowdown_error": self.lateral_slowdown_error,
            "minimum_lateral_speed_scale": self.minimum_lateral_speed_scale,
            "approach_min_command_speed_mps": self.approach_min_command_speed_mps,
            "max_linear_acceleration_mps2": self.max_linear_acceleration_mps2,
            "max_curvature_rate_per_sec": self.max_curvature_rate_per_sec,
            "approach_no_motion_timeout_sec": self.approach_no_motion_timeout_sec,
            "approach_no_progress_timeout_sec": self.approach_no_progress_timeout_sec,
            "approach_progress_step_m": self.approach_progress_step_m,
            "max_measured_speed_mps": self.max_measured_speed_mps,
            "max_pose_step_m": self.max_pose_step_m,
            "cleaning_reference_offset_m": self.cleaning_reference_offset_m,
            "cleaning_coverage_tolerance_m": self.cleaning_coverage_tolerance_m,
            "cleaning_min_forward_step_m": self.cleaning_min_forward_step_m,
            "cleaning_min_forward_speed_mps": self.cleaning_min_forward_speed_mps,
            "settle_wheel_angle_deg": self.settle_wheel_angle_deg,
            "settle_speed_mps": self.settle_speed_mps,
            "blind_speed_mps": self.blind_speed_mps,
            "blind_distance_m": self.blind_distance_m,
            "blind_hard_distance_m": self.blind_hard_distance_m,
            "blind_timeout_sec": self.blind_timeout_sec,
            "blind_no_motion_timeout_sec": self.blind_no_motion_timeout_sec,
            "blind_no_progress_timeout_sec": self.blind_no_progress_timeout_sec,
            "blind_progress_step_m": self.blind_progress_step_m,
            "final_stop_speed_mps": self.final_stop_speed_mps,
        }
        for name, value in numeric.items():
            if not math.isfinite(value):
                raise ValueError("%s must be finite" % name)

        required_absolute_topics = {
            "detections_topic": self.detections_topic,
            "metric_results_topic": self.metric_results_topic,
            "camera_info_topic": self.camera_info_topic,
            "odom_topic": self.odom_topic,
            "cmd_vel_topic": self.cmd_vel_topic,
            "ackermann_topic": self.ackermann_topic,
            "wheel_angle_topic": self.wheel_angle_topic,
            "chassis_status_topic": self.chassis_status_topic,
            "control_timeout_topic": self.control_timeout_topic,
            "canbus_topic": self.canbus_topic,
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
            "expected_cmd_vel_subscriber_node": (
                self.expected_cmd_vel_subscriber_node
            ),
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
        if self.expected_metric_backend != "detect_and_classify":
            raise ValueError("metric backend must be detect_and_classify")
        if self.expected_metric_frame != "map":
            raise ValueError("metric target frame must be map")
        if not MIN_LOCK_CONFIDENCE <= self.min_confidence <= MAX_LOCK_CONFIDENCE:
            raise ValueError("min_confidence must be between 0.25 and 0.95")
        if not 0.30 <= self.min_target_depth_m < self.max_target_depth_m:
            raise ValueError("target depth range is invalid")
        if not self.max_target_depth_m <= 15.0:
            raise ValueError("max_target_depth_m exceeds the configured ZED range")
        if not 10 <= self.min_depth_sample_count <= 10000:
            raise ValueError("min_depth_sample_count is invalid")
        if not 0.05 <= self.min_depth_valid_fraction <= 1.0:
            raise ValueError("min_depth_valid_fraction is invalid")
        if not 0.0 <= self.max_depth_mad_m <= 1.0:
            raise ValueError("max_depth_mad_m is invalid")
        if not 0.0 < self.max_depth_sync_delta_sec <= 0.20:
            raise ValueError("max_depth_sync_delta_sec is invalid")
        if not 0.0 <= self.nearest_depth_hysteresis_m <= 0.50:
            raise ValueError("nearest_depth_hysteresis_m is invalid")
        if not 1 <= self.locked_depth_invalid_abort_frames <= 30:
            raise ValueError(
                "locked_depth_invalid_abort_frames must be between 1 and 30"
            )
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
            ("metric_results_timeout_sec", self.metric_results_timeout_sec, 0.20, 1.00),
            ("metric_sync_tolerance_sec", self.metric_sync_tolerance_sec, 0.0001, 0.01),
            ("metric_bbox_min_iou", self.metric_bbox_min_iou, 0.70, 1.00),
            ("metric_max_position_jump_m", self.metric_max_position_jump_m, 0.03, 0.30),
            (
                "metric_max_selected_position_delta_m",
                self.metric_max_selected_position_delta_m,
                0.05,
                0.50,
            ),
            ("map_tf_timeout_sec", self.map_tf_timeout_sec, 0.20, 1.50),
            ("staged_target_timeout_sec", self.staged_target_timeout_sec, 5.0, 60.0),
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
            ("raw_can_timeout_sec", self.raw_can_timeout_sec, 1.00, 10.00),
            ("graph_check_interval_sec", self.graph_check_interval_sec, 0.20, 1.00),
        ):
            if not low <= value <= high:
                raise ValueError("%s must be between %.2f and %.2f" % (name, low, high))

        if not 0.0 <= self.cleaning_reference_offset_m <= 1.0:
            raise ValueError("cleaning reference offset must be between 0 and 1m")
        if not 0.01 <= self.cleaning_coverage_tolerance_m <= MAX_CLEANING_COVERAGE_TOLERANCE_M:
            raise ValueError("cleaning coverage tolerance exceeds the 0.10m limit")
        if not 0.0 <= self.cleaning_min_forward_step_m <= 0.01:
            raise ValueError("cleaning minimum forward step is invalid")
        if not 0.0 <= self.cleaning_min_forward_speed_mps <= 0.05:
            raise ValueError("cleaning minimum forward speed is invalid")

        if not 0.03 <= self.min_acquire_anchor_v_fraction < 0.70:
            raise ValueError("min acquisition vertical fraction is invalid")
        if not 0.50 <= self.max_acquire_anchor_v_fraction <= 0.97:
            raise ValueError("max acquisition vertical fraction is invalid")
        if self.min_acquire_anchor_v_fraction >= self.max_acquire_anchor_v_fraction:
            raise ValueError("acquisition vertical range is empty")
        if not 0.10 <= self.acquire_max_abs_horizontal_error <= 1.0:
            raise ValueError("acquisition horizontal gate is invalid")
        if not 3 <= self.machine_config.acquire_frames <= 20:
            raise ValueError("acquire_frames must be between 3 and 20")
        if not 0.0 <= self.association_config.min_iou <= 0.5:
            raise ValueError("association_min_iou is invalid")
        if not 0.02 <= self.association_config.max_anchor_distance_ratio <= 0.25:
            raise ValueError("association anchor distance is invalid")
        if not 0.60 <= self.association_config.duplicate_min_iou <= 0.95:
            raise ValueError("duplicate detection IoU gate is invalid")
        if not (
            0.001
            <= self.association_config.duplicate_max_anchor_distance_ratio
            <= 0.05
        ):
            raise ValueError("duplicate detection anchor distance is invalid")
        if not 0.02 <= self.association_config.duplicate_max_depth_delta_m <= 0.50:
            raise ValueError("duplicate detection depth delta is invalid")
        if not 1 <= self.machine_config.loss_confirm_frames <= 100:
            raise ValueError("loss_confirm_frames must be between 1 and 100")
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
        if not 0.05 <= self.approach_min_command_speed_mps <= self.far_speed_mps:
            raise ValueError("approach minimum command speed is invalid")
        if not 0.05 <= self.max_linear_acceleration_mps2 <= 4.00:
            raise ValueError("linear acceleration limit is invalid")
        if not 0.10 <= self.max_curvature_rate_per_sec <= 2.0:
            raise ValueError("curvature rate limit is invalid")
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
        if not 0.03 <= self.blind_speed_mps <= MAX_COMMAND_SPEED_MPS:
            raise ValueError("blind_speed_mps must be between 0.03 and 0.20")
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

    def _uses_selected_target(self):
        # Selection is independent of the tail's completion policy.
        return getattr(self, "staged_target", None) is not None

    def _requires_coordinate_coverage(self):
        return self._uses_selected_target() and self.staged_target.completion_policy != "visual_servo"

    def _set_cleaning_target_cb(self, request):
        with self.run_lock:
            if request.clear:
                if self.phase not in (DISABLED, COMPLETE, ABORT):
                    return SetFodCleaningTargetResponse(
                        False, "cannot clear the selected target while control is active"
                    )
                self.staged_target = None
                return SetFodCleaningTargetResponse(True, "selected target cleared")
            if self.phase != DISABLED:
                return SetFodCleaningTargetResponse(
                    False, "selected target can only be staged while disabled"
                )
            try:
                object_id = int(request.object_id)
                x = float(request.target_position.x)
                y = float(request.target_position.y)
                z = float(request.target_position.z)
                observed_at = float(request.observed_at.to_sec())
                label = str(request.label).strip()
                policy = str(request.completion_policy)
                token = str(request.request_id)
                if policy not in ("", "visual_servo"):
                    raise ValueError("unknown selected-target completion policy")
                if policy == "visual_servo" and (not token or len(token) > 64):
                    raise ValueError("visual completion requires a bounded mission token")
                if object_id <= 0:
                    raise ValueError("object_id must be positive")
                if request.target_frame != self.expected_metric_frame:
                    raise ValueError("selected target must use the map frame")
                if not all(math.isfinite(value) for value in (x, y, z, observed_at)):
                    raise ValueError("selected target contains a non-finite value")
                if observed_at <= 0.0 or not label or len(label) > 32:
                    raise ValueError("selected target identity is invalid")
            except (TypeError, ValueError) as exc:
                return SetFodCleaningTargetResponse(False, str(exc))
            self.staged_target = StagedTarget(
                object_id=object_id,
                x=x,
                y=y,
                frame=self.expected_metric_frame,
                observed_at_sec=observed_at,
                label=label,
                expires_monotonic=time.monotonic() + self.staged_target_timeout_sec,
                completion_policy=policy,
                request_id=token,
            )
            return SetFodCleaningTargetResponse(
                True,
                "selected target {} staged at map ({:.3f},{:.3f})".format(
                    label, x, y
                ),
            )

    def _metric_results_cb(self, msg):
        receipt = time.monotonic()
        stamp_sec = float(msg.header.stamp.to_sec())
        width = int(msg.image_width)
        height = int(msg.image_height)
        errors = []
        if not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
            errors.append("metric result source stamp is missing or invalid")
        if width != self.expected_image_width or height != self.expected_image_height:
            errors.append("metric result image dimensions do not match the controller")
        if msg.header.frame_id != self.expected_camera_frame:
            errors.append("metric result camera frame is invalid")
        if str(msg.backend_id) != self.expected_metric_backend:
            errors.append("metric result backend is not detect_and_classify")
        if str(msg.detector_model_sha256).strip().lower() != self.expected_model_sha256:
            errors.append("metric result detector SHA256 does not match")

        targets = []
        for index, item in enumerate(msg.detections):
            if not item.world_position_valid:
                continue
            values = (
                item.detect_confidence,
                item.world_position.x,
                item.world_position.y,
                item.world_position.z,
                item.bbox.x_offset,
                item.bbox.y_offset,
                item.bbox.width,
                item.bbox.height,
            )
            if not all(math.isfinite(float(value)) for value in values):
                errors.append("metric detection[%d] contains a non-finite value" % index)
                continue
            if int(item.object_id) <= 0:
                errors.append("metric detection[%d] has no object identity" % index)
                continue
            if str(item.backend_id) != self.expected_metric_backend:
                errors.append("metric detection[%d] backend mismatch" % index)
                continue
            if str(item.world_frame) != self.expected_metric_frame:
                errors.append("metric detection[%d] is not in map frame" % index)
                continue
            if item.bbox.width <= 0 or item.bbox.height <= 0:
                errors.append("metric detection[%d] has an invalid box" % index)
                continue
            state = str(item.state)
            if state in ("CLEANED", "EXPIRED", "LOST"):
                continue
            targets.append(
                MetricTarget(
                    object_id=int(item.object_id),
                    track_id=int(item.track_id),
                    class_name=str(item.material_class),
                    confidence=float(item.detect_confidence),
                    x=float(item.world_position.x),
                    y=float(item.world_position.y),
                    z=float(item.world_position.z),
                    world_frame=str(item.world_frame),
                    bbox_x=float(item.bbox.x_offset),
                    bbox_y=float(item.bbox.y_offset),
                    bbox_width=float(item.bbox.width),
                    bbox_height=float(item.bbox.height),
                    state=state,
                )
            )

        with self.sensor_lock:
            if (
                self.last_metric_stamp is not None
                and math.isfinite(stamp_sec)
                and stamp_sec <= self.last_metric_stamp
            ):
                errors.append("metric result source stamp did not increase")
            if math.isfinite(stamp_sec) and stamp_sec > 0.0:
                self.last_metric_stamp = max(
                    stamp_sec,
                    self.last_metric_stamp
                    if self.last_metric_stamp is not None
                    else stamp_sec,
                )
            self.metric_sequence += 1
            frame = MetricFrame(
                sequence=self.metric_sequence,
                receipt_monotonic=receipt,
                stamp_sec=stamp_sec,
                width=width,
                height=height,
                targets=tuple(targets),
                error="; ".join(errors),
            )
            if frame.error:
                self.invalid_metric_generation += 1
            self.metric_frames.append(frame)
            self.latest_metric = frame

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

        depth_frame_trusted = bool(msg.depth_synchronized)
        if depth_frame_trusted:
            depth_stamp_sec = float(msg.depth_header.stamp.to_sec())
            depth_delta_sec = float(msg.depth_sync_delta_sec)
            measured_delta_sec = abs(depth_stamp_sec - stamp_sec)
            if not math.isfinite(depth_stamp_sec) or depth_stamp_sec <= 0.0:
                errors.append("synchronized depth source stamp is invalid")
                depth_frame_trusted = False
            if msg.depth_header.frame_id != msg.header.frame_id:
                errors.append("registered depth frame does not match the RGB frame")
                depth_frame_trusted = False
            if (
                not math.isfinite(depth_delta_sec)
                or depth_delta_sec < 0.0
                or depth_delta_sec > self.max_depth_sync_delta_sec
            ):
                errors.append("registered depth synchronization delta is invalid")
                depth_frame_trusted = False
            elif (
                math.isfinite(measured_delta_sec)
                and abs(measured_delta_sec - depth_delta_sec) > 0.002
            ):
                errors.append("registered depth synchronization metadata is inconsistent")
                depth_frame_trusted = False

        observations = []
        candidates = []
        depth_rejected = []
        depth_rejections = []
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
                depth_valid=bool(item.depth_valid),
                depth_m=float(item.depth_m),
                depth_mad_m=float(item.depth_mad_m),
                depth_sample_count=int(item.depth_sample_count),
                depth_valid_fraction=float(item.depth_valid_fraction),
            )
            try:
                validate_detection(detection, width, height)
            except ValueError as exc:
                errors.append("detection[%d] is invalid: %s" % (index, exc))
                continue
            if detection.class_name in self.allowed_class_names:
                observations.append(detection)
                if detection.confidence >= self.min_confidence:
                    rejection = ""
                    if self.require_depth_for_acquisition:
                        if not depth_frame_trusted:
                            rejection = "registered RGB/depth frame is not synchronized"
                        else:
                            rejection = depth_rejection_reason(
                                detection,
                                self.min_target_depth_m,
                                self.max_target_depth_m,
                                self.min_depth_sample_count,
                                self.min_depth_valid_fraction,
                                self.max_depth_mad_m,
                            )
                    if rejection:
                        depth_rejected.append(detection)
                        depth_rejections.append(
                            "detection[%d] %s: %s"
                            % (index, detection.class_name, rejection)
                        )
                    else:
                        candidates.append(detection)

        with self.sensor_lock:
            metric_matches = [
                metric
                for metric in self.metric_frames
                if abs(metric.stamp_sec - stamp_sec) <= self.metric_sync_tolerance_sec
            ]
            if len(metric_matches) == 1:
                metric_targets = metric_matches[0].targets
                metric_error = metric_matches[0].error
            elif not metric_matches:
                metric_targets = tuple()
                metric_error = "matching source-stamped metric result is unavailable"
            else:
                metric_targets = tuple()
                metric_error = "metric result source-stamp match is ambiguous"
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
                depth_rejected=tuple(depth_rejected),
                depth_rejections=tuple(depth_rejections),
                metric_targets=tuple(metric_targets),
                metric_error=metric_error,
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
        expected_publishers = [
            (self.detections_topic, self.expected_detector_node),
            (self.camera_info_topic, self.expected_camera_node),
            (self.odom_topic, self.expected_driver_node),
            (self.wheel_angle_topic, self.expected_driver_node),
        ]
        if self._uses_selected_target():
            expected_publishers.append(
                (self.metric_results_topic, self.expected_detector_node)
            )
        if not self.external_estop_override:
            expected_publishers.extend(
                (
                    (self.chassis_status_topic, self.expected_driver_node),
                    (self.control_timeout_topic, self.expected_driver_node),
                    (self.canbus_topic, self.expected_canbus_node),
                )
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
        if self.expected_cmd_vel_subscriber_node not in command_subscribers:
            raise ControllerAbort(
                "%s is not connected to %s; subscribers: %s"
                % (
                    self.cmd_vel_topic,
                    self.expected_cmd_vel_subscriber_node,
                    self._format_nodes(command_subscribers),
                )
            )
        if self.cmd_pub.get_num_connections() < 1:
            raise ControllerAbort(
                "cmd_vel publisher is not connected to the expected command consumer"
            )
        if (
            not self.external_estop_override
            and rospy.get_param(
                self.expected_driver_node + "/is_pub_control_timeout", None
            )
            is not True
        ):
            raise ControllerAbort(
                self.expected_driver_node + "/is_pub_control_timeout must be true"
            )
        self.last_graph_check_monotonic = now

    def _check_raw_can(self):
        if self.external_estop_override:
            return

        # The NVIDIA M2 driver is the sole periodic-query owner.  This J6M
        # controller only observes /canbus_msg, so enabling visual control
        # cannot increase the VCU request rate or starve either query stream.
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
                "metric": self.latest_metric,
                "invalid_metric_generation": self.invalid_metric_generation,
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
            metric_sequence=self.metric_sequence if self._uses_selected_target() else 0,
            invalid_metric_generation=(
                self.invalid_metric_generation if self._uses_selected_target() else 0
            ),
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

        detection = self.latest_detection
        metric = self.latest_metric
        camera = self.latest_camera
        odom = self.latest_odom
        wheel_angle = self.latest_wheel_angle
        wheel_receipt = self.latest_wheel_angle_monotonic
        if detection is None or detection.error:
            raise ControllerAbort("terminal commit lacks valid detections")
        if self._uses_selected_target() and (metric is None or metric.error):
            raise ControllerAbort("terminal commit lacks valid metric results")
        if camera is None or camera.error:
            raise ControllerAbort("terminal commit lacks valid CameraInfo")
        if odom is None:
            raise ControllerAbort("terminal commit lacks valid odometry")
        if wheel_angle is None or wheel_receipt is None or not math.isfinite(
            wheel_angle
        ):
            raise ControllerAbort("terminal commit lacks valid wheel feedback")

        receipt_limits = [
            (detection.receipt_monotonic, self.detection_timeout_sec),
            (camera.receipt_monotonic, self.camera_info_timeout_sec),
            (odom.receipt_monotonic, self.odom_timeout_sec),
            (wheel_receipt, self.wheel_angle_timeout_sec),
        ]
        source_stamps = [detection.stamp_sec, camera.stamp_sec, odom.stamp_sec]
        if self._uses_selected_target():
            receipt_limits.append((metric.receipt_monotonic, self.metric_results_timeout_sec))
            source_stamps.append(metric.stamp_sec)
        if not self.external_estop_override:
            chassis = self.latest_chassis_status
            chassis_receipt = self.latest_chassis_status_monotonic
            if chassis is None or chassis_receipt is None:
                raise ControllerAbort("terminal commit lacks valid chassis status")
            if (
                chassis.hard_emergency
                or chassis.soft_emergency
                or chassis.gamepad_emergency
                or chassis.robot_emergency
            ):
                raise ControllerAbort(
                    "terminal commit observed an active chassis emergency"
                )
            receipt_limits.append(
                (chassis_receipt, self.chassis_status_timeout_sec)
            )
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
        # mode deadline after any scheduler pause.
        now_source_time = rospy.Time.now().to_sec()
        now_monotonic = time.monotonic()
        if not terminal_feedback_is_fresh(
            now_monotonic=now_monotonic,
            now_source_time=now_source_time,
            receipt_limits=receipt_limits,
            source_stamps=source_stamps,
            source_timeout=self.source_stamp_timeout_sec,
            absolute_deadlines=(
                self.mode_deadline_monotonic,
            ),
        ):
            raise ControllerAbort(
                "feedback or an absolute deadline became stale before COMPLETE"
            )

    def _arm_session_fault_latches(self):
        """Atomically verify current safety samples and capture generation floors."""

        with self.sensor_lock:
            metric = self.latest_metric
            camera = self.latest_camera
            odom = self.latest_odom
            wheel_receipt = self.latest_wheel_angle_monotonic
            if self._uses_selected_target() and (metric is None or metric.error):
                raise ControllerAbort("cannot arm with invalid metric results")
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
            if not self.external_estop_override:
                chassis = self.latest_chassis_status
                if chassis is None or (
                    chassis.hard_emergency
                    or chassis.soft_emergency
                    or chassis.gamepad_emergency
                    or chassis.robot_emergency
                ):
                    raise ControllerAbort(
                        "cannot arm while a chassis emergency is active"
                    )
                for msg_type, label in RAW_REQUIRED_TYPES.items():
                    status = self.raw_can_status.get(msg_type)
                    if status is None or status[3] or not status[1]:
                        raise ControllerAbort(
                            "cannot arm with unsafe raw CAN status: %s" % label
                        )

            # Any callback after this atomic capture advances its generation
            # above the floor and will be visible once ACQUIRE becomes active.
            self.session_invalid_metric_generation = self.invalid_metric_generation
            self.session_invalid_camera_generation = self.invalid_camera_generation
            self.session_invalid_odom_generation = self.invalid_odom_generation
            self.session_invalid_wheel_generation = self.invalid_wheel_generation
            self.session_chassis_fault_generation = self.chassis_fault_generation
            self.session_raw_can_fault_generation = self.raw_can_fault_generation
            self.session_m2_bypass_event_generation = (
                self.m2_bypass_event_generation
            )

    def _check_metric_health(self, snapshot, require_new_detection=False):
        if not self._uses_selected_target():
            return
        metric = snapshot["metric"]
        if metric is None:
            raise ControllerAbort("no metric detection results have been received")
        if metric.error:
            raise ControllerAbort("invalid metric result stream: %s" % metric.error)
        if (
            self.phase != PRECHECK
            and snapshot["invalid_metric_generation"]
            > self.session_invalid_metric_generation
        ):
            raise ControllerAbort(
                "an invalid metric result was observed during the active session"
            )
        metric_age = time.monotonic() - metric.receipt_monotonic
        if metric_age > self.metric_results_timeout_sec:
            raise ControllerAbort(
                "%s receipt timeout %.3fs" % (self.metric_results_topic, metric_age)
            )
        self._source_age_check("metric results", metric.stamp_sec)
        if require_new_detection and metric.sequence <= self.session_metric_floor:
            raise ControllerAbort("waiting for a new metric result after enable")

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

        self._check_metric_health(snapshot, require_new_detection)

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

        if not self.external_estop_override:
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
        if (
            not self.external_estop_override
            and snapshot["control_timeout_seen"]
            and not ignore_control_timeout
        ):
            raise ControllerAbort("VCU reported a 200ms command timeout")

    def _check_health(self, force_graph=False, require_new_detection=False, ignore_control_timeout=False):
        if rospy.is_shutdown():
            raise ControllerAbort("ROS is shutting down")
        self._check_graph(force=force_graph)
        self._check_raw_can()
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
                self.staged_target = None
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
            if (
                self.staged_target is not None
                and now > self.staged_target.expires_monotonic
            ):
                self.staged_target = None
                return SetBoolResponse(
                    success=False,
                    message="selected pending-litter target expired before enable",
                )
            with self.sensor_lock:
                self.session_detection_floor = self.detection_sequence
                self.session_metric_floor = self.metric_sequence
                self.detection_queue.clear()
                self.odom_queue.clear()
                self.detection_queue_overflow = False
                self.odom_queue_overflow = False
                self.control_timeout_seen = False
                self.session_invalid_odom_generation = self.invalid_odom_generation
                self.session_invalid_wheel_generation = self.invalid_wheel_generation
                self.session_invalid_metric_generation = self.invalid_metric_generation
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
            self._visual_receipt().begin()
            if self.external_estop_override:
                self.reason = (
                    "running visual, odometry, wheel, and ROS control-chain "
                    "prechecks; CAN/VCU safety gates are overridden"
                )
            else:
                self.reason = (
                    "running graph, sensor, emergency-stop, and chassis prechecks"
                )
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
        if self.external_estop_override:
            rospy.logwarn(
                "FOD visual-servo enable requested with external emergency-stop "
                "override. CAN/VCU safety state is not checked; the vehicle remains "
                "stopped until visual/odometry/wheel/control-chain prechecks pass and "
                "one target is stable. Keep the remote stop immediately operable."
            )
        else:
            rospy.logwarn(
                "FOD visual-servo enable requested. Vehicle remains stopped until all "
                "prechecks pass and the nearest depth-qualified target is stable. Keep the physical "
                "emergency stop ready."
            )
        return SetBoolResponse(
            success=True,
            message=(
                "enable accepted; motion waits for prechecks and one stable depth-selected target "
                "inside the steering capture range"
            ),
        )

    def _clear_run_metrics(self):
        self.target_u_px = None
        self.horizontal_error_value = None
        self.vertical_fraction_value = None
        self.target_visible = False
        self.locked_depth_invalid_frames = 0
        self.pending_metric_object_id = None
        self.locked_metric_target = None
        self.metric_target_distance_m = None
        self.metric_target_bearing_deg = None
        self.coverage_tracker = None
        self.coverage_progress = None
        self.coverage_last_odom_sequence = 0
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
        self._visual_receipt().disable()
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
        self._visual_receipt().abort()
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
            self._confirm_completion_stopped()
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
                "prechecks passed; waiting for the nearest stable depth-qualified target inside the steering capture range",
            )
            return

        self._check_health()

        if self.phase in (APPROACH, REACQUIRE, EDGE_ARMED, LOSS_CONFIRM):
            self._update_approach_odometry()
        elif self.phase == ACQUIRE:
            self._discard_odom_events()

        self._process_detection_events()

        if self.phase in (APPROACH, EDGE_ARMED):
            self._command_visual_approach()
        elif self.phase in (ACQUIRE, REACQUIRE, LOSS_CONFIRM):
            self._hard_stop()
        elif self.phase == BLIND_ADVANCE:
            self._blind_advance_tick()
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
            self._update_forward_coverage(state, self.max_pose_step_m)
            self.approach_path_m += step
            previous = state
            if self.approach_path_m >= self.approach_next_progress_m:
                self.approach_motion_seen = True
                self.approach_last_progress_monotonic = time.monotonic()
                while self.approach_path_m >= self.approach_next_progress_m:
                    self.approach_next_progress_m += self.approach_progress_step_m
        self.approach_last_odom = previous

    @staticmethod
    def _bbox_iou(pixel, metric):
        left = max(pixel.x, metric.bbox_x)
        top = max(pixel.y, metric.bbox_y)
        right = min(pixel.x + pixel.width, metric.bbox_x + metric.bbox_width)
        bottom = min(pixel.y + pixel.height, metric.bbox_y + metric.bbox_height)
        intersection = max(0.0, right - left) * max(0.0, bottom - top)
        union = (
            pixel.width * pixel.height
            + metric.bbox_width * metric.bbox_height
            - intersection
        )
        return intersection / union if union > 0.0 else 0.0

    def _metric_frame_for_detection(self, frame):
        # The detector publishes the metric result first, but the two topics
        # use independent TCPROS callback threads. Resolve the pair at consume
        # time as well as callback time so harmless callback scheduling cannot
        # turn a valid frame into a false mismatch.
        with self.sensor_lock:
            matches = tuple(
                item
                for item in self.metric_frames
                if abs(item.stamp_sec - frame.stamp_sec)
                <= self.metric_sync_tolerance_sec
            )
        if len(matches) != 1:
            raise ControllerAbort(
                "metric result source-stamp match is %s"
                % ("missing" if not matches else "ambiguous")
            )
        if matches[0].error:
            raise ControllerAbort(
                "source-stamped metric result is invalid: %s" % matches[0].error
            )
        return matches[0]

    def _metric_target_for_pixel(self, frame, pixel):
        metric_frame = self._metric_frame_for_detection(frame)
        matches = tuple(
            target
            for target in metric_frame.targets
            if self._bbox_iou(pixel, target) >= self.metric_bbox_min_iou
        )
        if len(matches) != 1:
            raise ControllerAbort(
                "pixel target has %d metric identity matches (requires exactly one)"
                % len(matches)
            )
        return matches[0]

    def _validate_selected_metric_target(self, target):
        staged = self.staged_target
        if staged is not None:
            if (time.monotonic() > staged.expires_monotonic and
                    (staged.completion_policy != "visual_servo" or self.locked_metric_target is None)):
                raise ControllerAbort("selected pending-litter target expired")
            if staged.completion_policy != "visual_servo" and target.object_id != staged.object_id:
                raise ControllerAbort(
                    "visual target identity %d differs from selected object %d"
                    % (target.object_id, staged.object_id)
                )
            delta = math.hypot(target.x - staged.x, target.y - staged.y)
            if delta > self.metric_max_selected_position_delta_m:
                raise ControllerAbort(
                    "selected target map position changed %.3fm, limit %.3fm"
                    % (delta, self.metric_max_selected_position_delta_m)
                )
        locked = self.locked_metric_target
        if locked is not None:
            if target.object_id != locked.object_id:
                raise ControllerAbort(
                    "visual target identity changed from %d to %d"
                    % (locked.object_id, target.object_id)
                )
            jump = math.hypot(target.x - locked.x, target.y - locked.y)
            if jump > self.metric_max_position_jump_m:
                raise ControllerAbort(
                    "locked target map position jumped %.3fm, limit %.3fm"
                    % (jump, self.metric_max_position_jump_m)
                )

    def _initialize_forward_coverage(self, metric_target):
        with self.sensor_lock:
            odom = self.latest_odom
        if odom is None:
            raise ControllerAbort("cannot initialize cleaning coverage without odometry")
        try:
            transform = self.tf_buffer.lookup_transform(
                self.expected_metric_frame,
                self.expected_base_frame,
                rospy.Time(0),
                rospy.Duration(self.map_tf_timeout_sec),
            )
            stamp_sec = float(transform.header.stamp.to_sec())
            age = rospy.Time.now().to_sec() - stamp_sec
            if (
                not math.isfinite(stamp_sec)
                or stamp_sec <= 0.0
                or age < -0.20
                or age > self.map_tf_timeout_sec
            ):
                raise ValueError("map-to-base transform is stale or invalid")
            translation = transform.transform.translation
            robot_map_yaw = quaternion_to_yaw(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
            target_map_x = (
                self.staged_target.x
                if self.staged_target is not None
                else metric_target.x
            )
            target_map_y = (
                self.staged_target.y
                if self.staged_target is not None
                else metric_target.y
            )
            target_odom_x, target_odom_y = map_target_to_wheel_odom(
                float(translation.x),
                float(translation.y),
                robot_map_yaw,
                target_map_x,
                target_map_y,
                odom.x,
                odom.y,
                odom.yaw,
            )
        except Exception as exc:
            raise ControllerAbort(
                "failed to freeze selected map target in wheel odometry: %s" % exc
            )

        dx = target_map_x - float(translation.x)
        dy = target_map_y - float(translation.y)
        self.metric_target_distance_m = math.hypot(dx, dy)
        self.metric_target_bearing_deg = math.degrees(
            math.atan2(
                -math.sin(robot_map_yaw) * dx + math.cos(robot_map_yaw) * dy,
                math.cos(robot_map_yaw) * dx + math.sin(robot_map_yaw) * dy,
            )
        )
        self.coverage_tracker = ForwardCoverageTracker(
            target_x=target_odom_x,
            target_y=target_odom_y,
            reference_offset_m=self.cleaning_reference_offset_m,
            tolerance_m=self.cleaning_coverage_tolerance_m,
            start_x=odom.x,
            start_y=odom.y,
            start_yaw=odom.yaw,
            start_linear_x=odom.linear_x,
            min_forward_step_m=self.cleaning_min_forward_step_m,
            min_forward_speed_mps=self.cleaning_min_forward_speed_mps,
        )
        self.coverage_progress = self.coverage_tracker.progress()
        self.coverage_last_odom_sequence = odom.sequence

    def _update_forward_coverage(self, state, max_pose_step_m):
        if not self._requires_coordinate_coverage():
            return None
        if self.coverage_tracker is None:
            raise ControllerAbort("selected-target coverage tracker is not initialized")
        if state.sequence <= self.coverage_last_odom_sequence:
            return self.coverage_progress
        try:
            self.coverage_progress = self.coverage_tracker.update(
                state.x,
                state.y,
                state.yaw,
                state.linear_x,
                max_pose_step_m,
            )
        except ValueError as exc:
            raise ControllerAbort("cleaning coverage odometry is invalid: %s" % exc)
        self.coverage_last_odom_sequence = state.sequence
        return self.coverage_progress

    def _acquisition_candidates(self, frame):
        if not frame.candidates:
            depth_rejections = tuple(
                getattr(frame, "depth_rejections", tuple())
            )
            if depth_rejections:
                return tuple(), depth_rejections[0]
            observations = tuple(getattr(frame, "observations", tuple()))
            if observations:
                best_confidence = max(item.confidence for item in observations)
                return (
                    tuple(),
                    "best target confidence %.3f is below acquisition threshold %.3f"
                    % (best_confidence, self.min_confidence),
                )
        candidates = frame.candidates
        if candidates and self.staged_target is not None:
            selected = []
            for candidate in candidates:
                try:
                    target = self._metric_target_for_pixel(frame, candidate)
                except ControllerAbort as exc:
                    if "source-stamp match is missing" in str(exc):
                        return tuple(), "waiting for source-stamped metric identity"
                    raise
                matches = target.object_id == self.staged_target.object_id
                if self.staged_target.completion_policy == "visual_servo":
                    # A tracked detection ID can legitimately change during
                    # the drive. Rebind only one stable target near the saved
                    # map point; two plausible objects remain ambiguous.
                    matches = (target.state == "CONFIRMED" and math.hypot(
                        target.x-self.staged_target.x, target.y-self.staged_target.y
                    ) <= self.metric_max_selected_position_delta_m)
                if matches:
                    self._validate_selected_metric_target(target)
                    selected.append(candidate)
            if len(selected) != 1:
                return (
                    tuple(),
                    "selected pending-litter object %d is not uniquely visible"
                    % self.staged_target.object_id,
                )
            candidates = tuple(selected)
        if len(candidates) > 1 and getattr(
            self, "require_depth_for_acquisition", False
        ):
            pending = getattr(getattr(self, "machine", None), "pending", None)
            candidate = nearest_depth_target(
                candidates,
                preferred=pending,
                preferred_hysteresis_m=self.nearest_depth_hysteresis_m,
                image_width=frame.width,
                image_height=frame.height,
                association=self.association_config,
            )
            if candidate is None:
                return tuple(), "multiple targets are visible but none has valid depth"
            if pending is not None:
                pending_matches = matching_detections(
                    pending,
                    candidates,
                    frame.width,
                    frame.height,
                    self.association_config,
                )
                # The phase machine sees only the depth-selected candidate. If
                # selection genuinely changes objects, reset its hit counter so
                # a broad association gate cannot blend two targets into one
                # six-frame acquisition.
                if not pending_matches or candidate is not pending_matches[0]:
                    self.machine.pending = None
                    self.machine.pending_hits = 0
            candidates = (candidate,)
        elif len(candidates) != 1:
            return candidates, ""
        candidate = candidates[0]
        q = candidate.anchor_v / float(frame.height - 1)
        error = horizontal_error(candidate.anchor_u, self.target_u_px, frame.width)
        if not (
            self.min_acquire_anchor_v_fraction
            <= q
            <= self.max_acquire_anchor_v_fraction
        ):
            return (
                tuple(),
                "target vertical fraction %.3f is outside capture range %.3f..%.3f"
                % (
                    q,
                    self.min_acquire_anchor_v_fraction,
                    self.max_acquire_anchor_v_fraction,
                ),
            )
        if abs(error) > self.acquire_max_abs_horizontal_error:
            return (
                tuple(),
                "target horizontal error %.3f exceeds steering capture limit %.3f"
                % (error, self.acquire_max_abs_horizontal_error),
            )
        return candidates, ""

    def _candidates_with_locked_depth_grace(self, frame):
        """Retain one locked target through bounded registered-depth jitter."""

        candidates = tuple(frame.candidates)
        if (
            self.phase == ACQUIRE
            or not self.require_depth_for_acquisition
            or self.machine.locked is None
        ):
            self.locked_depth_invalid_frames = 0
            return candidates, ""

        valid_matches = matching_detections(
            self.machine.locked,
            candidates,
            frame.width,
            frame.height,
            self.association_config,
        )
        if valid_matches:
            self.locked_depth_invalid_frames = 0
            return candidates, ""

        rejected_matches = matching_detections(
            self.machine.locked,
            frame.depth_rejected,
            frame.width,
            frame.height,
            self.association_config,
        )
        if not rejected_matches:
            # Only a depth-rejected observation of the same spatial target
            # contributes to the consecutive counter. A true dropout or an
            # unrelated rejected object follows the normal loss state machine.
            self.locked_depth_invalid_frames = 0
            return candidates, ""

        self.locked_depth_invalid_frames += 1
        rejection_reason = "invalid depth"
        for rejected, reason in zip(frame.depth_rejected, frame.depth_rejections):
            if rejected in rejected_matches:
                rejection_reason = reason
                break
        if (
            self.locked_depth_invalid_frames
            >= self.locked_depth_invalid_abort_frames
        ):
            raise ControllerAbort(
                "locked target lost motion-grade registered depth for %d "
                "consecutive frames: %s"
                % (self.locked_depth_invalid_frames, rejection_reason)
            )

        return (
            candidates + rejected_matches,
            "locked target registered depth invalid %d/%d; retaining visual "
            "lock: %s"
            % (
                self.locked_depth_invalid_frames,
                self.locked_depth_invalid_abort_frames,
                rejection_reason,
            ),
        )

    def _process_detection_events(self):
        frames = self._take_detection_events()
        for frame in frames:
            if frame.error:
                raise ControllerAbort("invalid detection frame: %s" % frame.error)

            if self.phase == BLIND_ADVANCE:
                # Ten-frame confirmation has finished. Complete this short pass;
                # later detections are not a new failure condition.
                continue

            if self.phase not in (
                ACQUIRE,
                APPROACH,
                REACQUIRE,
                EDGE_ARMED,
                LOSS_CONFIRM,
            ):
                continue
            acquisition_reason = ""
            depth_grace_reason = ""
            if self.phase == ACQUIRE:
                candidates, acquisition_reason = self._acquisition_candidates(frame)
            else:
                # Depth/confidence select the initial target only. A visible
                # locked object with poor stereo depth is NOT an absent object.
                candidates = frame.observations
            pending_metric = None
            if self._uses_selected_target() and self.phase == ACQUIRE and len(candidates) == 1:
                try:
                    pending_metric = self._metric_target_for_pixel(
                        frame, candidates[0]
                    )
                except ControllerAbort as exc:
                    if "source-stamp match is missing" in str(exc):
                        candidates = tuple()
                        acquisition_reason = (
                            "waiting for source-stamped metric identity"
                        )
                    else:
                        raise
                if pending_metric is not None:
                    self._validate_selected_metric_target(pending_metric)
                    if (
                        self.pending_metric_object_id is not None
                        and pending_metric.object_id
                        != self.pending_metric_object_id
                    ):
                        self.machine.pending = None
                        self.machine.pending_hits = 0
                    self.pending_metric_object_id = pending_metric.object_id
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
            frame_metric_target = None
            if self._uses_selected_target() and decision.target is not None and candidates and (decision.acquired or self._requires_coordinate_coverage()):
                frame_metric_target = self._metric_target_for_pixel(
                    frame, decision.target
                )
                self._validate_selected_metric_target(frame_metric_target)
            if decision.acquired:
                if self._uses_selected_target():
                    if frame_metric_target is None:
                        raise ControllerAbort(
                            "target acquired without a unique metric identity"
                        )
                    self.locked_metric_target = frame_metric_target
                    self.pending_metric_object_id = frame_metric_target.object_id
                    if self._requires_coordinate_coverage():
                        self._initialize_forward_coverage(frame_metric_target)
                self._start_approach_odometry()
            self.phase = decision.state
            self.reason = depth_grace_reason or acquisition_reason or decision.reason
            self.target_visible = bool(candidates) and self.phase in (
                APPROACH,
                EDGE_ARMED,
            )
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
            if self.phase == BLIND_ADVANCE and previous_state != BLIND_ADVANCE:
                self._begin_forward_pass()
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
        speed = max(
            self.approach_min_command_speed_mps,
            approach_speed(
                vertical_fraction=q,
                horizontal_error_abs=abs(error),
                far_speed=self.far_speed_mps,
                near_speed=self.near_speed_mps,
                slow_start_fraction=self.slow_start_fraction,
                near_start_fraction=self.near_start_fraction,
                lateral_slowdown_error=self.lateral_slowdown_error,
                minimum_lateral_scale=self.minimum_lateral_speed_scale,
            ),
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

    def _begin_forward_pass(self):
        # Start at confirmation, not at the first missing image. Travel during
        # the ten-frame wait does not subtract from the requested extra distance.
        now = time.monotonic()
        with self.sensor_lock:
            odom = self.latest_odom
            self.odom_queue.clear()
        if odom is None:
            raise ControllerAbort("forward pass needs odometry")
        self.blind_tracker = BlindDistanceTracker(odom.x, odom.y, odom.yaw)
        self.blind_progress = self.blind_tracker.update(
            odom.x, odom.y, odom.yaw, self.max_pose_step_m)
        self.blind_last_odom_state = odom
        self.last_processed_odom_sequence = odom.sequence
        self.blind_started_monotonic = now
        self.blind_drive_started_monotonic = now
        self.blind_last_progress_monotonic = now
        self.blind_motion_seen = False
        self.blind_next_progress_m = self.blind_progress_step_m
        self.blind_max_forward_m = 0.0

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
        if self.blind_tracker is None:
            raise ControllerAbort("forward pass has no odometry baseline")
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
            self._update_forward_coverage(state, dynamic_step_limit)
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
            if track_progress and progress.forward_m >= self.blind_next_progress_m:
                self.blind_motion_seen = True
                self.blind_last_progress_monotonic = now
                while progress.forward_m >= self.blind_next_progress_m:
                    self.blind_next_progress_m += self.blind_progress_step_m
        return self.blind_progress

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
        progress = self._update_blind_odometry(
            speed_limit=self.blind_speed_mps + 0.08,
            track_progress=True,
        )
        if progress is None:
            raise ControllerAbort("blind advance lost its synchronized odometry baseline")
        coverage = self.coverage_progress
        if not self._requires_coordinate_coverage() and blind_goal_reached(progress, self.blind_distance_m):
            self._complete_forward_pass(progress)
            return
        if coverage is not None and coverage.covered:
            self._complete_forward_pass(progress)
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

        minimum_distance = (
            coverage.minimum_distance_m
            if coverage is not None and math.isfinite(coverage.minimum_distance_m)
            else float("nan")
        )
        self.reason = (
            "post-loss net forward %.3fm (path %.3fm); selected-coordinate "
            "minimum distance %.3f/%.3fm"
        ) % (
            progress.forward_m,
            progress.path_m,
            minimum_distance,
            self.cleaning_coverage_tolerance_m,
        )
        if not self._requires_coordinate_coverage():
            self.reason = "post-loss net forward %.3f/%.3fm (path %.3fm)" % (
                progress.forward_m, self.blind_distance_m, progress.path_m
            )
        self._set_motion(self.blind_speed_mps, 0.0)

    def _complete_forward_pass(self, progress):
        self._hard_stop()
        odom = self.blind_last_odom_state
        expected_fence = TerminalSensorFence(
            odom_sequence=odom.sequence,
            wheel_sequence=self.wheel_sequence,
            detection_sequence=self.last_processed_detection_sequence,
            metric_sequence=self.metric_sequence if self._uses_selected_target() else 0,
            invalid_metric_generation=(
                self.session_invalid_metric_generation if self._uses_selected_target() else 0
            ),
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

        # Retain the existing atomic cancellation/fault fence, not a geometric
        # or stop-drift completion gate.
        with self.sensor_lock:
            if not terminal_sensor_fence_unchanged(
                    expected_fence, self._terminal_sensor_fence_locked()):
                return
            self._check_terminal_commit_health_locked()
            self.phase = COMPLETE
            self.reason = ("target absent for %d frames; forward %.3f/%.3fm; "
                           "cleaning complete, stop requested") % (
                self.machine_config.loss_confirm_frames, progress.forward_m,
                self.blind_distance_m)
            self.completed_odom_sequence = odom.sequence
            self._record_cleaning_completion(progress)
            self.target_visible = False
        rospy.logwarn("FOD visual servo COMPLETE: %s", self.reason)

    def _confirm_completion_stopped(self):
        # Cleaning is already COMPLETE. This only supplies truthful stop
        # feedback to existing AI/pending consumers; it cannot fail completion.
        # The unchanged mode manager still owns physical stopping and handoff.
        receipt = self._visual_receipt()
        if receipt.result != "COMPLETE" or receipt.last_completion.get("stopped"):
            return
        with self.sensor_lock:
            odom = self.latest_odom
            if (odom is None or
                    odom.sequence <= getattr(self, "completed_odom_sequence", odom.sequence) or
                    time.monotonic() - odom.receipt_monotonic > self.odom_timeout_sec or
                    not 0 <= rospy.Time.now().to_sec() - odom.stamp_sec <= self.source_stamp_timeout_sec or
                    abs(odom.linear_x) > 0.03 or abs(odom.angular_z) > 0.05):
                return
            receipt.last_completion["stopped"] = True
            if self.last_cleaning_completion:
                self.last_cleaning_completion["stopped"] = True

    def _visual_receipt(self):
        # Metadata only. Laziness also supports isolated __new__ test fixtures;
        # without a real begin() there can never be a completion receipt.
        if not hasattr(self, "visual_session"):
            self.visual_session = VisualSessionReceipt()
        return self.visual_session

    def _record_cleaning_completion(self, progress):
        # Completion and physical stop are separate facts. Preserve across disable.
        self._visual_receipt().complete(stopped=False)
        if not self._uses_selected_target():
            return
        staged = self.staged_target
        evidence = {
            "sequence": self.last_cleaning_completion.get("sequence", 0) + 1,
            "object_id": str(staged.object_id),
            "acquired_object_id": str(self.locked_metric_target.object_id),
            "target_x": staged.x, "target_y": staged.y,
            "completion_policy": staged.completion_policy,
            "request_id": staged.request_id, "stopped": False,
            "forward_m": progress.forward_m, "path_m": progress.path_m,
            "blind_target_m": self.blind_distance_m,
        }
        if self._requires_coordinate_coverage():
            evidence.update(covered=True, minimum_distance_m=self.coverage_progress.minimum_distance_m,
                            eligible_segments=self.coverage_progress.eligible_segments)
        self.last_cleaning_completion = evidence

    def _status_dictionary(self):
        now = time.monotonic()
        with self.command_lock:
            command_speed = self.motion_lease.linear_x
            command_curvature = self.motion_lease.curvature
            command_expired_reason = self.motion_lease.expired_reason
        with self.sensor_lock:
            detection = self.latest_detection
            metric = self.latest_metric
            camera = self.latest_camera
            odom = self.latest_odom
            wheel = self.latest_wheel_angle
            wheel_receipt = self.latest_wheel_angle_monotonic
            chassis_receipt = self.latest_chassis_status_monotonic
            raw_status = dict(self.raw_can_status)
        target = self.machine.locked
        blind_distance = self.blind_progress.path_m if self.blind_progress else 0.0
        blind_forward = self.blind_progress.forward_m if self.blind_progress else 0.0
        metric_target = self.locked_metric_target
        staged_target = self.staged_target
        coverage = self.coverage_progress
        raw_can_ages = {
            RAW_OBSERVED_TYPES.get(msg_type, "0x%02X" % msg_type): round(
                max(0.0, now - value[0]), 3
            )
            for msg_type, value in raw_status.items()
        }
        return {
            "pending_target_protocol": 2,
            "ai_completion_protocol": 1,
            "visual_session": self._visual_receipt().status(),
            "completion_policy": staged_target.completion_policy if staged_target else "",
            "pending_request_id": staged_target.request_id if staged_target else "",
            "state": self.phase,
            "reason": self.reason,
            "min_confidence": round(self.min_confidence, 4),
            "raw_can_query_owner": self.expected_driver_node,
            "raw_can_timeout_sec": self.raw_can_timeout_sec,
            "allow_motion": self.allow_motion,
            "external_estop_override": self.external_estop_override,
            "can_vcu_safety_checks_enabled": not self.external_estop_override,
            "active": self.phase
            not in (DISABLED, PRECHECK, COMPLETE, ABORT),
            "completed": self.phase == COMPLETE,
            "last_cleaning_completion": self.last_cleaning_completion,
            "target_visible": self.target_visible,
            "target_locked": target is not None,
            "metric_target_object_id": (
                metric_target.object_id
                if metric_target is not None
                else (
                    staged_target.object_id if staged_target is not None else 0
                )
            ),
            "metric_target_map_x": (
                round(staged_target.x, 4)
                if staged_target is not None
                else (
                    round(metric_target.x, 4)
                    if metric_target is not None
                    else None
                )
            ),
            "metric_target_map_y": (
                round(staged_target.y, 4)
                if staged_target is not None
                else (
                    round(metric_target.y, 4)
                    if metric_target is not None
                    else None
                )
            ),
            "metric_target_initial_distance_m": (
                round(self.metric_target_distance_m, 4)
                if self.metric_target_distance_m is not None
                else None
            ),
            "metric_target_initial_bearing_deg": (
                round(self.metric_target_bearing_deg, 3)
                if self.metric_target_bearing_deg is not None
                else None
            ),
            "cleaning_reference_offset_m": self.cleaning_reference_offset_m,
            "cleaning_coverage_tolerance_m": (
                self.cleaning_coverage_tolerance_m
            ),
            "cleaning_coordinate_covered": (
                coverage.covered if coverage is not None else False
            ),
            "cleaning_minimum_distance_m": (
                round(coverage.minimum_distance_m, 4)
                if coverage is not None
                and math.isfinite(coverage.minimum_distance_m)
                else None
            ),
            "cleaning_forward_path_m": (
                round(coverage.forward_path_m, 4)
                if coverage is not None
                else 0.0
            ),
            "cleaning_reverse_path_m_ignored": (
                round(coverage.reverse_path_m, 4)
                if coverage is not None
                else 0.0
            ),
            "cleaning_eligible_forward_segments": (
                coverage.eligible_segments if coverage is not None else 0
            ),
            "locked_depth_invalid_frames": self.locked_depth_invalid_frames,
            "locked_depth_invalid_abort_frames": (
                self.locked_depth_invalid_abort_frames
            ),
            "target_class": target.class_name if target is not None else "",
            "target_confidence": round(target.confidence, 4) if target is not None else None,
            "target_depth_m": (
                round(target.depth_m, 4)
                if target is not None
                and target.depth_valid
                and math.isfinite(target.depth_m)
                else None
            ),
            "target_depth_mad_m": (
                round(target.depth_mad_m, 4)
                if target is not None
                and target.depth_valid
                and math.isfinite(target.depth_mad_m)
                else None
            ),
            "target_depth_sample_count": (
                target.depth_sample_count if target is not None else None
            ),
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
            "selected_coordinate_completion": self._requires_coordinate_coverage(),
            "blind_hard_limit_m": self.blind_hard_distance_m,
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
            "metric_results_age_sec": (
                round(max(0.0, now - metric.receipt_monotonic), 3)
                if metric is not None
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
