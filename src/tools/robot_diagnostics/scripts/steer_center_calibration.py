#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Safely estimate M2 steering-center error from a straight, low-speed run.

The estimator uses the independent dual-antenna heading carried by /gps/odom.
It deliberately never publishes /m2_driver/steer_center_bias; the result is a
measurement and a suggested next trial only.
"""

import csv
from collections import deque
from dataclasses import dataclass
import datetime
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

import rosgraph
import rospy
from autolabor_canbus_driver.msg import CanBusMessage, ChassisStatusInfo
from autolabor_canbus_driver.srv import CanBusService, ChassisParameterServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64


DEFAULT_OUTPUT_DIR = str(Path(__file__).resolve().parents[4] / "test_results")
MAX_CALIBRATION_SPEED_MPS = 0.30
MAX_EXTERNAL_ESTOP_CALIBRATION_SPEED_MPS = 0.50
MAX_MEASURED_SPEED_MPS = 0.50
MAX_EXTERNAL_ESTOP_MEASURED_SPEED_MPS = 0.65
MAX_CALIBRATION_DISTANCE_M = 20.0
MIN_CALIBRATION_DISTANCE_M = 2.0
COURSE_RESIDUAL_WARNING_DEG = 0.5
MAX_RECOMMENDABLE_STEER_ERROR_DEG = 1.0
MAX_RECOMMENDATION_INCREMENT_DEG = 0.5
MAX_RECOMMENDED_ABSOLUTE_BIAS_DEG = 2.0
MAX_HEADING_FIT_RMSE_DEG = 0.5
MAX_EQUIVALENT_STEER_STANDARD_ERROR_DEG = 0.15
MAX_POSITION_STEER_DISAGREEMENT_DEG = 0.35
MIN_CHORD_TO_WHEEL_DISTANCE_RATIO = 0.80
MAX_CHORD_TO_WHEEL_DISTANCE_RATIO = 1.20
VCU_NODE_TYPE = 0x10
VCU_NODE_ID = 0x00
VCU_HARD_EMERGENCY = 0x17
VCU_SOFT_EMERGENCY = 0x18
VCU_GAMEPAD_EMERGENCY = 0x19
VCU_COMMON_STATE = 0x80
VCU_RUNNING_STATE = 0x10
RAW_EMERGENCY_TYPES = {
    VCU_HARD_EMERGENCY: "硬件急停",
    VCU_SOFT_EMERGENCY: "软件急停",
    VCU_GAMEPAD_EMERGENCY: "手柄急停",
    VCU_COMMON_STATE: "整车运行状态",
}
RAW_EMERGENCY_QUERY_ORDER = tuple(RAW_EMERGENCY_TYPES)


class CalibrationAbort(RuntimeError):
    """Expected fail-closed termination of a calibration run."""


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


@dataclass(frozen=True)
class MotionPoint:
    state: OdomState
    path_m: float
    gps_path_m: float
    yaw_unwrapped: float
    forward_m: float
    lateral_m: float


@dataclass(frozen=True)
class DriverSpeedState:
    receipt_monotonic: float
    stamp_sec: float
    linear_x: float


def normalize_angle(angle):
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x, y, z, w):
    values = (x, y, z, w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion contains a non-finite value")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-6 or abs(norm - 1.0) > 0.2:
        raise ValueError("quaternion norm is invalid: %.6f" % norm)
    x, y, z, w = (value / norm for value in values)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def circular_mean(angles):
    values = list(angles)
    if not values or not all(math.isfinite(value) for value in values):
        raise ValueError("circular mean requires finite samples")
    sin_mean = sum(math.sin(value) for value in values) / len(values)
    cos_mean = sum(math.cos(value) for value in values) / len(values)
    if math.hypot(sin_mean, cos_mean) < 1e-9:
        raise ValueError("heading samples have no unique circular mean")
    return math.atan2(sin_mean, cos_mean)


def circular_stddev(angles):
    values = list(angles)
    if not values or not all(math.isfinite(value) for value in values):
        raise ValueError("circular standard deviation requires finite samples")
    sin_mean = sum(math.sin(value) for value in values) / len(values)
    cos_mean = sum(math.cos(value) for value in values) / len(values)
    resultant = min(1.0, max(1e-12, math.hypot(sin_mean, cos_mean)))
    return math.sqrt(max(0.0, -2.0 * math.log(resultant)))


def local_displacement(start_x, start_y, start_yaw, end_x, end_y):
    values = (start_x, start_y, start_yaw, end_x, end_y)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("local displacement inputs must be finite")
    dx = end_x - start_x
    dy = end_y - start_y
    forward = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
    lateral = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
    return forward, lateral


def linear_fit_slope(xs, ys):
    xs = list(xs)
    ys = list(ys)
    if len(xs) != len(ys) or len(xs) < 2:
        raise ValueError("linear fit requires equally sized sample vectors")
    if not all(math.isfinite(value) for value in xs + ys):
        raise ValueError("linear fit samples must be finite")
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    denominator = sum((value - mean_x) ** 2 for value in xs)
    if denominator <= 1e-12:
        raise ValueError("linear fit distance span is too small")
    return sum(
        (x_value - mean_x) * (y_value - mean_y)
        for x_value, y_value in zip(xs, ys)
    ) / denominator


def fit_curvature(points, trim_start_m=0.5, trim_end_m=0.5):
    points = list(points)
    if len(points) < 3:
        raise ValueError("curvature fit requires at least three samples")
    total_distance = points[-1].path_m
    usable = [
        point
        for point in points
        if point.path_m >= trim_start_m
        and point.path_m <= total_distance - trim_end_m
    ]
    if len(usable) < 3:
        raise ValueError("not enough samples remain after distance trimming")
    if usable[-1].path_m - usable[0].path_m < 1.0:
        raise ValueError("curvature fit requires at least 1 m of steady data")
    return linear_fit_slope(
        [point.path_m for point in usable],
        [point.yaw_unwrapped for point in usable],
    ), len(usable), usable[-1].path_m - usable[0].path_m


def curvature_fit_metrics(points, trim_start_m=0.5, trim_end_m=0.5):
    points = list(points)
    curvature, count, span = fit_curvature(points, trim_start_m, trim_end_m)
    total_distance = points[-1].path_m
    usable = [
        point
        for point in points
        if point.path_m >= trim_start_m
        and point.path_m <= total_distance - trim_end_m
    ]
    xs = [point.path_m for point in usable]
    ys = [point.yaw_unwrapped for point in usable]
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    intercept = mean_y - curvature * mean_x
    residuals = [
        yaw - (intercept + curvature * distance)
        for distance, yaw in zip(xs, ys)
    ]
    squared_error = sum(value * value for value in residuals)
    rmse = math.sqrt(squared_error / len(residuals))
    denominator = sum((value - mean_x) ** 2 for value in xs)
    slope_standard_error = (
        math.sqrt((squared_error / (len(xs) - 2)) / denominator)
        if len(xs) > 2 and denominator > 1e-12
        else float("inf")
    )
    return {
        "curvature": curvature,
        "sample_count": count,
        "distance_span_m": span,
        "heading_rmse_rad": rmse,
        "curvature_standard_error_rad_m": slope_standard_error,
    }


def equivalent_steering_angle(curvature_rad_m, wheelbase_m):
    if not math.isfinite(curvature_rad_m):
        raise ValueError("curvature must be finite")
    if not math.isfinite(wheelbase_m) or wheelbase_m <= 0.0:
        raise ValueError("wheelbase must be a finite positive value")
    return math.atan(wheelbase_m * curvature_rad_m)


def recommended_bias_value(
    current_bias_deg,
    equivalent_steer_deg,
    gain=1.0,
    negative_bias_corrects_left=True,
):
    values = (current_bias_deg, equivalent_steer_deg, gain)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("bias recommendation inputs must be finite")
    if gain < 0.0 or gain > 1.0:
        raise ValueError("recommendation gain must be between 0 and 1")
    # Field convention supplied by the operator: a negative bias produces a
    # leftward correction. With that convention the numeric correction has the
    # same sign as the observed ROS-yaw equivalent steering error.
    direction = 1.0 if negative_bias_corrects_left else -1.0
    return current_bias_deg + direction * gain * equivalent_steer_deg


def progress_watchdog_expired(
    motion_seen,
    now_monotonic,
    last_progress_monotonic,
    timeout_sec,
    enabled=True,
):
    values = (now_monotonic, last_progress_monotonic, timeout_sec)
    if not all(math.isfinite(value) for value in values) or timeout_sec <= 0.0:
        raise ValueError("progress watchdog inputs must be finite and positive")
    return bool(
        enabled
        and motion_seen
        and now_monotonic - last_progress_monotonic > timeout_sec
    )


def calibration_speed_limit(external_estop_override):
    return (
        MAX_EXTERNAL_ESTOP_CALIBRATION_SPEED_MPS
        if external_estop_override
        else MAX_CALIBRATION_SPEED_MPS
    )


def measured_speed_limit(external_estop_override):
    return (
        MAX_EXTERNAL_ESTOP_MEASURED_SPEED_MPS
        if external_estop_override
        else MAX_MEASURED_SPEED_MPS
    )


def updated_heading_rate_violation_count(
    previous_count, heading_rate_deg_s, max_heading_rate_deg_s
):
    values = (heading_rate_deg_s, max_heading_rate_deg_s)
    if type(previous_count) is not int or previous_count < 0:
        raise ValueError("heading-rate violation count must be a non-negative integer")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("heading-rate values must be finite")
    if max_heading_rate_deg_s <= 0.0:
        raise ValueError("maximum heading rate must be positive")
    if abs(heading_rate_deg_s) > max_heading_rate_deg_s:
        return previous_count + 1
    return 0


def progress_step_reached(gps_progress_m, wheel_progress_m, step_m):
    values = (gps_progress_m, wheel_progress_m, step_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("progress distances must be finite")
    if gps_progress_m < 0.0 or wheel_progress_m < 0.0 or step_m <= 0.0:
        raise ValueError("progress distances must be non-negative and step positive")
    return max(gps_progress_m, wheel_progress_m) >= step_m


def iso_timestamp():
    return datetime.datetime.now().astimezone().isoformat(timespec="milliseconds")


def strict_bool_param(name, default):
    value = rospy.get_param(name, default)
    if type(value) is not bool:
        raise ValueError("%s must be a YAML boolean true/false" % name)
    return value


def strict_int_param(name, default):
    value = rospy.get_param(name, default)
    if type(value) is not int:
        raise ValueError("%s must be a YAML integer" % name)
    return value


def raw_emergency_response_is_safe(msg_type, payload):
    if msg_type not in RAW_EMERGENCY_TYPES:
        raise ValueError("unsupported raw emergency response type")
    if not payload:
        raise ValueError("raw emergency response has no payload")
    value = int(payload[0])
    if msg_type == VCU_COMMON_STATE:
        return value == VCU_RUNNING_STATE, value
    return value == 0, value


class SteerCenterCalibration:
    CSV_FIELDS = [
        "recorded_at",
        "phase",
        "ros_stamp",
        "elapsed_sec",
        "x_m",
        "y_m",
        "ros_yaw_deg_left_positive",
        "gps_heading_deg_clockwise_from_north",
        "gps_heading_age_sec",
        "linear_x_mps",
        "driver_linear_x_mps",
        "driver_speed_age_sec",
        "angular_z_radps",
        "wheel_angle_rad",
        "wheel_angle_deg",
        "wheel_angle_age_sec",
        "command_linear_x_mps",
        "wheel_distance_m",
        "gps_accumulated_path_m",
        "forward_m",
        "lateral_m_left_positive",
    ]

    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/gps/odom")
        self.raw_heading_topic = rospy.get_param("~raw_heading_topic", "/gps/heading")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.ackermann_topic = rospy.get_param("~ackermann_topic", "/ackerman_vel")
        self.steer_bias_topic = rospy.get_param(
            "~steer_bias_topic", "/m2_driver/steer_center_bias"
        )
        self.wheel_angle_topic = rospy.get_param(
            "~wheel_angle_topic", "/m2_driver/wheel_angle"
        )
        self.driver_odom_topic = rospy.get_param("~driver_odom_topic", "/odom")
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
        self.expected_odom_frame = rospy.get_param("~expected_odom_frame", "camera_init")
        self.expected_child_frame = rospy.get_param("~expected_child_frame", "base_link")
        self.expected_driver_odom_frame = rospy.get_param(
            "~expected_driver_odom_frame", "odom"
        )
        self.expected_odom_publisher = rospy.get_param(
            "~expected_odom_publisher", "/gps_localization"
        )
        self.expected_driver_node = rospy.get_param(
            "~expected_driver_node", "/m2_driver"
        )
        self.expected_canbus_node = rospy.get_param(
            "~expected_canbus_node", "/canbus_driver"
        )

        self.allow_motion = strict_bool_param("~allow_motion", False)
        self.use_raw_can_emergency_check = strict_bool_param(
            "~use_raw_can_emergency_check", True
        )
        self.external_estop_override = strict_bool_param(
            "~external_estop_override", False
        )
        self.use_progress_watchdog = strict_bool_param(
            "~use_progress_watchdog", True
        )
        self.command_speed_limit_mps = calibration_speed_limit(
            self.external_estop_override
        )
        self.hard_measured_speed_limit_mps = measured_speed_limit(
            self.external_estop_override
        )
        self.speed_mps = float(rospy.get_param("~speed_mps", 0.20))
        self.distance_m = float(rospy.get_param("~distance_m", 5.0))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.baseline_duration_sec = float(rospy.get_param("~baseline_duration_sec", 3.0))
        self.end_settle_sec = float(rospy.get_param("~end_settle_sec", 2.0))
        self.stop_publish_sec = float(rospy.get_param("~stop_publish_sec", 1.0))
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 0.6))
        self.source_stamp_timeout_sec = float(
            rospy.get_param("~source_stamp_timeout_sec", 1.0)
        )
        default_timeout = self.distance_m / max(self.speed_mps, 0.01) + 15.0
        self.run_timeout_sec = float(rospy.get_param("~run_timeout_sec", default_timeout))
        self.no_motion_timeout_sec = float(rospy.get_param("~no_motion_timeout_sec", 4.0))
        self.no_progress_timeout_sec = float(
            rospy.get_param("~no_progress_timeout_sec", 3.0)
        )
        self.progress_step_m = float(rospy.get_param("~progress_step_m", 0.02))
        self.command_lease_sec = float(rospy.get_param("~command_lease_sec", 0.30))
        self.wheel_angle_timeout_sec = float(
            rospy.get_param("~wheel_angle_timeout_sec", 0.6)
        )
        self.max_initial_wheel_angle_deg = float(
            rospy.get_param("~max_initial_wheel_angle_deg", 2.0)
        )
        self.max_initial_wheel_angle_std_deg = float(
            rospy.get_param("~max_initial_wheel_angle_std_deg", 0.25)
        )
        self.max_runtime_wheel_angle_deg = float(
            rospy.get_param("~max_runtime_wheel_angle_deg", 3.0)
        )
        self.raw_emergency_timeout_sec = float(
            rospy.get_param("~raw_emergency_timeout_sec", 2.5)
        )
        self.raw_emergency_query_interval_sec = float(
            rospy.get_param("~raw_emergency_query_interval_sec", 0.2)
        )
        default_max_measured_speed = min(
            self.hard_measured_speed_limit_mps,
            max(0.35, self.speed_mps + 0.15),
        )
        self.max_measured_speed_mps = float(
            rospy.get_param("~max_measured_speed_mps", default_max_measured_speed)
        )
        self.max_pose_step_m = float(rospy.get_param("~max_pose_step_m", 0.25))
        self.max_heading_rate_deg_s = float(
            rospy.get_param("~max_heading_rate_deg_s", 20.0)
        )
        self.heading_rate_abort_count = strict_int_param(
            "~heading_rate_abort_count",
            3 if self.external_estop_override else 1,
        )
        self.max_heading_change_deg = float(
            rospy.get_param("~max_heading_change_deg", 10.0)
        )
        self.max_lateral_deviation_m = float(
            rospy.get_param("~max_lateral_deviation_m", 0.75)
        )
        self.max_baseline_heading_std_deg = float(
            rospy.get_param("~max_baseline_heading_std_deg", 0.5)
        )
        self.fit_trim_start_m = float(rospy.get_param("~fit_trim_start_m", 0.5))
        self.fit_trim_end_m = float(rospy.get_param("~fit_trim_end_m", 0.5))
        self.wheelbase_override_m = float(rospy.get_param("~wheelbase_m", 0.0))
        self.current_bias_was_explicit = rospy.has_param("~current_bias_deg")
        self.current_bias_deg = float(rospy.get_param("~current_bias_deg", 0.0))
        self.recommendation_gain = float(rospy.get_param("~recommendation_gain", 0.5))
        self.negative_bias_corrects_left = strict_bool_param(
            "~negative_bias_corrects_left", True
        )
        self.output_dir = os.path.abspath(
            os.path.expanduser(rospy.get_param("~output_dir", DEFAULT_OUTPUT_DIR))
        )

        self.validate_parameters()

        self.lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.latest_odom = None
        self.odom_sequence = 0
        self.odom_history = deque(maxlen=4000)
        self.invalid_odom_reason = "waiting for odometry"
        self.invalid_odom_monotonic = time.monotonic()
        self.raw_heading_history = deque(maxlen=1000)
        self.latest_wheel_angle = None
        self.latest_wheel_angle_monotonic = None
        self.wheel_angle_history = deque(maxlen=1000)
        self.latest_driver_speed = None
        self.driver_speed_history = deque(maxlen=2000)
        self.invalid_driver_odom_reason = "waiting for driver odometry"
        self.invalid_driver_odom_monotonic = time.monotonic()
        self.latest_chassis_status = None
        self.latest_chassis_status_monotonic = None
        self.raw_emergency_status = {}
        self.last_raw_emergency_query_monotonic = 0.0
        self.raw_emergency_query_index = 0
        self.control_timeout_seen = False
        self.command_linear_x = 0.0
        self.command_lease_deadline_monotonic = None
        self.command_absolute_deadline_monotonic = None
        self.command_expired_reason = ""
        self.motion_points = []
        self.master = rosgraph.Master(rospy.get_name())
        self.master_pid = self.master.getPid()
        self.last_graph_check_monotonic = 0.0
        self.csv_handle = None
        self.csv_writer = None
        self.csv_path = ""
        self.summary_path = ""
        self.run_zero_monotonic = time.monotonic()
        self.shutdown_started = False

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_cb, queue_size=100
        )
        self.heading_sub = rospy.Subscriber(
            self.raw_heading_topic, Float64, self.heading_cb, queue_size=100
        )
        self.wheel_angle_sub = rospy.Subscriber(
            self.wheel_angle_topic, Float64, self.wheel_angle_cb, queue_size=100
        )
        self.driver_odom_sub = rospy.Subscriber(
            self.driver_odom_topic, Odometry, self.driver_odom_cb, queue_size=100
        )
        self.chassis_status_sub = rospy.Subscriber(
            self.chassis_status_topic,
            ChassisStatusInfo,
            self.chassis_status_cb,
            queue_size=20,
        )
        self.control_timeout_sub = rospy.Subscriber(
            self.control_timeout_topic, Bool, self.control_timeout_cb, queue_size=20
        )
        self.canbus_sub = rospy.Subscriber(
            self.canbus_topic, CanBusMessage, self.raw_canbus_cb, queue_size=100
        )
        self.canbus_service_proxy = rospy.ServiceProxy(
            self.canbus_service, CanBusService, persistent=True
        )
        self.command_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self.publish_command
        )
        rospy.on_shutdown(self.shutdown)

    def validate_parameters(self):
        numeric_values = {
            "speed_mps": self.speed_mps,
            "distance_m": self.distance_m,
            "publish_rate_hz": self.publish_rate_hz,
            "baseline_duration_sec": self.baseline_duration_sec,
            "end_settle_sec": self.end_settle_sec,
            "stop_publish_sec": self.stop_publish_sec,
            "odom_timeout_sec": self.odom_timeout_sec,
            "source_stamp_timeout_sec": self.source_stamp_timeout_sec,
            "run_timeout_sec": self.run_timeout_sec,
            "no_motion_timeout_sec": self.no_motion_timeout_sec,
            "no_progress_timeout_sec": self.no_progress_timeout_sec,
            "progress_step_m": self.progress_step_m,
            "command_lease_sec": self.command_lease_sec,
            "wheel_angle_timeout_sec": self.wheel_angle_timeout_sec,
            "max_initial_wheel_angle_deg": self.max_initial_wheel_angle_deg,
            "max_initial_wheel_angle_std_deg": self.max_initial_wheel_angle_std_deg,
            "max_runtime_wheel_angle_deg": self.max_runtime_wheel_angle_deg,
            "raw_emergency_timeout_sec": self.raw_emergency_timeout_sec,
            "raw_emergency_query_interval_sec": self.raw_emergency_query_interval_sec,
            "max_measured_speed_mps": self.max_measured_speed_mps,
            "max_pose_step_m": self.max_pose_step_m,
            "max_heading_rate_deg_s": self.max_heading_rate_deg_s,
            "max_heading_change_deg": self.max_heading_change_deg,
            "max_lateral_deviation_m": self.max_lateral_deviation_m,
            "max_baseline_heading_std_deg": self.max_baseline_heading_std_deg,
            "fit_trim_start_m": self.fit_trim_start_m,
            "fit_trim_end_m": self.fit_trim_end_m,
            "wheelbase_m": self.wheelbase_override_m,
            "current_bias_deg": self.current_bias_deg,
            "recommendation_gain": self.recommendation_gain,
        }
        for name, value in numeric_values.items():
            if not math.isfinite(value):
                raise ValueError("%s must be finite" % name)
        if not self.use_progress_watchdog and not self.external_estop_override:
            raise ValueError(
                "use_progress_watchdog=false requires external_estop_override=true"
            )
        if self.speed_mps <= 0.0 or self.speed_mps > self.command_speed_limit_mps:
            raise ValueError(
                "speed_mps must be in (0, %.2f] for this safety mode"
                % self.command_speed_limit_mps
            )
        if not MIN_CALIBRATION_DISTANCE_M <= self.distance_m <= MAX_CALIBRATION_DISTANCE_M:
            raise ValueError(
                "distance_m must be between %.1f and %.1f"
                % (MIN_CALIBRATION_DISTANCE_M, MAX_CALIBRATION_DISTANCE_M)
            )
        if not 10.0 <= self.publish_rate_hz <= 50.0:
            raise ValueError("publish_rate_hz must be between 10 and 50 Hz")
        if self.baseline_duration_sec < 2.0:
            raise ValueError("baseline_duration_sec must be at least 2 seconds")
        if self.end_settle_sec < 1.0:
            raise ValueError("end_settle_sec must be at least 1 second")
        if self.stop_publish_sec < 1.0:
            raise ValueError("stop_publish_sec must be at least 1 second")
        if not 0.1 <= self.odom_timeout_sec <= 1.0:
            raise ValueError("odom_timeout_sec must be between 0.1 and 1.0")
        if not 0.1 <= self.source_stamp_timeout_sec <= 1.5:
            raise ValueError("source_stamp_timeout_sec must be between 0.1 and 1.5")
        if self.run_timeout_sec <= self.distance_m / self.speed_mps:
            raise ValueError("run_timeout_sec leaves no stopping/progress margin")
        if self.run_timeout_sec > self.distance_m / self.speed_mps + 20.0:
            raise ValueError("run_timeout_sec exceeds the hard maximum margin")
        if not 0.5 <= self.no_motion_timeout_sec <= 5.0:
            raise ValueError("no_motion_timeout_sec must be between 0.5 and 5.0")
        if not 0.5 <= self.no_progress_timeout_sec <= 3.0:
            raise ValueError("no_progress_timeout_sec must be between 0.5 and 3.0")
        if not 0.01 <= self.progress_step_m <= 0.25:
            raise ValueError("progress_step_m must be between 0.01 and 0.25 m")
        minimum_lease = 3.0 / self.publish_rate_hz
        if not minimum_lease <= self.command_lease_sec <= 0.5:
            raise ValueError(
                "command_lease_sec must be between %.3f and 0.5 seconds"
                % minimum_lease
            )
        if not 0.1 <= self.wheel_angle_timeout_sec <= 1.0:
            raise ValueError("wheel_angle_timeout_sec must be between 0.1 and 1.0")
        if not 0.1 <= self.max_initial_wheel_angle_deg <= 3.0:
            raise ValueError("max_initial_wheel_angle_deg must be between 0.1 and 3")
        if not 0.01 <= self.max_initial_wheel_angle_std_deg <= 1.0:
            raise ValueError(
                "max_initial_wheel_angle_std_deg must be between 0.01 and 1"
            )
        if not 0.5 <= self.max_runtime_wheel_angle_deg <= 5.0:
            raise ValueError("max_runtime_wheel_angle_deg must be between 0.5 and 5")
        if not 1.0 <= self.raw_emergency_timeout_sec <= 5.0:
            raise ValueError("raw_emergency_timeout_sec must be between 1.0 and 5.0")
        if not 0.1 <= self.raw_emergency_query_interval_sec <= 0.5:
            raise ValueError(
                "raw_emergency_query_interval_sec must be between 0.1 and 0.5"
            )
        minimum_raw_emergency_timeout = (
            2.0
            * len(RAW_EMERGENCY_QUERY_ORDER)
            * self.raw_emergency_query_interval_sec
        )
        if self.raw_emergency_timeout_sec < minimum_raw_emergency_timeout:
            raise ValueError(
                "raw emergency timeout must cover at least two complete query rounds"
            )
        if not (
            self.speed_mps
            < self.max_measured_speed_mps
            <= self.hard_measured_speed_limit_mps
        ):
            raise ValueError(
                "max_measured_speed_mps must be above speed_mps and at most %.2f"
                % self.hard_measured_speed_limit_mps
            )
        if not 0.01 <= self.max_pose_step_m <= 0.5:
            raise ValueError("max_pose_step_m must be between 0.01 and 0.5 m")
        if not 1.0 <= self.max_heading_rate_deg_s <= 30.0:
            raise ValueError("max_heading_rate_deg_s must be between 1 and 30")
        if not 1 <= self.heading_rate_abort_count <= 10:
            raise ValueError("heading_rate_abort_count must be between 1 and 10")
        if not 1.0 <= self.max_heading_change_deg <= 15.0:
            raise ValueError("max_heading_change_deg must be between 1 and 15")
        if not 0.1 <= self.max_lateral_deviation_m <= 1.0:
            raise ValueError("max_lateral_deviation_m must be between 0.1 and 1.0")
        if not 0.05 <= self.max_baseline_heading_std_deg <= 1.0:
            raise ValueError(
                "max_baseline_heading_std_deg must be between 0.05 and 1.0"
            )
        if self.fit_trim_start_m < 0.0 or self.fit_trim_end_m < 0.0:
            raise ValueError("fit trim distances cannot be negative")
        if self.fit_trim_start_m + self.fit_trim_end_m + 1.0 > self.distance_m:
            raise ValueError("fit trims leave less than 1 m of steady data")
        if self.wheelbase_override_m < 0.0:
            raise ValueError("wheelbase_m cannot be negative")
        if not 0.0 <= self.recommendation_gain <= 1.0:
            raise ValueError("recommendation_gain must be between 0 and 1")

    def odom_cb(self, msg):
        now_monotonic = time.monotonic()
        try:
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            values = (
                position.x,
                position.y,
                msg.twist.twist.linear.x,
                msg.twist.twist.angular.z,
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError("position or twist contains a non-finite value")
            if msg.header.frame_id != self.expected_odom_frame:
                raise ValueError(
                    "unexpected odom frame %r (expected %r)"
                    % (msg.header.frame_id, self.expected_odom_frame)
                )
            if msg.child_frame_id != self.expected_child_frame:
                raise ValueError(
                    "unexpected child frame %r (expected %r)"
                    % (msg.child_frame_id, self.expected_child_frame)
                )
            stamp_sec = msg.header.stamp.to_sec()
            if stamp_sec <= 0.0 or not math.isfinite(stamp_sec):
                raise ValueError("odometry source stamp is missing or invalid")
            yaw = quaternion_to_yaw(
                orientation.x, orientation.y, orientation.z, orientation.w
            )
        except ValueError as exc:
            with self.lock:
                self.invalid_odom_reason = str(exc)
                self.invalid_odom_monotonic = now_monotonic
            return

        with self.lock:
            self.odom_sequence += 1
            state = OdomState(
                sequence=self.odom_sequence,
                receipt_monotonic=now_monotonic,
                stamp_sec=stamp_sec,
                x=position.x,
                y=position.y,
                yaw=yaw,
                linear_x=msg.twist.twist.linear.x,
                angular_z=msg.twist.twist.angular.z,
                frame_id=msg.header.frame_id,
                child_frame_id=msg.child_frame_id,
            )
            self.latest_odom = state
            self.odom_history.append(state)
            self.invalid_odom_reason = ""

    def heading_cb(self, msg):
        if math.isfinite(msg.data):
            with self.lock:
                self.raw_heading_history.append((time.monotonic(), msg.data % 360.0))

    def wheel_angle_cb(self, msg):
        if math.isfinite(msg.data):
            receipt = time.monotonic()
            with self.lock:
                self.latest_wheel_angle = msg.data
                self.latest_wheel_angle_monotonic = receipt
                self.wheel_angle_history.append((receipt, msg.data))

    def driver_odom_cb(self, msg):
        receipt = time.monotonic()
        try:
            speed = float(msg.twist.twist.linear.x)
            stamp_sec = msg.header.stamp.to_sec()
            if not math.isfinite(speed):
                raise ValueError("driver speed is not finite")
            if stamp_sec <= 0.0 or not math.isfinite(stamp_sec):
                raise ValueError("driver odometry stamp is invalid")
            if msg.header.frame_id != self.expected_driver_odom_frame:
                raise ValueError(
                    "unexpected driver odom frame %r" % msg.header.frame_id
                )
            if msg.child_frame_id != self.expected_child_frame:
                raise ValueError(
                    "unexpected driver odom child frame %r" % msg.child_frame_id
                )
        except ValueError as exc:
            with self.lock:
                self.invalid_driver_odom_reason = str(exc)
                self.invalid_driver_odom_monotonic = receipt
            return
        sample = DriverSpeedState(receipt, stamp_sec, speed)
        with self.lock:
            self.latest_driver_speed = sample
            self.driver_speed_history.append(sample)
            self.invalid_driver_odom_reason = ""

    def chassis_status_cb(self, msg):
        with self.lock:
            self.latest_chassis_status = msg
            self.latest_chassis_status_monotonic = time.monotonic()

    def raw_canbus_cb(self, msg):
        if msg.node_type != VCU_NODE_TYPE or msg.node_seq != VCU_NODE_ID:
            return
        if msg.msg_type not in RAW_EMERGENCY_TYPES:
            return
        receipt = time.monotonic()
        try:
            is_safe, raw_value = raw_emergency_response_is_safe(
                msg.msg_type, msg.payload
            )
            error = ""
        except ValueError as exc:
            is_safe = False
            raw_value = None
            error = str(exc)
        with self.lock:
            self.raw_emergency_status[msg.msg_type] = (
                receipt,
                is_safe,
                raw_value,
                error,
            )

    def control_timeout_cb(self, msg):
        if msg.data:
            with self.lock:
                self.control_timeout_seen = True

    def set_command(self, linear_x, absolute_deadline_monotonic=None):
        """Set zero, or arm a nonzero command with two independent deadlines."""
        now = time.monotonic()
        with self.command_lock:
            if abs(linear_x) <= 1e-12:
                self.command_linear_x = 0.0
                self.command_lease_deadline_monotonic = None
                self.command_absolute_deadline_monotonic = None
                return
            if absolute_deadline_monotonic is None or absolute_deadline_monotonic <= now:
                raise CalibrationAbort("非零速度命令缺少有效的绝对截止时间")
            self.command_linear_x = linear_x
            self.command_lease_deadline_monotonic = min(
                now + self.command_lease_sec, absolute_deadline_monotonic
            )
            self.command_absolute_deadline_monotonic = absolute_deadline_monotonic
            self.command_expired_reason = ""

    def renew_motion_lease(self):
        """Renew only from the healthy main loop; never resurrect an expired command."""
        now = time.monotonic()
        with self.command_lock:
            if self.command_expired_reason:
                raise CalibrationAbort(self.command_expired_reason)
            if abs(self.command_linear_x) <= 1e-12:
                raise CalibrationAbort("速度命令已被安全租约切为零")
            if (
                self.command_absolute_deadline_monotonic is None
                or now >= self.command_absolute_deadline_monotonic
            ):
                self.command_linear_x = 0.0
                self.command_expired_reason = "速度命令达到绝对运行截止时间"
                raise CalibrationAbort(self.command_expired_reason)
            if (
                self.command_lease_deadline_monotonic is None
                or now >= self.command_lease_deadline_monotonic
            ):
                self.command_linear_x = 0.0
                self.command_expired_reason = "主循环心跳超时，速度命令已自动切为零"
                raise CalibrationAbort(self.command_expired_reason)
            self.command_lease_deadline_monotonic = min(
                now + self.command_lease_sec,
                self.command_absolute_deadline_monotonic,
            )

    def complete_motion_command(self):
        """Atomically accept a live run as complete and switch it to zero."""
        now = time.monotonic()
        with self.command_lock:
            if self.command_expired_reason:
                raise CalibrationAbort(self.command_expired_reason)
            if abs(self.command_linear_x) <= 1e-12:
                raise CalibrationAbort("速度命令已在到达目标前被切为零")
            if (
                self.command_lease_deadline_monotonic is None
                or self.command_absolute_deadline_monotonic is None
                or now >= self.command_lease_deadline_monotonic
                or now >= self.command_absolute_deadline_monotonic
            ):
                self.command_linear_x = 0.0
                self.command_expired_reason = "到达距离条件时速度安全租约已过期"
                raise CalibrationAbort(self.command_expired_reason)
            self.command_linear_x = 0.0
            self.command_lease_deadline_monotonic = None
            self.command_absolute_deadline_monotonic = None

    def publish_command(self, _event=None):
        expired_reason = ""
        now = time.monotonic()
        with self.command_lock:
            if abs(self.command_linear_x) > 1e-12:
                if (
                    self.command_absolute_deadline_monotonic is None
                    or now >= self.command_absolute_deadline_monotonic
                ):
                    expired_reason = "速度命令达到绝对运行截止时间"
                elif (
                    self.command_lease_deadline_monotonic is None
                    or now >= self.command_lease_deadline_monotonic
                ):
                    expired_reason = "主循环心跳超时，速度命令已自动切为零"
                if expired_reason:
                    self.command_linear_x = 0.0
                    self.command_expired_reason = expired_reason
            linear_x = self.command_linear_x
            command = Twist()
            command.linear.x = linear_x
            command.angular.z = 0.0
            # Keep publication serialized with every command transition so a
            # previously sampled positive command cannot overtake a stop.
            self.cmd_pub.publish(command)
        if expired_reason:
            rospy.logerr("%s", expired_reason)

    def stop_burst(self):
        self.set_command(0.0)
        deadline = time.monotonic() + max(0.0, self.stop_publish_sec)
        period = 1.0 / self.publish_rate_hz
        while time.monotonic() < deadline:
            try:
                self.publish_command()
            except Exception:
                pass
            time.sleep(period)

    def shutdown(self):
        if self.shutdown_started:
            return
        self.shutdown_started = True
        self.stop_burst()

    def latest_state(self):
        with self.lock:
            return self.latest_odom

    def states_after(self, sequence):
        with self.lock:
            return [state for state in self.odom_history if state.sequence > sequence]

    def auxiliary_values_at(self, receipt_monotonic):
        with self.lock:
            raw_candidates = [
                (receipt, value)
                for receipt, value in self.raw_heading_history
                if receipt <= receipt_monotonic
            ]
            wheel_candidates = [
                (receipt, value)
                for receipt, value in self.wheel_angle_history
                if receipt <= receipt_monotonic
            ]
        raw = raw_candidates[-1] if raw_candidates else None
        wheel = wheel_candidates[-1] if wheel_candidates else None
        return (
            None if raw is None else raw[1],
            None if raw is None else receipt_monotonic - raw[0],
            None if wheel is None else wheel[1],
            None if wheel is None else receipt_monotonic - wheel[0],
        )

    def check_odom_fresh(self):
        with self.lock:
            state = self.latest_odom
            invalid_reason = self.invalid_odom_reason
            invalid_time = self.invalid_odom_monotonic
        now_monotonic = time.monotonic()
        if state is None:
            raise CalibrationAbort("没有收到有效的 %s：%s" % (self.odom_topic, invalid_reason))
        if invalid_reason and invalid_time >= state.receipt_monotonic:
            raise CalibrationAbort("收到无效 GPS odom：%s" % invalid_reason)
        receipt_age = now_monotonic - state.receipt_monotonic
        if receipt_age > self.odom_timeout_sec:
            raise CalibrationAbort(
                "%s 接收超时 %.3fs" % (self.odom_topic, receipt_age)
            )
        source_age = rospy.Time.now().to_sec() - state.stamp_sec
        if source_age < -0.2 or source_age > self.source_stamp_timeout_sec:
            raise CalibrationAbort(
                "%s 源时间戳异常，age=%.3fs" % (self.odom_topic, source_age)
            )
        return state

    def check_chassis_status(self, require_no_control_timeout=True):
        with self.lock:
            status = self.latest_chassis_status
            receipt = self.latest_chassis_status_monotonic
            timeout_seen = self.control_timeout_seen
        if status is None or receipt is None:
            raise CalibrationAbort("没有收到 %s" % self.chassis_status_topic)
        if time.monotonic() - receipt > 3.0:
            raise CalibrationAbort("底盘状态超过 3s 未更新")
        if (
            status.hard_emergency
            or status.soft_emergency
            or status.gamepad_emergency
            or status.robot_emergency
        ):
            raise CalibrationAbort(
                "底盘急停未释放：hard=%s soft=%s gamepad=%s robot=%s"
                % (
                    status.hard_emergency,
                    status.soft_emergency,
                    status.gamepad_emergency,
                    status.robot_emergency,
                )
            )
        if require_no_control_timeout and timeout_seen:
            raise CalibrationAbort("VCU 报告了 200ms 控制命令超时")

    def check_wheel_angle_fresh(self):
        with self.lock:
            wheel_angle = self.latest_wheel_angle
            receipt = self.latest_wheel_angle_monotonic
        if wheel_angle is None or receipt is None:
            raise CalibrationAbort("没有收到 %s" % self.wheel_angle_topic)
        age = time.monotonic() - receipt
        if age > self.wheel_angle_timeout_sec:
            raise CalibrationAbort(
                "%s 超时 %.3fs" % (self.wheel_angle_topic, age)
            )
        if not math.isfinite(wheel_angle):
            raise CalibrationAbort("前轮转角反馈不是有限值")
        if abs(math.degrees(wheel_angle)) > self.max_runtime_wheel_angle_deg:
            raise CalibrationAbort(
                "前轮转角反馈超限：%+.3fdeg"
                % math.degrees(wheel_angle)
            )
        return wheel_angle

    def check_driver_speed_fresh(self):
        with self.lock:
            sample = self.latest_driver_speed
            invalid_reason = self.invalid_driver_odom_reason
            invalid_time = self.invalid_driver_odom_monotonic
        if sample is None:
            raise CalibrationAbort(
                "没有收到有效的 %s：%s"
                % (self.driver_odom_topic, invalid_reason)
            )
        if invalid_reason and invalid_time >= sample.receipt_monotonic:
            raise CalibrationAbort("收到无效底盘 odom：%s" % invalid_reason)
        receipt_age = time.monotonic() - sample.receipt_monotonic
        if receipt_age > self.odom_timeout_sec:
            raise CalibrationAbort(
                "%s 接收超时 %.3fs" % (self.driver_odom_topic, receipt_age)
            )
        source_age = rospy.Time.now().to_sec() - sample.stamp_sec
        if source_age < -0.2 or source_age > self.source_stamp_timeout_sec:
            raise CalibrationAbort(
                "%s 源时间戳异常，age=%.3fs"
                % (self.driver_odom_topic, source_age)
            )
        return sample

    def query_and_check_raw_emergency_status(self):
        if self.external_estop_override or not self.use_raw_can_emergency_check:
            return

        now = time.monotonic()
        if (
            now - self.last_raw_emergency_query_monotonic
            >= self.raw_emergency_query_interval_sec
        ):
            # The serial CAN bridge writes every message in one service request
            # back-to-back.  The VCU can drop replies when all four status
            # queries arrive as a burst, so issue one request at a time and
            # rotate through the four safety states.  This also leaves room for
            # the driver's own status and motion-control traffic.
            msg_type = RAW_EMERGENCY_QUERY_ORDER[
                self.raw_emergency_query_index
            ]
            request = CanBusMessage()
            request.node_type = VCU_NODE_TYPE
            request.node_seq = VCU_NODE_ID
            request.msg_type = msg_type
            request.payload = []
            try:
                self.canbus_service_proxy([request])
            except (rospy.ROSException, rospy.ServiceException) as exc:
                raise CalibrationAbort("查询底层 CAN 急停状态失败：%s" % exc)
            self.raw_emergency_query_index = (
                self.raw_emergency_query_index + 1
            ) % len(RAW_EMERGENCY_QUERY_ORDER)
            self.last_raw_emergency_query_monotonic = now

        with self.lock:
            statuses = dict(self.raw_emergency_status)
        now = time.monotonic()
        for msg_type, label in RAW_EMERGENCY_TYPES.items():
            status = statuses.get(msg_type)
            if status is None:
                raise CalibrationAbort("尚未收到%s的原始 CAN 响应" % label)
            receipt, is_safe, raw_value, error = status
            age = now - receipt
            if age > self.raw_emergency_timeout_sec:
                raise CalibrationAbort(
                    "%s原始 CAN 响应已过期 %.3fs" % (label, age)
                )
            if error:
                raise CalibrationAbort("%s原始 CAN 响应无效：%s" % (label, error))
            if not is_safe:
                raise CalibrationAbort(
                    "%s未释放，原始值=0x%02X" % (label, raw_value)
                )

    def driver_speed_at(self, receipt_monotonic):
        with self.lock:
            candidates = [
                sample
                for sample in self.driver_speed_history
                if sample.receipt_monotonic <= receipt_monotonic
            ]
        if not candidates:
            raise CalibrationAbort("GPS 样本之前没有可配对的底盘轮速")
        sample = candidates[-1]
        age = receipt_monotonic - sample.receipt_monotonic
        if age > self.odom_timeout_sec:
            raise CalibrationAbort("GPS 样本配对的底盘轮速已过期 %.3fs" % age)
        return sample.linear_x, age

    @staticmethod
    def topic_nodes(entries, topic):
        for candidate, nodes in entries:
            if candidate == topic:
                return set(nodes)
        return set()

    def check_command_graph(self, force=False):
        now = time.monotonic()
        if not force and now - self.last_graph_check_monotonic < 0.5:
            return
        if self.master.getPid() != self.master_pid:
            raise CalibrationAbort("ROS master 已被替换")
        if rospy.get_param("/use_sim_time", False) is not False:
            raise CalibrationAbort("真车标定要求 /use_sim_time=false")
        publishers, subscribers, services = self.master.getSystemState()
        canbus_publishers = self.topic_nodes(publishers, self.canbus_topic)
        if canbus_publishers != {self.expected_canbus_node}:
            raise CalibrationAbort(
                "%s 发布者必须且只能是 %s，当前：%s"
                % (
                    self.canbus_topic,
                    self.expected_canbus_node,
                    ", ".join(sorted(canbus_publishers)) or "无",
                )
            )
        canbus_service_nodes = self.topic_nodes(services, self.canbus_service)
        if canbus_service_nodes != {self.expected_canbus_node}:
            raise CalibrationAbort(
                "%s 服务必须且只能由 %s 提供，当前：%s"
                % (
                    self.canbus_service,
                    self.expected_canbus_node,
                    ", ".join(sorted(canbus_service_nodes)) or "无",
                )
            )
        cmd_publishers = self.topic_nodes(publishers, self.cmd_topic)
        other_publishers = cmd_publishers - {rospy.get_name()}
        if other_publishers:
            raise CalibrationAbort(
                "%s 存在其他发布者，拒绝抢占控制：%s"
                % (self.cmd_topic, ", ".join(sorted(other_publishers)))
            )
        ackermann_publishers = self.topic_nodes(publishers, self.ackermann_topic)
        if ackermann_publishers:
            raise CalibrationAbort(
                "%s 存在发布者，可能与直行命令冲突：%s"
                % (self.ackermann_topic, ", ".join(sorted(ackermann_publishers)))
            )
        bias_publishers = self.topic_nodes(publishers, self.steer_bias_topic)
        if bias_publishers:
            raise CalibrationAbort(
                "%s 存在发布者，标定期间偏置可能变化：%s"
                % (self.steer_bias_topic, ", ".join(sorted(bias_publishers)))
            )
        odom_publishers = self.topic_nodes(publishers, self.odom_topic)
        if odom_publishers != {self.expected_odom_publisher}:
            raise CalibrationAbort(
                "%s 发布者必须且只能是 %s，当前：%s"
                % (
                    self.odom_topic,
                    self.expected_odom_publisher,
                    ", ".join(sorted(odom_publishers)) or "无",
                )
            )
        driver_odom_publishers = self.topic_nodes(
            publishers, self.driver_odom_topic
        )
        if driver_odom_publishers != {self.expected_driver_node}:
            raise CalibrationAbort(
                "%s 发布者必须且只能是 %s，当前：%s"
                % (
                    self.driver_odom_topic,
                    self.expected_driver_node,
                    ", ".join(sorted(driver_odom_publishers)) or "无",
                )
            )
        heading_publishers = self.topic_nodes(publishers, self.raw_heading_topic)
        if heading_publishers != {self.expected_odom_publisher}:
            raise CalibrationAbort(
                "%s 发布者必须且只能是 %s，当前：%s"
                % (
                    self.raw_heading_topic,
                    self.expected_odom_publisher,
                    ", ".join(sorted(heading_publishers)) or "无",
                )
            )
        gps_heading_source_param = self.expected_odom_publisher + "/heading_source"
        heading_source = rospy.get_param(gps_heading_source_param, None)
        if heading_source not in ("dual_antenna", "uniheading", "heading"):
            raise CalibrationAbort(
                "%s 必须使用双天线航向，当前=%r"
                % (gps_heading_source_param, heading_source)
            )
        gps_use_wheel_odom_param = self.expected_odom_publisher + "/use_wheel_odom"
        if rospy.get_param(gps_use_wheel_odom_param, None) is not False:
            raise CalibrationAbort(
                "%s 必须为 false，位置测量才能独立于底盘里程计"
                % gps_use_wheel_odom_param
            )
        gps_use_wheel_twist_param = self.expected_odom_publisher + "/use_wheel_twist"
        if rospy.get_param(gps_use_wheel_twist_param, None) is not True:
            raise CalibrationAbort(
                "%s 必须为 true，距离和实测速度才能使用底盘轮速"
                % gps_use_wheel_twist_param
            )
        if not self.external_estop_override:
            timeout_publishers = self.topic_nodes(
                publishers, self.control_timeout_topic
            )
            if timeout_publishers != {self.expected_driver_node}:
                raise CalibrationAbort(
                    "%s 发布者必须且只能是 %s，当前：%s"
                    % (
                        self.control_timeout_topic,
                        self.expected_driver_node,
                        ", ".join(sorted(timeout_publishers)) or "无",
                    )
                )
            timeout_param = self.expected_driver_node + "/is_pub_control_timeout"
            if rospy.get_param(timeout_param, None) is not True:
                raise CalibrationAbort("底盘参数 %s 必须为 true" % timeout_param)
        wheel_publishers = self.topic_nodes(publishers, self.wheel_angle_topic)
        if wheel_publishers != {self.expected_driver_node}:
            raise CalibrationAbort(
                "%s 发布者必须且只能是 %s，当前：%s"
                % (
                    self.wheel_angle_topic,
                    self.expected_driver_node,
                    ", ".join(sorted(wheel_publishers)) or "无",
                )
            )
        if not self.external_estop_override:
            chassis_publishers = self.topic_nodes(
                publishers, self.chassis_status_topic
            )
            if chassis_publishers != {self.expected_driver_node}:
                raise CalibrationAbort(
                    "%s 发布者必须且只能是 %s，当前：%s"
                    % (
                        self.chassis_status_topic,
                        self.expected_driver_node,
                        ", ".join(sorted(chassis_publishers)) or "无",
                    )
                )
        parameter_service_nodes = self.topic_nodes(
            services, self.chassis_parameter_service
        )
        if parameter_service_nodes != {self.expected_driver_node}:
            raise CalibrationAbort(
                "%s 服务必须且只能由 %s 提供，当前：%s"
                % (
                    self.chassis_parameter_service,
                    self.expected_driver_node,
                    ", ".join(sorted(parameter_service_nodes)) or "无",
                )
            )
        cmd_subscribers = self.topic_nodes(subscribers, self.cmd_topic)
        if self.expected_driver_node not in cmd_subscribers:
            raise CalibrationAbort(
                "%s 没有连接 %s，当前订阅者：%s"
                % (
                    self.cmd_topic,
                    self.expected_driver_node,
                    ", ".join(sorted(cmd_subscribers)) or "无",
                )
            )
        if self.cmd_pub.get_num_connections() < 1:
            raise CalibrationAbort("标定节点尚未与 /m2_driver 建立 TCPROS 连接")
        self.last_graph_check_monotonic = now

    def check_runtime_health(self, force_graph=False, require_no_control_timeout=True):
        if rospy.is_shutdown():
            raise CalibrationAbort("ROS 正在关闭")
        self.check_odom_fresh()
        self.check_driver_speed_fresh()
        self.check_wheel_angle_fresh()
        if not self.external_estop_override:
            self.query_and_check_raw_emergency_status()
            self.check_chassis_status(
                require_no_control_timeout=require_no_control_timeout
            )
        self.check_command_graph(force=force_graph)

    def wait_for_preflight(self, timeout_sec=15.0):
        deadline = time.monotonic() + timeout_sec
        last_error = ""
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                # A timeout may have been reported while this node was still
                # connecting. The command timer is already publishing zero;
                # require a quiet baseline after operator confirmation instead
                # of permanently latching a preflight startup event.
                self.check_runtime_health(
                    force_graph=True, require_no_control_timeout=False
                )
                return
            except CalibrationAbort as exc:
                last_error = str(exc)
                rospy.loginfo_throttle(1.0, "标定预检等待中：%s", last_error)
            rate.sleep()
        raise CalibrationAbort("标定预检超时：%s" % (last_error or "未知原因"))

    def revalidate_after_operator_confirmation(
        self, timeout_sec=15.0, quiet_sec=0.5
    ):
        """Refresh every motion prerequisite after the blocking START prompt.

        The operator is intentionally allowed to spend as long as necessary at
        the confirmation prompt.  In the default safety mode, raw CAN emergency
        responses collected before that wait must therefore never authorize
        motion afterwards.  Keep the command at zero, refresh every enabled
        prerequisite, then require a short stable interval before baseline
        collection can begin.
        """
        if timeout_sec <= 0.0 or quiet_sec < 0.0:
            raise ValueError("post-confirmation health intervals are invalid")

        self.set_command(0.0)
        with self.lock:
            self.raw_emergency_status.clear()
            # A timeout reported while the node was connecting or while the
            # operator was reading the prompt is not allowed to authorize
            # motion, but neither should it permanently poison the new zero-
            # command validation interval below.
            self.control_timeout_seen = False
        self.last_raw_emergency_query_monotonic = 0.0
        self.raw_emergency_query_index = 0

        if self.external_estop_override:
            rospy.logwarn(
                "已收到 START；外部遥控急停覆盖已启用，跳过 CAN/VCU "
                "安全状态门禁，仅重新检查 GPS、轮角、速度和 ROS 控制链"
            )
        elif self.use_raw_can_emergency_check:
            rospy.loginfo(
                "已收到 START；保持零速并重新获取 GPS、底盘和四项 CAN 急停状态"
            )
        else:
            rospy.logwarn(
                "已收到 START；原始 CAN 急停轮询已由操作者显式关闭，"
                "仅重新检查 GPS、底盘聚合状态和 ROS 控制链"
            )
        self.wait_for_preflight(timeout_sec=timeout_sec)

        # The preflight wait deliberately tolerates connection-time command
        # timeout reports.  Start a new strict observation window only after
        # all other inputs, including newly queried raw CAN responses, are
        # current.
        with self.lock:
            self.control_timeout_seen = False
        self.check_runtime_health(force_graph=True)

        quiet_deadline = time.monotonic() + quiet_sec
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and time.monotonic() < quiet_deadline:
            self.check_runtime_health()
            rate.sleep()
        if rospy.is_shutdown():
            raise CalibrationAbort("ROS 在确认后安全复检期间关闭")

        rospy.loginfo("确认后安全复检通过，即将采集静止航向基线")

    def read_wheelbase(self):
        if self.wheelbase_override_m > 0.0:
            if not 0.2 <= self.wheelbase_override_m <= 3.0:
                raise CalibrationAbort("显式 wheelbase_m 超出合理范围")
            rospy.logwarn(
                "使用显式轴距 wheelbase_m=%.4fm；请确认它与底盘参数一致",
                self.wheelbase_override_m,
            )
            return self.wheelbase_override_m, "ros_parameter_override"
        try:
            rospy.wait_for_service(self.chassis_parameter_service, timeout=8.0)
            response = rospy.ServiceProxy(
                self.chassis_parameter_service, ChassisParameterServer
            )()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise CalibrationAbort("读取底盘轴距失败：%s" % exc)
        wheelbase = float(response.parameters.robot_length)
        if not response.success or not math.isfinite(wheelbase) or not 0.2 <= wheelbase <= 3.0:
            raise CalibrationAbort(
                "底盘返回的 robot_length/轴距无效：success=%s value=%r message=%s"
                % (response.success, wheelbase, response.message)
            )
        return wheelbase, "m2_driver_chassis_parameter.robot_length"

    def prepare_output(self):
        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        base = os.path.join(self.output_dir, "steer_center_calibration_" + stamp)
        self.csv_path = base + ".csv"
        self.summary_path = base + "_summary.json"
        self.csv_handle = open(self.csv_path, "w", encoding="utf-8", newline="")
        self.csv_writer = csv.DictWriter(self.csv_handle, fieldnames=self.CSV_FIELDS)
        self.csv_writer.writeheader()
        self.csv_handle.flush()

    def write_sample(
        self,
        phase,
        state,
        path_m="",
        gps_path_m="",
        forward_m="",
        lateral_m="",
    ):
        (
            raw_heading,
            raw_heading_age,
            wheel_angle,
            wheel_angle_age,
        ) = self.auxiliary_values_at(state.receipt_monotonic)
        driver_speed, driver_speed_age = self.driver_speed_at(
            state.receipt_monotonic
        )
        with self.command_lock:
            command_linear_x = self.command_linear_x
        row = {
            "recorded_at": iso_timestamp(),
            "phase": phase,
            "ros_stamp": "%.9f" % state.stamp_sec,
            "elapsed_sec": "%.3f" % (state.receipt_monotonic - self.run_zero_monotonic),
            "x_m": "%.6f" % state.x,
            "y_m": "%.6f" % state.y,
            "ros_yaw_deg_left_positive": "%.6f" % math.degrees(state.yaw),
            "gps_heading_deg_clockwise_from_north": (
                "" if raw_heading is None else "%.6f" % raw_heading
            ),
            "gps_heading_age_sec": (
                "" if raw_heading_age is None else "%.6f" % raw_heading_age
            ),
            "linear_x_mps": "%.6f" % state.linear_x,
            "driver_linear_x_mps": "%.6f" % driver_speed,
            "driver_speed_age_sec": "%.6f" % driver_speed_age,
            "angular_z_radps": "%.6f" % state.angular_z,
            "wheel_angle_rad": "" if wheel_angle is None else "%.6f" % wheel_angle,
            "wheel_angle_deg": (
                "" if wheel_angle is None else "%.6f" % math.degrees(wheel_angle)
            ),
            "wheel_angle_age_sec": (
                "" if wheel_angle_age is None else "%.6f" % wheel_angle_age
            ),
            "command_linear_x_mps": "%.6f" % command_linear_x,
            "wheel_distance_m": "" if path_m == "" else "%.6f" % path_m,
            "gps_accumulated_path_m": (
                "" if gps_path_m == "" else "%.6f" % gps_path_m
            ),
            "forward_m": "" if forward_m == "" else "%.6f" % forward_m,
            "lateral_m_left_positive": (
                "" if lateral_m == "" else "%.6f" % lateral_m
            ),
        }
        self.csv_writer.writerow(row)
        self.csv_handle.flush()

    def collect_baseline(self):
        rospy.loginfo(
            "保持车辆静止并连续发送零速，采集 %.1fs 双天线航向基线",
            self.baseline_duration_sec,
        )
        start = time.monotonic()
        last_sequence = self.latest_state().sequence
        samples = []
        rate = rospy.Rate(20.0)
        while time.monotonic() - start < self.baseline_duration_sec:
            self.check_runtime_health()
            for state in self.states_after(last_sequence):
                samples.append(state)
                last_sequence = state.sequence
                self.write_sample("baseline", state)
            rate.sleep()
        if len(samples) < int(self.baseline_duration_sec * 5.0):
            raise CalibrationAbort("基线 /gps/odom 更新率低于 5Hz")
        baseline_driver_speeds = [
            self.driver_speed_at(state.receipt_monotonic)[0] for state in samples
        ]
        if max(abs(speed) for speed in baseline_driver_speeds) > 0.05:
            raise CalibrationAbort("基线采集期间车辆并非静止")
        yaw_mean = circular_mean(state.yaw for state in samples)
        yaw_std = circular_stddev(state.yaw for state in samples)
        if math.degrees(yaw_std) > self.max_baseline_heading_std_deg:
            raise CalibrationAbort(
                "静止航向不稳定：std=%.3fdeg > %.3fdeg"
                % (math.degrees(yaw_std), self.max_baseline_heading_std_deg)
            )
        start_x = sum(state.x for state in samples) / len(samples)
        start_y = sum(state.y for state in samples) / len(samples)
        raw_values = []
        wheel_values = []
        with self.lock:
            raw_values = [
                value for receipt, value in self.raw_heading_history if receipt >= start
            ]
            wheel_values = [
                value for receipt, value in self.wheel_angle_history if receipt >= start
            ]
        raw_heading_mean = None
        if raw_values:
            raw_heading_mean = math.degrees(
                circular_mean(math.radians(value) for value in raw_values)
            ) % 360.0
        if len(wheel_values) < 3:
            raise CalibrationAbort("基线期间前轮转角反馈样本不足")
        wheel_angle_mean = sum(wheel_values) / len(wheel_values)
        wheel_angle_std = math.sqrt(
            sum((value - wheel_angle_mean) ** 2 for value in wheel_values)
            / len(wheel_values)
        )
        if max(abs(math.degrees(value)) for value in wheel_values) > self.max_initial_wheel_angle_deg:
            raise CalibrationAbort(
                "零转角命令下前轮未回中：反馈最大绝对值 %.3fdeg > %.3fdeg"
                % (
                    max(abs(math.degrees(value)) for value in wheel_values),
                    self.max_initial_wheel_angle_deg,
                )
            )
        if math.degrees(wheel_angle_std) > self.max_initial_wheel_angle_std_deg:
            raise CalibrationAbort(
                "起步前轮角不稳定：std=%.3fdeg > %.3fdeg"
                % (
                    math.degrees(wheel_angle_std),
                    self.max_initial_wheel_angle_std_deg,
                )
            )
        return {
            "samples": samples,
            "start_x": start_x,
            "start_y": start_y,
            "start_yaw": yaw_mean,
            "yaw_std": yaw_std,
            "raw_heading": raw_heading_mean,
            "wheel_angle_mean": wheel_angle_mean,
            "wheel_angle_std": wheel_angle_std,
            "last_sequence": last_sequence,
        }

    def operator_confirmation(self, wheelbase_m):
        rospy.logwarn("=" * 68)
        rospy.logwarn("本程序将直接成为 /cmd_vel 的唯一发布者并驱动车辆")
        rospy.logwarn(
            "计划：速度 %.2fm/s，距离 %.1fm，angular.z=0，轴距 %.3fm",
            self.speed_mps,
            self.distance_m,
            wheelbase_m,
        )
        rospy.logwarn("需要前方至少 %.1fm 净空，左右各至少 1m；物理急停人员必须就位", self.distance_m + 2.0)
        rospy.logwarn("不要在完整 bringup/move_base/键盘遥控运行时执行")
        if self.external_estop_override:
            rospy.logwarn(
                "外部急停安全覆盖已启用：跳过原始 CAN、底盘聚合急停和 "
                "VCU 控制超时检查"
            )
            rospy.logwarn(
                "仍保留 GPS/轮角/速度、唯一控制发布者、0.30s 命令租约和绝对超时"
            )
            rospy.logwarn("现场遥控器急停人员必须全程保持可立即操作")
        elif not self.use_raw_can_emergency_check:
            rospy.logwarn(
                "安全覆盖已启用：不主动查询或校验四项原始 CAN 急停响应；"
                "仍检查底盘聚合急停状态、控制超时和软件零速看门狗"
            )
            rospy.logwarn("现场遥控器急停人员必须全程保持可立即操作")
        if self.speed_mps > MAX_CALIBRATION_SPEED_MPS:
            rospy.logwarn(
                "外部急停模式高速覆盖：标定速度 %.2fm/s，高于默认硬限制 %.2fm/s",
                self.speed_mps,
                MAX_CALIBRATION_SPEED_MPS,
            )
        if not self.use_progress_watchdog:
            rospy.logwarn(
                "有效进展看门狗已关闭：车辆一旦被判定起步，后续停滞将等待距离条件"
                "或 %.1fs 绝对运行超时才由软件停车",
                self.run_timeout_sec,
            )
        if self.heading_rate_abort_count > 1:
            rospy.logwarn(
                "双天线航向尖峰容错：单点超限会丢弃，连续 %d 次才中止",
                self.heading_rate_abort_count,
            )
        rospy.logwarn("=" * 68)
        try:
            answer = input(
                "确认场地安全后输入 START 并回车；其他输入取消"
                "（输入时间不限，START 后会自动重新预检）："
            ).strip()
        except (EOFError, KeyboardInterrupt):
            raise CalibrationAbort("操作者取消")
        if answer != "START":
            raise CalibrationAbort("操作者未输入 START")

    def drive_straight(self, baseline):
        start_x = baseline["start_x"]
        start_y = baseline["start_y"]
        start_yaw = baseline["start_yaw"]
        previous = baseline["samples"][-1]
        last_sequence = baseline["last_sequence"]
        previous_yaw = previous.yaw
        previous_driver_speed = self.driver_speed_at(
            previous.receipt_monotonic
        )[0]
        yaw_unwrapped = start_yaw
        path_m = 0.0
        gps_path_m = 0.0
        points = []
        self.motion_points = points
        drive_start = time.monotonic()
        motion_seen = False
        progress_anchor_x = previous.x
        progress_anchor_y = previous.y
        progress_anchor_path_m = 0.0
        last_progress_monotonic = drive_start
        overspeed_sample_count = 0
        heading_rate_violation_count = 0
        with self.lock:
            self.control_timeout_seen = False
        self.set_command(
            self.speed_mps,
            absolute_deadline_monotonic=drive_start + self.run_timeout_sec,
        )
        rospy.logwarn("直线标定开始：%.2fm/s，目标 %.1fm", self.speed_mps, self.distance_m)
        rate = rospy.Rate(20.0)

        while not rospy.is_shutdown():
            self.check_runtime_health()
            new_states = self.states_after(last_sequence)
            for state in new_states:
                if state.stamp_sec <= previous.stamp_sec:
                    raise CalibrationAbort("/gps/odom 源时间戳未严格递增")
                dt = state.stamp_sec - previous.stamp_sec
                current_driver_speed = self.driver_speed_at(
                    state.receipt_monotonic
                )[0]
                ds = math.hypot(state.x - previous.x, state.y - previous.y)
                if ds > self.max_pose_step_m:
                    raise CalibrationAbort("GPS 位置单步跳变 %.3fm" % ds)
                delta_yaw = normalize_angle(state.yaw - previous_yaw)
                heading_rate_deg_s = math.degrees(delta_yaw / dt)
                heading_rate_violation_count = updated_heading_rate_violation_count(
                    heading_rate_violation_count,
                    heading_rate_deg_s,
                    self.max_heading_rate_deg_s,
                )
                if heading_rate_violation_count:
                    last_sequence = state.sequence
                    if heading_rate_violation_count >= self.heading_rate_abort_count:
                        raise CalibrationAbort(
                            "双天线航向连续 %d 次突变，当前 %.2fdeg/s"
                            % (heading_rate_violation_count, heading_rate_deg_s)
                        )
                    rospy.logwarn(
                        "忽略双天线航向瞬时尖峰 %.2fdeg/s（%d/%d）",
                        heading_rate_deg_s,
                        heading_rate_violation_count,
                        self.heading_rate_abort_count,
                    )
                    continue
                path_m += max(
                    0.0, 0.5 * (previous_driver_speed + current_driver_speed)
                ) * dt
                gps_path_m += ds
                yaw_unwrapped += delta_yaw
                forward_m, lateral_m = local_displacement(
                    start_x, start_y, start_yaw, state.x, state.y
                )
                point = MotionPoint(
                    state=state,
                    path_m=path_m,
                    gps_path_m=gps_path_m,
                    yaw_unwrapped=yaw_unwrapped,
                    forward_m=forward_m,
                    lateral_m=lateral_m,
                )
                points.append(point)
                self.write_sample(
                    "driving",
                    state,
                    path_m,
                    gps_path_m,
                    forward_m,
                    lateral_m,
                )
                previous = state
                previous_yaw = state.yaw
                previous_driver_speed = current_driver_speed
                last_sequence = state.sequence
                if abs(current_driver_speed) >= 0.03 and not motion_seen:
                    motion_seen = True
                    progress_anchor_x = state.x
                    progress_anchor_y = state.y
                    progress_anchor_path_m = path_m
                    last_progress_monotonic = time.monotonic()
                gps_progress_from_anchor = math.hypot(
                    state.x - progress_anchor_x, state.y - progress_anchor_y
                )
                wheel_progress_from_anchor = max(
                    0.0, path_m - progress_anchor_path_m
                )
                if progress_step_reached(
                    gps_progress_from_anchor,
                    wheel_progress_from_anchor,
                    self.progress_step_m,
                ):
                    motion_seen = True
                    progress_anchor_x = state.x
                    progress_anchor_y = state.y
                    progress_anchor_path_m = path_m
                    last_progress_monotonic = time.monotonic()
                heading_change = math.degrees(yaw_unwrapped - start_yaw)
                if abs(heading_change) > self.max_heading_change_deg:
                    raise CalibrationAbort(
                        "航向变化达到安全上限 %.2fdeg" % heading_change
                    )
                if abs(lateral_m) > self.max_lateral_deviation_m:
                    raise CalibrationAbort(
                        "横向偏移达到安全上限 %.3fm" % lateral_m
                    )
                if current_driver_speed < -0.03:
                    raise CalibrationAbort("直行前进命令下检测到底盘倒车")
                if current_driver_speed > self.hard_measured_speed_limit_mps:
                    raise CalibrationAbort(
                        "实测车速达到硬上限：%.3fm/s > %.3fm/s"
                        % (
                            current_driver_speed,
                            self.hard_measured_speed_limit_mps,
                        )
                    )
                if current_driver_speed > self.max_measured_speed_mps:
                    overspeed_sample_count += 1
                else:
                    overspeed_sample_count = 0
                if overspeed_sample_count >= 3:
                    raise CalibrationAbort(
                        "实测车速连续超限：%.3fm/s > %.3fm/s"
                        % (current_driver_speed, self.max_measured_speed_mps)
                    )
                chord_m = math.hypot(state.x - start_x, state.y - start_y)
                if max(path_m, chord_m) >= self.distance_m:
                    # Do not turn a delayed/stalled main-loop run into a
                    # seemingly valid completion after the timer cut motion.
                    self.complete_motion_command()
                    self.publish_command()
                    rospy.logwarn(
                        "已到目标距离：轮速积分 %.3fm、GPS 弦长 %.3fm，发送零速",
                        path_m,
                        chord_m,
                    )
                    return (
                        points,
                        path_m,
                        gps_path_m,
                        last_sequence,
                        previous,
                        previous_driver_speed,
                        yaw_unwrapped,
                    )

            elapsed = time.monotonic() - drive_start
            if not motion_seen and elapsed > self.no_motion_timeout_sec:
                raise CalibrationAbort("发出直行命令后车辆未在限定时间内起步")
            if progress_watchdog_expired(
                motion_seen,
                time.monotonic(),
                last_progress_monotonic,
                self.no_progress_timeout_sec,
                enabled=self.use_progress_watchdog,
            ):
                raise CalibrationAbort(
                    "车辆已起步但 GPS/轮速积分连续 %.1fs 无 %.2fm 有效进展"
                    % (self.no_progress_timeout_sec, self.progress_step_m)
                )
            if elapsed > self.run_timeout_sec:
                raise CalibrationAbort("直行标定超过 %.1fs 绝对超时" % self.run_timeout_sec)
            self.renew_motion_lease()
            rate.sleep()
        raise CalibrationAbort("ROS 在直行过程中关闭")

    def collect_settle(
        self,
        baseline,
        path_m,
        gps_path_m,
        last_sequence,
        previous,
        previous_driver_speed,
        yaw_unwrapped,
    ):
        self.set_command(0.0)
        start = time.monotonic()
        settle_samples = []
        stationary_samples = []
        previous_yaw = previous.yaw
        stationary_since = None
        heading_rate_violation_count = 0
        rate = rospy.Rate(20.0)
        max_wait = max(5.0, self.end_settle_sec + 3.0)
        while time.monotonic() - start < max_wait and not rospy.is_shutdown():
            self.check_runtime_health()
            for state in self.states_after(last_sequence):
                if state.stamp_sec <= previous.stamp_sec:
                    raise CalibrationAbort("停车阶段 /gps/odom 源时间戳未严格递增")
                dt = state.stamp_sec - previous.stamp_sec
                current_driver_speed = self.driver_speed_at(
                    state.receipt_monotonic
                )[0]
                ds = math.hypot(state.x - previous.x, state.y - previous.y)
                if ds > self.max_pose_step_m:
                    raise CalibrationAbort("停车阶段 GPS 位置单步跳变 %.3fm" % ds)
                delta_yaw = normalize_angle(state.yaw - previous_yaw)
                heading_rate_deg_s = math.degrees(delta_yaw / dt)
                heading_rate_violation_count = updated_heading_rate_violation_count(
                    heading_rate_violation_count,
                    heading_rate_deg_s,
                    self.max_heading_rate_deg_s,
                )
                if heading_rate_violation_count:
                    last_sequence = state.sequence
                    if heading_rate_violation_count >= self.heading_rate_abort_count:
                        raise CalibrationAbort(
                            "停车阶段双天线航向连续 %d 次突变，当前 %.2fdeg/s"
                            % (heading_rate_violation_count, heading_rate_deg_s)
                        )
                    rospy.logwarn(
                        "停车阶段忽略双天线航向瞬时尖峰 %.2fdeg/s（%d/%d）",
                        heading_rate_deg_s,
                        heading_rate_violation_count,
                        self.heading_rate_abort_count,
                    )
                    continue
                path_m += max(
                    0.0, 0.5 * (previous_driver_speed + current_driver_speed)
                ) * dt
                gps_path_m += ds
                yaw_unwrapped += delta_yaw
                forward_m, lateral_m = local_displacement(
                    baseline["start_x"],
                    baseline["start_y"],
                    baseline["start_yaw"],
                    state.x,
                    state.y,
                )
                self.write_sample(
                    "settling",
                    state,
                    path_m,
                    gps_path_m,
                    forward_m,
                    lateral_m,
                )
                settle_samples.append(state)
                previous = state
                previous_yaw = state.yaw
                previous_driver_speed = current_driver_speed
                last_sequence = state.sequence
                heading_change = math.degrees(
                    yaw_unwrapped - baseline["start_yaw"]
                )
                if abs(heading_change) > self.max_heading_change_deg:
                    raise CalibrationAbort(
                        "停车阶段航向变化达到安全上限 %.2fdeg" % heading_change
                    )
                if abs(lateral_m) > self.max_lateral_deviation_m:
                    raise CalibrationAbort(
                        "停车阶段横向偏移达到安全上限 %.3fm" % lateral_m
                    )
                if current_driver_speed < -0.03:
                    raise CalibrationAbort("停车阶段检测到底盘倒车")
                if current_driver_speed > self.max_measured_speed_mps:
                    raise CalibrationAbort(
                        "停车阶段实测车速超限：%.3fm/s"
                        % current_driver_speed
                    )
                if abs(current_driver_speed) <= 0.03:
                    if stationary_since is None:
                        stationary_since = time.monotonic()
                        stationary_samples = []
                    stationary_samples.append(state)
                else:
                    stationary_since = None
                    stationary_samples = []
            elapsed = time.monotonic() - start
            if (
                elapsed >= self.end_settle_sec
                and stationary_since is not None
                and time.monotonic() - stationary_since >= 0.5
            ):
                return stationary_samples, path_m, gps_path_m, yaw_unwrapped
            rate.sleep()
        if rospy.is_shutdown():
            raise CalibrationAbort("ROS 在停车确认阶段关闭")
        if not settle_samples:
            raise CalibrationAbort("停车后没有收到新的 /gps/odom")
        raise CalibrationAbort(
            "发送零速后 %.1fs 内未确认连续静止 0.5s，拒绝生成完成结果"
            % max_wait
        )

    def build_summary(
        self,
        status,
        reason,
        wheelbase_m,
        wheelbase_source,
        baseline=None,
        points=None,
        settle_samples=None,
        final_path_m=None,
        final_gps_path_m=None,
    ):
        now_monotonic = time.monotonic()
        with self.lock:
            raw_statuses = dict(self.raw_emergency_status)
        raw_status_summary = {}
        for msg_type, label in RAW_EMERGENCY_TYPES.items():
            status_entry = raw_statuses.get(msg_type)
            if status_entry is None:
                raw_status_summary[label] = {"seen": False}
                continue
            receipt, is_safe, raw_value, error = status_entry
            raw_status_summary[label] = {
                "seen": True,
                "safe": is_safe,
                "raw_value": raw_value,
                "error": error,
                "age_sec": now_monotonic - receipt,
            }
        summary = {
            "recorded_at": iso_timestamp(),
            "status": status,
            "reason": reason,
            "odom_topic": self.odom_topic,
            "raw_heading_topic": self.raw_heading_topic,
            "cmd_topic": self.cmd_topic,
            "command_speed_mps": self.speed_mps,
            "requested_distance_m": self.distance_m,
            "wheelbase_m": wheelbase_m,
            "wheelbase_source": wheelbase_source,
            "current_bias_deg_operator_input": self.current_bias_deg,
            "current_bias_was_explicit": self.current_bias_was_explicit,
            "recommendation_gain": self.recommendation_gain,
            "bias_sign_convention": (
                "field convention: negative steer_center_bias corrects left"
                if self.negative_bias_corrects_left
                else "configured convention: positive steer_center_bias corrects left"
            ),
            "bias_units_and_persistence_verified_by_source": False,
            "expected_odom_publisher": self.expected_odom_publisher,
            "expected_driver_node": self.expected_driver_node,
            "expected_canbus_node": self.expected_canbus_node,
            "use_raw_can_emergency_check": self.use_raw_can_emergency_check,
            "external_estop_override": self.external_estop_override,
            "use_progress_watchdog": self.use_progress_watchdog,
            "raw_can_emergency_status": raw_status_summary,
            "safety_parameters": {
                "external_estop_override": self.external_estop_override,
                "command_speed_limit_mps": self.command_speed_limit_mps,
                "hard_measured_speed_limit_mps": self.hard_measured_speed_limit_mps,
                "use_progress_watchdog": self.use_progress_watchdog,
                "command_lease_sec": self.command_lease_sec,
                "run_timeout_sec": self.run_timeout_sec,
                "no_motion_timeout_sec": self.no_motion_timeout_sec,
                "no_progress_timeout_sec": self.no_progress_timeout_sec,
                "progress_step_m": self.progress_step_m,
                "max_measured_speed_mps": self.max_measured_speed_mps,
                "max_pose_step_m": self.max_pose_step_m,
                "max_heading_rate_deg_s": self.max_heading_rate_deg_s,
                "heading_rate_abort_count": self.heading_rate_abort_count,
                "max_heading_change_deg": self.max_heading_change_deg,
                "max_lateral_deviation_m": self.max_lateral_deviation_m,
                "raw_emergency_timeout_sec": self.raw_emergency_timeout_sec,
                "raw_emergency_query_interval_sec": self.raw_emergency_query_interval_sec,
            },
            "csv_path": self.csv_path,
        }
        if baseline is None:
            return summary
        summary.update(
            {
                "baseline_sample_count": len(baseline["samples"]),
                "baseline_heading_std_deg": math.degrees(baseline["yaw_std"]),
                "start_x_m": baseline["start_x"],
                "start_y_m": baseline["start_y"],
                "start_ros_yaw_deg_left_positive": math.degrees(baseline["start_yaw"]),
                "start_gps_heading_deg_clockwise_from_north": baseline["raw_heading"],
                "baseline_wheel_angle_mean_deg": math.degrees(
                    baseline["wheel_angle_mean"]
                ),
                "baseline_wheel_angle_std_deg": math.degrees(
                    baseline["wheel_angle_std"]
                ),
            }
        )
        if not points:
            return summary
        cutoff = points[-1]
        end_state = settle_samples[-1] if settle_samples else cutoff.state
        end_yaws = [state.yaw for state in (settle_samples or [cutoff.state])]
        end_yaw = circular_mean(end_yaws)
        (
            end_raw_heading,
            end_raw_heading_age,
            end_wheel_angle,
            end_wheel_angle_age,
        ) = self.auxiliary_values_at(end_state.receipt_monotonic)
        forward_m, lateral_m = local_displacement(
            baseline["start_x"],
            baseline["start_y"],
            baseline["start_yaw"],
            end_state.x,
            end_state.y,
        )
        chord_m = math.hypot(forward_m, lateral_m)
        net_heading_change = normalize_angle(end_yaw - baseline["start_yaw"])
        track_bearing_error = (
            normalize_angle(math.atan2(end_state.y - baseline["start_y"], end_state.x - baseline["start_x"]) - baseline["start_yaw"])
            if chord_m > 1e-6
            else 0.0
        )
        estimated_crab_angle = normalize_angle(
            track_bearing_error - 0.5 * net_heading_change
        )
        position_curvature = 2.0 * lateral_m / (chord_m * chord_m) if chord_m > 1e-6 else 0.0
        distance_for_ratio = (
            final_path_m
            if final_path_m is not None and final_path_m > 1e-6
            else cutoff.path_m
        )
        chord_to_wheel_ratio = (
            chord_m / distance_for_ratio if distance_for_ratio > 1e-6 else None
        )
        summary.update(
            {
                "drive_sample_count": len(points),
                "command_cutoff_path_m": cutoff.path_m,
                "command_cutoff_gps_accumulated_path_m": cutoff.gps_path_m,
                "final_path_m_including_stop": final_path_m,
                "final_gps_accumulated_path_m_including_stop": final_gps_path_m,
                "end_x_m": end_state.x,
                "end_y_m": end_state.y,
                "end_ros_yaw_deg_left_positive": math.degrees(end_yaw),
                "net_heading_change_deg_left_positive": math.degrees(net_heading_change),
                "end_gps_heading_deg_clockwise_from_north": end_raw_heading,
                "end_gps_heading_age_sec": end_raw_heading_age,
                "end_wheel_angle_deg": (
                    None if end_wheel_angle is None else math.degrees(end_wheel_angle)
                ),
                "end_wheel_angle_age_sec": end_wheel_angle_age,
                "gps_heading_change_deg_right_positive": (
                    None
                    if baseline["raw_heading"] is None or end_raw_heading is None
                    else math.degrees(
                        normalize_angle(
                            math.radians(end_raw_heading - baseline["raw_heading"])
                        )
                    )
                ),
                "forward_displacement_m": forward_m,
                "lateral_displacement_m_left_positive": lateral_m,
                "chord_distance_m": chord_m,
                "track_bearing_error_deg_left_positive": math.degrees(track_bearing_error),
                "estimated_crab_angle_deg_left_positive": math.degrees(
                    estimated_crab_angle
                ),
                "position_curvature_rad_per_m_left_positive": position_curvature,
                "chord_to_wheel_distance_ratio": chord_to_wheel_ratio,
            }
        )
        try:
            metrics = curvature_fit_metrics(
                points, self.fit_trim_start_m, self.fit_trim_end_m
            )
            curvature = metrics["curvature"]
            equivalent_rad = equivalent_steering_angle(curvature, wheelbase_m)
            equivalent_deg = math.degrees(equivalent_rad)
            equivalent_standard_error_deg = math.degrees(
                wheelbase_m
                * metrics["curvature_standard_error_rad_m"]
                / (1.0 + (wheelbase_m * curvature) ** 2)
            )
            position_equivalent_deg = math.degrees(
                equivalent_steering_angle(position_curvature, wheelbase_m)
            )
            recommended_bias = recommended_bias_value(
                self.current_bias_deg,
                equivalent_deg,
                self.recommendation_gain,
                self.negative_bias_corrects_left,
            )
            summary.update(
                {
                    "fit_sample_count": metrics["sample_count"],
                    "fit_distance_span_m": metrics["distance_span_m"],
                    "fit_heading_rmse_deg": math.degrees(
                        metrics["heading_rmse_rad"]
                    ),
                    "fitted_curvature_standard_error_rad_per_m": metrics[
                        "curvature_standard_error_rad_m"
                    ],
                    "fitted_curvature_rad_per_m_left_positive": curvature,
                    "fitted_heading_drift_deg_per_m_left_positive": math.degrees(curvature),
                    "equivalent_front_steer_error_deg_left_positive": equivalent_deg,
                    "equivalent_front_steer_standard_error_deg": equivalent_standard_error_deg,
                    "position_equivalent_front_steer_deg_left_positive": position_equivalent_deg,
                    "observed_curve_direction": (
                        "left" if equivalent_deg > 0.0 else "right" if equivalent_deg < 0.0 else "straight"
                    ),
                    "recommendation_is_automatic_command": False,
                }
            )
            suppression_reasons = []
            if status != "completed":
                suppression_reasons.append("试验未完成全部安全与停车检查")
            if abs(equivalent_deg) > MAX_RECOMMENDABLE_STEER_ERROR_DEG:
                suppression_reasons.append(
                    "等效转向误差超过 %.1fdeg"
                    % MAX_RECOMMENDABLE_STEER_ERROR_DEG
                )
            if abs(math.degrees(estimated_crab_angle)) > COURSE_RESIDUAL_WARNING_DEG:
                suppression_reasons.append(
                    "蟹行角估计超过 %.1fdeg"
                    % COURSE_RESIDUAL_WARNING_DEG
                )
            if math.degrees(metrics["heading_rmse_rad"]) > MAX_HEADING_FIT_RMSE_DEG:
                suppression_reasons.append(
                    "航向拟合 RMSE 超过 %.1fdeg" % MAX_HEADING_FIT_RMSE_DEG
                )
            if (
                equivalent_standard_error_deg
                > MAX_EQUIVALENT_STEER_STANDARD_ERROR_DEG
            ):
                suppression_reasons.append(
                    "等效转角标准误超过 %.2fdeg"
                    % MAX_EQUIVALENT_STEER_STANDARD_ERROR_DEG
                )
            if (
                chord_to_wheel_ratio is None
                or not MIN_CHORD_TO_WHEEL_DISTANCE_RATIO
                <= chord_to_wheel_ratio
                <= MAX_CHORD_TO_WHEEL_DISTANCE_RATIO
            ):
                suppression_reasons.append("GNSS 弦长与轮速积分距离不一致")
            if abs(position_equivalent_deg - equivalent_deg) > MAX_POSITION_STEER_DISAGREEMENT_DEG:
                suppression_reasons.append(
                    "位置与航向的转角估计相差超过 %.2fdeg"
                    % MAX_POSITION_STEER_DISAGREEMENT_DEG
                )
            increment = recommended_bias - self.current_bias_deg
            if abs(increment) > MAX_RECOMMENDATION_INCREMENT_DEG:
                suppression_reasons.append(
                    "候选偏置增量超过 %.1fdeg"
                    % MAX_RECOMMENDATION_INCREMENT_DEG
                )
            if abs(recommended_bias) > MAX_RECOMMENDED_ABSOLUTE_BIAS_DEG:
                suppression_reasons.append(
                    "候选绝对偏置超过 %.1fdeg"
                    % MAX_RECOMMENDED_ABSOLUTE_BIAS_DEG
                )
            if not self.current_bias_was_explicit:
                suppression_reasons.append("未显式传入 current_bias_deg")
            summary["recommendation_suppressed"] = bool(suppression_reasons)
            if suppression_reasons:
                summary["recommendation_suppression_reasons"] = suppression_reasons
            else:
                summary["suggested_bias_increment_deg"] = increment
                summary["suggested_next_absolute_bias_deg"] = recommended_bias
        except ValueError as exc:
            summary["fit_error"] = str(exc)
        return summary

    def print_summary(self, summary):
        rospy.loginfo("标定记录：%s", self.csv_path)
        rospy.loginfo("结果摘要：%s", self.summary_path)
        if "equivalent_front_steer_error_deg_left_positive" not in summary:
            rospy.logwarn("本次没有得到有效转向偏差估计：%s", summary.get("reason", ""))
            return
        heading_change = summary["net_heading_change_deg_left_positive"]
        lateral = summary["lateral_displacement_m_left_positive"]
        steer_error = summary["equivalent_front_steer_error_deg_left_positive"]
        crab_angle = summary["estimated_crab_angle_deg_left_positive"]
        rospy.logwarn("=" * 68)
        if summary.get("status") != "completed":
            rospy.logwarn("本次试验已中止；以下数值仅作故障诊断，不得用于调整偏置")
        rospy.logwarn(
            "实测航向变化：%+.3fdeg（左正右负）；横向偏移：%+.3fm（左正右负）",
            heading_change,
            lateral,
        )
        rospy.logwarn(
            "拟合曲率：%+.6frad/m，等效前轮中心误差：%+.3fdeg（左正右负）",
            summary["fitted_curvature_rad_per_m_left_positive"],
            steer_error,
        )
        rospy.logwarn(
            "轨迹弦方向偏角：%+.3fdeg；扣除圆弧效应后的蟹行角估计：%+.3fdeg",
            summary["track_bearing_error_deg_left_positive"],
            crab_angle,
        )
        if summary.get("recommendation_suppressed", True):
            rospy.logerr("质量/安全门限未通过，本次不生成 steer_center_bias 建议")
            for suppression_reason in summary.get(
                "recommendation_suppression_reasons", []
            ):
                rospy.logerr("  - %s", suppression_reason)
        else:
            convention_text = (
                "负 bias 向左修正"
                if self.negative_bias_corrects_left
                else "正 bias 向左修正"
            )
            rospy.logwarn(
                "依据配置约定“%s”，建议偏置增量：%+.3fdeg",
                convention_text,
                summary["suggested_bias_increment_deg"],
            )
            rospy.logwarn(
                "若本次 current_bias_deg=%.3f 填写正确，下一次试验候选值：%.3f",
                self.current_bias_deg,
                summary["suggested_next_absolute_bias_deg"],
            )
        rospy.logwarn("程序没有发布 steer_center_bias；建议至少重复 3 次并取中位数")
        rospy.logwarn("=" * 68)

    def run(self):
        return_code = 1
        status = "aborted"
        reason = "initialization incomplete"
        wheelbase_m = None
        wheelbase_source = ""
        baseline = None
        points = []
        settle_samples = []
        final_path_m = None
        final_gps_path_m = None
        try:
            self.prepare_output()
        except OSError as exc:
            rospy.logerr("无法创建标定记录文件：%s", exc)
            return 1
        try:
            if not self.allow_motion:
                raise CalibrationAbort(
                    "allow_motion=false；确认现场安全后显式传入 _allow_motion:=true"
                )
            self.wait_for_preflight()
            wheelbase_m, wheelbase_source = self.read_wheelbase()
            self.operator_confirmation(wheelbase_m)
            self.revalidate_after_operator_confirmation()
            baseline = self.collect_baseline()
            (
                points,
                cutoff_path,
                cutoff_gps_path,
                last_sequence,
                previous,
                previous_driver_speed,
                yaw_unwrapped,
            ) = self.drive_straight(baseline)
            (
                settle_samples,
                final_path_m,
                final_gps_path_m,
                _final_unwrapped,
            ) = self.collect_settle(
                baseline,
                cutoff_path,
                cutoff_gps_path,
                last_sequence,
                previous,
                previous_driver_speed,
                yaw_unwrapped,
            )
            status = "completed"
            reason = "requested distance reached and zero command issued"
            return_code = 0
        except (CalibrationAbort, ValueError) as exc:
            reason = str(exc)
            rospy.logerr("直线标定中止：%s", reason)
            return_code = 1
        except Exception as exc:
            reason = "unexpected %s: %s" % (type(exc).__name__, exc)
            rospy.logerr("直线标定异常：%s", reason)
            return_code = 1
        finally:
            self.stop_burst()
            points_for_summary = points or self.motion_points
            summary = None
            try:
                summary = self.build_summary(
                    status,
                    reason,
                    wheelbase_m,
                    wheelbase_source,
                    baseline,
                    points_for_summary,
                    settle_samples,
                    final_path_m,
                    final_gps_path_m,
                )
                with open(self.summary_path, "w", encoding="utf-8") as handle:
                    json.dump(
                        summary,
                        handle,
                        ensure_ascii=False,
                        indent=2,
                        sort_keys=True,
                        allow_nan=False,
                    )
                    handle.write("\n")
            except (OSError, TypeError, ValueError) as exc:
                rospy.logerr("写入标定摘要失败：%s", exc)
                return_code = 1
            finally:
                if self.csv_handle is not None:
                    self.csv_handle.close()
            if summary is not None:
                self.print_summary(summary)
        return return_code


def main():
    rospy.init_node("steer_center_calibration")
    try:
        node = SteerCenterCalibration()
    except (ValueError, rospy.ROSException) as exc:
        rospy.logfatal("标定节点初始化失败：%s", exc)
        return 2
    return node.run()


if __name__ == "__main__":
    sys.exit(main())
