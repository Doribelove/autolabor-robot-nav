#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
import threading
import unittest
from unittest import mock


SCRIPT_PATH = (
    Path(__file__).resolve().parents[1] / "scripts" / "steer_center_calibration.py"
)
SPEC = importlib.util.spec_from_file_location("steer_center_calibration", str(SCRIPT_PATH))
CALIBRATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CALIBRATION)


def motion_point(distance, yaw, x=None, y=None):
    state = CALIBRATION.OdomState(
        sequence=int(distance * 10),
        receipt_monotonic=distance,
        stamp_sec=distance + 1.0,
        x=distance if x is None else x,
        y=0.0 if y is None else y,
        yaw=CALIBRATION.normalize_angle(yaw),
        linear_x=0.2,
        angular_z=0.0,
        frame_id="camera_init",
        child_frame_id="base_link",
    )
    return CALIBRATION.MotionPoint(
        state=state,
        path_m=distance,
        gps_path_m=distance,
        yaw_unwrapped=yaw,
        forward_m=distance,
        lateral_m=0.0,
    )


def arc_motion_point(distance, curvature):
    if abs(curvature) < 1e-12:
        return motion_point(distance, 0.0)
    yaw = curvature * distance
    x = math.sin(yaw) / curvature
    y = (1.0 - math.cos(yaw)) / curvature
    return motion_point(distance, yaw, x=x, y=y)


class AngleMathTest(unittest.TestCase):
    def test_wrap_across_pi_uses_short_direction(self):
        delta = CALIBRATION.normalize_angle(math.radians(-179.0 - 179.0))
        self.assertAlmostEqual(math.degrees(delta), 2.0)

    def test_circular_mean_handles_heading_wrap(self):
        values = [math.radians(179.0), math.radians(-179.0)]
        mean = CALIBRATION.circular_mean(values)
        self.assertAlmostEqual(abs(math.degrees(mean)), 180.0)
        self.assertAlmostEqual(math.degrees(CALIBRATION.circular_stddev(values)), 1.0, places=2)

    def test_quaternion_to_yaw(self):
        half = math.pi / 4.0
        yaw = CALIBRATION.quaternion_to_yaw(0.0, 0.0, math.sin(half), math.cos(half))
        self.assertAlmostEqual(yaw, math.pi / 2.0)


class GeometryTest(unittest.TestCase):
    def test_local_displacement_reports_left_positive(self):
        forward, lateral = CALIBRATION.local_displacement(
            2.0, 3.0, math.pi / 2.0, 1.0, 5.0
        )
        self.assertAlmostEqual(forward, 2.0)
        self.assertAlmostEqual(lateral, 1.0)

    def test_curvature_fit_uses_unwrapped_yaw(self):
        expected_curvature = 0.02
        points = [
            motion_point(index / 10.0, expected_curvature * index / 10.0)
            for index in range(61)
        ]
        curvature, count, span = CALIBRATION.fit_curvature(points, 0.5, 0.5)
        self.assertAlmostEqual(curvature, expected_curvature)
        self.assertGreater(count, 40)
        self.assertGreaterEqual(span, 5.0)

    def test_equivalent_steering_uses_wheelbase_and_curvature(self):
        angle = CALIBRATION.equivalent_steering_angle(0.02, 0.65)
        self.assertAlmostEqual(angle, math.atan(0.013))

    def test_short_fit_is_rejected(self):
        points = [motion_point(0.0, 0.0), motion_point(0.5, 0.01), motion_point(1.0, 0.02)]
        with self.assertRaises(ValueError):
            CALIBRATION.fit_curvature(points, 0.25, 0.25)

    def test_fit_metrics_report_zero_residual_for_constant_curvature(self):
        points = [arc_motion_point(index / 10.0, -0.01) for index in range(51)]
        metrics = CALIBRATION.curvature_fit_metrics(points, 0.5, 0.5)
        self.assertAlmostEqual(metrics["curvature"], -0.01)
        self.assertAlmostEqual(metrics["heading_rmse_rad"], 0.0, places=12)


class SafetyPrimitiveTest(unittest.TestCase):
    def test_progress_watchdog_expires_only_after_motion(self):
        self.assertFalse(CALIBRATION.progress_watchdog_expired(False, 4.0, 1.0, 2.0))
        self.assertTrue(CALIBRATION.progress_watchdog_expired(True, 4.0, 1.0, 2.0))
        self.assertFalse(
            CALIBRATION.progress_watchdog_expired(
                True, 4.0, 1.0, 2.0, enabled=False
            )
        )

    def test_half_meter_speed_requires_external_estop_override(self):
        self.assertEqual(CALIBRATION.calibration_speed_limit(False), 0.30)
        self.assertEqual(CALIBRATION.calibration_speed_limit(True), 0.50)
        self.assertEqual(CALIBRATION.measured_speed_limit(False), 0.50)
        self.assertEqual(CALIBRATION.measured_speed_limit(True), 0.65)

    def test_heading_rate_spike_count_resets_after_normal_sample(self):
        count = CALIBRATION.updated_heading_rate_violation_count(0, 23.99, 20.0)
        self.assertEqual(count, 1)
        count = CALIBRATION.updated_heading_rate_violation_count(count, -24.0, 20.0)
        self.assertEqual(count, 2)
        self.assertEqual(
            CALIBRATION.updated_heading_rate_violation_count(count, 4.0, 20.0),
            0,
        )

    def test_wheel_distance_can_confirm_progress_while_gps_position_is_held(self):
        self.assertTrue(CALIBRATION.progress_step_reached(0.001, 0.021, 0.02))
        self.assertTrue(CALIBRATION.progress_step_reached(0.021, 0.0, 0.02))
        self.assertFalse(CALIBRATION.progress_step_reached(0.001, 0.019, 0.02))

    def test_string_false_is_not_accepted_as_motion_authorization(self):
        with mock.patch.object(CALIBRATION.rospy, "get_param", return_value="false"):
            with self.assertRaises(ValueError):
                CALIBRATION.strict_bool_param("~allow_motion", False)

    def test_raw_can_emergency_responses_are_decoded_fail_closed(self):
        for msg_type in (
            CALIBRATION.VCU_HARD_EMERGENCY,
            CALIBRATION.VCU_SOFT_EMERGENCY,
            CALIBRATION.VCU_GAMEPAD_EMERGENCY,
        ):
            self.assertEqual(
                CALIBRATION.raw_emergency_response_is_safe(msg_type, [0]),
                (True, 0),
            )
            self.assertEqual(
                CALIBRATION.raw_emergency_response_is_safe(msg_type, [1]),
                (False, 1),
            )
        self.assertEqual(
            CALIBRATION.raw_emergency_response_is_safe(
                CALIBRATION.VCU_COMMON_STATE,
                [CALIBRATION.VCU_RUNNING_STATE],
            ),
            (True, CALIBRATION.VCU_RUNNING_STATE),
        )
        self.assertEqual(
            CALIBRATION.raw_emergency_response_is_safe(
                CALIBRATION.VCU_COMMON_STATE, [0xFF]
            ),
            (False, 0xFF),
        )
        with self.assertRaises(ValueError):
            CALIBRATION.raw_emergency_response_is_safe(
                CALIBRATION.VCU_HARD_EMERGENCY, []
            )

    def test_raw_can_status_queries_are_single_message_round_robin(self):
        node = CALIBRATION.SteerCenterCalibration.__new__(
            CALIBRATION.SteerCenterCalibration
        )
        node.lock = threading.Lock()
        node.raw_emergency_timeout_sec = 2.5
        node.raw_emergency_query_interval_sec = 0.2
        node.use_raw_can_emergency_check = True
        node.external_estop_override = False
        node.use_progress_watchdog = True
        node.command_speed_limit_mps = 0.30
        node.hard_measured_speed_limit_mps = 0.50
        node.raw_emergency_status = {
            msg_type: (9.5, True, 0, "")
            for msg_type in CALIBRATION.RAW_EMERGENCY_QUERY_ORDER
        }
        node.last_raw_emergency_query_monotonic = 0.0
        node.raw_emergency_query_index = 0
        node.canbus_service_proxy = mock.Mock()

        with mock.patch.object(CALIBRATION.time, "monotonic", return_value=10.0):
            node.query_and_check_raw_emergency_status()
        with mock.patch.object(CALIBRATION.time, "monotonic", return_value=10.1):
            node.query_and_check_raw_emergency_status()
        with mock.patch.object(CALIBRATION.time, "monotonic", return_value=10.21):
            node.query_and_check_raw_emergency_status()

        self.assertEqual(node.canbus_service_proxy.call_count, 2)
        queried_types = [
            call.args[0][0].msg_type
            for call in node.canbus_service_proxy.call_args_list
        ]
        self.assertEqual(
            queried_types,
            [
                CALIBRATION.VCU_HARD_EMERGENCY,
                CALIBRATION.VCU_SOFT_EMERGENCY,
            ],
        )
        self.assertTrue(
            all(len(call.args[0]) == 1 for call in node.canbus_service_proxy.call_args_list)
        )

    def test_round_robin_monitor_still_fails_closed(self):
        node = CALIBRATION.SteerCenterCalibration.__new__(
            CALIBRATION.SteerCenterCalibration
        )
        node.lock = threading.Lock()
        node.raw_emergency_timeout_sec = 2.5
        node.raw_emergency_query_interval_sec = 0.2
        node.use_raw_can_emergency_check = True
        node.external_estop_override = False
        node.use_progress_watchdog = True
        node.command_speed_limit_mps = 0.30
        node.hard_measured_speed_limit_mps = 0.50
        node.last_raw_emergency_query_monotonic = 10.0
        node.raw_emergency_query_index = 0
        node.canbus_service_proxy = mock.Mock()
        node.raw_emergency_status = {
            msg_type: (9.5, True, 0, "")
            for msg_type in CALIBRATION.RAW_EMERGENCY_QUERY_ORDER
        }

        node.raw_emergency_status[CALIBRATION.VCU_HARD_EMERGENCY] = (
            7.49,
            True,
            0,
            "",
        )
        with mock.patch.object(CALIBRATION.time, "monotonic", return_value=10.0):
            with self.assertRaisesRegex(CALIBRATION.CalibrationAbort, "已过期"):
                node.query_and_check_raw_emergency_status()

        node.raw_emergency_status[CALIBRATION.VCU_HARD_EMERGENCY] = (
            9.5,
            False,
            1,
            "",
        )
        with mock.patch.object(CALIBRATION.time, "monotonic", return_value=10.0):
            with self.assertRaisesRegex(CALIBRATION.CalibrationAbort, "未释放"):
                node.query_and_check_raw_emergency_status()

    def test_raw_can_emergency_check_can_be_explicitly_disabled(self):
        node = CALIBRATION.SteerCenterCalibration.__new__(
            CALIBRATION.SteerCenterCalibration
        )
        node.use_raw_can_emergency_check = False
        node.external_estop_override = False
        node.canbus_service_proxy = mock.Mock()

        node.query_and_check_raw_emergency_status()

        node.canbus_service_proxy.assert_not_called()

    def test_external_estop_override_skips_can_and_vcu_health_gates(self):
        node = CALIBRATION.SteerCenterCalibration.__new__(
            CALIBRATION.SteerCenterCalibration
        )
        node.external_estop_override = True
        node.check_odom_fresh = mock.Mock()
        node.check_driver_speed_fresh = mock.Mock()
        node.check_wheel_angle_fresh = mock.Mock()
        node.query_and_check_raw_emergency_status = mock.Mock()
        node.check_chassis_status = mock.Mock()
        node.check_command_graph = mock.Mock()

        with mock.patch.object(CALIBRATION.rospy, "is_shutdown", return_value=False):
            node.check_runtime_health(force_graph=True)

        node.check_odom_fresh.assert_called_once_with()
        node.check_driver_speed_fresh.assert_called_once_with()
        node.check_wheel_angle_fresh.assert_called_once_with()
        node.query_and_check_raw_emergency_status.assert_not_called()
        node.check_chassis_status.assert_not_called()
        node.check_command_graph.assert_called_once_with(force=True)

    def test_command_timer_cuts_speed_when_main_loop_lease_expires(self):
        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = CALIBRATION.SteerCenterCalibration.__new__(
            CALIBRATION.SteerCenterCalibration
        )
        node.lock = threading.Lock()
        node.raw_emergency_status = {}
        node.command_lock = threading.Lock()
        node.command_lease_sec = 0.30
        node.command_linear_x = 0.0
        node.command_lease_deadline_monotonic = None
        node.command_absolute_deadline_monotonic = None
        node.command_expired_reason = ""
        node.cmd_pub = Publisher()
        with mock.patch.object(CALIBRATION.time, "monotonic", return_value=10.0):
            node.set_command(0.2, absolute_deadline_monotonic=20.0)
        with mock.patch.object(CALIBRATION.time, "monotonic", return_value=10.31):
            with mock.patch.object(CALIBRATION.rospy, "logerr"):
                node.publish_command()
        self.assertEqual(node.command_linear_x, 0.0)
        self.assertIn("心跳超时", node.command_expired_reason)
        self.assertEqual(node.cmd_pub.messages[-1].linear.x, 0.0)

    def test_command_timer_enforces_nonrenewable_absolute_deadline(self):
        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        node = CALIBRATION.SteerCenterCalibration.__new__(
            CALIBRATION.SteerCenterCalibration
        )
        node.lock = threading.Lock()
        node.raw_emergency_status = {}
        node.command_lock = threading.Lock()
        node.command_lease_sec = 0.30
        node.command_linear_x = 0.0
        node.command_lease_deadline_monotonic = None
        node.command_absolute_deadline_monotonic = None
        node.command_expired_reason = ""
        node.cmd_pub = Publisher()
        with mock.patch.object(CALIBRATION.time, "monotonic", return_value=10.0):
            node.set_command(0.2, absolute_deadline_monotonic=10.2)
        with mock.patch.object(CALIBRATION.time, "monotonic", return_value=10.21):
            with mock.patch.object(CALIBRATION.rospy, "logerr"):
                node.publish_command()
        self.assertEqual(node.command_linear_x, 0.0)
        self.assertIn("绝对运行截止时间", node.command_expired_reason)
        self.assertEqual(node.cmd_pub.messages[-1].linear.x, 0.0)

    def test_driver_speed_pairing_rejects_stale_sample(self):
        node = CALIBRATION.SteerCenterCalibration.__new__(
            CALIBRATION.SteerCenterCalibration
        )
        node.lock = threading.Lock()
        node.odom_timeout_sec = 0.6
        node.driver_speed_history = [
            CALIBRATION.DriverSpeedState(1.0, 101.0, 0.1),
            CALIBRATION.DriverSpeedState(2.0, 102.0, 0.2),
        ]
        speed, age = node.driver_speed_at(2.4)
        self.assertAlmostEqual(speed, 0.2)
        self.assertAlmostEqual(age, 0.4)
        with self.assertRaises(CALIBRATION.CalibrationAbort):
            node.driver_speed_at(2.7)

    def test_post_confirmation_revalidation_replaces_stale_can_state(self):
        node = CALIBRATION.SteerCenterCalibration.__new__(
            CALIBRATION.SteerCenterCalibration
        )
        node.lock = threading.Lock()
        node.raw_emergency_status = {
            CALIBRATION.VCU_HARD_EMERGENCY: (1.0, True, 0, ""),
            CALIBRATION.VCU_SOFT_EMERGENCY: (1.0, True, 0, ""),
        }
        node.control_timeout_seen = True
        node.last_raw_emergency_query_monotonic = 42.0
        node.raw_emergency_query_index = 3
        node.use_raw_can_emergency_check = True
        node.external_estop_override = False
        node.set_command = mock.Mock()
        node.check_runtime_health = mock.Mock()

        observed = {}

        def fresh_preflight(timeout_sec):
            del timeout_sec
            with node.lock:
                observed["cache_was_empty"] = not node.raw_emergency_status
                observed["timeout_was_cleared"] = not node.control_timeout_seen
                for msg_type in CALIBRATION.RAW_EMERGENCY_TYPES:
                    node.raw_emergency_status[msg_type] = (10.0, True, 0, "")
                # Simulate a connection-time timeout that the preflight wait is
                # intentionally allowed to observe.  The strict quiet window
                # must start with a new latch afterwards.
                node.control_timeout_seen = True

        node.wait_for_preflight = fresh_preflight

        with mock.patch.object(CALIBRATION.rospy, "loginfo"):
            with mock.patch.object(CALIBRATION.rospy, "Rate"):
                with mock.patch.object(
                    CALIBRATION.rospy, "is_shutdown", return_value=False
                ):
                    with mock.patch.object(
                        CALIBRATION.time,
                        "monotonic",
                        side_effect=[10.0, 10.1],
                    ):
                        node.revalidate_after_operator_confirmation(
                            timeout_sec=15.0, quiet_sec=0.0
                        )

        node.set_command.assert_called_once_with(0.0)
        self.assertTrue(observed["cache_was_empty"])
        self.assertTrue(observed["timeout_was_cleared"])
        self.assertEqual(node.last_raw_emergency_query_monotonic, 0.0)
        self.assertEqual(node.raw_emergency_query_index, 0)
        self.assertEqual(
            set(node.raw_emergency_status), set(CALIBRATION.RAW_EMERGENCY_TYPES)
        )
        self.assertFalse(node.control_timeout_seen)
        node.check_runtime_health.assert_called_once_with(force_graph=True)


class BiasRecommendationTest(unittest.TestCase):
    def test_field_convention_keeps_observed_error_sign(self):
        # A rightward observed curve is negative ROS yaw. The user's field
        # convention says a negative bias produces the required left correction.
        result = CALIBRATION.recommended_bias_value(-0.3, -0.1, 1.0, True)
        self.assertAlmostEqual(result, -0.4)

    def test_alternate_bias_convention_reverses_adjustment(self):
        result = CALIBRATION.recommended_bias_value(0.0, -0.1, 1.0, False)
        self.assertAlmostEqual(result, 0.1)

    def test_recommendation_gain_is_bounded(self):
        with self.assertRaises(ValueError):
            CALIBRATION.recommended_bias_value(0.0, 0.1, 1.1, True)


class SummaryTest(unittest.TestCase):
    def test_summary_reports_fitted_error_and_next_bias(self):
        node = CALIBRATION.SteerCenterCalibration.__new__(
            CALIBRATION.SteerCenterCalibration
        )
        node.lock = threading.Lock()
        node.raw_emergency_status = {}
        node.odom_topic = "/gps/odom"
        node.raw_heading_topic = "/gps/heading"
        node.cmd_topic = "/cmd_vel"
        node.speed_mps = 0.2
        node.distance_m = 5.0
        node.current_bias_deg = -0.3
        node.current_bias_was_explicit = True
        node.recommendation_gain = 1.0
        node.negative_bias_corrects_left = True
        node.use_raw_can_emergency_check = True
        node.external_estop_override = False
        node.use_progress_watchdog = True
        node.command_speed_limit_mps = 0.30
        node.hard_measured_speed_limit_mps = 0.50
        node.max_heading_rate_deg_s = 20.0
        node.heading_rate_abort_count = 1
        node.csv_path = "/tmp/synthetic.csv"
        node.expected_odom_publisher = "/gps_localization"
        node.expected_driver_node = "/m2_driver"
        node.expected_canbus_node = "/canbus_driver"
        node.command_lease_sec = 0.3
        node.run_timeout_sec = 40.0
        node.no_motion_timeout_sec = 4.0
        node.no_progress_timeout_sec = 3.0
        node.progress_step_m = 0.02
        node.max_measured_speed_mps = 0.35
        node.max_pose_step_m = 0.25
        node.max_heading_change_deg = 10.0
        node.max_lateral_deviation_m = 0.75
        node.raw_emergency_timeout_sec = 2.5
        node.raw_emergency_query_interval_sec = 0.2
        node.fit_trim_start_m = 0.5
        node.fit_trim_end_m = 0.5
        node.auxiliary_values_at = lambda _receipt: (93.0, 0.01, 0.0, 0.01)

        curvature = -0.01
        points = [
            arc_motion_point(index / 10.0, curvature)
            for index in range(51)
        ]
        baseline = {
            "samples": [points[0].state] * 20,
            "start_x": 0.0,
            "start_y": 0.0,
            "start_yaw": 0.0,
            "yaw_std": math.radians(0.1),
            "raw_heading": 90.0,
            "wheel_angle_mean": 0.0,
            "wheel_angle_std": math.radians(0.05),
        }
        summary = node.build_summary(
            "completed",
            "synthetic",
            0.65,
            "test",
            baseline,
            points,
            [points[-1].state],
            5.0,
        )
        expected_error = math.degrees(math.atan(0.65 * curvature))
        self.assertAlmostEqual(
            summary["equivalent_front_steer_error_deg_left_positive"],
            expected_error,
        )
        self.assertAlmostEqual(
            summary["suggested_next_absolute_bias_deg"],
            -0.3 + expected_error,
        )
        self.assertAlmostEqual(summary["gps_heading_change_deg_right_positive"], 3.0)
        self.assertFalse(summary["recommendation_suppressed"])

    def test_aborted_run_never_contains_bias_suggestion(self):
        node = CALIBRATION.SteerCenterCalibration.__new__(
            CALIBRATION.SteerCenterCalibration
        )
        node.lock = threading.Lock()
        node.raw_emergency_status = {}
        node.odom_topic = "/gps/odom"
        node.raw_heading_topic = "/gps/heading"
        node.cmd_topic = "/cmd_vel"
        node.speed_mps = 0.2
        node.distance_m = 5.0
        node.current_bias_deg = 0.0
        node.current_bias_was_explicit = True
        node.recommendation_gain = 0.5
        node.negative_bias_corrects_left = True
        node.use_raw_can_emergency_check = True
        node.external_estop_override = False
        node.use_progress_watchdog = True
        node.command_speed_limit_mps = 0.30
        node.hard_measured_speed_limit_mps = 0.50
        node.max_heading_rate_deg_s = 20.0
        node.heading_rate_abort_count = 1
        node.csv_path = "/tmp/synthetic.csv"
        node.expected_odom_publisher = "/gps_localization"
        node.expected_driver_node = "/m2_driver"
        node.expected_canbus_node = "/canbus_driver"
        node.command_lease_sec = 0.3
        node.run_timeout_sec = 40.0
        node.no_motion_timeout_sec = 4.0
        node.no_progress_timeout_sec = 3.0
        node.progress_step_m = 0.02
        node.max_measured_speed_mps = 0.35
        node.max_pose_step_m = 0.25
        node.max_heading_change_deg = 10.0
        node.max_lateral_deviation_m = 0.75
        node.raw_emergency_timeout_sec = 2.5
        node.raw_emergency_query_interval_sec = 0.2
        node.fit_trim_start_m = 0.5
        node.fit_trim_end_m = 0.5
        node.auxiliary_values_at = lambda _receipt: (90.0, 0.01, 0.0, 0.01)
        points = [arc_motion_point(index / 10.0, 0.01) for index in range(51)]
        baseline = {
            "samples": [points[0].state] * 20,
            "start_x": 0.0,
            "start_y": 0.0,
            "start_yaw": 0.0,
            "yaw_std": math.radians(0.1),
            "raw_heading": 90.0,
            "wheel_angle_mean": 0.0,
            "wheel_angle_std": math.radians(0.05),
        }
        summary = node.build_summary(
            "aborted", "synthetic safety stop", 0.65, "test", baseline, points
        )
        self.assertTrue(summary["recommendation_suppressed"])
        self.assertNotIn("suggested_bias_increment_deg", summary)
        self.assertNotIn("suggested_next_absolute_bias_deg", summary)


if __name__ == "__main__":
    unittest.main()
