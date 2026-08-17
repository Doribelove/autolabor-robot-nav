#!/usr/bin/env python3

import importlib.util
import math
import pathlib
from types import SimpleNamespace
import unittest


MODULE_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "gps_localization_node.py"
)
SPEC = importlib.util.spec_from_file_location("gps_localization_node", MODULE_PATH)
GPS_LOCALIZATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GPS_LOCALIZATION)


class MotionSampleFreshnessTest(unittest.TestCase):
    def test_sample_expires_at_configured_timeout(self):
        self.assertTrue(GPS_LOCALIZATION.motion_sample_is_fresh(10.0, 9.1, 1.0))
        self.assertFalse(GPS_LOCALIZATION.motion_sample_is_fresh(10.0, 8.9, 1.0))

    def test_future_or_missing_sample_is_not_fresh(self):
        self.assertFalse(GPS_LOCALIZATION.motion_sample_is_fresh(10.0, 10.1, 1.0))
        self.assertFalse(GPS_LOCALIZATION.motion_sample_is_fresh(10.0, None, 1.0))


class HeadingPolicyTest(unittest.TestCase):
    @staticmethod
    def valid_heading():
        return {
            "solution_status": "SOL_COMPUTED",
            "position_type": "NARROW_INT",
            "heading_deg": 90.0,
        }

    def evaluate(self, heading, now_sec=10.0, sample_sec=9.8):
        return GPS_LOCALIZATION.evaluate_dual_antenna_heading(
            heading,
            now_sec,
            sample_sec,
            0.5,
            "SOL_COMPUTED",
            {"NARROW_INT"},
        )

    def test_strict_sources_block_missing_heading(self):
        yaw, state = self.evaluate(None, sample_sec=None)

        self.assertIsNone(yaw)
        self.assertEqual(state, "missing")
        for source in ("dual_antenna", "uniheading", "heading"):
            self.assertFalse(
                GPS_LOCALIZATION.navigation_heading_is_ready(source, yaw)
            )

    def test_stale_heading_is_rejected(self):
        yaw, state = self.evaluate(self.valid_heading(), sample_sec=9.0)

        self.assertIsNone(yaw)
        self.assertEqual(state, "stale")
        self.assertFalse(
            GPS_LOCALIZATION.navigation_heading_is_ready("dual_antenna", yaw)
        )

    def test_fresh_valid_heading_opens_strict_source(self):
        yaw, state = self.evaluate(self.valid_heading())

        self.assertEqual(state, "ok")
        self.assertAlmostEqual(yaw, 0.0)
        self.assertTrue(
            GPS_LOCALIZATION.navigation_heading_is_ready("dual_antenna", yaw)
        )

    def test_invalid_heading_quality_is_rejected(self):
        heading = self.valid_heading()
        heading["position_type"] = "SINGLE"

        yaw, state = self.evaluate(heading)

        self.assertIsNone(yaw)
        self.assertEqual(state, "invalid")

    def test_auto_and_gps_course_keep_fallback(self):
        for source in ("auto", "gps_course"):
            self.assertTrue(
                GPS_LOCALIZATION.navigation_heading_is_ready(source, None)
            )

    def test_unknown_heading_source_is_rejected(self):
        with self.assertRaises(ValueError):
            GPS_LOCALIZATION.validate_heading_source("unknown")

    def test_navigation_uses_guarded_yaw_while_raw_heading_stays_fresh(self):
        node = GPS_LOCALIZATION.GpsLocalizationNode.__new__(
            GPS_LOCALIZATION.GpsLocalizationNode
        )
        node.latest_heading = self.valid_heading()
        node.latest_heading["heading_deg"] = 92.4
        node.latest_heading_stamp = GPS_LOCALIZATION.rospy.Time.from_sec(10.0)
        node.latest_navigation_heading_yaw = 0.0
        node.heading_timeout = 0.5
        node.heading_required_solution_status = "SOL_COMPUTED"
        node.heading_required_position_types = {"NARROW_INT"}
        node.heading_source = "dual_antenna"

        yaw = node.latest_heading_yaw(
            GPS_LOCALIZATION.rospy.Time.from_sec(10.1)
        )

        self.assertAlmostEqual(yaw, 0.0)


class WheelOdomTimestampTest(unittest.TestCase):
    def test_callback_preserves_chassis_measurement_timestamp(self):
        node = GPS_LOCALIZATION.GpsLocalizationNode.__new__(
            GPS_LOCALIZATION.GpsLocalizationNode
        )
        source_stamp = GPS_LOCALIZATION.rospy.Time.from_sec(12.5)
        message = SimpleNamespace(header=SimpleNamespace(stamp=source_stamp))

        node.wheel_odom_cb(message)

        self.assertEqual(node.latest_wheel_odom_stamp, source_stamp)


class ParameterValidationTest(unittest.TestCase):
    def test_finite_range_and_strict_minimum(self):
        self.assertEqual(
            GPS_LOCALIZATION.validate_float_parameter(
                "alpha", 1.0, minimum=0.0, maximum=1.0
            ),
            1.0,
        )
        with self.assertRaises(ValueError):
            GPS_LOCALIZATION.validate_float_parameter("timeout", float("nan"))
        with self.assertRaises(ValueError):
            GPS_LOCALIZATION.validate_float_parameter(
                "min_dt", 0.0, minimum=0.0, minimum_inclusive=False
            )
        with self.assertRaises(ValueError):
            GPS_LOCALIZATION.validate_float_parameter(
                "direction_threshold", 1.1, minimum=0.0, maximum=1.0
            )

    def test_boolean_and_integer_parameters_are_strict(self):
        self.assertTrue(GPS_LOCALIZATION.validate_bool_parameter("enabled", "true"))
        self.assertFalse(GPS_LOCALIZATION.validate_bool_parameter("enabled", False))
        self.assertEqual(
            GPS_LOCALIZATION.validate_int_parameter("samples", 3, minimum=1),
            3,
        )
        with self.assertRaises(ValueError):
            GPS_LOCALIZATION.validate_bool_parameter("enabled", "yes")
        with self.assertRaises(ValueError):
            GPS_LOCALIZATION.validate_int_parameter("samples", 1.5, minimum=1)


class SignedSpeedTest(unittest.TestCase):
    def test_forward_course_keeps_positive_speed(self):
        speed = GPS_LOCALIZATION.signed_speed_from_course(1.2, 0.1, 0.0)
        self.assertAlmostEqual(speed, 1.2)

    def test_reverse_course_makes_speed_negative(self):
        speed = GPS_LOCALIZATION.signed_speed_from_course(0.8, math.pi, 0.0)
        self.assertAlmostEqual(speed, -0.8)

    def test_ambiguous_sideways_course_is_rejected(self):
        speed = GPS_LOCALIZATION.signed_speed_from_course(0.8, math.pi / 2.0, 0.0)
        self.assertIsNone(speed)

    def test_stationary_noise_is_zero(self):
        speed = GPS_LOCALIZATION.signed_speed_from_course(0.03, math.pi, 0.0)
        self.assertEqual(speed, 0.0)


class HeadingRateTest(unittest.TestCase):
    def test_heading_wrap_is_continuous(self):
        rate = GPS_LOCALIZATION.heading_rate_from_samples(
            math.radians(179.0),
            math.radians(-179.0),
            0.1,
            3.0,
        )
        self.assertAlmostEqual(rate, math.radians(20.0), places=6)

    def test_implausible_heading_jump_is_rejected(self):
        rate = GPS_LOCALIZATION.heading_rate_from_samples(0.0, 1.0, 0.1, 3.0)
        self.assertIsNone(rate)


class HeadingJumpGuardTest(unittest.TestCase):
    @staticmethod
    def make_guard(enabled=True, recovery_samples=3):
        return GPS_LOCALIZATION.HeadingJumpGuard(
            enabled=enabled,
            jump_threshold_rad=math.radians(1.5),
            recovery_tolerance_rad=math.radians(0.8),
            recovery_samples=recovery_samples,
            max_prediction_dt=1.0,
        )

    def test_straight_line_heading_jump_is_held_without_losing_output(self):
        guard = self.make_guard()
        guard.update(0.0, 1.0, 0.0)

        yaw, state, innovation = guard.update(math.radians(2.4), 1.1, 0.0)

        self.assertEqual(state, "rejected")
        self.assertAlmostEqual(yaw, 0.0)
        self.assertAlmostEqual(math.degrees(innovation), 2.4)
        self.assertTrue(guard.holding)

    def test_live_heading_resumes_only_after_consecutive_stable_samples(self):
        guard = self.make_guard(recovery_samples=3)
        guard.update(0.0, 1.0, 0.0)
        guard.update(math.radians(2.4), 1.1, 0.0)

        for index, heading_deg in enumerate((0.2, 0.1), start=1):
            yaw, state, _ = guard.update(
                math.radians(heading_deg), 1.1 + 0.1 * index, 0.0
            )
            self.assertEqual(state, "holding")
            self.assertAlmostEqual(yaw, 0.0)

        yaw, state, _ = guard.update(math.radians(0.15), 1.4, 0.0)

        self.assertEqual(state, "recovered")
        self.assertAlmostEqual(yaw, math.radians(0.15))
        self.assertFalse(guard.holding)

    def test_real_turn_matching_chassis_yaw_rate_is_not_rejected(self):
        guard = self.make_guard()
        guard.update(0.0, 1.0, math.radians(20.0))

        yaw, state, innovation = guard.update(
            math.radians(2.0), 1.1, math.radians(20.0)
        )

        self.assertEqual(state, "accepted")
        self.assertAlmostEqual(yaw, math.radians(2.0))
        self.assertAlmostEqual(innovation, 0.0)

    def test_held_heading_follows_chassis_turn_until_gnss_recovers(self):
        guard = self.make_guard()
        guard.update(0.0, 1.0, 0.0)
        guard.update(math.radians(2.4), 1.1, 0.0)

        yaw, state, _ = guard.update(
            math.radians(5.0), 1.2, math.radians(10.0)
        )

        self.assertEqual(state, "holding")
        self.assertAlmostEqual(yaw, math.radians(1.0))

    def test_missing_wheel_feedback_fails_open_before_a_rejection(self):
        guard = self.make_guard()
        guard.update(0.0, 1.0, None)

        yaw, state, _ = guard.update(math.radians(3.0), 1.1, None)

        self.assertEqual(state, "unmonitored")
        self.assertAlmostEqual(yaw, math.radians(3.0))
        self.assertFalse(guard.holding)

    def test_disabled_guard_preserves_live_heading(self):
        guard = self.make_guard(enabled=False)
        guard.update(0.0, 1.0, 0.0)

        yaw, state, _ = guard.update(math.radians(3.0), 1.1, 0.0)

        self.assertEqual(state, "disabled")
        self.assertAlmostEqual(yaw, math.radians(3.0))


class PositionSpeedTest(unittest.TestCase):
    def test_projection_preserves_reverse_sign(self):
        speed = GPS_LOCALIZATION.longitudinal_speed_from_positions(
            1.0, 0.0, 0.8, 0.0, 0.0, 0.1
        )
        self.assertAlmostEqual(speed, -2.0)

    def test_tiny_interval_and_implausible_speed_are_rejected(self):
        self.assertIsNone(
            GPS_LOCALIZATION.longitudinal_speed_from_positions(
                0.0, 0.0, 0.1, 0.0, 0.0, 0.01
            )
        )
        self.assertIsNone(
            GPS_LOCALIZATION.longitudinal_speed_from_positions(
                0.0, 0.0, 1.0, 0.0, 0.0, 0.1, max_abs_speed=3.5
            )
        )


class AntennaOffsetTest(unittest.TestCase):
    @staticmethod
    def make_node():
        node = GPS_LOCALIZATION.GpsLocalizationNode.__new__(
            GPS_LOCALIZATION.GpsLocalizationNode
        )
        node.gps_antenna_offset_x = -0.3
        node.gps_antenna_offset_y = -0.05
        return node

    def test_rear_right_antenna_is_shifted_to_chassis_center(self):
        node = self.make_node()

        base_x, base_y = node.antenna_to_base_position(10.0, 20.0, 0.0)

        self.assertAlmostEqual(base_x, 10.3)
        self.assertAlmostEqual(base_y, 20.05)

    def test_offset_rotates_with_vehicle_yaw(self):
        node = self.make_node()

        base_x, base_y = node.antenna_to_base_position(
            10.0, 20.0, math.pi / 2.0
        )

        self.assertAlmostEqual(base_x, 9.95)
        self.assertAlmostEqual(base_y, 20.3)


class PositionFilterMotionTest(unittest.TestCase):
    @staticmethod
    def make_node():
        node = GPS_LOCALIZATION.GpsLocalizationNode.__new__(
            GPS_LOCALIZATION.GpsLocalizationNode
        )
        node.filtered_x = 0.0
        node.filtered_y = 0.0
        node.max_fix_jump = 5.0
        node.stationary_speed_threshold = 0.05
        node.stationary_hold_radius = 0.8
        node.stationary_filter_alpha = 0.05
        node.position_filter_alpha = 0.25
        return node

    def test_wheel_speed_takes_priority_over_zero_rmc_speed(self):
        speed = GPS_LOCALIZATION.position_filter_motion_speed((-0.4, 0.2), 0.0)
        self.assertAlmostEqual(speed, 0.4)

        node = self.make_node()
        x, _ = node.filter_position(0.5, 0.0, speed)
        self.assertAlmostEqual(x, 0.125)

    def test_missing_wheel_twist_keeps_stationary_rmc_hold(self):
        speed = GPS_LOCALIZATION.position_filter_motion_speed(None, 0.0)
        node = self.make_node()

        x, _ = node.filter_position(0.5, 0.0, speed)

        self.assertEqual(x, 0.0)


class NodeMotionSelectionTest(unittest.TestCase):
    @staticmethod
    def make_node():
        node = GPS_LOCALIZATION.GpsLocalizationNode.__new__(
            GPS_LOCALIZATION.GpsLocalizationNode
        )
        node.last_yaw = 0.0
        node.last_x = None
        node.last_y = None
        node.last_stamp = None
        node.last_linear_speed = 0.0
        node.heading_source = "dual_antenna"
        node.heading_min_speed = 0.15
        node.min_course_distance = 0.5
        node.stationary_speed_threshold = 0.05
        node.rmc_direction_cos_threshold = 0.5
        node.position_speed_min_dt = 0.05
        node.position_speed_max_abs = 3.5
        return node

    def test_wheel_twist_does_not_replace_dual_antenna_pose_yaw(self):
        node = self.make_node()
        node.latest_heading_yaw = lambda _stamp: 0.7
        node.latest_wheel_twist = lambda _stamp: (-0.4, -0.2)

        yaw, linear_speed, angular_speed = node.update_motion(
            object(), 0.0, 0.0, 180.0, 0.4
        )

        self.assertAlmostEqual(yaw, 0.7)
        self.assertAlmostEqual(linear_speed, -0.4)
        self.assertAlmostEqual(angular_speed, -0.2)

    def test_gnss_fallback_recovers_reverse_and_heading_rate(self):
        node = self.make_node()
        node.latest_heading_yaw = lambda _stamp: 0.0
        node.latest_wheel_twist = lambda _stamp: None
        node.latest_heading_rate = lambda _stamp: 0.3

        _, linear_speed, angular_speed = node.update_motion(
            object(), 0.0, 0.0, 270.0, 0.8
        )

        self.assertAlmostEqual(linear_speed, -0.8)
        self.assertAlmostEqual(angular_speed, 0.3)


if __name__ == "__main__":
    unittest.main()
