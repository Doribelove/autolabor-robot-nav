#!/usr/bin/env python3

from dataclasses import replace
import math
import unittest

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
    matching_detections,
    renew_motion_lease_now,
    terminal_feedback_is_fresh,
    terminal_sensor_fence_unchanged,
    validate_detection,
)


WIDTH = 1280
HEIGHT = 1024
TARGET_U = 620.0


def detection(
    u=TARGET_U,
    q=0.50,
    class_id=0,
    class_name="Metal",
    confidence=0.90,
    box_width=100.0,
    box_height=80.0,
):
    v = q * (HEIGHT - 1)
    return PixelDetection(
        class_id=class_id,
        class_name=class_name,
        confidence=confidence,
        x=u - 0.5 * box_width,
        y=v - box_height,
        width=box_width,
        height=box_height,
        anchor_u=u,
        anchor_v=v,
    )


class PixelControllerTest(unittest.TestCase):
    def test_image_right_commands_negative_ros_yaw_curvature(self):
        error = horizontal_error(TARGET_U + 128.0, TARGET_U, WIDTH)
        curvature = curvature_from_pixel_error(
            error, gain=0.65, steering_sign=-1.0, deadband=0.0, max_curvature=0.4
        )
        self.assertGreater(error, 0.0)
        self.assertLess(curvature, 0.0)

    def test_image_left_commands_positive_ros_yaw_curvature(self):
        error = horizontal_error(TARGET_U - 128.0, TARGET_U, WIDTH)
        curvature = curvature_from_pixel_error(
            error, gain=0.65, steering_sign=-1.0, deadband=0.0, max_curvature=0.4
        )
        self.assertLess(error, 0.0)
        self.assertGreater(curvature, 0.0)

    def test_deadband_centers_steering(self):
        curvature = curvature_from_pixel_error(
            0.01, gain=0.65, steering_sign=-1.0, deadband=0.025, max_curvature=0.4
        )
        self.assertEqual(curvature, 0.0)

    def test_speed_reduces_near_image_bottom(self):
        far = approach_speed(0.50, 0.0, 0.15, 0.06, 0.65, 0.82, 0.45, 0.35)
        near = approach_speed(0.90, 0.0, 0.15, 0.06, 0.65, 0.82, 0.45, 0.35)
        lateral = approach_speed(0.50, 0.45, 0.15, 0.06, 0.65, 0.82, 0.45, 0.35)
        self.assertAlmostEqual(far, 0.15)
        self.assertAlmostEqual(near, 0.06)
        self.assertLess(lateral, far)

    def test_malformed_pixel_detection_is_rejected(self):
        invalid = detection(u=1400.0)
        with self.assertRaises(ValueError):
            validate_detection(invalid, WIDTH, HEIGHT)


class TargetPhaseMachineTest(unittest.TestCase):
    def setUp(self):
        self.config = TargetMachineConfig(
            acquire_frames=3,
            bottom_fraction=0.88,
            bottom_center_tolerance_fraction=0.05,
            bottom_confirm_frames=3,
            min_approach_distance_m=0.10,
            min_vertical_progress_fraction=0.06,
            loss_confirm_frames=5,
            loss_confirm_min_sec=0.20,
            early_loss_max_frames=4,
            filter_alpha=0.35,
        )
        self.machine = TargetPhaseMachine(self.config, AssociationConfig())
        self.stamp = 1.0

    def step(self, candidates, distance=0.0, dt=0.05):
        self.stamp += dt
        return self.machine.process_frame(
            candidates,
            WIDTH,
            HEIGHT,
            TARGET_U,
            distance,
            self.stamp,
        )

    def acquire(self):
        decision = None
        for _ in range(self.config.acquire_frames):
            decision = self.step([detection()])
        self.assertEqual(decision.state, APPROACH)
        self.assertTrue(decision.acquired)
        return decision

    def arm_bottom_gate(self):
        self.acquire()
        decision = None
        # Move down in small spatially-associated increments, then hold long
        # enough for both the pixel EMA and consecutive bottom gate.
        for index in range(1, 19):
            q = 0.50 + index * (0.42 / 18.0)
            decision = self.step([detection(q=q)], distance=0.20)
        for _ in range(8):
            decision = self.step([detection(q=0.92)], distance=0.20)
        self.assertEqual(decision.state, EDGE_ARMED)
        return decision

    def test_requires_consecutive_unique_frames_to_acquire(self):
        first = self.step([detection()])
        ambiguous = self.step([detection(), detection(u=850.0)])
        self.assertEqual(first.state, ACQUIRE)
        self.assertEqual(ambiguous.state, ACQUIRE)
        self.assertIn("multiple", ambiguous.reason)
        self.assertIsNone(ambiguous.target)

    def test_early_loss_stops_and_never_enters_blind_transition(self):
        self.acquire()
        first_loss = self.step([], distance=0.02)
        self.assertEqual(first_loss.state, REACQUIRE)
        self.assertFalse(first_loss.enter_steer_settle)
        decision = first_loss
        for _ in range(self.config.early_loss_max_frames - 1):
            decision = self.step([], distance=0.02)
        self.assertTrue(decision.fault)
        self.assertNotEqual(decision.state, STEER_SETTLE)

    def test_bottom_gate_and_five_fresh_empty_frames_enable_settle(self):
        self.arm_bottom_gate()
        decision = self.step([], distance=0.20, dt=0.06)
        self.assertEqual(decision.state, LOSS_CONFIRM)
        self.assertFalse(decision.enter_steer_settle)
        for _ in range(3):
            decision = self.step([], distance=0.20, dt=0.06)
            self.assertFalse(decision.enter_steer_settle)
        decision = self.step([], distance=0.20, dt=0.06)
        self.assertEqual(decision.state, STEER_SETTLE)
        self.assertTrue(decision.enter_steer_settle)

    def test_single_bottom_dropout_can_reacquire_same_target(self):
        self.arm_bottom_gate()
        self.assertEqual(self.step([], distance=0.20).state, LOSS_CONFIRM)
        reacquired = self.step([detection(q=0.92)], distance=0.20)
        self.assertEqual(reacquired.state, EDGE_ARMED)
        self.assertFalse(reacquired.fault)

    def test_prearm_dropout_resets_consecutive_bottom_frames(self):
        self.acquire()
        # alpha=0.35 needs a gradual descent before bottom-ready frames.
        for index in range(1, 19):
            q = 0.50 + index * (0.42 / 18.0)
            self.step([detection(q=q)], distance=0.20)
        self.machine.bottom_hits = self.config.bottom_confirm_frames - 1
        lost = self.step([], distance=0.20)
        self.assertEqual(lost.state, REACQUIRE)
        self.assertEqual(self.machine.bottom_hits, 0)
        recovered = self.step([detection(q=0.92)], distance=0.20)
        self.assertEqual(recovered.state, APPROACH)
        self.assertEqual(self.machine.bottom_hits, 1)

    def test_distant_interference_does_not_replace_locked_target(self):
        self.acquire()
        locked = detection(u=625.0, q=0.51)
        interference = detection(u=1000.0, q=0.30, class_id=2, class_name="Plastic")
        decision = self.step([interference, locked], distance=0.01)
        self.assertEqual(decision.state, APPROACH)
        self.assertAlmostEqual(decision.target.anchor_u, locked.anchor_u)

    def test_two_plausible_matches_abort_instead_of_switching(self):
        self.acquire()
        left = detection(u=605.0, q=0.51)
        right = detection(u=635.0, q=0.51)
        decision = self.step([left, right], distance=0.01)
        self.assertTrue(decision.fault)
        self.assertIn("multiple", decision.fault)

    def test_raw_lateral_jump_cannot_hide_behind_pixel_filter_before_loss(self):
        self.arm_bottom_gate()
        # 160 px remains within the association distance on a 1280x1024
        # image, while EMA(alpha=.35) moves only 56 px.  Safety gating must use
        # the raw observation as well as the filtered control value.
        jumped = self.step([detection(u=TARGET_U + 160.0, q=0.92)], distance=0.20)
        self.assertTrue(jumped.fault)
        self.assertIn("laterally", jumped.fault)

    def test_raw_out_of_gate_frames_do_not_accumulate_bottom_confirmation(self):
        self.acquire()
        for index in range(1, 19):
            q = 0.50 + index * (0.42 / 18.0)
            self.step([detection(q=q)], distance=0.20)
        for offset in (70.0, -70.0, 70.0, -70.0):
            decision = self.step(
                [detection(u=TARGET_U + offset, q=0.92)], distance=0.20
            )
            self.assertNotEqual(decision.state, EDGE_ARMED)
            self.assertEqual(self.machine.bottom_hits, 0)

    def test_matching_can_survive_brief_class_jitter_without_target_jump(self):
        original = detection()
        jittered = detection(u=623.0, q=0.505, class_id=4, class_name="Tool")
        matches = matching_detections(
            original, [jittered], WIDTH, HEIGHT, AssociationConfig()
        )
        self.assertEqual(matches, (jittered,))


class BlindOdometryAndLeaseTest(unittest.TestCase):
    def test_first_loss_pose_is_interpolated_at_its_source_timestamp(self):
        ratio, x, y, yaw = interpolate_planar_pose(
            10.0,
            1.0,
            2.0,
            math.radians(179.0),
            10.2,
            1.2,
            2.4,
            math.radians(-179.0),
            10.05,
        )
        self.assertAlmostEqual(ratio, 0.25)
        self.assertAlmostEqual(x, 1.05)
        self.assertAlmostEqual(y, 2.10)
        self.assertAlmostEqual(math.degrees(yaw), 179.5)

    def test_pose_interpolation_rejects_unbracketed_timestamp(self):
        with self.assertRaises(ValueError):
            interpolate_planar_pose(1.0, 0.0, 0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 2.1)

    def test_blind_distance_uses_odometry_and_reaches_half_meter(self):
        tracker = BlindDistanceTracker(1.0, 2.0, 0.0)
        progress = tracker.update(1.49, 2.0, 0.0, max_pose_step_m=0.50)
        self.assertAlmostEqual(progress.path_m, 0.49)
        self.assertLess(progress.path_m, 0.50)
        progress = tracker.update(1.50, 2.0, 0.0, max_pose_step_m=0.10)
        self.assertAlmostEqual(progress.path_m, 0.50)
        self.assertAlmostEqual(progress.forward_m, 0.50)

    def test_blind_tracker_reports_lateral_and_heading_deviation(self):
        tracker = BlindDistanceTracker(0.0, 0.0, 0.0)
        progress = tracker.update(0.10, 0.04, math.radians(3.0), 0.20)
        self.assertAlmostEqual(progress.lateral_m, 0.04)
        self.assertAlmostEqual(math.degrees(progress.yaw_change_rad), 3.0)

    def test_blind_tracker_rejects_pose_jump(self):
        tracker = BlindDistanceTracker(0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            tracker.update(0.30, 0.0, 0.0, max_pose_step_m=0.15)

    def test_forward_then_backward_path_does_not_complete(self):
        tracker = BlindDistanceTracker(0.0, 0.0, 0.0)
        tracker.update(0.25, 0.0, 0.0, max_pose_step_m=0.30)
        progress = tracker.update(0.0, 0.0, 0.0, max_pose_step_m=0.30)
        self.assertAlmostEqual(progress.path_m, 0.50)
        self.assertAlmostEqual(progress.forward_m, 0.0)
        self.assertFalse(blind_goal_reached(progress, 0.50))

    def test_command_lease_fails_closed_before_point_three_seconds(self):
        lease = MotionLease(lease_sec=0.25)
        lease.set(0.08, 0.0, now=10.0, absolute_deadline=20.0)
        self.assertEqual(lease.sample(10.24)[:2], (0.08, 0.0))
        linear_x, curvature, reason = lease.sample(10.251)
        self.assertEqual((linear_x, curvature), (0.0, 0.0))
        self.assertIn("heartbeat", reason)
        with self.assertRaises(RuntimeError):
            lease.set(0.08, 0.0, now=10.26, absolute_deadline=20.0)

    def test_late_renewal_cannot_race_the_publisher_and_revive_command(self):
        lease = MotionLease(lease_sec=0.25)
        lease.set(0.08, 0.0, now=10.0, absolute_deadline=20.0)
        with self.assertRaises(RuntimeError):
            lease.set(0.08, 0.0, now=10.30, absolute_deadline=20.0)
        linear_x, curvature, reason = lease.sample(10.30)
        self.assertEqual((linear_x, curvature), (0.0, 0.0))
        self.assertIn("heartbeat", reason)

    def test_renewal_clock_is_sampled_after_a_hypothetical_lock_wait(self):
        lease = MotionLease(lease_sec=0.25)
        lease.set(0.08, 0.0, now=10.0, absolute_deadline=20.0)
        stale_time_sampled_before_lock = 10.24
        self.assertLess(stale_time_sampled_before_lock, lease.lease_deadline)

        with self.assertRaises(RuntimeError):
            renew_motion_lease_now(
                lease,
                0.08,
                0.0,
                absolute_deadline=20.0,
                clock=lambda: 10.26,
            )
        linear_x, curvature, reason = lease.sample(10.26)
        self.assertEqual((linear_x, curvature), (0.0, 0.0))
        self.assertIn("heartbeat", reason)

    def test_live_renewal_does_not_extend_absolute_deadline(self):
        lease = MotionLease(lease_sec=0.25)
        lease.set(0.08, 0.0, now=1.0, absolute_deadline=2.0)
        lease.set(0.08, 0.0, now=1.20, absolute_deadline=9.0)
        self.assertEqual(lease.absolute_deadline, 2.0)

    def test_absolute_deadline_also_fails_closed(self):
        lease = MotionLease(lease_sec=1.0)
        lease.set(0.08, 0.0, now=1.0, absolute_deadline=1.2)
        linear_x, _, reason = lease.sample(1.2)
        self.assertEqual(linear_x, 0.0)
        self.assertIn("absolute", reason)


class FreshConfirmationWindowTest(unittest.TestCase):
    def test_cached_safe_sample_cannot_advance_a_confirmation_window(self):
        first = advance_confirmation_window(9, 0, None, 10, 10.0, True, 0)
        cached = advance_confirmation_window(
            first.last_sequence,
            first.seen_unsafe_sequence,
            first.start_time,
            10,
            99.0,
            True,
            0,
        )
        self.assertTrue(first.new_sample)
        self.assertFalse(cached.new_sample)
        self.assertEqual(cached.start_time, 10.0)

    def test_unsafe_then_safe_in_one_control_batch_restarts_the_window(self):
        first = advance_confirmation_window(9, 0, None, 10, 10.0, True, 0)
        batch = advance_confirmation_window(
            first.last_sequence,
            first.seen_unsafe_sequence,
            first.start_time,
            12,
            10.5,
            True,
            11,
        )
        self.assertTrue(batch.new_sample)
        self.assertEqual(batch.seen_unsafe_sequence, 11)
        self.assertEqual(batch.start_time, 10.5)

    def test_unsafe_latch_may_lead_an_unconsumed_odom_sample(self):
        pending = advance_confirmation_window(10, 0, 10.0, 10, 10.0, True, 11)
        self.assertFalse(pending.new_sample)
        self.assertEqual(pending.last_sequence, 10)
        self.assertEqual(pending.seen_unsafe_sequence, 11)
        self.assertIsNone(pending.start_time)

    def test_lagging_safe_sample_cannot_restore_pre_unsafe_start_time(self):
        lagging = advance_confirmation_window(9, 9, 8.0, 10, 10.0, True, 11)
        self.assertIsNone(lagging.start_time)
        recovered = advance_confirmation_window(
            lagging.last_sequence,
            lagging.seen_unsafe_sequence,
            lagging.start_time,
            12,
            12.0,
            True,
            11,
        )
        self.assertEqual(recovered.start_time, 12.0)


class GraphPolicyTest(unittest.TestCase):
    def test_all_m2_bypass_publishers_are_reported_and_unrelated_topics_ignored(self):
        forbidden = (
            "/m2_driver/steer_center_bias",
            "/m2_driver/reset_odom",
            "/m2_driver/brake_set",
            "/m2_driver/emergency_stop",
        )
        publishers = [
            ("/fod/detections", ["/fod_detector"]),
            ("/m2_driver/reset_odom", ["/rqt_gui"]),
            ("/m2_driver/steer_center_bias", ["/calibrator"]),
            ("/m2_driver/reset_odom", ["/maintenance_tool"]),
        ]
        self.assertEqual(
            find_forbidden_publishers(publishers, forbidden),
            (
                (
                    "/m2_driver/reset_odom",
                    ("/maintenance_tool", "/rqt_gui"),
                ),
                ("/m2_driver/steer_center_bias", ("/calibrator",)),
            ),
        )

    def test_no_forbidden_publishers_is_an_empty_result(self):
        self.assertEqual(
            find_forbidden_publishers(
                [("/cmd_vel", ["/fod_visual_servo"])],
                ["/m2_driver/steer_center_bias"],
            ),
            (),
        )


class TerminalSensorFenceTest(unittest.TestCase):
    def test_any_unprocessed_feedback_or_fault_change_blocks_complete(self):
        expected = TerminalSensorFence(
            odom_sequence=100,
            wheel_sequence=200,
            detection_sequence=300,
            invalid_camera_generation=1,
            invalid_odom_generation=2,
            invalid_wheel_generation=3,
            chassis_fault_generation=4,
            raw_can_fault_generation=5,
            m2_bypass_event_generation=6,
            control_timeout_seen=False,
            detection_queue_size=0,
            odom_queue_size=0,
            detection_queue_overflow=False,
            odom_queue_overflow=False,
        )
        self.assertTrue(terminal_sensor_fence_unchanged(expected, expected))

        changed_fields = {
            "odom_sequence": 101,
            "wheel_sequence": 201,
            "detection_sequence": 301,
            "invalid_camera_generation": 2,
            "invalid_odom_generation": 3,
            "invalid_wheel_generation": 4,
            "chassis_fault_generation": 5,
            "raw_can_fault_generation": 6,
            "m2_bypass_event_generation": 7,
            "control_timeout_seen": True,
            "detection_queue_size": 1,
            "odom_queue_size": 1,
            "detection_queue_overflow": True,
            "odom_queue_overflow": True,
        }
        for field, value in changed_fields.items():
            with self.subTest(field=field):
                current = replace(expected, **{field: value})
                self.assertFalse(
                    terminal_sensor_fence_unchanged(expected, current)
                )

    def test_time_advance_without_callbacks_invalidates_terminal_feedback(self):
        receipt_limits = (
            (10.0, 0.35),
            (10.0, 0.60),
            (10.0, 0.60),
        )
        self.assertTrue(
            terminal_feedback_is_fresh(
                now_monotonic=10.20,
                now_source_time=100.20,
                receipt_limits=receipt_limits,
                source_stamps=(100.0, 100.0, 100.0),
                source_timeout=0.80,
                absolute_deadlines=(11.0, 11.0),
            )
        )
        self.assertFalse(
            terminal_feedback_is_fresh(
                now_monotonic=10.61,
                now_source_time=100.61,
                receipt_limits=receipt_limits,
                source_stamps=(100.0, 100.0, 100.0),
                source_timeout=0.80,
                absolute_deadlines=(11.0, 11.0),
            )
        )

    def test_expired_mode_or_final_deadline_blocks_terminal_commit(self):
        common = dict(
            now_source_time=100.10,
            receipt_limits=((10.0, 0.60),),
            source_stamps=(100.0,),
            source_timeout=0.80,
        )
        self.assertFalse(
            terminal_feedback_is_fresh(
                now_monotonic=10.50,
                absolute_deadlines=(10.50, 20.0),
                **common
            )
        )
        self.assertFalse(
            terminal_feedback_is_fresh(
                now_monotonic=10.50,
                absolute_deadlines=(20.0, 10.40),
                **common
            )
        )


if __name__ == "__main__":
    unittest.main()
