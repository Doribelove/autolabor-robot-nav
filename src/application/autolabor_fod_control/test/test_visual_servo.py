#!/usr/bin/env python3

from dataclasses import replace
import importlib.util
import math
import pathlib
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import yaml

from autolabor_fod_control.visual_servo import (
    ACQUIRE,
    BLIND_ADVANCE,
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
    depth_rejection_reason,
    find_forbidden_publishers,
    horizontal_error,
    interpolate_planar_pose,
    matching_detections,
    nearest_depth_target,
    pure_pursuit_curvature,
    renew_motion_lease_now,
    terminal_feedback_is_fresh,
    terminal_sensor_fence_unchanged,
    target_relative_geometry,
    validate_detection,
)


NODE_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "fod_visual_servo_node.py"
)
CONFIG_PATH = (
    pathlib.Path(__file__).resolve().parents[1] / "config" / "visual_servo.yaml"
)
LAUNCH_PATH = (
    pathlib.Path(__file__).resolve().parents[1] / "launch" / "visual_recovery.launch"
)
CFG_PATH = pathlib.Path(__file__).resolve().parents[1] / "cfg" / "VisualServo.cfg"
CMAKE_PATH = pathlib.Path(__file__).resolve().parents[1] / "CMakeLists.txt"
NODE_SPEC = importlib.util.spec_from_file_location(
    "fod_visual_servo_node_under_test", NODE_PATH
)
FOD_NODE = importlib.util.module_from_spec(NODE_SPEC)
sys.modules[NODE_SPEC.name] = FOD_NODE
NODE_SPEC.loader.exec_module(FOD_NODE)


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
    depth_valid=True,
    depth_m=2.0,
    depth_mad_m=0.02,
    depth_sample_count=200,
    depth_valid_fraction=0.90,
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
        depth_valid=depth_valid,
        depth_m=depth_m,
        depth_mad_m=depth_mad_m,
        depth_sample_count=depth_sample_count,
        depth_valid_fraction=depth_valid_fraction,
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


class AcquisitionCaptureRangeTest(unittest.TestCase):
    def test_expanded_vertical_capture_boundaries_and_six_frame_lock(self):
        for q, allowed in ((.029, False), (.03, True), (.05, True), (.10, True),
                           (.90, True), (.953, True), (.97, True), (.971, False)):
            with self.subTest(q=q):
                node = self.make_node()
                target = detection(q=q)
                for i in range(6):
                    candidates, reason = node._acquisition_candidates(self.frame(target))
                    decision = node.machine.process_frame(candidates, WIDTH, HEIGHT,
                                                          TARGET_U, 0., frame_stamp=10+i*.1)
                self.assertEqual(decision.acquired, allowed)
                self.assertEqual(bool(candidates), allowed)
                if not allowed:
                    self.assertIn("outside capture range 0.030..0.970", reason)

    @staticmethod
    def make_node():
        node = FOD_NODE.FodVisualServoNode.__new__(FOD_NODE.FodVisualServoNode)
        node.min_acquire_anchor_v_fraction = 0.03
        node.max_acquire_anchor_v_fraction = 0.97
        node.acquire_max_abs_horizontal_error = 0.65
        node.min_confidence = 0.30
        node.target_u_px = TARGET_U
        node.require_depth_for_acquisition = True
        node.nearest_depth_hysteresis_m = 0.10
        node.staged_target = None
        node.association_config = AssociationConfig()
        node.machine = TargetPhaseMachine(TargetMachineConfig(), node.association_config)
        return node

    @staticmethod
    def frame(candidate):
        return SimpleNamespace(
            candidates=(candidate,),
            width=WIDTH,
            height=HEIGHT,
        )

    def test_default_capture_range_matches_runtime_steering_envelope(self):
        with CONFIG_PATH.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.assertEqual(config["acquire_max_abs_horizontal_error"], 0.65)
        self.assertEqual(config["min_acquire_anchor_v_fraction"], 0.03)
        self.assertEqual(config["max_acquire_anchor_v_fraction"], 0.97)
        self.assertNotIn("max_runtime_horizontal_error", config)
        self.assertEqual(config["min_confidence"], 0.30)
        self.assertNotIn("raw_can_query_interval_sec", config)
        self.assertNotIn("canbus_service", config)
        self.assertEqual(config["raw_can_timeout_sec"], 8.00)
        self.assertTrue(config["require_depth_for_acquisition"])
        self.assertEqual(config["min_target_depth_m"], 0.35)
        self.assertEqual(config["max_target_depth_m"], 8.0)
        self.assertEqual(config["nearest_depth_hysteresis_m"], 0.10)
        self.assertEqual(config["locked_depth_invalid_abort_frames"], 5)
        self.assertEqual(config["association_max_anchor_distance_ratio"], 0.18)
        self.assertEqual(config["duplicate_detection_min_iou"], 0.80)
        self.assertEqual(
            config["duplicate_detection_max_anchor_distance_ratio"], 0.02
        )
        self.assertEqual(config["duplicate_detection_max_depth_delta_m"], 0.15)
        self.assertNotIn("early_loss_grace_frames", config)
        self.assertNotIn("early_loss_max_frames", config)
        self.assertEqual(config["far_speed_mps"], 0.20)
        self.assertEqual(config["near_speed_mps"], 0.20)
        self.assertEqual(config["approach_min_command_speed_mps"], 0.20)
        self.assertEqual(config["detection_timeout_sec"], 0.50)
        self.assertEqual(config["metric_results_timeout_sec"], 0.60)
        self.assertEqual(config["cleaning_coverage_tolerance_m"], 0.10)
        self.assertEqual(config["source_stamp_timeout_sec"], 0.50)
        self.assertEqual(config["max_linear_acceleration_mps2"], 4.00)
        self.assertEqual(config["blind_speed_mps"], 0.20)

    def test_launch_confidence_override_matches_yaml_default(self):
        with CONFIG_PATH.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        root = ET.parse(str(LAUNCH_PATH)).getroot()
        launch_arg = next(
            item for item in root.findall("arg") if item.get("name") == "min_confidence"
        )
        self.assertEqual(float(launch_arg.get("default")), config["min_confidence"])

    def test_lock_confidence_is_exposed_as_a_bounded_dynamic_parameter(self):
        cfg = CFG_PATH.read_text(encoding="utf-8")
        cmake = CMAKE_PATH.read_text(encoding="utf-8")
        self.assertIn('"min_confidence"', cfg)
        self.assertRegex(
            cfg,
            r'"min_confidence",\s*double_t,[\s\S]*?0\.30,\s*0\.25,\s*0\.95',
        )
        self.assertIn("generate_dynamic_reconfigure_options(", cmake)
        self.assertIn("cfg/VisualServo.cfg", cmake)

    def test_visual_controller_observes_can_but_never_owns_periodic_queries(self):
        source = NODE_PATH.read_text(encoding="utf-8")
        self.assertNotIn("CanBusService", source)
        self.assertNotIn("/canbus_server", source)
        self.assertNotIn("canbus_proxy", source)
        self.assertIn("def _check_raw_can(self):", source)
        self.assertIn('"raw_can_query_owner": self.expected_driver_node', source)

    def test_launch_exposes_the_approved_model_contract(self):
        with CONFIG_PATH.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        root = ET.parse(str(LAUNCH_PATH)).getroot()
        launch_args = {
            item.get("name"): item.get("default") for item in root.findall("arg")
        }
        self.assertEqual(
            config["expected_model_sha256"],
            launch_args["expected_model_sha256"],
        )
        self.assertEqual(
            config["allowed_class_names"],
            launch_args["allowed_class_names"].split(","),
        )
        node = root.find("node")
        parameters = {
            item.get("name"): item.get("value") for item in node.findall("param")
        }
        self.assertEqual(
            "$(arg expected_model_sha256)", parameters["expected_model_sha256"]
        )
        self.assertEqual(
            "$(arg allowed_class_names)", parameters["allowed_class_names"]
        )

    def test_stable_targets_well_left_or_right_of_center_can_be_acquired(self):
        node = self.make_node()
        for signed_error in (-0.60, 0.60):
            with self.subTest(signed_error=signed_error):
                candidate = detection(
                    u=TARGET_U + signed_error * 0.5 * WIDTH,
                    q=0.50,
                )
                candidates, reason = node._acquisition_candidates(
                    self.frame(candidate)
                )
                self.assertEqual(candidates, (candidate,))
                self.assertEqual(reason, "")

    def test_target_outside_capture_range_stays_stopped_with_clear_reason(self):
        node = self.make_node()
        candidate = detection(u=TARGET_U - 0.66 * 0.5 * WIDTH, q=0.50)
        candidates, reason = node._acquisition_candidates(self.frame(candidate))
        self.assertEqual(candidates, tuple())
        self.assertIn("exceeds steering capture limit 0.650", reason)

    def test_visible_low_confidence_target_reports_actual_rejection_reason(self):
        node = self.make_node()
        low_confidence = detection(confidence=0.29)
        frame = SimpleNamespace(
            candidates=tuple(),
            observations=(low_confidence,),
            width=WIDTH,
            height=HEIGHT,
        )
        candidates, reason = node._acquisition_candidates(frame)
        self.assertEqual(candidates, tuple())
        self.assertIn("confidence 0.290", reason)
        self.assertIn("threshold 0.300", reason)

    def test_multiple_targets_select_the_nearest_valid_depth(self):
        node = self.make_node()
        farther = detection(u=TARGET_U - 120.0, depth_m=3.20)
        nearest = detection(u=TARGET_U + 90.0, depth_m=1.45)
        frame = SimpleNamespace(
            candidates=(farther, nearest),
            observations=(farther, nearest),
            width=WIDTH,
            height=HEIGHT,
        )
        candidates, reason = node._acquisition_candidates(frame)
        self.assertEqual(candidates, (nearest,))
        self.assertEqual(reason, "")

    def test_pending_target_hysteresis_prevents_depth_jitter_switch(self):
        node = self.make_node()
        pending = detection(u=TARGET_U - 100.0, depth_m=2.00)
        node.machine.pending = pending
        same_target = detection(u=TARGET_U - 96.0, depth_m=2.06)
        # This second object is still inside the broad association envelope;
        # the best spatial match must retain the pending target.
        marginally_nearer = detection(u=TARGET_U + 100.0, depth_m=2.00)
        frame = SimpleNamespace(
            candidates=(same_target, marginally_nearer),
            observations=(same_target, marginally_nearer),
            width=WIDTH,
            height=HEIGHT,
        )
        candidates, _ = node._acquisition_candidates(frame)
        self.assertEqual(candidates, (same_target,))

    def test_clear_nearest_target_change_resets_acquisition_hits(self):
        node = self.make_node()
        pending = detection(u=TARGET_U - 100.0, depth_m=2.00)
        node.machine.pending = pending
        node.machine.pending_hits = 4
        same_target = detection(u=TARGET_U - 96.0, depth_m=2.05)
        clearly_nearer = detection(u=TARGET_U + 100.0, depth_m=1.70)
        frame = SimpleNamespace(
            candidates=(same_target, clearly_nearer),
            observations=(same_target, clearly_nearer),
            width=WIDTH,
            height=HEIGHT,
        )
        candidates, _ = node._acquisition_candidates(frame)
        self.assertEqual(candidates, (clearly_nearer,))
        self.assertIsNone(node.machine.pending)
        self.assertEqual(node.machine.pending_hits, 0)

    def test_unusable_depth_has_an_explicit_motion_rejection(self):
        target = detection(depth_valid=False, depth_m=float("nan"))
        reason = depth_rejection_reason(target, 0.35, 15.0, 20, 0.20, 0.35)
        self.assertIn("unavailable", reason)
        self.assertIsNone(nearest_depth_target((target,)))

    def test_off_center_target_keeps_proven_chassis_start_speed(self):
        error = -0.60
        shaped_speed = approach_speed(
            0.50, abs(error), 0.20, 0.20, 0.65, 0.82, 0.45, 0.35
        )
        speed = max(0.20, shaped_speed)
        max_curvature = math.tan(math.radians(12.0)) / 0.65
        curvature = curvature_from_pixel_error(
            error,
            gain=0.65,
            steering_sign=-1.0,
            deadband=0.025,
            max_curvature=max_curvature,
        )
        self.assertAlmostEqual(speed, 0.20)
        self.assertGreater(curvature, 0.0)


class ApprovedBottomGateTest(unittest.TestCase):
    @staticmethod
    def configured_node():
        with CONFIG_PATH.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        node = FOD_NODE.FodVisualServoNode.__new__(FOD_NODE.FodVisualServoNode)
        for name, value in config.items():
            setattr(node, name, value)
        node.machine_config = TargetMachineConfig(**{
            name: config.get("pixel_filter_alpha" if name == "filter_alpha" else name, field.default)
            for name, field in TargetMachineConfig.__dataclass_fields__.items()
        })
        node.association_config = AssociationConfig()
        node.m2_bypass_topics = [node.steer_center_bias_topic, node.reset_odom_topic,
                                 node.brake_set_topic, node.emergency_stop_topic]
        return node

    def test_simple_production_completion_parameters(self):
        node = self.configured_node()
        node._validate_parameters()
        self.assertEqual(node.machine_config.loss_confirm_frames, 10)
        self.assertEqual(node.blind_distance_m, 0.50)
        self.assertEqual((node.far_speed_mps, node.blind_speed_mps), (0.20, 0.20))


class ApprovedVisualRangeAndStopTest(unittest.TestCase):
    def test_all_launch_distance_defaults_match_production_yaml(self):
        config = ApprovedBottomGateTest.configured_node()
        for path in (LAUNCH_PATH, LAUNCH_PATH.with_name("gps_visual_recovery_standby.launch")):
            root = ET.parse(str(path)).getroot()
            self.assertEqual(float(root.find("arg[@name='blind_distance_m']").get("default")),
                             config.blind_distance_m, str(path))

    def test_depth_range_includes_eight_metres_but_not_more(self):
        node = ApprovedBottomGateTest.configured_node()
        for depth, accepted in ((0.349, False), (0.35, True), (5.7, True),
                                (8.0, True), (8.000001, False)):
            with self.subTest(depth=depth):
                reason = depth_rejection_reason(
                    detection(depth_m=depth), node.min_target_depth_m,
                    node.max_target_depth_m, node.min_depth_sample_count,
                    node.min_depth_valid_fraction, node.max_depth_mad_m)
                self.assertEqual(not reason, accepted)

    def test_launch_entry_limits_match_visual_depth_range(self):
        config = ApprovedBottomGateTest.configured_node()
        standby = ET.parse(str(LAUNCH_PATH.with_name("gps_visual_recovery_standby.launch"))).getroot()
        entry_arg = standby.find("arg[@name='fod_entry_distance_m']")
        self.assertEqual(float(entry_arg.get("default")), config.max_target_depth_m)
        integrated = ET.parse(str(CONFIG_PATH.parents[3] / "platform" / "autolabor_dual_host" /
                                  "launch" / "j6m_fastlio_navigation.launch")).getroot()
        entry_param = integrated.find(".//node[@name='fod_navigation_mode']/param[@name='fod_entry_distance_m']")
        self.assertIsNotNone(entry_param)
        self.assertEqual(float(entry_param.get("value")), config.max_target_depth_m)


class TargetPhaseMachineTest(unittest.TestCase):
    def setUp(self):
        self.machine = TargetPhaseMachine(TargetMachineConfig())
        self.stamp = 1.0

    def step(self, candidates, distance=0.0, dt=0.10):
        self.stamp += dt
        return self.machine.process_frame(candidates, WIDTH, HEIGHT, TARGET_U,
                                          distance, self.stamp)

    def lock(self, target=None):
        target = target or detection()
        for _ in range(6):
            decision = self.step((target,))
        self.assertTrue(decision.acquired)
        return target

    def test_no_acquired_target_never_completes(self):
        for _ in range(100):
            self.assertEqual(self.step(()).state, ACQUIRE)

    def test_exactly_ten_missing_frames_at_any_image_position(self):
        for q in (0.25, 0.5, 0.86, 0.99):
            for offset in (-200, 0, 200):
                with self.subTest(q=q, offset=offset):
                    self.machine.reset()
                    self.lock(detection(q=q, u=TARGET_U + offset))
                    for count in range(1, 11):
                        decision = self.step(())
                        self.assertFalse(decision.fault)
                        self.assertEqual(self.machine.missing_frames, count)
                        self.assertEqual(decision.state,
                                         LOSS_CONFIRM if count < 10 else BLIND_ADVANCE)

    def test_nine_misses_then_reappearance_resets_counter(self):
        target = self.lock()
        for _ in range(9):
            self.step(())
        self.assertEqual(self.step((target,)).state, APPROACH)
        self.assertEqual(self.machine.missing_frames, 0)
        for _ in range(9):
            self.assertEqual(self.step(()).state, LOSS_CONFIRM)
        self.assertEqual(self.step(()).state, BLIND_ADVANCE)

    def test_visible_low_confidence_or_invalid_depth_is_not_loss(self):
        target = self.lock()
        for _ in range(30):
            decision = self.step((replace(target, depth_valid=False,
                                           depth_m=math.nan, confidence=0.1),))
            self.assertEqual(decision.state, APPROACH)
            self.assertEqual(self.machine.missing_frames, 0)

    def test_other_distant_object_does_not_replace_lock_or_block_loss(self):
        target = self.lock(detection(u=100))
        other = detection(u=1100)
        for _ in range(10):
            decision = self.step((other,))
        self.assertEqual(decision.state, BLIND_ADVANCE)
        self.assertEqual(decision.target, target)

    def test_no_extra_time_gate(self):
        self.lock()
        for _ in range(10):
            decision = self.step((), dt=0.001)
        self.assertEqual(decision.state, BLIND_ADVANCE)

    def test_large_box_growth_retains_the_target(self):
        self.lock()
        decision = self.step((detection(box_width=400, box_height=200),))
        self.assertEqual(decision.state, APPROACH)
        self.assertFalse(decision.fault)

    def test_pending_acquisition_still_needs_six_frames(self):
        for _ in range(5):
            self.assertEqual(self.step((detection(),)).state, ACQUIRE)
        self.assertEqual(self.step((detection(),)).state, APPROACH)

    def test_pass_is_one_way_after_tenth_frame(self):
        target = self.lock()
        for _ in range(10):
            self.step(())
        self.assertEqual(self.step((target,)).state, BLIND_ADVANCE)

    def test_recorded_1605_five_bottom_frames_no_longer_need_six(self):
        # Recorded seq 2063--2067; pure target logic replay, not vehicle replay.
        def frame(u, v):
            self.stamp += 0.13
            candidates = () if u is None else (
                replace(detection(), x=u-30, y=v-30, width=60, height=30,
                        anchor_u=u, anchor_v=v),)
            return self.machine.process_frame(
                candidates, 640, 360, 317.458, 1.05, self.stamp)
        for _ in range(6):
            frame(279, 326)
        for u, v in ((279, 326), (280, 337), (276.5, 355),
                     (271.5, 359), (273.5, 359)):
            frame(u, v)
        for _ in range(10):
            decision = frame(None, None)
        self.assertEqual(decision.state, BLIND_ADVANCE)
        self.assertFalse(decision.fault)


class AreaIndependentAssociationTest(unittest.TestCase):
    @staticmethod
    def recorded_targets():
        # Actual /fod/detections frames from 2026-09-12 15:11:16--17,
        # seq 17936/17938/17939/17940. The last three touch the image bottom.
        boxes = ((306, 282, 58, 37), (259, 292, 113, 68),
                 (255, 296, 119, 64), (264, 304, 119, 56))
        depths = (0.7530065775, 0.7028478384, 0.6910436153, 0.6604517102)
        return tuple(
            PixelDetection(
                class_id=0, class_name="paper", confidence=0.47,
                x=x, y=y, width=width, height=height,
                anchor_u=x + (width - 1) / 2.0,
                anchor_v=y + height - 1,
                depth_valid=True, depth_m=depth, depth_mad_m=0.008,
                depth_sample_count=879, depth_valid_fraction=0.74,
            )
            for (x, y, width, height), depth in zip(boxes, depths)
        )

    def recorded_machine(self):
        # Seed only the phase machine from the last accepted live observation.
        # This is not a full sensor/vehicle replay or proof of physical cleaning.
        config = ApprovedBottomGateTest.configured_node().machine_config
        machine = TargetPhaseMachine(config, AssociationConfig())
        machine.state = APPROACH
        machine.locked = self.recorded_targets()[0]
        machine.filtered_u = 332.892
        machine.filtered_v = 300.132
        machine.acquired_v_fraction = 0.52368
        machine.maximum_v_fraction = 0.83602
        return machine

    def test_area_ratio_parameters_are_removed_from_production(self):
        with CONFIG_PATH.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        for name in ("min_area_ratio", "max_area_ratio"):
            self.assertNotIn(name, AssociationConfig.__dataclass_fields__)
            self.assertNotIn("association_" + name, config)
            self.assertNotIn("association_" + name, NODE_PATH.read_text())

    def test_size_growth_and_shrinkage_do_not_break_a_spatial_match(self):
        original = detection()
        for scale in (0.10, 0.50, 2.0, 4.0):
            with self.subTest(scale=scale):
                candidate = detection(box_width=100.0 * scale,
                                      box_height=80.0 * scale)
                validate_detection(candidate, WIDTH, HEIGHT)
                self.assertEqual(matching_detections(
                    original, (candidate,), WIDTH, HEIGHT, AssociationConfig()
                ), (candidate,))

    def test_recorded_bottom_boxes_all_match_the_last_locked_target(self):
        previous, *candidates = self.recorded_targets()
        for candidate in candidates:
            with self.subTest(box=(candidate.width, candidate.height)):
                self.assertEqual(matching_detections(
                    previous, (candidate,), 640, 360, AssociationConfig()
                ), (candidate,))

    def test_size_change_does_not_admit_a_distant_unrelated_target(self):
        original = detection(u=200.0, q=0.30)
        for scale in (0.25, 3.0):
            with self.subTest(scale=scale):
                candidate = detection(u=1000.0, q=0.75,
                                      box_width=100.0 * scale,
                                      box_height=80.0 * scale)
                self.assertEqual(matching_detections(
                    original, (candidate,), WIDTH, HEIGHT, AssociationConfig()
                ), ())

    def test_different_sizes_do_not_resolve_two_plausible_targets(self):
        machine = self.recorded_machine()
        previous, enlarged, *_ = self.recorded_targets()
        decision = machine.process_frame((previous, enlarged), 640, 360,
                                         317.459, 0.75, 1.0)
        self.assertIn("multiple detections", decision.fault)
        self.assertFalse(decision.enter_steer_settle)

    def test_recorded_three_bottom_frames_followed_by_ten_misses_start_pass(self):
        machine = self.recorded_machine()
        for index, target in enumerate(self.recorded_targets()[1:]):
            decision = machine.process_frame((target,), 640, 360, 317.459,
                                             0.75, 1.0 + index * 0.13)
            self.assertEqual(decision.state, APPROACH)
            self.assertFalse(decision.fault)
        for index in range(10):
            decision = machine.process_frame((), 640, 360, 317.459,
                                             0.99, 2.0 + index * 0.13)
        self.assertEqual(decision.state, BLIND_ADVANCE)
        self.assertFalse(decision.fault)


class MetricIdentityBindingTest(unittest.TestCase):
    def test_metric_requirements_are_exclusive_to_selected_pending_litter(self):
        node = FOD_NODE.FodVisualServoNode.__new__(FOD_NODE.FodVisualServoNode)
        node.staged_target = None
        node._check_metric_health({"metric": None})
        self.assertIsNone(node._update_forward_coverage(None, 0.05))
        node.staged_target = SimpleNamespace(object_id=42)
        with self.assertRaisesRegex(FOD_NODE.ControllerAbort, "no metric"):
            node._check_metric_health({"metric": None})

    @staticmethod
    def metric_for(pixel, object_id=42, x=3.0, y=4.0):
        return FOD_NODE.MetricTarget(
            object_id=object_id,
            track_id=7,
            class_name=pixel.class_name,
            confidence=pixel.confidence,
            x=x,
            y=y,
            z=0.0,
            world_frame="map",
            bbox_x=pixel.x,
            bbox_y=pixel.y,
            bbox_width=pixel.width,
            bbox_height=pixel.height,
            state="CONFIRMED",
        )

    @staticmethod
    def make_node(targets):
        node = FOD_NODE.FodVisualServoNode.__new__(FOD_NODE.FodVisualServoNode)
        node.sensor_lock = threading.Lock()
        node.metric_sync_tolerance_sec = 0.002
        node.metric_bbox_min_iou = 0.90
        node.metric_frames = (
            FOD_NODE.MetricFrame(
                sequence=1,
                receipt_monotonic=9.0,
                stamp_sec=100.0,
                width=WIDTH,
                height=HEIGHT,
                targets=tuple(targets),
                error="",
            ),
        )
        return node

    def test_pixel_binds_to_one_source_stamped_metric_identity(self):
        pixel = detection()
        node = self.make_node((self.metric_for(pixel),))
        frame = SimpleNamespace(stamp_sec=100.0)
        self.assertEqual(node._metric_target_for_pixel(frame, pixel).object_id, 42)

    def test_missing_or_duplicate_metric_identity_fails_closed(self):
        pixel = detection()
        frame = SimpleNamespace(stamp_sec=100.0)
        with self.assertRaisesRegex(FOD_NODE.ControllerAbort, "0 metric"):
            self.make_node(tuple())._metric_target_for_pixel(frame, pixel)
        targets = (self.metric_for(pixel, 42), self.metric_for(pixel, 43))
        with self.assertRaisesRegex(FOD_NODE.ControllerAbort, "2 metric"):
            self.make_node(targets)._metric_target_for_pixel(frame, pixel)

    def test_selected_object_and_map_position_cannot_silently_change(self):
        pixel = detection()
        node = self.make_node(tuple())
        node.metric_max_selected_position_delta_m = 0.25
        node.metric_max_position_jump_m = 0.10
        node.locked_metric_target = None
        node.staged_target = FOD_NODE.StagedTarget(
            object_id=42,
            x=3.0,
            y=4.0,
            frame="map",
            observed_at_sec=99.0,
            label="a",
            expires_monotonic=20.0,
        )
        with mock.patch.object(FOD_NODE.time, "monotonic", return_value=10.0):
            node._validate_selected_metric_target(self.metric_for(pixel, 42, 3.1, 4.0))
            with self.assertRaisesRegex(FOD_NODE.ControllerAbort, "identity"):
                node._validate_selected_metric_target(self.metric_for(pixel, 43))
            with self.assertRaisesRegex(FOD_NODE.ControllerAbort, "position changed"):
                node._validate_selected_metric_target(
                    self.metric_for(pixel, 42, 3.26, 4.0)
                )


class SelectedVisualPolicyTest(unittest.TestCase):
    def node(self):
        node = MetricIdentityBindingTest.make_node(tuple())
        node.staged_target = FOD_NODE.StagedTarget(42, 3., 4., 'map', 99., 'a', 1e12,
                                                 'visual_servo', 'request-a')
        node.metric_max_selected_position_delta_m = .25
        node.metric_max_position_jump_m = .10
        node.locked_metric_target = None
        node.require_depth_for_acquisition = False
        node.target_u_px = TARGET_U
        node.min_acquire_anchor_v_fraction = 0.
        node.max_acquire_anchor_v_fraction = 1.
        node.acquire_max_abs_horizontal_error = 1.
        node.last_cleaning_completion = {'sequence':2}
        node.blind_distance_m = .5
        return node

    def test_new_detection_id_can_rebind_only_near_selected_coordinate(self):
        node = self.node()
        pixel = detection()
        near = MetricIdentityBindingTest.metric_for(pixel, 77, 3.05, 4.)
        node._metric_target_for_pixel = lambda *_: near
        frame = SimpleNamespace(candidates=(pixel,), width=WIDTH, height=HEIGHT)
        candidates, _ = node._acquisition_candidates(frame)
        self.assertEqual(candidates, (pixel,))
        self.assertFalse(node._requires_coordinate_coverage())
        far = replace(near, x=3.5)
        node._metric_target_for_pixel = lambda *_: far
        self.assertEqual(node._acquisition_candidates(frame)[0], tuple())

    def test_two_nearby_targets_are_ambiguous_even_when_one_has_original_id(self):
        node = self.node()
        pixels = (detection(), detection(u=TARGET_U+100))
        node._metric_target_for_pixel = lambda frame, p: MetricIdentityBindingTest.metric_for(
            p, 42 if p == pixels[0] else 77, 3., 4.)
        candidates, reason = node._acquisition_candidates(SimpleNamespace(candidates=pixels))
        self.assertEqual(candidates, tuple())
        self.assertIn('not uniquely visible', reason)

    def test_locked_identity_still_cannot_jump(self):
        node = self.node()
        pixel = detection()
        node.locked_metric_target = MetricIdentityBindingTest.metric_for(pixel, 77, 3., 4.)
        with self.assertRaisesRegex(FOD_NODE.ControllerAbort, 'identity changed'):
            node._validate_selected_metric_target(MetricIdentityBindingTest.metric_for(pixel, 78, 3., 4.))

    def test_visual_completion_records_session_without_coordinate_gate(self):
        node = self.node()
        node.locked_metric_target = MetricIdentityBindingTest.metric_for(detection(), 77, 3., 4.)
        node._record_cleaning_completion(SimpleNamespace(forward_m=.51, path_m=.52))
        proof = node.last_cleaning_completion
        self.assertEqual(proof['sequence'], 3)
        self.assertEqual(proof['object_id'], '42')
        self.assertEqual(proof['acquired_object_id'], '77')
        self.assertEqual(proof['request_id'], 'request-a')
        self.assertFalse(proof['stopped'])  # Zero command is not physical-stop proof.
        self.assertNotIn('minimum_distance_m', proof)

    def test_legacy_selected_policy_and_ordinary_visual_are_unchanged(self):
        node = self.node()
        node.staged_target = replace(node.staged_target, completion_policy='')
        self.assertTrue(node._requires_coordinate_coverage())
        node.staged_target = None
        self.assertFalse(node._requires_coordinate_coverage())
        node._record_cleaning_completion(SimpleNamespace(forward_m=.51, path_m=.52))
        self.assertEqual(node.last_cleaning_completion, {'sequence':2})


class BlindOdometryAndLeaseTest(unittest.TestCase):
    def test_direction_change_segment_is_not_forward_coverage(self):
        tracker = ForwardCoverageTracker(
            target_x=0.54, target_y=0.0, reference_offset_m=0.52,
            tolerance_m=0.01, start_x=0.0, start_y=0.0, start_yaw=0.0,
            start_linear_x=0.20,
        )
        progress = tracker.update(0.04, 0.0, 0.0, -0.10, 0.05)
        self.assertFalse(progress.covered)
        self.assertEqual(progress.eligible_segments, 0)

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

    def test_front_cleaning_reference_covers_target_on_forward_segment(self):
        tracker = ForwardCoverageTracker(
            target_x=1.0,
            target_y=0.06,
            reference_offset_m=0.52,
            tolerance_m=0.10,
            start_x=0.0,
            start_y=0.0,
            start_yaw=0.0,
            start_linear_x=0.0,
        )
        progress = tracker.update(0.60, 0.0, 0.0, 0.20, 0.70)
        self.assertTrue(progress.covered)
        self.assertAlmostEqual(progress.minimum_distance_m, 0.06)
        self.assertEqual(progress.eligible_segments, 1)

    def test_reverse_crossing_never_counts_as_cleaning(self):
        tracker = ForwardCoverageTracker(
            target_x=0.75,
            target_y=0.0,
            reference_offset_m=0.52,
            tolerance_m=0.10,
            start_x=0.60,
            start_y=0.0,
            start_yaw=0.0,
            start_linear_x=0.0,
        )
        progress = tracker.update(0.0, 0.0, 0.0, -0.20, 0.70)
        self.assertFalse(progress.covered)
        self.assertEqual(progress.eligible_segments, 0)
        self.assertAlmostEqual(progress.reverse_path_m, 0.60)

    def test_sparse_forward_samples_cannot_skip_over_target(self):
        tracker = ForwardCoverageTracker(
            target_x=0.82,
            target_y=0.0,
            reference_offset_m=0.52,
            tolerance_m=0.10,
            start_x=0.0,
            start_y=0.0,
            start_yaw=0.0,
            start_linear_x=0.20,
        )
        progress = tracker.update(0.60, 0.0, 0.0, 0.20, 0.70)
        self.assertTrue(progress.covered)
        self.assertAlmostEqual(progress.minimum_distance_m, 0.0)

    def test_lateral_miss_outside_ten_centimeters_is_rejected(self):
        tracker = ForwardCoverageTracker(
            target_x=0.82,
            target_y=0.101,
            reference_offset_m=0.52,
            tolerance_m=0.10,
            start_x=0.0,
            start_y=0.0,
            start_yaw=0.0,
            start_linear_x=0.20,
        )
        progress = tracker.update(0.60, 0.0, 0.0, 0.20, 0.70)
        self.assertFalse(progress.covered)
        self.assertAlmostEqual(progress.minimum_distance_m, 0.101)

    def test_stationary_jitter_without_positive_speed_does_not_count(self):
        tracker = ForwardCoverageTracker(
            target_x=0.525,
            target_y=0.0,
            reference_offset_m=0.52,
            tolerance_m=0.10,
            start_x=0.0,
            start_y=0.0,
            start_yaw=0.0,
            start_linear_x=0.0,
        )
        progress = tracker.update(0.005, 0.0, 0.0, 0.0, 0.01)
        self.assertFalse(progress.covered)
        self.assertEqual(progress.eligible_segments, 0)

    def test_metric_geometry_and_curvature_are_vehicle_relative(self):
        distance, forward, lateral, bearing = target_relative_geometry(
            10.0, 20.0, math.pi / 2.0, 9.5, 22.0
        )
        self.assertAlmostEqual(distance, math.hypot(2.0, 0.5))
        self.assertAlmostEqual(forward, 2.0)
        self.assertAlmostEqual(lateral, 0.5)
        self.assertAlmostEqual(bearing, math.atan2(0.5, 2.0))
        self.assertAlmostEqual(
            pure_pursuit_curvature(forward, lateral, 0.50),
            1.0 / 4.25,
        )

    def test_metric_curvature_is_bounded_and_rejects_target_behind(self):
        self.assertEqual(pure_pursuit_curvature(0.10, 1.0, 0.40), 0.40)
        with self.assertRaises(ValueError):
            pure_pursuit_curvature(-0.10, 0.0, 0.40)

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


class DynamicLockConfidenceTest(unittest.TestCase):
    @staticmethod
    def make_node(phase):
        node = FOD_NODE.FodVisualServoNode.__new__(FOD_NODE.FodVisualServoNode)
        node.run_lock = threading.Lock()
        node.phase = phase
        node.min_confidence = 0.30
        node._publish_status = mock.Mock()
        return node

    def test_stopped_controller_accepts_and_reports_new_threshold(self):
        node = self.make_node(FOD_NODE.DISABLED)
        with mock.patch.object(FOD_NODE.rospy, "loginfo"):
            config = node._dynamic_reconfigure_cb({"min_confidence": 0.35}, 0)
        self.assertEqual(config["min_confidence"], 0.35)
        self.assertEqual(node.min_confidence, 0.35)
        node._publish_status.assert_called_once_with(force=True)

    def test_active_controller_keeps_current_threshold(self):
        node = self.make_node(FOD_NODE.APPROACH)
        with mock.patch.object(FOD_NODE.rospy, "logwarn"):
            config = node._dynamic_reconfigure_cb({"min_confidence": 0.25}, 0)
        self.assertEqual(config["min_confidence"], 0.30)
        self.assertEqual(node.min_confidence, 0.30)
        node._publish_status.assert_not_called()

    def test_non_finite_threshold_is_rejected(self):
        node = self.make_node(FOD_NODE.ABORT)
        with mock.patch.object(FOD_NODE.rospy, "logwarn"):
            config = node._dynamic_reconfigure_cb({"min_confidence": math.nan}, 0)
        self.assertEqual(config["min_confidence"], 0.30)
        self.assertEqual(node.min_confidence, 0.30)
        node._publish_status.assert_not_called()


class LockedDepthGraceTest(unittest.TestCase):
    @staticmethod
    def make_node(phase=FOD_NODE.APPROACH):
        node = FOD_NODE.FodVisualServoNode.__new__(FOD_NODE.FodVisualServoNode)
        node.phase = phase
        node.require_depth_for_acquisition = True
        node.locked_depth_invalid_abort_frames = 5
        node.locked_depth_invalid_frames = 0
        node.association_config = AssociationConfig()
        node.machine = TargetPhaseMachine(TargetMachineConfig(), node.association_config)
        node.machine.state = phase
        node.machine.locked = detection()
        node.machine.filtered_u = TARGET_U
        node.machine.filtered_v = 0.50 * (HEIGHT - 1)
        node.machine.acquired_v_fraction = 0.50
        node.machine.maximum_v_fraction = 0.50
        node._metric_target_for_pixel = mock.Mock(
            return_value=SimpleNamespace(object_id=17, x=1.0, y=2.0)
        )
        node._validate_selected_metric_target = mock.Mock()
        return node

    @staticmethod
    def frame(*, valid=tuple(), rejected=tuple(), reasons=tuple()):
        return SimpleNamespace(
            candidates=valid,
            depth_rejected=rejected,
            depth_rejections=reasons,
            width=WIDTH,
            height=HEIGHT,
        )

    def test_first_four_matching_invalid_depth_frames_retain_visual_lock(self):
        node = self.make_node()
        invalid = detection(depth_valid=False, depth_m=math.nan)
        frame = self.frame(
            rejected=(invalid,),
            reasons=("detection[0] Metal: registered depth is unavailable",),
        )

        for expected_count in range(1, 5):
            candidates, reason = node._candidates_with_locked_depth_grace(frame)
            self.assertEqual(candidates, (invalid,))
            self.assertEqual(node.locked_depth_invalid_frames, expected_count)
            self.assertIn("%d/5" % expected_count, reason)

    def test_fifth_matching_invalid_depth_frame_aborts(self):
        node = self.make_node()
        invalid = detection(depth_valid=False, depth_m=math.nan)
        frame = self.frame(
            rejected=(invalid,),
            reasons=("detection[0] Metal: registered depth is unavailable",),
        )

        for _ in range(4):
            node._candidates_with_locked_depth_grace(frame)
        with self.assertRaisesRegex(
            FOD_NODE.ControllerAbort, "for 5 consecutive frames"
        ):
            node._candidates_with_locked_depth_grace(frame)

    def test_detection_pipeline_tracks_visible_target_without_depth_abort(self):
        node = self.make_node()
        node.target_u_px = TARGET_U
        node.approach_path_m = 0.20
        node.horizontal_error_value = None
        node.vertical_fraction_value = None
        node.target_visible = True
        node._start_approach_odometry = mock.Mock()
        node._hard_stop = mock.Mock()
        node._pause_approach_watchdog = mock.Mock()
        invalid = detection(depth_valid=False, depth_m=math.nan)
        frame = SimpleNamespace(
            candidates=tuple(),
            depth_rejected=(invalid,),
            depth_rejections=(
                "detection[0] Metal: registered depth is unavailable",
            ),
            observations=(invalid,),
            width=WIDTH,
            height=HEIGHT,
            stamp_sec=100.0,
            error="",
        )
        node._take_detection_events = mock.Mock(return_value=(frame,))

        for _ in range(30):
            node._process_detection_events()
            self.assertEqual(node.phase, FOD_NODE.APPROACH)
            self.assertTrue(node.target_visible)
            self.assertEqual(node.machine.missing_frames, 0)
        node._hard_stop.assert_not_called()

    def test_edge_armed_depth_jitter_does_not_look_like_disappearance(self):
        node = self.make_node(FOD_NODE.EDGE_ARMED)
        armed = detection(q=0.90)
        node.machine.locked = armed
        node.machine.filtered_u = armed.anchor_u
        node.machine.filtered_v = armed.anchor_v
        node.machine.acquired_v_fraction = 0.50
        node.machine.maximum_v_fraction = 0.90
        invalid = detection(q=0.90, depth_valid=False, depth_m=math.nan)
        frame = self.frame(
            rejected=(invalid,),
            reasons=("detection[0] Metal: registered depth is unavailable",),
        )

        candidates, _reason = node._candidates_with_locked_depth_grace(frame)
        decision = node.machine.process_frame(
            candidates=candidates,
            image_width=WIDTH,
            image_height=HEIGHT,
            target_u=TARGET_U,
            approach_distance_m=0.20,
            frame_stamp=100.0,
        )

        self.assertEqual(decision.state, EDGE_ARMED)
        self.assertFalse(decision.fault)
        self.assertEqual(node.locked_depth_invalid_frames, 1)

    def test_valid_or_nonmatching_frame_resets_invalid_depth_counter(self):
        node = self.make_node()
        invalid = detection(depth_valid=False, depth_m=math.nan)
        invalid_frame = self.frame(
            rejected=(invalid,),
            reasons=("detection[0] Metal: registered depth is unavailable",),
        )
        node._candidates_with_locked_depth_grace(invalid_frame)
        node._candidates_with_locked_depth_grace(invalid_frame)

        valid = detection(depth_valid=True, depth_m=1.8)
        candidates, reason = node._candidates_with_locked_depth_grace(
            self.frame(valid=(valid,))
        )
        self.assertEqual(candidates, (valid,))
        self.assertEqual(reason, "")
        self.assertEqual(node.locked_depth_invalid_frames, 0)

        node._candidates_with_locked_depth_grace(invalid_frame)
        unrelated = detection(
            u=TARGET_U + 0.40 * WIDTH,
            depth_valid=False,
            depth_m=math.nan,
        )
        candidates, reason = node._candidates_with_locked_depth_grace(
            self.frame(
                rejected=(unrelated,),
                reasons=(
                    "detection[0] Metal: registered depth is unavailable",
                ),
            )
        )
        self.assertEqual(candidates, tuple())
        self.assertEqual(reason, "")
        self.assertEqual(node.locked_depth_invalid_frames, 0)


class RawCanObservationTest(unittest.TestCase):
    @staticmethod
    def make_node():
        node = FOD_NODE.FodVisualServoNode.__new__(FOD_NODE.FodVisualServoNode)
        node.sensor_lock = threading.Lock()
        node.raw_can_timeout_sec = 5.0
        node.raw_can_status = {
            msg_type: (9.5, True, 0, "")
            for msg_type in FOD_NODE.RAW_REQUIRED_TYPES
        }
        node.raw_can_fault_generation = 0
        node.session_raw_can_fault_generation = 0
        node.last_raw_can_fault_reason = ""
        node.external_estop_override = False
        node.phase = FOD_NODE.PRECHECK
        return node

    def test_all_fresh_safe_m2_responses_pass_without_sending_a_query(self):
        node = self.make_node()

        with mock.patch.object(FOD_NODE.time, "monotonic", return_value=10.0):
            node._check_raw_can()

    def test_observer_still_aborts_on_an_unsafe_value(self):
        node = self.make_node()
        node.raw_can_status[FOD_NODE.VCU_HARD_EMERGENCY] = (
            9.9,
            False,
            1,
            "",
        )

        with mock.patch.object(FOD_NODE.time, "monotonic", return_value=10.0):
            with self.assertRaisesRegex(FOD_NODE.ControllerAbort, "unsafe"):
                node._check_raw_can()

    def test_observer_aborts_when_an_m2_response_is_stale(self):
        node = self.make_node()
        node.raw_can_status[FOD_NODE.VCU_GAMEPAD_EMERGENCY] = (
            4.9,
            True,
            0,
            "",
        )

        with mock.patch.object(FOD_NODE.time, "monotonic", return_value=10.0):
            with self.assertRaisesRegex(FOD_NODE.ControllerAbort, "stale"):
                node._check_raw_can()

    def test_external_estop_override_does_not_require_can_status(self):
        node = self.make_node()
        node.external_estop_override = True
        node.raw_can_status.clear()

        with mock.patch.object(FOD_NODE.time, "monotonic", return_value=10.0):
            node._check_raw_can()


class ExternalEstopOverrideTest(unittest.TestCase):
    @staticmethod
    def make_graph_node(override):
        node = FOD_NODE.FodVisualServoNode.__new__(FOD_NODE.FodVisualServoNode)
        node.external_estop_override = override
        node.last_graph_check_monotonic = 0.0
        node.graph_check_interval_sec = 0.5
        node.master_pid = 42
        node.master = mock.Mock()
        node.master.getPid.return_value = 42
        node.detections_topic = "/fod/detections"
        node.metric_results_topic = "/fod/vision/results"
        node.camera_info_topic = "/fod_camera/camera_info"
        node.odom_topic = "/odom"
        node.wheel_angle_topic = "/m2_driver/wheel_angle"
        node.chassis_status_topic = "/m2_driver/chassis_info"
        node.control_timeout_topic = "/m2_driver/control_timeout"
        node.canbus_topic = "/canbus_msg"
        node.chassis_parameter_service = "/m2_driver/chassis_parameter"
        node.cmd_vel_topic = "/cmd_vel"
        node.ackermann_topic = "/ackerman_vel"
        node.m2_bypass_topics = (
            "/m2_driver/steer_center_bias",
            "/m2_driver/reset_odom",
            "/m2_driver/brake_set",
        )
        node.expected_detector_node = "/fod_detector"
        node.expected_camera_node = "/fod_camera/driver"
        node.expected_driver_node = "/m2_driver"
        node.expected_cmd_vel_subscriber_node = "/m2_driver"
        node.expected_canbus_node = "/canbus_driver"
        node.cmd_pub = mock.Mock()
        node.cmd_pub.get_num_connections.return_value = 1
        return node

    def test_graph_override_does_not_require_can_or_vcu_safety_endpoints(self):
        node = self.make_graph_node(override=True)
        node.master.getSystemState.return_value = (
            [
                ("/fod/detections", ["/fod_detector"]),
                ("/fod/vision/results", ["/fod_detector"]),
                ("/fod_camera/camera_info", ["/fod_camera/driver"]),
                ("/odom", ["/m2_driver"]),
                ("/m2_driver/wheel_angle", ["/m2_driver"]),
                ("/cmd_vel", ["/fod_visual_servo"]),
            ],
            [("/cmd_vel", ["/m2_driver"])],
            [("/m2_driver/chassis_parameter", ["/m2_driver"])],
        )

        def get_param(name, default=None):
            return False if name == "/use_sim_time" else default

        with mock.patch.object(
            FOD_NODE.rospy, "get_name", return_value="/fod_visual_servo"
        ), mock.patch.object(
            FOD_NODE.rospy, "get_param", side_effect=get_param
        ), mock.patch.object(
            FOD_NODE.time, "monotonic", return_value=10.0
        ):
            node._check_graph(force=True)

        self.assertEqual(node.last_graph_check_monotonic, 10.0)

    def test_runtime_override_ignores_missing_chassis_and_vcu_timeout(self):
        node = FOD_NODE.FodVisualServoNode.__new__(FOD_NODE.FodVisualServoNode)
        node.external_estop_override = True
        node.phase = FOD_NODE.ACQUIRE
        node.session_detection_floor = 0
        node.session_metric_floor = 0
        node.session_invalid_metric_generation = 0
        node.session_invalid_camera_generation = 0
        node.session_invalid_odom_generation = 0
        node.session_invalid_wheel_generation = 0
        node.session_chassis_fault_generation = 0
        node.session_m2_bypass_event_generation = 0
        node.detection_timeout_sec = 0.35
        node.metric_results_timeout_sec = 0.60
        node.camera_info_timeout_sec = 0.60
        node.odom_timeout_sec = 0.60
        node.wheel_angle_timeout_sec = 0.60
        node.max_measured_speed_mps = 0.30
        node.max_steering_angle_deg = 12.0
        node.detections_topic = "/fod/detections"
        node.metric_results_topic = "/fod/vision/results"
        node.camera_info_topic = "/fod_camera/camera_info"
        node.odom_topic = "/odom"
        node.wheel_angle_topic = "/m2_driver/wheel_angle"
        valid_sample = SimpleNamespace(
            error="", receipt_monotonic=9.9, stamp_sec=100.0, sequence=1
        )
        odom = SimpleNamespace(
            receipt_monotonic=9.9,
            stamp_sec=100.0,
            linear_x=0.0,
        )
        node._sensor_snapshot = mock.Mock(
            return_value={
                "detection": valid_sample,
                "metric": valid_sample,
                "invalid_metric_generation": 0,
                "camera": valid_sample,
                "invalid_camera_generation": 0,
                "odom": odom,
                "invalid_odom_reason": "",
                "invalid_odom_monotonic": 0.0,
                "invalid_odom_generation": 0,
                "wheel_angle": 0.0,
                "wheel_receipt": 9.9,
                "invalid_wheel_reason": "",
                "invalid_wheel_monotonic": 0.0,
                "invalid_wheel_generation": 0,
                "chassis": None,
                "chassis_receipt": None,
                "chassis_fault_generation": 999,
                "last_chassis_fault_reason": "forced test fault",
                "m2_bypass_event_generation": 0,
                "last_m2_bypass_event_topic": "",
                "control_timeout_seen": True,
                "detection_overflow": False,
                "odom_overflow": False,
            }
        )
        node._source_age_check = mock.Mock()

        with mock.patch.object(FOD_NODE.time, "monotonic", return_value=10.0):
            node._check_sensor_health()

    def test_override_can_arm_without_chassis_or_raw_can_samples(self):
        node = FOD_NODE.FodVisualServoNode.__new__(FOD_NODE.FodVisualServoNode)
        node.external_estop_override = True
        node.sensor_lock = threading.Lock()
        node.latest_metric = SimpleNamespace(error="")
        node.latest_camera = SimpleNamespace(error="")
        node.latest_odom = SimpleNamespace(receipt_monotonic=10.0)
        node.latest_wheel_angle = 0.0
        node.latest_wheel_angle_monotonic = 10.0
        node.invalid_odom_reason = ""
        node.invalid_odom_monotonic = 0.0
        node.invalid_wheel_reason = ""
        node.invalid_wheel_monotonic = 0.0
        node.latest_chassis_status = None
        node.raw_can_status = {}
        node.invalid_camera_generation = 1
        node.invalid_metric_generation = 0
        node.invalid_odom_generation = 2
        node.invalid_wheel_generation = 3
        node.chassis_fault_generation = 4
        node.raw_can_fault_generation = 5
        node.m2_bypass_event_generation = 6

        node._arm_session_fault_latches()

        self.assertEqual(node.session_raw_can_fault_generation, 5)

    def test_override_terminal_commit_does_not_reintroduce_can_vcu_gates(self):
        node = FOD_NODE.FodVisualServoNode.__new__(FOD_NODE.FodVisualServoNode)
        node.external_estop_override = True
        node.operator_disable_requested = threading.Event()
        node.mode_deadline_monotonic = 11.0
        node.final_stop_started_monotonic = 9.0
        node.final_stop_timeout_sec = 3.0
        node.detection_timeout_sec = 0.35
        node.metric_results_timeout_sec = 0.60
        node.camera_info_timeout_sec = 0.60
        node.odom_timeout_sec = 0.60
        node.wheel_angle_timeout_sec = 0.60
        node.source_stamp_timeout_sec = 0.80
        node.latest_detection = SimpleNamespace(
            error="", receipt_monotonic=9.9, stamp_sec=100.0
        )
        node.latest_metric = SimpleNamespace(
            error="", receipt_monotonic=9.9, stamp_sec=100.0
        )
        node.latest_camera = SimpleNamespace(
            error="", receipt_monotonic=9.9, stamp_sec=100.0
        )
        node.latest_odom = SimpleNamespace(
            receipt_monotonic=9.9, stamp_sec=100.0
        )
        node.latest_wheel_angle = 0.0
        node.latest_wheel_angle_monotonic = 9.9
        node.latest_chassis_status = None
        node.latest_chassis_status_monotonic = None
        node.raw_can_status = {}
        ros_now = SimpleNamespace(to_sec=lambda: 100.1)

        with mock.patch.object(
            FOD_NODE.rospy, "is_shutdown", return_value=False
        ), mock.patch.object(
            FOD_NODE.rospy.Time, "now", return_value=ros_now
        ), mock.patch.object(
            FOD_NODE.time, "monotonic", return_value=10.0
        ):
            node._check_terminal_commit_health_locked()


class TerminalSensorFenceTest(unittest.TestCase):
    def test_any_unprocessed_feedback_or_fault_change_blocks_complete(self):
        expected = TerminalSensorFence(
            odom_sequence=100,
            wheel_sequence=200,
            detection_sequence=300,
            metric_sequence=400,
            invalid_metric_generation=0,
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
            "metric_sequence": 401,
            "invalid_metric_generation": 1,
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
