#!/usr/bin/env python3

import importlib.util
import math
import os
import unittest


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "m2_truth_sim.py",
)
SPEC = importlib.util.spec_from_file_location("m2_truth_sim", SCRIPT)
PLANT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PLANT)


class M2TwistContractTest(unittest.TestCase):
    def test_steering_uses_commanded_velocity_not_lagging_actual_velocity(self):
        wheelbase = 0.65
        radius = 1.35
        velocity = 1.0
        omega = velocity / radius
        steer = PLANT.twist_to_front_steering(
            velocity, omega, wheelbase, 1.0
        )
        self.assertAlmostEqual(math.atan(wheelbase / radius), steer)

    def test_reverse_command_preserves_base_frame_yaw_sign(self):
        forward = PLANT.twist_to_front_steering(1.0, 0.4, 0.65, 1.0)
        reverse = PLANT.twist_to_front_steering(-1.0, 0.4, 0.65, 1.0)
        self.assertAlmostEqual(forward, -reverse)

    def test_linear_speed_clamp_preserves_curvature(self):
        limited_omega = PLANT.angular_velocity_after_linear_limit(
            2.0, 1.6, 1.0
        )
        original_curvature = 1.0 / 2.0
        limited_curvature = limited_omega / 1.6
        self.assertAlmostEqual(original_curvature, limited_curvature)

    def test_integrator_does_not_saturate_steer_while_speed_catches_up(self):
        plant = PLANT.M2TruthPlant.__new__(PLANT.M2TruthPlant)
        plant.wheelbase = 0.65
        plant.effective_steer_limit = math.atan(0.65 / 1.35)
        plant.linear_accel = 2.0
        plant.velocity = 0.10
        plant.x = 0.0
        plant.y = 0.0
        plant.yaw = 0.0
        plant.actual_omega = 0.0
        plant.actual_steer = 0.0
        plant._desired_motion = lambda _stamp: (1.0, 1.0 / 2.0)

        plant._integrate(None, 0.02)

        self.assertAlmostEqual(math.atan(0.65 / 2.0), plant.actual_steer)
        self.assertLess(plant.actual_steer, plant.effective_steer_limit)


if __name__ == "__main__":
    unittest.main()
