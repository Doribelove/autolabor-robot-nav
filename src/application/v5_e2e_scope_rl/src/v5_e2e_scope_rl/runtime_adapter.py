"""Sensor-only Ackermann candidate adapter for V6 ROS/Gazebo evaluation."""

import math
import time

import numpy as np
import torch

from .asymmetric_ppo import observations_to_torch, stack_observations


ACTION_NAMES = (
    "certified_stop",
    "straight_v040",
    "straight_v080",
    "straight_v120",
    "spatial_left_v080",
    "spatial_right_v080",
    "spatial_left_v120",
    "spatial_right_v120",
)
TARGET_SPEEDS = np.asarray([0.0, 0.4, 0.8, 1.2, 0.8, 0.8, 1.2, 1.2])
LATERAL_OFFSETS = np.asarray([0.0, 0.0, 0.0, 0.0, 1.2, -1.2, 1.2, -1.2])


def _clip(value, lower, upper):
    return max(lower, min(upper, value))


class SensorRuntimeAdapter:
    """Build the exact actor observation without simulator world-state inputs."""

    def __init__(self, actor=None, advantage_gate_margin=0.10):
        self.actor = actor
        self.advantage_gate_margin = float(advantage_gate_margin)
        self.dt = 0.25
        self.preview_dt_s = 0.25
        self.preview_horizon_s = 3.0
        self.wheelbase_m = 0.65
        self.maximum_acceleration_mps2 = 0.60
        self.maximum_deceleration_mps2 = 0.90
        self.maximum_steering_rad = 0.40
        self.maximum_steering_rate_radps = 0.75
        self.robot_radius_m = 0.46
        self.shield_inflation_m = 0.12
        self.shield_clearance_minimum_m = 0.0
        self.maximum_dynamic_obstacle_speed_mps = 1.35
        self.maximum_dynamic_expansion_m = 0.75
        self.maximum_range_m = 8.0
        self.wall_half_width_m = 2.35
        self.lateral_offsets = LATERAL_OFFSETS.astype(np.float64).copy()
        self.lateral_offset_normalizer_m = 1.2
        self.target_speeds = TARGET_SPEEDS.astype(np.float64).copy()
        self.action_names = tuple(ACTION_NAMES)
        self.start_x_m = -4.2
        self.goal_x_m = 5.2
        self.beam_angles = np.linspace(math.radians(-135.0), math.radians(135.0), 72)
        self.scan_history = np.full((6, 72), self.maximum_range_m, dtype=np.float32)
        self.previous_action = 2
        self.previous_target_speed = 0.8
        self.consecutive_stop_count = 0
        self.decision_count = 0

    def resample_scan(self, ranges, angle_min, angle_increment, range_max):
        ranges = np.asarray(ranges, dtype=np.float64)
        indices = np.rint((self.beam_angles - float(angle_min)) / float(angle_increment)).astype(int)
        valid = (indices >= 0) & (indices < len(ranges))
        result = np.full(72, self.maximum_range_m, dtype=np.float64)
        selected = ranges[np.clip(indices, 0, max(0, len(ranges) - 1))]
        finite = valid & np.isfinite(selected) & (selected > 0.0)
        result[finite] = np.minimum(
            selected[finite], min(float(range_max), self.maximum_range_m)
        )
        return np.clip(result, 0.02, self.maximum_range_m).astype(np.float32)

    def append_scan(self, scan):
        scan = np.asarray(scan, dtype=np.float32)
        if scan.shape != (72,):
            raise ValueError("RUNTIME_SCAN_SHAPE_MISMATCH")
        self.scan_history = np.concatenate((self.scan_history[1:], scan[None, :]), axis=0)

    def _target_lateral(self, x_m, offset_m):
        if abs(offset_m) < 1.0e-12 or x_m <= -4.0 or x_m >= 4.0:
            return 0.0
        phase = (x_m + 4.0) / 8.0
        return offset_m * math.sin(math.pi * _clip(phase, 0.0, 1.0))

    def _kinematic_step(self, state, target_speed, lateral_offset):
        x_m, y_m, yaw_rad, speed_mps, steering_rad = state
        delta_speed = _clip(
            target_speed - speed_mps,
            -self.maximum_deceleration_mps2 * self.preview_dt_s,
            self.maximum_acceleration_mps2 * self.preview_dt_s,
        )
        next_speed = _clip(speed_mps + delta_speed, 0.0, 1.2)
        target_y = self._target_lateral(x_m, lateral_offset)
        desired_steering = _clip(
            0.78 * (target_y - y_m) - 0.90 * yaw_rad,
            -self.maximum_steering_rad,
            self.maximum_steering_rad,
        )
        next_steering = steering_rad + _clip(
            desired_steering - steering_rad,
            -self.maximum_steering_rate_radps * self.preview_dt_s,
            self.maximum_steering_rate_radps * self.preview_dt_s,
        )
        average_speed = 0.5 * (speed_mps + next_speed)
        average_steering = 0.5 * (steering_rad + next_steering)
        next_x = x_m + average_speed * math.cos(yaw_rad) * self.preview_dt_s
        next_y = y_m + average_speed * math.sin(yaw_rad) * self.preview_dt_s
        next_yaw = yaw_rad + average_speed * math.tan(average_steering) * self.preview_dt_s / self.wheelbase_m
        return next_x, next_y, next_yaw, next_speed, next_steering

    def _preview(self, pose, speed, steering, action, stop_tail=False):
        state = (pose[0], pose[1], pose[2], speed, steering)
        states = [state]
        if stop_tail:
            state = self._kinematic_step(
                state, float(self.target_speeds[action]), float(self.lateral_offsets[action])
            )
            states.append(state)
            for _ in range(8):
                state = self._kinematic_step(state, 0.0, float(self.lateral_offsets[action]))
                states.append(state)
                if state[3] <= 1.0e-8:
                    break
        else:
            for _ in range(int(round(self.preview_horizon_s / self.preview_dt_s))):
                state = self._kinematic_step(
                    state, float(self.target_speeds[action]), float(self.lateral_offsets[action])
                )
                states.append(state)
        return states

    def _scan_points_world(self, pose):
        scan = self.scan_history[-1].astype(np.float64)
        valid = scan < self.maximum_range_m - 0.05
        angles = pose[2] + self.beam_angles
        if not np.any(valid):
            return np.empty((0, 2), dtype=np.float64)
        return np.stack(
            (
                pose[0] + scan[valid] * np.cos(angles[valid]),
                pose[1] + scan[valid] * np.sin(angles[valid]),
            ),
            axis=1,
        )

    def _clearance(self, preview, scan_points, dynamic_reachable=False):
        minimum = float("inf")
        if len(scan_points):
            wall_like = np.abs(np.abs(scan_points[:, 1]) - self.wall_half_width_m) <= 0.10
            dynamic_points = scan_points[~wall_like]
            static_points = scan_points[wall_like]
        else:
            dynamic_points = static_points = scan_points
        for index, state in enumerate(preview):
            minimum = min(
                minimum,
                self.wall_half_width_m - abs(state[1]) - self.robot_radius_m,
            )
            if len(static_points):
                distance = np.hypot(static_points[:, 0] - state[0], static_points[:, 1] - state[1])
                minimum = min(
                    minimum,
                    float(np.min(distance)) - self.robot_radius_m - self.shield_inflation_m,
                )
            if len(dynamic_points):
                distance = np.hypot(
                    dynamic_points[:, 0] - state[0], dynamic_points[:, 1] - state[1]
                )
                expansion = 0.0
                if dynamic_reachable:
                    expansion = min(
                        self.maximum_dynamic_expansion_m,
                        self.maximum_dynamic_obstacle_speed_mps * index * self.preview_dt_s,
                    )
                minimum = min(
                    minimum,
                    float(np.min(distance))
                    - self.robot_radius_m
                    - self.shield_inflation_m
                    - expansion,
                )
        return minimum

    def build(self, pose, speed, yaw_rate, elapsed_s):
        speed = _clip(float(speed), 0.0, 1.2)
        steering = 0.0
        if speed > 0.05:
            steering = _clip(
                math.atan(float(yaw_rate) * self.wheelbase_m / speed),
                -self.maximum_steering_rad,
                self.maximum_steering_rad,
            )
        scan_points = self._scan_points_world(pose)
        features = []
        priors = []
        masks = []
        previews = []
        for action in range(8):
            preview = self._preview(pose, speed, steering, action)
            backup = self._preview(pose, speed, steering, action, stop_tail=True)
            full_clearance = self._clearance(preview, scan_points, dynamic_reachable=False)
            shield_clearance = self._clearance(backup, scan_points, dynamic_reachable=True)
            progress = preview[-1][0] - pose[0]
            acceleration_cost = abs(float(self.target_speeds[action]) - speed)
            lateral_cost = abs(float(self.lateral_offsets[action]))
            switch_cost = 1.0 if action != self.previous_action else 0.0
            priors.append(
                progress
                + 1.50 * _clip(full_clearance, -1.0, 2.0)
                - 0.035 * acceleration_cost
                - 0.025 * lateral_cost
                - 0.01 * switch_cost
            )
            masks.append(
                action == 0
                or shield_clearance >= self.shield_clearance_minimum_m - 1.0e-8
            )
            features.append(
                [
                    float(self.target_speeds[action]) / 1.2,
                    float(self.lateral_offsets[action]) / self.lateral_offset_normalizer_m,
                    progress / 3.6,
                    _clip(full_clearance, -1.5, 3.0) / 3.0,
                    acceleration_cost / 1.2,
                    lateral_cost / 1.2,
                    switch_cost,
                ]
            )
            previews.append(preview)
        masks = np.asarray(masks, dtype=bool)
        masks[0] = True
        observation = {
            "scans": (self.scan_history / self.maximum_range_m).astype(np.float32),
            "robot": np.asarray(
                [
                    (pose[0] - self.start_x_m) / (self.goal_x_m - self.start_x_m),
                    pose[1] / self.wall_half_width_m,
                    pose[2] / 0.7,
                    speed / 1.2,
                    self.previous_target_speed / 1.2,
                    (self.goal_x_m - pose[0]) / 9.0,
                    self.previous_action / 7.0,
                    min(elapsed_s / 20.0, 1.0),
                    min(self.consecutive_stop_count, 12) / 12.0,
                    float(np.min(self.scan_history[-1])) / self.maximum_range_m,
                ],
                dtype=np.float32,
            ),
            "candidates": np.asarray(features, dtype=np.float32),
            "analytic_prior": np.asarray(priors, dtype=np.float32),
            "action_mask": masks.astype(np.int8),
        }
        return observation, previews

    def select(self, pose, speed, yaw_rate, elapsed_s):
        observation, previews = self.build(pose, speed, yaw_rate, elapsed_s)
        started = time.perf_counter_ns()
        if self.actor is None:
            values = np.where(
                observation["action_mask"].astype(bool),
                observation["analytic_prior"],
                -np.inf,
            )
            action = int(np.argmax(values))
            metadata = {
                "learned_action": action,
                "analytic_action": action,
                "intervened": False,
                "residual_margin": 0.0,
            }
        else:
            batch = observations_to_torch(
                stack_observations([observation]), torch.device("cpu")
            )
            with torch.no_grad():
                actions, result = self.actor.choose_actions(
                    batch,
                    deterministic=True,
                    advantage_gate_margin=self.advantage_gate_margin,
                )
            action = int(actions[0])
            metadata = {
                "learned_action": int(result["learned_action"][0]),
                "analytic_action": int(result["analytic_action"][0]),
                "intervened": bool(result["intervened"][0]),
                "residual_margin": float(result["residual_margin"][0]),
            }
        latency_ms = (time.perf_counter_ns() - started) / 1.0e6
        self.previous_action = action
        self.previous_target_speed = float(self.target_speeds[action])
        self.consecutive_stop_count = self.consecutive_stop_count + 1 if action == 0 else 0
        self.decision_count += 1
        points = [
            {
                "x_m": float(state[0]),
                "y_m": float(state[1]),
                "longitudinal_velocity_mps": float(state[3]),
            }
            for state in previews[action]
        ]
        return {
            "action": action,
            "action_name": self.action_names[action],
            "points": points,
            "observation": observation,
            "latency_ms": latency_ms,
            **metadata,
        }
