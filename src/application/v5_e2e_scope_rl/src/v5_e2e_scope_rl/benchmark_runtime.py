"""Auditable runtime contracts for the PCT/analytic Gazebo benchmark.

This module is deliberately separate from the frozen V6 runtime.  It repairs
deployment parity and execution atomicity without changing historical files.
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
import torch

from .asymmetric_ppo import observations_to_torch, stack_observations
from .runtime_adapter import (
    ACTION_NAMES,
    LATERAL_OFFSETS,
    TARGET_SPEEDS,
    SensorRuntimeAdapter,
    _clip,
)
from .sensor_env import MODE_NAMES, SensorCandidateEnv


@dataclass(frozen=True)
class RuntimeInputHealth:
    scan_age_s: float
    odometry_age_s: float
    transform_age_s: float
    scan_frame_ok: bool = True
    odometry_frame_ok: bool = True
    transform_ok: bool = True

    def failures(self, maximum_age_s=0.20):
        result = []
        for name, value in (
            ("scan", self.scan_age_s),
            ("odometry", self.odometry_age_s),
            ("transform", self.transform_age_s),
        ):
            if not math.isfinite(float(value)) or float(value) < 0.0:
                result.append(name + "_age_invalid")
            elif float(value) > float(maximum_age_s):
                result.append(name + "_stale")
        if not self.scan_frame_ok:
            result.append("scan_frame_invalid")
        if not self.odometry_frame_ok:
            result.append("odometry_frame_invalid")
        if not self.transform_ok:
            result.append("transform_invalid")
        return result


class BenchmarkSensorCandidateEnv(SensorCandidateEnv):
    """Development/training backend with the deployed response-delay shield."""

    def __init__(self, config=None):
        config = dict(config or {})
        config.setdefault("maximum_episode_decisions", 88)
        config.setdefault("lateral_offsets_m", [0.0, 0.0, 0.0, 0.0, 1.35, -1.35, 1.35, -1.35])
        config.setdefault("target_speeds_mps", [0.0, 0.4, 0.8, 1.2, 0.4, 0.4, 1.2, 1.2])
        config.setdefault("action_names", [
            "certified_stop", "straight_v040", "straight_v080", "straight_v120",
            "spatial_left_v040", "spatial_right_v040", "spatial_left_v120", "spatial_right_v120",
        ])
        config.setdefault("shield_clearance_minimum_m", 0.20)
        config.setdefault("maximum_steering_rad", 0.48)
        config.setdefault("maximum_steering_rate_radps", 0.80)
        self.safety_response_delay_s = float(config.pop("safety_response_delay_s", 0.35))
        self.certified_stop_dwell_s = float(config.pop("certified_stop_dwell_s", 1.50))
        self.route_departure_x_m = float(config.pop("route_departure_x_m", -5.5))
        self.route_hold_start_x_m = float(config.pop("route_hold_start_x_m", -3.0))
        self.route_hold_end_x_m = float(config.pop("route_hold_end_x_m", 0.8))
        self.route_recovery_x_m = float(config.pop("route_recovery_x_m", 2.0))
        self.certified_stop_prior_penalty = float(config.pop("certified_stop_prior_penalty", 1.0))
        self.spatial_commitment_bonus = float(config.pop("spatial_commitment_bonus", 1.0))
        self.candidate_switch_penalty = float(config.pop("candidate_switch_penalty", 0.15))
        self.opposite_spatial_side_switch_penalty = float(
            config.pop("opposite_spatial_side_switch_penalty", 2.0)
        )
        self.spatial_speed_escalation_penalty = float(
            config.pop("spatial_speed_escalation_penalty", 2.0)
        )
        self.spatial_speed_escalation_release_clearance_m = float(
            config.pop("spatial_speed_escalation_release_clearance_m", 0.22)
        )
        self.static_route_commitment_bonus = float(
            config.pop("static_route_commitment_bonus", 3.0)
        )
        self.committed_spatial_side = 0
        self.committed_spatial_speed_mps = 0.0
        self.confirmed_straight_run_length = 0
        super().__init__(config)
        if self.safety_response_delay_s < 0.0:
            raise ValueError("NEGATIVE_SAFETY_RESPONSE_DELAY")
        if self.certified_stop_dwell_s < 0.0:
            raise ValueError("NEGATIVE_CERTIFIED_STOP_DWELL")
        if not (
            self.route_departure_x_m
            < self.route_hold_start_x_m
            < self.route_hold_end_x_m
            < self.route_recovery_x_m
        ):
            raise ValueError("LATERAL_ROUTE_INTERVAL_INVALID")

    def _configure_scene(self, options):
        """Match the four dynamic training families to the Gazebo templates.

        The historical sensor backend used nearby artificial corridor walls,
        a different occluder location, and a centre-lane head-on actor.  Those
        differences are large enough to change both scans and safe candidate
        sets, so the benchmark training backend owns an explicit, reviewed
        mapping instead of inheriting those historical fixtures.
        """
        self.actors = []
        self.static_circles = []
        mode = options.get(
            "actor_mode",
            MODE_NAMES[int(self.np_random.integers(0, len(MODE_NAMES)))],
        )
        speed = float(options.get("actor_speed_mps", self.np_random.uniform(0.84, 0.99)))
        if self.family == "bidirectional_reactive_crossing":
            side = float(options.get("actor_side", self.np_random.choice([-1.0, 1.0])))
            self.wall_half_width_m = 20.0
            self.actors.append(self._actor(0.0, side * 3.60, 0.0, -side * speed, mode))
        elif self.family == "sequential_two_actor_crossing":
            first_side = float(options.get("actor_side", self.np_random.choice([-1.0, 1.0])))
            second_speed = float(self.np_random.uniform(0.84, 0.99))
            second_mode = MODE_NAMES[int(self.np_random.integers(0, len(MODE_NAMES)))]
            self.wall_half_width_m = 20.0
            self.actors.append(
                self._actor(-1.0, first_side * 3.60, 0.0, -first_side * speed, mode)
            )
            self.actors.append(
                self._actor(
                    1.35,
                    -first_side * 3.60,
                    0.0,
                    first_side * second_speed,
                    second_mode,
                )
            )
        elif self.family == "partially_occluded_crossing":
            side = float(options.get("actor_side", self.np_random.choice([-1.0, 1.0])))
            self.wall_half_width_m = 20.0
            # Circle is the conservative scan-equivalent of the Gazebo
            # 0.80 x 0.60 m box centred at (-0.10, +/-1.65).
            self.static_circles.append(
                {"x": -0.10, "y": side * 1.65, "radius": 0.50}
            )
            self.actors.append(self._actor(0.0, side * 3.60, 0.0, -side * speed, mode))
        elif self.family == "narrow_corridor_yield":
            side = float(options.get("actor_side", self.np_random.choice([-1.0, 1.0])))
            self.wall_half_width_m = 2.55
            self.actors.append(
                self._actor(3.60, side * 1.40, -speed, 0.0, mode, response_gain=0.8)
            )
        else:
            raise ValueError("UNKNOWN_SCENE_FAMILY:" + str(self.family))

    def reset(self, *, seed=None, options=None):
        self.committed_spatial_side = 0
        self.committed_spatial_speed_mps = 0.0
        self.confirmed_straight_run_length = 0
        return super().reset(seed=seed, options=options)

    def step(self, action):
        action = int(action)
        if action in (4, 5, 6, 7):
            new_side = 1 if self.lateral_offsets[action] > 0.0 else -1
            if self.committed_spatial_side != new_side:
                self.committed_spatial_speed_mps = float(self.target_speeds[action])
            self.committed_spatial_side = new_side
            self.confirmed_straight_run_length = 0
        elif action in (1, 2, 3):
            self.confirmed_straight_run_length += 1
            if (
                self.confirmed_straight_run_length >= 4
                and self.ego_x > self._central_static_route_release_x()
            ):
                self.committed_spatial_side = 0
                self.committed_spatial_speed_mps = 0.0
        else:
            self.confirmed_straight_run_length = 0
        return super().step(action)

    def _target_lateral(self, x_m, offset_m):
        if abs(offset_m) < 1.0e-12 or x_m <= self.route_departure_x_m:
            return 0.0
        if x_m < self.route_hold_start_x_m:
            phase = (x_m - self.route_departure_x_m) / (
                self.route_hold_start_x_m - self.route_departure_x_m
            )
            return offset_m * math.sin(0.5 * math.pi * _clip(phase, 0.0, 1.0))
        if x_m <= self.route_hold_end_x_m:
            return float(offset_m)
        if x_m < self.route_recovery_x_m:
            phase = (x_m - self.route_hold_end_x_m) / (
                self.route_recovery_x_m - self.route_hold_end_x_m
            )
            return offset_m * math.cos(0.5 * math.pi * _clip(phase, 0.0, 1.0))
        return 0.0

    def _refresh_candidates(self):
        release_x = self._central_static_route_release_x()
        if self.committed_spatial_side and math.isfinite(release_x) and self.ego_x > release_x:
            self.committed_spatial_side = 0
            self.committed_spatial_speed_mps = 0.0
            self.confirmed_straight_run_length = 0
        super()._refresh_candidates()
        self._analytic_prior[0] -= self.certified_stop_prior_penalty
        extra_switch_penalty = max(0.0, self.candidate_switch_penalty - 0.01)
        full_clearances = self._candidate_features[:, 3].astype(np.float64) * 3.0
        for action in range(8):
            if action != self.previous_action:
                self._analytic_prior[action] -= extra_switch_penalty
            if self.committed_spatial_side * float(self.lateral_offsets[action]) < 0.0:
                self._analytic_prior[action] -= self.opposite_spatial_side_switch_penalty
            if (
                self.committed_spatial_side != 0
                and action != 0
                and float(self.target_speeds[action]) > self.committed_spatial_speed_mps + 1.0e-9
                and not self._spatial_speed_upgrade_allowed(action, full_clearances[action])
            ):
                self._analytic_prior[action] -= self.spatial_speed_escalation_penalty
        if self.committed_spatial_side and self.ego_x <= release_x:
            for action in (4, 5, 6, 7):
                if (
                    self.committed_spatial_side * float(self.lateral_offsets[action]) > 0.0
                    and (
                        float(self.target_speeds[action])
                        <= self.committed_spatial_speed_mps + 1.0e-9
                        or self._spatial_speed_upgrade_allowed(
                            action, full_clearances[action]
                        )
                    )
                    and full_clearances[action] > 0.0
                ):
                    self._analytic_prior[action] += self.static_route_commitment_bonus
        if full_clearances[3] <= 0.0:
            for action in (4, 5):
                if full_clearances[action] > 0.0:
                    self._analytic_prior[action] += self.spatial_commitment_bonus

    def _spatial_speed_upgrade_allowed(self, action, full_clearance_m):
        return bool(
            self.committed_spatial_side != 0
            and self.committed_spatial_side * float(self.lateral_offsets[action]) > 0.0
            and float(self.target_speeds[action])
            > self.committed_spatial_speed_mps + 1.0e-9
            and float(full_clearance_m)
            >= self.spatial_speed_escalation_release_clearance_m - 1.0e-9
        )

    def _central_static_route_release_x(self):
        protected = self.robot_radius_m + self.shield_inflation_m
        releases = []
        for circle in self.static_circles:
            radius = float(circle["radius"])
            if abs(float(circle["y"])) - radius <= protected:
                releases.append(
                    float(circle["x"])
                    + radius
                    + protected
                    + self.shield_clearance_minimum_m
                )
        return max(releases) if releases else float("-inf")

    def _certified_backup_preview(self, action):
        state = (self.ego_x, self.ego_y, self.ego_yaw, self.ego_speed, self.ego_steering)
        states = [state]
        delay_steps = int(math.ceil(self.safety_response_delay_s / self.preview_dt_s))
        previous_offset = float(self.lateral_offsets[self.previous_action])
        held_measured_speed = float(state[3])
        for _ in range(delay_steps):
            state = self._kinematic_step(
                state, held_measured_speed, previous_offset, self.preview_dt_s
            )
            states.append(state)
        state = self._kinematic_step(
            state,
            float(self.target_speeds[action]),
            float(self.lateral_offsets[action]),
            self.preview_dt_s,
        )
        states.append(state)
        maximum_tail_steps = int(
            math.ceil(1.2 / self.maximum_deceleration_mps2 / self.preview_dt_s)
        ) + 2
        for _ in range(maximum_tail_steps):
            state = self._kinematic_step(
                state, 0.0, float(self.lateral_offsets[action]), self.preview_dt_s
            )
            states.append(state)
            if state[3] <= 1.0e-8:
                break
        for _ in range(int(math.ceil(self.certified_stop_dwell_s / self.preview_dt_s))):
            states.append(state)
        return states

    def _observable_clearance(self, preview, horizon_s=None, reachable_dynamic=False):
        """Use the evaluator footprint for mapped corridor walls as well.

        The parent sensor environment already applies ``shield_inflation_m``
        to scan surfaces.  Its analytic wall fallback historically used only
        the circumscribed robot radius, which made clearance depend on whether
        a wall happened to be visible in the latest scan.  The benchmark uses
        one evaluator-aligned radius for both cases.
        """
        minimum = super()._observable_clearance(
            preview,
            horizon_s=horizon_s,
            reachable_dynamic=reachable_dynamic,
        )
        maximum_index = len(preview)
        if horizon_s is not None:
            maximum_index = min(
                maximum_index,
                int(round(horizon_s / self.preview_dt_s)) + 1,
            )
        for state in preview[:maximum_index]:
            minimum = min(
                minimum,
                self.wall_half_width_m
                - abs(float(state[1]))
                - self.robot_radius_m
                - self.shield_inflation_m,
            )
        return minimum

    def _oracle_clearance(self, action):
        """Align the training-only rank label with the deployed footprint."""
        preview = self._candidate_preview(action)
        minimum = float("inf")
        for index, state in enumerate(preview):
            time_s = index * self.preview_dt_s
            minimum = min(
                minimum,
                self.wall_half_width_m
                - abs(float(state[1]))
                - self.robot_radius_m
                - self.shield_inflation_m,
            )
            for actor in self.actors:
                actor_x = actor["x"] + actor["vx"] * time_s
                actor_y = actor["y"] + actor["vy"] * time_s
                minimum = min(
                    minimum,
                    math.hypot(state[0] - actor_x, state[1] - actor_y)
                    - self.robot_radius_m
                    - self.shield_inflation_m
                    - actor["radius"],
                )
            for circle in self.static_circles:
                minimum = min(
                    minimum,
                    math.hypot(state[0] - circle["x"], state[1] - circle["y"])
                    - self.robot_radius_m
                    - self.shield_inflation_m
                    - circle["radius"],
                )
        return minimum

    def _physical_clearance(self):
        """Report the same conservative radius used by the Gazebo evaluator."""
        minimum = (
            self.wall_half_width_m
            - abs(self.ego_y)
            - self.robot_radius_m
            - self.shield_inflation_m
        )
        for actor in self.actors:
            minimum = min(
                minimum,
                math.hypot(self.ego_x - actor["x"], self.ego_y - actor["y"])
                - self.robot_radius_m
                - self.shield_inflation_m
                - actor["radius"],
            )
        for circle in self.static_circles:
            minimum = min(
                minimum,
                math.hypot(self.ego_x - circle["x"], self.ego_y - circle["y"])
                - self.robot_radius_m
                - self.shield_inflation_m
                - circle["radius"],
            )
        return minimum


class BenchmarkRuntimeAdapter(SensorRuntimeAdapter):
    """Training-aligned candidate builder with fail-closed execution semantics."""

    def __init__(
        self,
        actor=None,
        advantage_gate_margin=0.10,
        *,
        start_x_m=-4.0,
        goal_x_m=5.0,
        goal_tolerance_m=0.25,
        maximum_episode_duration_s=22.0,
        wall_half_width_m=2.35,
        static_circles=(),
        static_boxes=(),
        shield_inflation_m=0.33,
        inference_deadline_ms=20.0,
        maximum_input_age_s=0.20,
        goal_speed_envelope=True,
        safety_response_delay_s=0.0,
        certified_stop_dwell_s=0.0,
        route_departure_x_m=-5.5,
        route_hold_start_x_m=-3.0,
        route_hold_end_x_m=0.8,
        route_recovery_x_m=2.0,
        certified_stop_prior_penalty=1.0,
        lateral_offsets_m=(0.0, 0.0, 0.0, 0.0, 1.35, -1.35, 1.35, -1.35),
        static_surface_association_tolerance_m=0.15,
        shield_clearance_minimum_m=0.20,
        spatial_commitment_bonus=1.0,
        candidate_switch_penalty=0.15,
        opposite_spatial_side_switch_penalty=2.0,
        spatial_speed_escalation_penalty=2.0,
        spatial_speed_escalation_release_clearance_m=0.22,
        static_route_commitment_bonus=3.0,
        target_speeds_mps=(0.0, 0.4, 0.8, 1.2, 0.4, 0.4, 1.2, 1.2),
        action_names=(
            "certified_stop", "straight_v040", "straight_v080", "straight_v120",
            "spatial_left_v040", "spatial_right_v040", "spatial_left_v120", "spatial_right_v120",
        ),
        maximum_steering_rad=0.48,
        maximum_steering_rate_radps=0.80,
    ):
        super().__init__(actor=actor, advantage_gate_margin=advantage_gate_margin)
        self.maximum_steering_rad = float(maximum_steering_rad)
        self.maximum_steering_rate_radps = float(maximum_steering_rate_radps)
        self.start_x_m = float(start_x_m)
        self.goal_x_m = float(goal_x_m)
        self.goal_tolerance_m = float(goal_tolerance_m)
        self.maximum_episode_duration_s = float(maximum_episode_duration_s)
        self.wall_half_width_m = float(wall_half_width_m)
        self.static_circles = tuple(dict(item) for item in static_circles)
        self.static_boxes = tuple(dict(item) for item in static_boxes)
        self.shield_inflation_m = float(shield_inflation_m)
        self.inference_deadline_ms = float(inference_deadline_ms)
        self.maximum_input_age_s = float(maximum_input_age_s)
        self.goal_speed_envelope = bool(goal_speed_envelope)
        self.safety_response_delay_s = float(safety_response_delay_s)
        self.certified_stop_dwell_s = float(certified_stop_dwell_s)
        self.route_departure_x_m = float(route_departure_x_m)
        self.route_hold_start_x_m = float(route_hold_start_x_m)
        self.route_hold_end_x_m = float(route_hold_end_x_m)
        self.route_recovery_x_m = float(route_recovery_x_m)
        self.certified_stop_prior_penalty = float(certified_stop_prior_penalty)
        self.lateral_offsets = np.asarray(lateral_offsets_m, dtype=np.float64)
        if self.lateral_offsets.shape != (8,) or not np.all(np.isfinite(self.lateral_offsets)):
            raise ValueError("LATERAL_OFFSETS_CONTRACT_VIOLATION")
        self.lateral_offset_normalizer_m = max(1.0, float(np.max(np.abs(self.lateral_offsets))))
        self.static_surface_association_tolerance_m = float(static_surface_association_tolerance_m)
        self.shield_clearance_minimum_m = float(shield_clearance_minimum_m)
        self.spatial_commitment_bonus = float(spatial_commitment_bonus)
        self.candidate_switch_penalty = float(candidate_switch_penalty)
        self.opposite_spatial_side_switch_penalty = float(
            opposite_spatial_side_switch_penalty
        )
        self.spatial_speed_escalation_penalty = float(spatial_speed_escalation_penalty)
        self.spatial_speed_escalation_release_clearance_m = float(
            spatial_speed_escalation_release_clearance_m
        )
        self.static_route_commitment_bonus = float(static_route_commitment_bonus)
        self.target_speeds = np.asarray(target_speeds_mps, dtype=np.float64)
        self.action_names = tuple(action_names)
        if self.target_speeds.shape != (8,) or not np.all(np.isfinite(self.target_speeds)):
            raise ValueError("TARGET_SPEEDS_CONTRACT_VIOLATION")
        if not 0.0 < self.maximum_steering_rad <= 0.4964:
            raise ValueError("MAXIMUM_STEERING_CONTRACT_VIOLATION")
        if self.maximum_steering_rate_radps <= 0.0:
            raise ValueError("MAXIMUM_STEERING_RATE_CONTRACT_VIOLATION")
        if len(self.action_names) != 8:
            raise ValueError("ACTION_NAMES_CONTRACT_VIOLATION")
        if self.static_surface_association_tolerance_m <= 0.0:
            raise ValueError("STATIC_SURFACE_ASSOCIATION_TOLERANCE_INVALID")
        if self.safety_response_delay_s < 0.0:
            raise ValueError("NEGATIVE_SAFETY_RESPONSE_DELAY")
        if self.certified_stop_dwell_s < 0.0:
            raise ValueError("NEGATIVE_CERTIFIED_STOP_DWELL")
        if self.maximum_episode_duration_s <= 0.0:
            raise ValueError("NONPOSITIVE_MAXIMUM_EPISODE_DURATION")
        if not (
            self.route_departure_x_m
            < self.route_hold_start_x_m
            < self.route_hold_end_x_m
            < self.route_recovery_x_m
        ):
            raise ValueError("LATERAL_ROUTE_INTERVAL_INVALID")
        self.last_suggested_action = None
        self.last_validated_action = None
        self.last_executed_action = self.previous_action
        self.committed_spatial_side = 0
        self.committed_spatial_speed_mps = 0.0
        self.confirmed_straight_run_length = 0
        self.latest_pose_x_m = self.start_x_m
        self.execution_sequence = 0

    def _target_lateral(self, x_m, offset_m):
        if abs(offset_m) < 1.0e-12 or x_m <= self.route_departure_x_m:
            return 0.0
        if x_m < self.route_hold_start_x_m:
            phase = (x_m - self.route_departure_x_m) / (
                self.route_hold_start_x_m - self.route_departure_x_m
            )
            return offset_m * math.sin(0.5 * math.pi * _clip(phase, 0.0, 1.0))
        if x_m <= self.route_hold_end_x_m:
            return float(offset_m)
        if x_m < self.route_recovery_x_m:
            phase = (x_m - self.route_hold_end_x_m) / (
                self.route_recovery_x_m - self.route_hold_end_x_m
            )
            return offset_m * math.cos(0.5 * math.pi * _clip(phase, 0.0, 1.0))
        return 0.0

    def build(self, pose, speed, yaw_rate, elapsed_s):
        self.latest_pose_x_m = float(pose[0])
        release_x = self._central_static_route_release_x()
        if (
            self.committed_spatial_side
            and math.isfinite(release_x)
            and float(pose[0]) > release_x
        ):
            self.committed_spatial_side = 0
            self.committed_spatial_speed_mps = 0.0
            self.confirmed_straight_run_length = 0
        observation, previews = super().build(pose, speed, yaw_rate, elapsed_s)
        observation["robot"][7] = min(
            float(elapsed_s) / self.maximum_episode_duration_s, 1.0
        )
        observation["analytic_prior"][0] -= self.certified_stop_prior_penalty
        extra_switch_penalty = max(0.0, self.candidate_switch_penalty - 0.01)
        full_clearances = observation["candidates"][:, 3].astype(np.float64) * 3.0
        for action in range(8):
            if action != self.previous_action:
                observation["analytic_prior"][action] -= extra_switch_penalty
            if self.committed_spatial_side * float(self.lateral_offsets[action]) < 0.0:
                observation["analytic_prior"][action] -= self.opposite_spatial_side_switch_penalty
            if (
                self.committed_spatial_side != 0
                and action != 0
                and float(self.target_speeds[action]) > self.committed_spatial_speed_mps + 1.0e-9
                and not self._spatial_speed_upgrade_allowed(action, full_clearances[action])
            ):
                observation["analytic_prior"][action] -= self.spatial_speed_escalation_penalty
        if self.committed_spatial_side and float(pose[0]) <= release_x:
            for action in (4, 5, 6, 7):
                if (
                    self.committed_spatial_side * float(self.lateral_offsets[action]) > 0.0
                    and (
                        float(self.target_speeds[action])
                        <= self.committed_spatial_speed_mps + 1.0e-9
                        or self._spatial_speed_upgrade_allowed(
                            action, full_clearances[action]
                        )
                    )
                    and full_clearances[action] > 0.0
                ):
                    observation["analytic_prior"][action] += self.static_route_commitment_bonus
        if full_clearances[3] <= 0.0:
            for action in (4, 5):
                if full_clearances[action] > 0.0:
                    observation["analytic_prior"][action] += self.spatial_commitment_bonus
        return observation, previews

    def _spatial_speed_upgrade_allowed(self, action, full_clearance_m):
        return bool(
            self.committed_spatial_side != 0
            and self.committed_spatial_side * float(self.lateral_offsets[action]) > 0.0
            and float(self.target_speeds[action])
            > self.committed_spatial_speed_mps + 1.0e-9
            and float(full_clearance_m)
            >= self.spatial_speed_escalation_release_clearance_m - 1.0e-9
        )

    def _central_static_route_release_x(self):
        protected = self.robot_radius_m + self.shield_inflation_m
        releases = []
        for circle in self.static_circles:
            radius = float(circle["radius"])
            if abs(float(circle["y"])) - radius <= protected:
                releases.append(
                    float(circle["x"])
                    + radius
                    + protected
                    + self.shield_clearance_minimum_m
                )
        for box in self.static_boxes:
            half_length = 0.5 * float(box["length"])
            half_width = 0.5 * float(box["width"])
            if abs(float(box["y"])) - half_width <= protected:
                releases.append(
                    float(box["x"])
                    + half_length
                    + protected
                    + self.shield_clearance_minimum_m
                )
        return max(releases) if releases else float("-inf")

    def resample_scan(self, ranges, angle_min, angle_increment, range_max):
        ranges = np.asarray(ranges, dtype=np.float64)
        if ranges.ndim != 1 or len(ranges) < 2:
            raise ValueError("RUNTIME_SCAN_CARDINALITY_INVALID")
        if not math.isfinite(float(angle_min)):
            raise ValueError("RUNTIME_SCAN_ANGLE_MIN_INVALID")
        if not math.isfinite(float(angle_increment)) or float(angle_increment) <= 0.0:
            raise ValueError("RUNTIME_SCAN_ANGLE_INCREMENT_INVALID")
        if not math.isfinite(float(range_max)) or float(range_max) <= 0.0:
            raise ValueError("RUNTIME_SCAN_RANGE_MAX_INVALID")
        return super().resample_scan(ranges, angle_min, angle_increment, range_max)

    def append_scan(self, scan):
        scan = np.asarray(scan, dtype=np.float32)
        if scan.shape != (72,) or not np.all(np.isfinite(scan)):
            raise ValueError("RUNTIME_SCAN_NONFINITE_OR_SHAPE_MISMATCH")
        if np.any(scan < 0.02) or np.any(scan > self.maximum_range_m):
            raise ValueError("RUNTIME_SCAN_RANGE_CONTRACT_VIOLATION")
        super().append_scan(scan)

    def _known_static_point_mask(self, points):
        if not len(points):
            return np.zeros(0, dtype=bool)
        tolerance = self.static_surface_association_tolerance_m
        result = np.abs(np.abs(points[:, 1]) - self.wall_half_width_m) <= tolerance
        for circle in self.static_circles:
            radial = np.hypot(
                points[:, 0] - float(circle["x"]),
                points[:, 1] - float(circle["y"]),
            )
            result |= np.abs(radial - float(circle["radius"])) <= tolerance
        for box in self.static_boxes:
            dx = np.abs(points[:, 0] - float(box["x"]))
            dy = np.abs(points[:, 1] - float(box["y"]))
            half_length = 0.5 * float(box["length"])
            half_width = 0.5 * float(box["width"])
            on_vertical = (np.abs(dx - half_length) <= tolerance) & (dy <= half_width + tolerance)
            on_horizontal = (np.abs(dy - half_width) <= tolerance) & (dx <= half_length + tolerance)
            result |= on_vertical | on_horizontal
        return result

    def _known_static_geometry_clearance(self, state):
        minimum = (
            self.wall_half_width_m
            - abs(float(state[1]))
            - self.robot_radius_m
            - self.shield_inflation_m
        )
        for circle in self.static_circles:
            minimum = min(
                minimum,
                math.hypot(float(state[0]) - float(circle["x"]), float(state[1]) - float(circle["y"]))
                - float(circle["radius"])
                - self.robot_radius_m
                - self.shield_inflation_m,
            )
        for box in self.static_boxes:
            dx = max(abs(float(state[0]) - float(box["x"])) - 0.5 * float(box["length"]), 0.0)
            dy = max(abs(float(state[1]) - float(box["y"])) - 0.5 * float(box["width"]), 0.0)
            minimum = min(
                minimum,
                math.hypot(dx, dy) - self.robot_radius_m - self.shield_inflation_m,
            )
        return minimum

    def _clearance(self, preview, scan_points, dynamic_reachable=False):
        static_mask = self._known_static_point_mask(scan_points)
        static_points = scan_points[static_mask]
        dynamic_points = scan_points[~static_mask]
        minimum = float("inf")
        for index, state in enumerate(preview):
            time_s = index * self.preview_dt_s
            minimum = min(minimum, self._known_static_geometry_clearance(state))
            if len(static_points):
                distances = np.hypot(
                    static_points[:, 0] - state[0], static_points[:, 1] - state[1]
                )
                minimum = min(
                    minimum,
                    float(np.min(distances)) - self.robot_radius_m - self.shield_inflation_m,
                )
            if len(dynamic_points):
                distances = np.hypot(
                    dynamic_points[:, 0] - state[0], dynamic_points[:, 1] - state[1]
                )
                reachable_expansion = (
                    self.maximum_dynamic_obstacle_speed_mps * time_s
                    if dynamic_reachable
                    else 0.0
                )
                minimum = min(
                    minimum,
                    float(np.min(distances))
                    - self.robot_radius_m
                    - self.shield_inflation_m
                    - reachable_expansion,
                )
        return minimum

    def _goal_speed_limit(self, x_m):
        distance_to_goal_boundary = max(
            0.0, self.goal_x_m - self.goal_tolerance_m - float(x_m)
        )
        return math.sqrt(
            2.0 * self.maximum_deceleration_mps2 * distance_to_goal_boundary
        )

    def terminal_command_speed(self, action, x_m, proposed_speed_mps):
        """Avoid pure-pursuit stopping one lookahead radius before the goal ball."""
        if int(action) == 0:
            return 0.0
        proposed = _clip(float(proposed_speed_mps), 0.0, 1.2)
        if float(x_m) < self.goal_x_m - 1.5:
            return proposed
        # A small center-side target compensates only the controller lookahead;
        # the common evaluator still terminates at the unchanged 0.25 m ball.
        remaining = self.goal_x_m + 0.05 - float(x_m)
        if remaining <= 0.0:
            return 0.0
        stopping_limit = math.sqrt(2.0 * self.maximum_deceleration_mps2 * remaining)
        approach_floor = min(0.20, stopping_limit)
        return min(stopping_limit, max(proposed, approach_floor))

    def terminal_command(self, action, pose, proposed_speed_mps, proposed_yaw_rate_radps):
        """Preserve the speed envelope and converge to the frozen terminal pose.

        Candidate previews are intentionally optimized for interaction rather than
        terminal pose regulation.  Inside the last metre, track a point just beyond
        the unchanged goal centre so that a lateral candidate cannot strand the
        vehicle inside the position ball with an invalid heading.
        """
        x_m, y_m, yaw_rad = (float(value) for value in pose[:3])
        speed = self.terminal_command_speed(action, x_m, proposed_speed_mps)
        if int(action) == 0 or speed <= 0.0:
            return 0.0, 0.0
        if x_m < self.goal_x_m - 1.5:
            return speed, float(proposed_yaw_rate_radps)
        # Regulate heading explicitly instead of aiming at the goal centre: a
        # centre-seeking carrot can enter the position ball diagonally and then
        # lose all Ackermann authority as the speed envelope reaches zero.
        target_heading = _clip(-1.20 * y_m, -0.40, 0.40)
        heading_error = math.atan2(
            math.sin(target_heading - yaw_rad), math.cos(target_heading - yaw_rad)
        )
        maximum_curvature = math.tan(0.40) / self.wheelbase_m
        curvature = _clip(2.0 * heading_error, -maximum_curvature, maximum_curvature)
        yaw_rate = speed * curvature
        return speed, yaw_rate

    def _kinematic_step(self, state, target_speed, lateral_offset):
        if self.goal_speed_envelope:
            target_speed = min(float(target_speed), self._goal_speed_limit(state[0]))
        return super()._kinematic_step(state, target_speed, lateral_offset)

    def _preview(self, pose, speed, steering, action, stop_tail=False):
        if not stop_tail or self.safety_response_delay_s <= 0.0:
            return super()._preview(pose, speed, steering, action, stop_tail=stop_tail)
        state = (pose[0], pose[1], pose[2], speed, steering)
        states = [state]
        delay_steps = int(math.ceil(self.safety_response_delay_s / self.preview_dt_s))
        previous_offset = float(self.lateral_offsets[self.previous_action])
        held_measured_speed = float(state[3])
        for _ in range(delay_steps):
            state = self._kinematic_step(state, held_measured_speed, previous_offset)
            states.append(state)
        state = self._kinematic_step(
            state, float(self.target_speeds[action]), float(self.lateral_offsets[action])
        )
        states.append(state)
        for _ in range(8):
            state = self._kinematic_step(state, 0.0, float(self.lateral_offsets[action]))
            states.append(state)
            if state[3] <= 1.0e-8:
                break
        for _ in range(int(math.ceil(self.certified_stop_dwell_s / self.preview_dt_s))):
            states.append(state)
        return states

    @staticmethod
    def _observation_finite(observation):
        return all(np.all(np.isfinite(np.asarray(value))) for value in observation.values())

    @staticmethod
    def _analytic_action(observation):
        mask = np.asarray(observation["action_mask"], dtype=bool)
        prior = np.asarray(observation["analytic_prior"], dtype=np.float64)
        if mask.shape != (8,) or prior.shape != (8,) or not np.any(mask):
            return 0
        values = np.where(mask & np.isfinite(prior), prior, -np.inf)
        return int(np.argmax(values)) if np.any(np.isfinite(values)) else 0

    def suggest(self, pose, speed, yaw_rate, elapsed_s, input_health):
        health_failures = list(input_health.failures(self.maximum_input_age_s))
        try:
            observation, previews = self.build(pose, speed, yaw_rate, elapsed_s)
        except Exception as error:  # fail closed before any learned code
            self.last_suggested_action = 0
            return {
                "suggested_action": 0,
                "suggested_action_name": self.action_names[0],
                "analytic_action": 0,
                "learned_action": None,
                "intervened": False,
                "residual_margin": None,
                "latency_ms": 0.0,
                "fallback_reason": "observation_build_error:" + type(error).__name__,
                "input_health_failures": health_failures,
                "observation": None,
                "previews": None,
            }
        analytic = self._analytic_action(observation)
        if health_failures or not self._observation_finite(observation):
            action = 0
            reason = "input_health_failure" if health_failures else "nonfinite_observation"
            learned = None
            intervened = False
            margin = None
            latency_ms = 0.0
        elif self.actor is None:
            action = analytic
            reason = None
            learned = analytic
            intervened = False
            margin = 0.0
            latency_ms = 0.0
        else:
            started = time.perf_counter_ns()
            try:
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
                learned = int(result["learned_action"][0])
                intervened = bool(result["intervened"][0])
                margin = float(result["residual_margin"][0])
                reason = None
            except Exception as error:
                action = analytic
                learned = None
                intervened = False
                margin = None
                reason = "inference_error:" + type(error).__name__
            latency_ms = (time.perf_counter_ns() - started) / 1.0e6
            if latency_ms > self.inference_deadline_ms:
                action = analytic
                reason = "inference_deadline_miss"
        mask = np.asarray(observation["action_mask"], dtype=bool)
        if action < 0 or action >= 8 or not bool(mask[action]):
            action = analytic
            reason = "suggestion_permission_fallback"
        self.last_suggested_action = action
        return {
            "suggested_action": action,
            "suggested_action_name": self.action_names[action],
            "analytic_action": analytic,
            "learned_action": learned,
            "intervened": intervened,
            "residual_margin": margin,
            "latency_ms": latency_ms,
            "fallback_reason": reason,
            "input_health_failures": health_failures,
            "observation": observation,
            "previews": previews,
        }

    def revalidate(self, suggestion, pose, speed, yaw_rate, elapsed_s, input_health):
        failures = list(input_health.failures(self.maximum_input_age_s))
        if failures:
            validated = 0
            reason = "activation_input_health_failure"
            observation = None
            previews = None
        else:
            try:
                observation, previews = self.build(pose, speed, yaw_rate, elapsed_s)
                if not self._observation_finite(observation):
                    raise ValueError("NONFINITE_OBSERVATION")
                mask = np.asarray(observation["action_mask"], dtype=bool)
                requested = int(suggestion["suggested_action"])
                if 0 <= requested < 8 and bool(mask[requested]):
                    validated = requested
                    reason = None
                else:
                    validated = self._analytic_action(observation)
                    reason = "activation_mask_changed"
            except Exception as error:
                validated = 0
                reason = "activation_validation_error:" + type(error).__name__
                observation = None
                previews = None
        self.last_validated_action = validated
        return {
            "validated_action": validated,
            "validated_action_name": self.action_names[validated],
            "activation_fallback_reason": reason,
            "input_health_failures": failures,
            "observation": observation,
            "previews": previews,
        }

    def commit_executed(self, validated_action):
        action = int(validated_action)
        if action < 0 or action >= 8:
            raise ValueError("EXECUTED_ACTION_OUT_OF_RANGE")
        if self.last_validated_action is None or action != self.last_validated_action:
            raise ValueError("EXECUTION_SEMANTIC_MISMATCH")
        if action in (4, 5, 6, 7):
            new_side = 1 if self.lateral_offsets[action] > 0.0 else -1
            if self.committed_spatial_side != new_side:
                self.committed_spatial_speed_mps = float(self.target_speeds[action])
            self.committed_spatial_side = new_side
            self.confirmed_straight_run_length = 0
        elif action in (1, 2, 3):
            self.confirmed_straight_run_length += 1
            if (
                self.confirmed_straight_run_length >= 4
                and self.latest_pose_x_m > self._central_static_route_release_x()
            ):
                self.committed_spatial_side = 0
                self.committed_spatial_speed_mps = 0.0
        else:
            self.confirmed_straight_run_length = 0
        self.previous_action = action
        self.previous_target_speed = float(self.target_speeds[action])
        self.consecutive_stop_count = self.consecutive_stop_count + 1 if action == 0 else 0
        self.decision_count += 1
        self.execution_sequence += 1
        self.last_executed_action = action
        return {
            "execution_sequence": self.execution_sequence,
            "executed_action": action,
            "executed_action_name": self.action_names[action],
        }

    def execution_audit(self):
        return {
            "suggested_action": self.last_suggested_action,
            "validated_action": self.last_validated_action,
            "executed_action": self.last_executed_action,
            "previous_action_feature_source": "last_confirmed_executed_action",
            "committed_spatial_side": self.committed_spatial_side,
            "committed_spatial_speed_mps": self.committed_spatial_speed_mps,
            "spatial_speed_escalation_release_clearance_m": self.spatial_speed_escalation_release_clearance_m,
            "confirmed_straight_run_length": self.confirmed_straight_run_length,
            "central_static_route_release_x_m": self._central_static_route_release_x(),
            "certified_stop_preserves_spatial_side_commitment": True,
            "execution_sequence": self.execution_sequence,
        }

    def candidate_safety_diagnostics(self, pose, speed, yaw_rate):
        speed = _clip(float(speed), 0.0, 1.2)
        steering = 0.0
        if speed > 0.05:
            steering = _clip(
                math.atan(float(yaw_rate) * self.wheelbase_m / speed),
                -self.maximum_steering_rad,
                self.maximum_steering_rad,
            )
        points = self._scan_points_world(pose)
        static_mask = self._known_static_point_mask(points)
        dynamic_points = points[~static_mask]
        rows = []
        for action in range(8):
            full = self._preview(pose, speed, steering, action)
            backup = self._preview(pose, speed, steering, action, stop_tail=True)
            rows.append(
                {
                    "action": action,
                    "full_clearance_m": self._clearance(full, points, False),
                    "backup_clearance_m": self._clearance(backup, points, True),
                }
            )
        return {
            "scan_minimum_m": float(np.min(self.scan_history[-1])),
            "scan_point_count": int(len(points)),
            "static_point_count": int(np.sum(static_mask)),
            "dynamic_point_count": int(np.sum(~static_mask)),
            "dynamic_point_sample_world_m": dynamic_points[:20].tolist(),
            "dynamic_point_bounds_world_m": {
                "minimum": np.min(dynamic_points, axis=0).tolist(),
                "maximum": np.max(dynamic_points, axis=0).tolist(),
            } if len(dynamic_points) else None,
            "candidates": rows,
        }


class DirectionalBenchmarkRuntimeAdapter(BenchmarkRuntimeAdapter):
    """Map-rejected temporal scan tracker with directional reachable occupancy."""

    def __init__(
        self,
        *args,
        dynamic_surface_radius_m=0.36,
        dynamic_track_maximum_age_s=0.50,
        **kwargs
    ):
        super().__init__(*args, **kwargs)
        self.dynamic_surface_radius_m = float(dynamic_surface_radius_m)
        self.dynamic_acceleration_bound_mps2 = 0.40
        self.dynamic_uncertainty_rate_mps = 0.12
        self.dynamic_uncertainty_cap_m = 0.75
        self.dynamic_association_radius_m = 1.20
        self.dynamic_track_maximum_age_s = float(dynamic_track_maximum_age_s)
        self.track_history = deque(maxlen=8)
        self.dynamic_center = None
        self.dynamic_velocity = np.zeros(2, dtype=np.float64)
        self.dynamic_direction = np.zeros(2, dtype=np.float64)
        self.dynamic_track_valid = False
        self.dynamic_track_residual_m = None
        self.dynamic_track_stamp_s = None
        self.last_dynamic_surface_points = np.empty((0, 2), dtype=np.float64)
        self.latest_scan_pose_world = None
        self.latest_scan_stamp_s = None

    def _clear_dynamic_track(self):
        self.track_history.clear()
        self.dynamic_center = None
        self.dynamic_velocity = np.zeros(2, dtype=np.float64)
        self.dynamic_direction = np.zeros(2, dtype=np.float64)
        self.dynamic_track_valid = False
        self.dynamic_track_residual_m = None
        self.dynamic_track_stamp_s = None
        self.last_dynamic_surface_points = np.empty((0, 2), dtype=np.float64)

    def _world_cloud(self, scan, pose):
        scan = np.asarray(scan, dtype=np.float64)
        valid = scan < self.maximum_range_m - 0.05
        angles = float(pose[2]) + self.beam_angles
        if not np.any(valid):
            return np.empty((0, 2), dtype=np.float64)
        return np.stack(
            (
                float(pose[0]) + scan[valid] * np.cos(angles[valid]),
                float(pose[1]) + scan[valid] * np.sin(angles[valid]),
            ),
            axis=1,
        )

    def append_scan_with_pose(self, scan, pose, stamp_s):
        super().append_scan(scan)
        self.latest_scan_pose_world = tuple(float(value) for value in pose[:3])
        stamp_s = float(stamp_s)
        self.latest_scan_stamp_s = stamp_s
        cloud = self._world_cloud(scan, pose)
        static_mask = self._known_static_point_mask(cloud)
        dynamic = cloud[~static_mask]
        if len(dynamic) and self.dynamic_center is not None:
            predicted = self.dynamic_center.copy()
            if self.dynamic_track_stamp_s is not None:
                predicted += self.dynamic_velocity * max(0.0, float(stamp_s) - self.dynamic_track_stamp_s)
            distances = np.linalg.norm(dynamic - predicted[None, :], axis=1)
            associated = dynamic[distances <= self.dynamic_association_radius_m]
            if len(associated):
                dynamic = associated
            elif (
                self.dynamic_track_stamp_s is not None
                and stamp_s - self.dynamic_track_stamp_s
                <= self.dynamic_track_maximum_age_s
            ):
                # A short association miss must not let an unrelated scan
                # cluster hijack the current participant identity.
                dynamic = np.empty((0, 2), dtype=np.float64)
            else:
                self._clear_dynamic_track()
        elif (
            not len(dynamic)
            and self.dynamic_track_stamp_s is not None
            and stamp_s - self.dynamic_track_stamp_s
            > self.dynamic_track_maximum_age_s
        ):
            self._clear_dynamic_track()
        if len(dynamic):
            surface_center = np.median(dynamic, axis=0)
            direction = surface_center - np.asarray(pose[:2], dtype=np.float64)
            norm = float(np.linalg.norm(direction))
            center = surface_center.copy()
            if norm > 1.0e-8:
                center += self.dynamic_surface_radius_m * direction / norm
            self.track_history.append((stamp_s, center.copy()))
            self.dynamic_center = center
            self.dynamic_track_stamp_s = stamp_s
            self.last_dynamic_surface_points = dynamic.copy()
            if len(self.track_history) >= 3:
                stamps = np.asarray([item[0] for item in self.track_history], dtype=np.float64)
                centers = np.stack([item[1] for item in self.track_history])
                relative = stamps - stamps[-1]
                span = float(stamps[-1] - stamps[0])
                design = np.stack((relative, np.ones_like(relative)), axis=1)
                coefficients = np.linalg.lstsq(design, centers, rcond=None)[0]
                velocity = coefficients[0]
                fitted = design @ coefficients
                residual = float(np.sqrt(np.mean(np.sum((centers - fitted) ** 2, axis=1))))
                speed = float(np.linalg.norm(velocity))
                if speed > 1.50:
                    velocity *= 1.50 / speed
                self.dynamic_velocity = velocity
                velocity_norm = float(np.linalg.norm(velocity))
                if velocity_norm >= 0.05:
                    self.dynamic_direction = velocity / velocity_norm
                self.dynamic_track_residual_m = residual
                self.dynamic_track_valid = span >= 0.35 and residual <= 0.25

    def _scan_points_world(self, pose):
        scan_pose = self.latest_scan_pose_world or pose
        return self._world_cloud(self.scan_history[-1], scan_pose)

    def _partition_points(self, scan_points):
        static_mask = self._known_static_point_mask(scan_points)
        static_points = scan_points[static_mask]
        dynamic_points = scan_points[~static_mask]
        return static_points, dynamic_points

    def _clearance(self, preview, scan_points, dynamic_reachable=False):
        if not self.dynamic_track_valid:
            return super()._clearance(preview, scan_points, dynamic_reachable)
        static_points, dynamic_points = self._partition_points(scan_points)
        if not len(dynamic_points) and len(self.last_dynamic_surface_points):
            dynamic_points = self.last_dynamic_surface_points
        minimum = float("inf")
        for index, state in enumerate(preview):
            time_s = index * self.preview_dt_s
            minimum = min(minimum, self._known_static_geometry_clearance(state))
            if len(static_points):
                distances = np.hypot(
                    static_points[:, 0] - state[0], static_points[:, 1] - state[1]
                )
                minimum = min(
                    minimum,
                    float(np.min(distances)) - self.robot_radius_m - self.shield_inflation_m,
                )
            if len(dynamic_points):
                predicted = dynamic_points
                uncertainty = 0.0
                if dynamic_reachable:
                    safety_velocity = self.dynamic_velocity
                    if float(np.linalg.norm(self.dynamic_direction)) > 0.5:
                        safety_velocity = (
                            self.dynamic_direction * self.maximum_dynamic_obstacle_speed_mps
                        )
                    predicted = dynamic_points + safety_velocity[None, :] * time_s
                    uncertainty = min(
                        self.dynamic_uncertainty_cap_m,
                        self.dynamic_uncertainty_rate_mps * time_s
                        + 0.5 * self.dynamic_acceleration_bound_mps2 * time_s * time_s,
                    )
                distances = np.hypot(
                    predicted[:, 0] - state[0], predicted[:, 1] - state[1]
                )
                minimum = min(
                    minimum,
                    float(np.min(distances))
                    - self.robot_radius_m
                    - self.shield_inflation_m
                    - uncertainty,
                )
        return minimum

    def tracker_audit(self):
        return {
            "valid": bool(self.dynamic_track_valid),
            "center_world_m": self.dynamic_center.tolist() if self.dynamic_center is not None else None,
            "velocity_world_mps": self.dynamic_velocity.tolist(),
            "safety_direction_world": self.dynamic_direction.tolist(),
            "safety_prediction_speed_mps": self.maximum_dynamic_obstacle_speed_mps
            if float(np.linalg.norm(self.dynamic_direction)) > 0.5
            else float(np.linalg.norm(self.dynamic_velocity)),
            "speed_mps": float(np.linalg.norm(self.dynamic_velocity)),
            "fit_residual_m": self.dynamic_track_residual_m,
            "history_count": len(self.track_history),
            "track_age_s": (
                max(0.0, self.latest_scan_stamp_s - self.dynamic_track_stamp_s)
                if self.latest_scan_stamp_s is not None
                and self.dynamic_track_stamp_s is not None
                else None
            ),
            "maximum_track_age_s": self.dynamic_track_maximum_age_s,
            "association_miss_policy": "hold_then_clear_before_reacquisition",
            "source": "LaserScan+Odometry+static_map_rejection_only",
        }
