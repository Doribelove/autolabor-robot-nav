"""Sensor-level sequential interaction backend for V5-E2E-SCOPE-RL.

The policy observation contains only robot-available quantities.  Simulation
truth is exposed by a separate ``privileged_observation`` method for the
training critic and never appears in ``reset`` or ``step`` observations.
"""

import copy
import math

import gymnasium as gym
import numpy as np
from gymnasium import spaces


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
TARGET_SPEEDS = np.asarray([0.0, 0.4, 0.8, 1.2, 0.8, 0.8, 1.2, 1.2], dtype=np.float64)
LATERAL_OFFSETS = np.asarray([0.0, 0.0, 0.0, 0.0, 1.2, -1.2, 1.2, -1.2], dtype=np.float64)
FAMILY_NAMES = (
    "bidirectional_reactive_crossing",
    "sequential_two_actor_crossing",
    "partially_occluded_crossing",
    "narrow_corridor_yield",
)
MODE_NAMES = ("cautious", "hesitant", "assertive")


def _clip(value, lower, upper):
    return max(lower, min(upper, value))


def _angle_delta(target, source):
    return math.atan2(math.sin(target - source), math.cos(target - source))


class SensorCandidateEnv(gym.Env):
    """2-D LiDAR interaction simulator with an observable hard shield."""

    metadata = {"render_modes": []}

    def __init__(self, config=None):
        super().__init__()
        config = dict(config or {})
        self.dt = float(config.get("decision_period_s", 0.25))
        self.max_steps = int(config.get("maximum_episode_decisions", 80))
        self.history_frames = int(config.get("scan_history_frames", 6))
        self.beam_count = int(config.get("laser_beams", 72))
        self.field_of_view_rad = math.radians(float(config.get("laser_field_of_view_deg", 270.0)))
        self.maximum_range_m = float(config.get("laser_maximum_range_m", 8.0))
        self.scan_noise_std_m = float(config.get("scan_noise_std_m", 0.012))
        self.scan_dropout_probability = float(config.get("scan_dropout_probability", 0.006))
        self.preview_horizon_s = float(config.get("preview_horizon_s", 3.0))
        self.shield_horizon_s = float(config.get("shield_horizon_s", 0.75))
        self.maximum_dynamic_obstacle_speed_mps = float(
            config.get("maximum_dynamic_obstacle_speed_mps", 1.35)
        )
        self.preview_dt_s = float(config.get("preview_dt_s", 0.25))
        self.wheelbase_m = float(config.get("wheelbase_m", 0.65))
        self.maximum_acceleration_mps2 = float(config.get("maximum_acceleration_mps2", 0.60))
        self.maximum_deceleration_mps2 = float(config.get("maximum_deceleration_mps2", 0.90))
        self.maximum_steering_rad = float(config.get("maximum_steering_rad", 0.40))
        self.maximum_steering_rate_radps = float(config.get("maximum_steering_rate_radps", 0.75))
        self.robot_radius_m = float(config.get("robot_radius_m", 0.46))
        self.shield_inflation_m = float(config.get("shield_inflation_m", 0.12))
        self.shield_clearance_minimum_m = float(config.get("shield_clearance_minimum_m", 0.0))
        self.start_x_m = float(config.get("start_x_m", -4.0))
        self.goal_x_m = float(config.get("goal_x_m", 5.0))
        self.goal_tolerance_m = float(config.get("goal_tolerance_m", 0.30))
        self.advantage_gate_margin = float(config.get("advantage_gate_margin", 0.10))
        self.fixed_family = config.get("family")
        self.no_scan_history = bool(config.get("no_scan_history", False))
        self.lateral_offsets = np.asarray(
            config.get("lateral_offsets_m", LATERAL_OFFSETS), dtype=np.float64
        )
        self.target_speeds = np.asarray(
            config.get("target_speeds_mps", TARGET_SPEEDS), dtype=np.float64
        )
        self.action_names = tuple(config.get("action_names", ACTION_NAMES))
        if self.lateral_offsets.shape != (len(ACTION_NAMES),) or not np.all(np.isfinite(self.lateral_offsets)):
            raise ValueError("LATERAL_OFFSETS_CONTRACT_VIOLATION")
        if self.target_speeds.shape != (len(ACTION_NAMES),) or not np.all(np.isfinite(self.target_speeds)):
            raise ValueError("TARGET_SPEEDS_CONTRACT_VIOLATION")
        if len(self.action_names) != len(ACTION_NAMES):
            raise ValueError("ACTION_NAMES_CONTRACT_VIOLATION")
        self.lateral_offset_normalizer_m = max(1.0, float(np.max(np.abs(self.lateral_offsets))))

        self.action_space = spaces.Discrete(len(ACTION_NAMES))
        self.robot_feature_count = 10
        self.candidate_feature_count = 7
        # 2 actors x 6 state fields + 8 oracle clearances + 8 oracle TTC
        # proxies + 4 scene-family indicators.
        self.privileged_feature_count = 32
        self.observation_space = spaces.Dict(
            {
                "scans": spaces.Box(
                    0.0,
                    1.0,
                    shape=(self.history_frames, self.beam_count),
                    dtype=np.float32,
                ),
                "robot": spaces.Box(-5.0, 5.0, shape=(self.robot_feature_count,), dtype=np.float32),
                "candidates": spaces.Box(
                    -5.0,
                    5.0,
                    shape=(len(ACTION_NAMES), self.candidate_feature_count),
                    dtype=np.float32,
                ),
                "analytic_prior": spaces.Box(-10.0, 10.0, shape=(len(ACTION_NAMES),), dtype=np.float32),
                "action_mask": spaces.MultiBinary(len(ACTION_NAMES)),
            }
        )

        self.ego_x = self.start_x_m
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_speed = 0.8
        self.ego_steering = 0.0
        self.previous_action = 2
        self.previous_target_speed = 0.8
        self.consecutive_stop_count = 0
        self.step_count = 0
        self.family = FAMILY_NAMES[0]
        self.wall_half_width_m = 2.25
        self.actors = []
        self.static_circles = []
        self.scan_history = np.ones((self.history_frames, self.beam_count), dtype=np.float32)
        self._candidate_features = np.zeros((len(ACTION_NAMES), self.candidate_feature_count), dtype=np.float32)
        self._analytic_prior = np.zeros(len(ACTION_NAMES), dtype=np.float32)
        self._action_mask = np.ones(len(ACTION_NAMES), dtype=bool)
        self.minimum_physical_clearance_m = float("inf")
        self.mask_violation_count = 0
        self.activation_revalidation_failure_count = 0
        self.contact_count = 0
        self.intervention_count = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = dict(options or {})
        self.ego_x = self.start_x_m
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_speed = 0.8
        self.ego_steering = 0.0
        self.previous_action = 2
        self.previous_target_speed = 0.8
        self.consecutive_stop_count = 0
        self.step_count = 0
        self.minimum_physical_clearance_m = float("inf")
        self.mask_violation_count = 0
        self.activation_revalidation_failure_count = 0
        self.contact_count = 0
        self.intervention_count = 0
        self.family = options.get(
            "family",
            self.fixed_family
            or FAMILY_NAMES[int(self.np_random.integers(0, len(FAMILY_NAMES)))],
        )
        self._configure_scene(options)
        self._initialize_scan_history()
        self._refresh_candidates()
        observation = self.policy_observation()
        if not self.observation_space.contains(observation):
            raise RuntimeError("POLICY_OBSERVATION_CONTRACT_VIOLATION")
        return observation, self._info()

    def _actor(self, x, y, vx, vy, mode, response_gain=1.0):
        return {
            "x": float(x),
            "y": float(y),
            "vx": float(vx),
            "vy": float(vy),
            "base_vx": float(vx),
            "base_vy": float(vy),
            "mode": str(mode),
            "radius": 0.38,
            "response_gain": float(response_gain),
            "hesitation_phase": int(self.np_random.integers(0, 8)),
        }

    def _configure_scene(self, options):
        self.actors = []
        self.static_circles = []
        mode = options.get("actor_mode", MODE_NAMES[int(self.np_random.integers(0, len(MODE_NAMES)))])
        if self.family == "bidirectional_reactive_crossing":
            side = float(options.get("actor_side", self.np_random.choice([-1.0, 1.0])))
            speed = float(options.get("actor_speed_mps", self.np_random.uniform(0.72, 1.12)))
            x = float(self.np_random.uniform(-0.35, 0.35))
            y = side * float(self.np_random.uniform(1.45, 2.05))
            self.wall_half_width_m = 2.35
            self.actors.append(self._actor(x, y, 0.0, -side * speed, mode))
        elif self.family == "sequential_two_actor_crossing":
            first_side = float(self.np_random.choice([-1.0, 1.0]))
            speed_a = float(self.np_random.uniform(0.68, 1.02))
            speed_b = float(self.np_random.uniform(0.62, 0.98))
            self.wall_half_width_m = 2.45
            self.actors.append(
                self._actor(-0.65, first_side * self.np_random.uniform(1.35, 1.85), 0.0, -first_side * speed_a, mode)
            )
            second_mode = MODE_NAMES[int(self.np_random.integers(0, len(MODE_NAMES)))]
            self.actors.append(
                self._actor(1.05, -first_side * self.np_random.uniform(1.75, 2.25), 0.0, first_side * speed_b, second_mode)
            )
        elif self.family == "partially_occluded_crossing":
            side = float(self.np_random.choice([-1.0, 1.0]))
            speed = float(self.np_random.uniform(0.78, 1.10))
            self.wall_half_width_m = 2.35
            self.static_circles.append({"x": -0.35, "y": side * 0.78, "radius": 0.48})
            self.actors.append(
                self._actor(0.05, side * self.np_random.uniform(1.55, 1.95), 0.0, -side * speed, mode)
            )
        elif self.family == "narrow_corridor_yield":
            side = float(self.np_random.choice([-1.0, 1.0]))
            self.wall_half_width_m = 1.85
            speed = float(self.np_random.uniform(0.46, 0.72))
            lateral = side * float(self.np_random.uniform(0.50, 0.70))
            self.actors.append(
                self._actor(self.np_random.uniform(1.25, 1.85), lateral, -speed, 0.0, mode, response_gain=0.8)
            )
        else:
            raise ValueError("UNKNOWN_SCENE_FAMILY:" + str(self.family))

    def _scan_angles(self):
        return np.linspace(
            -0.5 * self.field_of_view_rad,
            0.5 * self.field_of_view_rad,
            self.beam_count,
            dtype=np.float64,
        )

    def _raycast_scan(self, actor_override=None, add_noise=True):
        angles = self.ego_yaw + self._scan_angles()
        directions = np.stack((np.cos(angles), np.sin(angles)), axis=1)
        ranges = np.full(self.beam_count, self.maximum_range_m, dtype=np.float64)
        circles = list(self.static_circles)
        actors = actor_override if actor_override is not None else self.actors
        circles.extend(
            {"x": actor["x"], "y": actor["y"], "radius": actor["radius"]}
            for actor in actors
        )
        origin = np.asarray([self.ego_x, self.ego_y], dtype=np.float64)
        for circle in circles:
            relative = np.asarray([circle["x"], circle["y"]], dtype=np.float64) - origin
            projection = directions @ relative
            perpendicular_sq = float(relative @ relative) - projection * projection
            radius_sq = float(circle["radius"]) ** 2
            valid = (projection > 0.0) & (perpendicular_sq <= radius_sq)
            root = np.sqrt(np.maximum(0.0, radius_sq - perpendicular_sq))
            hit = projection - root
            valid &= hit >= 0.0
            ranges = np.where(valid, np.minimum(ranges, hit), ranges)
        sine = directions[:, 1]
        for wall_y in (-self.wall_half_width_m, self.wall_half_width_m):
            valid = np.abs(sine) > 1.0e-8
            hit = np.where(valid, (wall_y - self.ego_y) / sine, np.inf)
            valid &= hit >= 0.0
            ranges = np.where(valid, np.minimum(ranges, hit), ranges)
        if add_noise:
            ranges += self.np_random.normal(0.0, self.scan_noise_std_m, size=self.beam_count)
            dropout = self.np_random.random(self.beam_count) < self.scan_dropout_probability
            ranges[dropout] = self.maximum_range_m
        return np.clip(ranges, 0.02, self.maximum_range_m).astype(np.float32)

    def _initialize_scan_history(self):
        scans = []
        for history_index in range(self.history_frames):
            lag_s = (self.history_frames - 1 - history_index) * self.dt
            past = []
            for actor in self.actors:
                value = dict(actor)
                value["x"] = actor["x"] - actor["vx"] * lag_s
                value["y"] = actor["y"] - actor["vy"] * lag_s
                past.append(value)
            scans.append(self._raycast_scan(past, add_noise=True))
        self.scan_history = np.stack(scans).astype(np.float32)

    def _target_lateral(self, x_m, offset_m):
        # The bump begins at the planning origin so an Ackermann platform has
        # enough steering distance to establish lateral clearance before the
        # conflict zone, then rejoins the goal line before termination.
        lateral_start_x_m = self.start_x_m
        lateral_end_x_m = 4.0
        if (
            abs(offset_m) < 1.0e-12
            or x_m <= lateral_start_x_m
            or x_m >= lateral_end_x_m
        ):
            return 0.0
        phase = (x_m - lateral_start_x_m) / (lateral_end_x_m - lateral_start_x_m)
        return offset_m * math.sin(math.pi * _clip(phase, 0.0, 1.0))

    def _kinematic_step(self, state, target_speed, lateral_offset, dt):
        x_m, y_m, yaw_rad, speed_mps, steering_rad = state
        delta_speed = _clip(
            target_speed - speed_mps,
            -self.maximum_deceleration_mps2 * dt,
            self.maximum_acceleration_mps2 * dt,
        )
        next_speed = _clip(speed_mps + delta_speed, 0.0, 1.2)
        target_y = self._target_lateral(x_m, lateral_offset)
        desired_steering = _clip(
            0.78 * (target_y - y_m) - 0.90 * yaw_rad,
            -self.maximum_steering_rad,
            self.maximum_steering_rad,
        )
        delta_steering = _clip(
            desired_steering - steering_rad,
            -self.maximum_steering_rate_radps * dt,
            self.maximum_steering_rate_radps * dt,
        )
        next_steering = _clip(
            steering_rad + delta_steering,
            -self.maximum_steering_rad,
            self.maximum_steering_rad,
        )
        average_speed = 0.5 * (speed_mps + next_speed)
        average_steering = 0.5 * (steering_rad + next_steering)
        next_x = x_m + average_speed * math.cos(yaw_rad) * dt
        next_y = y_m + average_speed * math.sin(yaw_rad) * dt
        next_yaw = yaw_rad + average_speed * math.tan(average_steering) * dt / self.wheelbase_m
        return next_x, next_y, next_yaw, next_speed, next_steering

    def _candidate_preview(self, action):
        state = (self.ego_x, self.ego_y, self.ego_yaw, self.ego_speed, self.ego_steering)
        states = [state]
        count = int(round(self.preview_horizon_s / self.preview_dt_s))
        for _ in range(count):
            state = self._kinematic_step(
                state,
                float(self.target_speeds[action]),
                float(self.lateral_offsets[action]),
                self.preview_dt_s,
            )
            states.append(state)
        return states

    def _scan_points_world(self):
        scan = self.scan_history[-1].astype(np.float64)
        angles = self.ego_yaw + self._scan_angles()
        valid = scan < self.maximum_range_m - 0.05
        return np.stack(
            (
                self.ego_x + scan[valid] * np.cos(angles[valid]),
                self.ego_y + scan[valid] * np.sin(angles[valid]),
            ),
            axis=1,
        ) if np.any(valid) else np.empty((0, 2), dtype=np.float64)

    def _known_static_point_mask(self, points):
        """Identify scan returns explained by the local static costmap.

        This is deliberately a sensor/map operation rather than simulator actor
        access.  Returns not explained by walls or mapped circles are treated
        as dynamic and receive a reachable-occupancy expansion.
        """
        if not len(points):
            return np.zeros(0, dtype=bool)
        result = np.abs(np.abs(points[:, 1]) - self.wall_half_width_m) <= 0.07
        for circle in self.static_circles:
            radial = np.hypot(points[:, 0] - circle["x"], points[:, 1] - circle["y"])
            result |= np.abs(radial - circle["radius"]) <= 0.07
        return result

    def _observable_clearance(self, preview, horizon_s=None, reachable_dynamic=False):
        points = self._scan_points_world()
        static_mask = self._known_static_point_mask(points)
        static_points = points[static_mask]
        dynamic_points = points[~static_mask]
        minimum = float("inf")
        maximum_index = len(preview)
        if horizon_s is not None:
            maximum_index = min(
                maximum_index,
                int(round(horizon_s / self.preview_dt_s)) + 1,
            )
        for index, state in enumerate(preview[:maximum_index]):
            time_s = index * self.preview_dt_s
            wall_clearance = self.wall_half_width_m - abs(state[1]) - self.robot_radius_m
            minimum = min(minimum, wall_clearance)
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
                    if reachable_dynamic
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

    def _certified_backup_preview(self, action):
        """One decision followed by maximum-deceleration certified stop."""
        state = (self.ego_x, self.ego_y, self.ego_yaw, self.ego_speed, self.ego_steering)
        states = [state]
        state = self._kinematic_step(
            state,
            float(self.target_speeds[action]),
            float(self.lateral_offsets[action]),
            self.preview_dt_s,
        )
        states.append(state)
        maximum_tail_steps = int(math.ceil(1.2 / self.maximum_deceleration_mps2 / self.preview_dt_s)) + 2
        for _ in range(maximum_tail_steps):
            state = self._kinematic_step(
                state,
                0.0,
                float(self.lateral_offsets[action]),
                self.preview_dt_s,
            )
            states.append(state)
            if state[3] <= 1.0e-8:
                break
        return states

    def _oracle_clearance(self, action):
        preview = self._candidate_preview(action)
        minimum = float("inf")
        for index, state in enumerate(preview):
            time_s = index * self.preview_dt_s
            minimum = min(
                minimum,
                self.wall_half_width_m - abs(state[1]) - self.robot_radius_m,
            )
            for actor in self.actors:
                actor_x = actor["x"] + actor["vx"] * time_s
                actor_y = actor["y"] + actor["vy"] * time_s
                minimum = min(
                    minimum,
                    math.hypot(state[0] - actor_x, state[1] - actor_y)
                    - self.robot_radius_m
                    - actor["radius"],
                )
            for circle in self.static_circles:
                minimum = min(
                    minimum,
                    math.hypot(state[0] - circle["x"], state[1] - circle["y"])
                    - self.robot_radius_m
                    - circle["radius"],
                )
        return minimum

    def _refresh_candidates(self):
        features = []
        priors = []
        masks = []
        for action in range(len(ACTION_NAMES)):
            preview = self._candidate_preview(action)
            full_clearance = self._observable_clearance(preview)
            backup_preview = self._certified_backup_preview(action)
            shield_clearance = self._observable_clearance(
                backup_preview,
                reachable_dynamic=True,
            )
            progress = preview[-1][0] - self.ego_x
            acceleration_cost = abs(float(self.target_speeds[action]) - self.ego_speed)
            lateral_cost = abs(float(self.lateral_offsets[action]))
            switch_cost = 1.0 if action != self.previous_action else 0.0
            prior = (
                progress
                + 1.50 * _clip(full_clearance, -1.0, 2.0)
                - 0.035 * acceleration_cost
                - 0.025 * lateral_cost
                - 0.01 * switch_cost
            )
            accepted = shield_clearance >= self.shield_clearance_minimum_m - 1.0e-8
            if action == 0:
                accepted = True
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
            priors.append(prior)
            masks.append(accepted)
        self._candidate_features = np.asarray(features, dtype=np.float32)
        self._analytic_prior = np.asarray(priors, dtype=np.float32)
        self._action_mask = np.asarray(masks, dtype=bool)
        self._action_mask[0] = True
        if not bool(np.any(self._action_mask)):
            raise RuntimeError("EMPTY_SAFE_ACTION_SET")

    def action_masks(self):
        return self._action_mask.copy()

    def analytic_action(self):
        values = np.where(self._action_mask, self._analytic_prior, -np.inf)
        return int(np.argmax(values))

    def policy_observation(self):
        scans = self.scan_history / self.maximum_range_m
        if self.no_scan_history:
            scans = np.repeat(scans[-1:, :], self.history_frames, axis=0)
        robot = np.asarray(
            [
                (self.ego_x - self.start_x_m) / (self.goal_x_m - self.start_x_m),
                self.ego_y / max(1.0, self.wall_half_width_m),
                self.ego_yaw / 0.7,
                self.ego_speed / 1.2,
                self.previous_target_speed / 1.2,
                (self.goal_x_m - self.ego_x) / 9.0,
                self.previous_action / float(len(ACTION_NAMES) - 1),
                self.step_count / float(self.max_steps),
                min(self.consecutive_stop_count, 12) / 12.0,
                float(np.min(self.scan_history[-1])) / self.maximum_range_m,
            ],
            dtype=np.float32,
        )
        return {
            "scans": scans.astype(np.float32).copy(),
            "robot": np.clip(robot, -5.0, 5.0),
            "candidates": self._candidate_features.copy(),
            "analytic_prior": self._analytic_prior.copy(),
            "action_mask": self._action_mask.astype(np.int8).copy(),
        }

    def privileged_observation(self):
        values = []
        for index in range(2):
            if index < len(self.actors):
                actor = self.actors[index]
                values.extend(
                    [
                        actor["x"] / 5.0,
                        actor["y"] / 2.5,
                        actor["vx"] / 1.5,
                        actor["vy"] / 1.5,
                        actor["radius"],
                        MODE_NAMES.index(actor["mode"]) / 2.0,
                    ]
                )
            else:
                values.extend([0.0] * 6)
        oracle_clearances = [
            _clip(self._oracle_clearance(action), -2.0, 3.0) / 3.0
            for action in range(len(ACTION_NAMES))
        ]
        values.extend(oracle_clearances)
        # Oracle time-to-conflict proxy for each candidate.
        values.extend(
            [
                _clip(2.0 + 2.0 * clearance, 0.0, 6.0) / 6.0
                for clearance in oracle_clearances
            ]
        )
        values.extend([1.0 if self.family == name else 0.0 for name in FAMILY_NAMES])
        result = np.asarray(values, dtype=np.float32)
        if result.shape != (self.privileged_feature_count,):
            raise RuntimeError("PRIVILEGED_SCHEMA_SHAPE_MISMATCH")
        return result

    def _actor_desired_velocity(self, actor, chosen_target_speed):
        base = np.asarray([actor["base_vx"], actor["base_vy"]], dtype=np.float64)
        speed = float(np.linalg.norm(base))
        if speed <= 1.0e-8:
            return base
        direction = base / speed
        distance_to_crossing = math.hypot(actor["x"], actor["y"])
        ego_eta = max(0.0, -self.ego_x) / max(0.10, self.ego_speed)
        actor_eta = distance_to_crossing / max(0.10, speed)
        conflict = abs(ego_eta - actor_eta) < 1.15 and self.ego_x < 1.2
        target_speed = speed
        if conflict and actor["mode"] == "cautious" and chosen_target_speed >= 0.75:
            target_speed = 0.18 * actor["response_gain"]
        elif conflict and actor["mode"] == "hesitant":
            phase = (self.step_count + actor["hesitation_phase"]) % 8
            target_speed = (0.25 if phase < 4 else min(1.25, speed + 0.15)) * actor["response_gain"]
        # All actors retain a physical collision-avoidance boundary.  Their
        # interaction modes affect negotiation, but none is an adversarial
        # rammer that can make certified stop intrinsically unsafe.
        robot_velocity = self.ego_speed * np.asarray(
            [math.cos(self.ego_yaw), math.sin(self.ego_yaw)], dtype=np.float64
        )
        relative_position = np.asarray(
            [actor["x"] - self.ego_x, actor["y"] - self.ego_y], dtype=np.float64
        )
        candidate_velocity = direction * target_speed
        relative_velocity = candidate_velocity - robot_velocity
        relative_speed_sq = float(relative_velocity @ relative_velocity)
        closest_time = 0.0
        if relative_speed_sq > 1.0e-8:
            closest_time = _clip(
                -float(relative_position @ relative_velocity) / relative_speed_sq,
                0.0,
                2.75,
            )
        closest_separation = float(
            np.linalg.norm(relative_position + closest_time * relative_velocity)
        )
        protected_separation = self.robot_radius_m + actor["radius"] + 0.18
        if closest_time > 0.0 and closest_separation < protected_separation:
            current_separation = float(np.linalg.norm(relative_position))
            speed_cap = _clip(0.75 * (current_separation - protected_separation), 0.0, speed)
            target_speed = min(target_speed, speed_cap)
        return direction * target_speed

    def _advance_actors(self, chosen_target_speed):
        for actor in self.actors:
            desired = self._actor_desired_velocity(actor, chosen_target_speed)
            current = np.asarray([actor["vx"], actor["vy"]], dtype=np.float64)
            delta = desired - current
            norm = float(np.linalg.norm(delta))
            maximum = 1.20 * self.dt
            if norm > maximum:
                delta *= maximum / norm
            updated = current + delta
            actor["x"] += 0.5 * (current[0] + updated[0]) * self.dt
            actor["y"] += 0.5 * (current[1] + updated[1]) * self.dt
            actor["vx"] = float(updated[0])
            actor["vy"] = float(updated[1])

    def _physical_clearance(self):
        minimum = self.wall_half_width_m - abs(self.ego_y) - self.robot_radius_m
        for actor in self.actors:
            minimum = min(
                minimum,
                math.hypot(self.ego_x - actor["x"], self.ego_y - actor["y"])
                - self.robot_radius_m
                - actor["radius"],
            )
        for circle in self.static_circles:
            minimum = min(
                minimum,
                math.hypot(self.ego_x - circle["x"], self.ego_y - circle["y"])
                - self.robot_radius_m
                - circle["radius"],
            )
        return minimum

    def _info(self):
        return {
            "family_evaluator_only": self.family,
            "actor_modes_evaluator_only": [actor["mode"] for actor in self.actors],
            "action_mask": self.action_masks(),
            "minimum_physical_clearance_m": self.minimum_physical_clearance_m,
            "mask_violation_count": self.mask_violation_count,
            "activation_revalidation_failure_count": self.activation_revalidation_failure_count,
            "contact_count": self.contact_count,
            "step_count": self.step_count,
        }

    def step(self, action):
        action = int(action)
        if not self.action_space.contains(action):
            raise ValueError("ACTION_OUT_OF_RANGE")
        if not bool(self._action_mask[action]):
            self.mask_violation_count += 1
            raise ValueError("ACTION_REJECTED_BY_HARD_MASK")
        # The same observable shield is rerun immediately before activation.
        self._refresh_candidates()
        if not bool(self._action_mask[action]):
            self.activation_revalidation_failure_count += 1
            action = 0

        previous_x = self.ego_x
        previous_speed = self.ego_speed
        switched = action != self.previous_action
        target_speed = float(self.target_speeds[action])
        state = self._kinematic_step(
            (self.ego_x, self.ego_y, self.ego_yaw, self.ego_speed, self.ego_steering),
            target_speed,
            float(self.lateral_offsets[action]),
            self.dt,
        )
        self.ego_x, self.ego_y, self.ego_yaw, self.ego_speed, self.ego_steering = state
        self._advance_actors(target_speed)
        clearance = self._physical_clearance()
        self.minimum_physical_clearance_m = min(self.minimum_physical_clearance_m, clearance)
        contact = clearance < -1.0e-7
        if contact:
            self.contact_count += 1

        self.step_count += 1
        self.previous_target_speed = target_speed
        self.previous_action = action
        self.consecutive_stop_count = self.consecutive_stop_count + 1 if action == 0 else 0
        new_scan = self._raycast_scan(add_noise=True)
        self.scan_history = np.concatenate((self.scan_history[1:], new_scan[None, :]), axis=0)
        self._refresh_candidates()

        goal_reached = (
            self.ego_x >= self.goal_x_m - self.goal_tolerance_m
            and abs(self.ego_y) <= 0.55
            and abs(self.ego_yaw) <= 0.35
        )
        timeout = self.step_count >= self.max_steps
        terminated = bool(goal_reached or contact)
        truncated = bool(timeout and not terminated)
        progress = max(-0.05, self.ego_x - previous_x)
        reward = progress - 0.30 * self.dt
        reward -= 0.01 * abs(self.ego_speed - previous_speed)
        reward -= 0.005 if switched else 0.0
        if goal_reached:
            reward += 8.0
        if contact:
            reward -= 30.0
        elif truncated:
            reward -= 4.0
        info = self._info()
        info.update(
            {
                "goal_reached": bool(goal_reached),
                "contact": bool(contact),
                "timeout": bool(truncated),
                "restricted_elapsed_s": self.step_count * self.dt,
                "ego_x_m": self.ego_x,
                "ego_y_m": self.ego_y,
                "action_id": action,
                "action_name": self.action_names[action],
            }
        )
        return self.policy_observation(), float(reward), terminated, truncated, info

    def clone(self):
        clone = SensorCandidateEnv(
            {
                "decision_period_s": self.dt,
                "maximum_episode_decisions": self.max_steps,
                "scan_history_frames": self.history_frames,
                "laser_beams": self.beam_count,
                "laser_field_of_view_deg": math.degrees(self.field_of_view_rad),
                "laser_maximum_range_m": self.maximum_range_m,
                "scan_noise_std_m": self.scan_noise_std_m,
                "scan_dropout_probability": self.scan_dropout_probability,
                "preview_horizon_s": self.preview_horizon_s,
                "shield_horizon_s": self.shield_horizon_s,
                "maximum_dynamic_obstacle_speed_mps": self.maximum_dynamic_obstacle_speed_mps,
                "preview_dt_s": self.preview_dt_s,
                "wheelbase_m": self.wheelbase_m,
                "maximum_acceleration_mps2": self.maximum_acceleration_mps2,
                "maximum_deceleration_mps2": self.maximum_deceleration_mps2,
                "maximum_steering_rad": self.maximum_steering_rad,
                "maximum_steering_rate_radps": self.maximum_steering_rate_radps,
                "robot_radius_m": self.robot_radius_m,
                "shield_inflation_m": self.shield_inflation_m,
                "start_x_m": self.start_x_m,
                "goal_x_m": self.goal_x_m,
                "goal_tolerance_m": self.goal_tolerance_m,
                "advantage_gate_margin": self.advantage_gate_margin,
                "family": self.fixed_family,
                "no_scan_history": self.no_scan_history,
            }
        )
        state = copy.deepcopy(self.__dict__)
        state.pop("action_space", None)
        state.pop("observation_space", None)
        clone.__dict__.update(state)
        return clone
