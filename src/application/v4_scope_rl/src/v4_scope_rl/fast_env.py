"""Deterministic fast sequential backend for V4-SCOPE-RL.

The backend models one reactive actor crossing an Ackermann ego path.  It is a
training and learning-value environment, not Gazebo evidence.  Every policy
action is filtered by the same conservative reachable-occupancy calculation;
the learned policy never receives an unsafe action.
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
    "spatial_left_v100",
    "spatial_right_v100",
)
TARGET_SPEEDS = np.asarray([0.0, 0.4, 0.8, 1.2, 1.0, 1.0], dtype=np.float64)
LATERAL_OFFSETS = np.asarray([0.0, 0.0, 0.0, 0.0, 1.15, -1.15], dtype=np.float64)


def _clip(value, lower, upper):
    return max(lower, min(upper, value))


def _distance_to_segment(px, py, ax, ay, bx, by):
    abx = bx - ax
    aby = by - ay
    denominator = abx * abx + aby * aby
    if denominator <= 1.0e-12:
        return math.hypot(px - ax, py - ay)
    ratio = _clip(((px - ax) * abx + (py - ay) * aby) / denominator, 0.0, 1.0)
    return math.hypot(px - (ax + ratio * abx), py - (ay + ratio * aby))


class CounterfactualCrossingEnv(gym.Env):
    """Single-reactive-actor crossing with hard-masked candidate actions."""

    metadata = {"render_modes": []}

    def __init__(self, config=None):
        super().__init__()
        config = dict(config or {})
        self.dt = float(config.get("replanning_period_s", 0.25))
        self.max_steps = int(config.get("maximum_episode_decisions", 72))
        self.preview_horizon_s = float(config.get("preview_horizon_s", 3.0))
        self.preview_dt_s = float(config.get("preview_dt_s", 0.25))
        self.goal_x = float(config.get("goal_x_m", 5.2))
        self.start_x = float(config.get("start_x_m", -2.6))
        self.wheelbase_m = float(config.get("wheelbase_m", 0.65))
        self.maximum_steering_rad = float(config.get("maximum_steering_rad", 0.38))
        self.maximum_steering_rate_radps = float(config.get("maximum_steering_rate_radps", 0.70))
        self.maximum_acceleration_mps2 = float(config.get("maximum_acceleration_mps2", 0.60))
        self.maximum_deceleration_mps2 = float(config.get("maximum_deceleration_mps2", 0.80))
        self.combined_radius_m = float(config.get("combined_radius_m", 0.82))
        self.reachable_position_margin_m = float(config.get("reachable_position_margin_m", 0.18))
        self.reachable_speed_margin_mps = float(config.get("reachable_speed_margin_mps", 0.32))
        self.cautious_yield_memory_s = float(config.get("cautious_yield_memory_s", 1.25))
        self.hesitant_yield_memory_s = float(config.get("hesitant_yield_memory_s", 0.75))
        self.elapsed_reward_per_s = float(config.get("elapsed_reward_per_s", -0.12))
        self.maximum_actor_deceleration_mps2 = float(
            config.get("maximum_actor_deceleration_mps2", 1.35)
        )
        self.maximum_actor_acceleration_mps2 = float(
            config.get("maximum_actor_acceleration_mps2", 1.35)
        )

        self.action_space = spaces.Discrete(len(ACTION_NAMES))
        self.global_feature_count = 12
        self.candidate_feature_count = 5
        size = self.global_feature_count + len(ACTION_NAMES) * self.candidate_feature_count
        self.observation_space = spaces.Box(-5.0, 5.0, shape=(size,), dtype=np.float32)

        self.ego_x = self.start_x
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_speed = 0.8
        self.ego_steering = 0.0
        self.actor_coordinate = -2.0
        self.actor_speed = 1.0
        self.actor_acceleration = 0.0
        self.actor_angle_deg = 90.0
        self.actor_policy = "cautious"
        self.actor_base_speed = 1.0
        self.actor_yield_memory_s = 0.0
        self.previous_target_speed = 0.8
        self.previous_action = 2
        self.consecutive_stop_count = 0
        self.step_count = 0
        self.minimum_clearance_m = float("inf")
        self.mask_violation_count = 0
        self.action_switch_count = 0
        self._candidate_features = None
        self._action_mask = None

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        options = dict(options or {})
        self.ego_x = self.start_x
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_speed = 0.8
        self.ego_steering = 0.0
        policies = ("cautious", "hesitant", "assertive")
        self.actor_policy = options.get("actor_policy", policies[int(self.np_random.integers(0, len(policies)))])
        self.actor_angle_deg = float(options.get("actor_angle_deg", self.np_random.choice([70.0, 90.0, 110.0])))
        speed_ranges = {
            "cautious": (0.82, 1.02),
            "hesitant": (0.98, 1.18),
            "assertive": (1.24, 1.44),
        }
        low, high = speed_ranges[self.actor_policy]
        self.actor_base_speed = float(options.get("actor_speed_mps", self.np_random.uniform(low, high)))
        self.actor_speed = self.actor_base_speed
        self.actor_acceleration = 0.0
        self.actor_yield_memory_s = 0.0
        self.actor_coordinate = float(options.get("actor_coordinate_m", self.np_random.uniform(-2.45, -1.75)))
        self.previous_target_speed = 0.8
        self.previous_action = 2
        self.consecutive_stop_count = 0
        self.step_count = 0
        self.minimum_clearance_m = float("inf")
        self.mask_violation_count = 0
        self.action_switch_count = 0
        self._refresh_candidates()
        return self._observation(), self._info()

    def _target_lateral(self, x_m, offset_m):
        if abs(offset_m) < 1.0e-12 or x_m <= -2.2 or x_m >= 2.2:
            return 0.0
        phase = (x_m + 2.2) / 4.4
        return offset_m * math.sin(math.pi * _clip(phase, 0.0, 1.0))

    def _kinematic_step(self, state, target_speed, lateral_offset, dt):
        x_m, y_m, yaw_rad, speed_mps, steering_rad = state
        speed_delta = _clip(
            target_speed - speed_mps,
            -self.maximum_deceleration_mps2 * dt,
            self.maximum_acceleration_mps2 * dt,
        )
        next_speed = _clip(speed_mps + speed_delta, 0.0, 1.2)
        target_y = self._target_lateral(x_m, lateral_offset)
        desired_steering = _clip(
            0.72 * (target_y - y_m) - 0.85 * yaw_rad,
            -self.maximum_steering_rad,
            self.maximum_steering_rad,
        )
        steering_delta = _clip(
            desired_steering - steering_rad,
            -self.maximum_steering_rate_radps * dt,
            self.maximum_steering_rate_radps * dt,
        )
        next_steering = _clip(
            steering_rad + steering_delta,
            -self.maximum_steering_rad,
            self.maximum_steering_rad,
        )
        average_speed = 0.5 * (speed_mps + next_speed)
        average_steering = 0.5 * (steering_rad + next_steering)
        next_x = x_m + average_speed * math.cos(yaw_rad) * dt
        next_y = y_m + average_speed * math.sin(yaw_rad) * dt
        next_yaw = yaw_rad + average_speed * math.tan(average_steering) * dt / self.wheelbase_m
        return next_x, next_y, next_yaw, next_speed, next_steering

    def _actor_world(self, coordinate):
        angle = math.radians(self.actor_angle_deg)
        return coordinate * math.cos(angle), coordinate * math.sin(angle)

    def _candidate_preview(self, action):
        state = (self.ego_x, self.ego_y, self.ego_yaw, self.ego_speed, self.ego_steering)
        target_speed = float(TARGET_SPEEDS[action])
        lateral_offset = float(LATERAL_OFFSETS[action])
        minimum_clearance = float("inf")
        sample_count = int(round(self.preview_horizon_s / self.preview_dt_s))
        for index in range(sample_count + 1):
            time_s = index * self.preview_dt_s
            lower_speed = max(0.0, self.actor_speed - self.reachable_speed_margin_mps)
            upper_speed = self.actor_speed + self.reachable_speed_margin_mps
            expanding_margin = self.reachable_position_margin_m + 0.06 * time_s
            coordinate_a = self.actor_coordinate + lower_speed * time_s - expanding_margin
            coordinate_b = self.actor_coordinate + upper_speed * time_s + expanding_margin
            actor_a = self._actor_world(coordinate_a)
            actor_b = self._actor_world(coordinate_b)
            distance = _distance_to_segment(state[0], state[1], actor_a[0], actor_a[1], actor_b[0], actor_b[1])
            minimum_clearance = min(minimum_clearance, distance - self.combined_radius_m)
            if index < sample_count:
                state = self._kinematic_step(state, target_speed, lateral_offset, self.preview_dt_s)
        projected_progress = state[0] - self.ego_x
        acceleration_cost = abs(target_speed - self.ego_speed)
        lateral_excursion = abs(lateral_offset)
        accepted = minimum_clearance >= -1.0e-9
        if action == 0:
            # Certified braking is the fail-closed action.  The episode starts
            # outside the conflict zone and the verifier never masks it.
            accepted = True
        return np.asarray(
            [
                1.0 if accepted else 0.0,
                projected_progress / 3.6,
                _clip(minimum_clearance, -1.0, 3.0) / 3.0,
                acceleration_cost / 1.2,
                lateral_excursion / 1.15,
            ],
            dtype=np.float32,
        )

    def _refresh_candidates(self):
        self._candidate_features = np.stack(
            [self._candidate_preview(action) for action in range(len(ACTION_NAMES))]
        )
        self._action_mask = self._candidate_features[:, 0] > 0.5
        self._action_mask[0] = True
        if not bool(np.any(self._action_mask)):
            raise RuntimeError("EMPTY_SAFE_ACTION_SET")

    def action_masks(self):
        return self._action_mask.copy()

    def analytic_action(self):
        features = self._candidate_features
        score = (
            features[:, 1]
            + 0.10 * np.clip(features[:, 2], 0.0, 1.0)
            - 0.035 * features[:, 3]
            - 0.025 * features[:, 4]
        )
        score = np.where(self._action_mask, score, -np.inf)
        return int(np.argmax(score))

    def _eta_difference(self):
        ego_eta = max(0.0, -self.ego_x) / max(self.ego_speed, 0.08)
        actor_eta = max(0.0, -self.actor_coordinate) / max(self.actor_speed, 0.08)
        return _clip(ego_eta - actor_eta, -6.0, 6.0)

    def _actor_desired_speed(self, chosen_target_speed):
        eta_difference = self._eta_difference()
        near_crossing = self.actor_coordinate < 1.4 and self.ego_x < 1.6
        if not near_crossing:
            self.actor_yield_memory_s = max(0.0, self.actor_yield_memory_s - self.dt)
            return self.actor_base_speed
        if self.actor_policy == "assertive":
            return self.actor_base_speed
        if self.actor_policy == "cautious":
            if chosen_target_speed >= 0.75 and eta_difference <= 0.70:
                # A visible high-commitment candidate makes the cautious actor
                # commit to yielding for a short response window.  The delay
                # is intentionally longer than one decision, which is the
                # sequential effect that the one-shot V3 score omits.
                self.actor_yield_memory_s = max(
                    self.actor_yield_memory_s, self.cautious_yield_memory_s
                )
            if self.actor_yield_memory_s > 0.0:
                self.actor_yield_memory_s = max(0.0, self.actor_yield_memory_s - self.dt)
                return 0.05
            if chosen_target_speed <= 0.41 and eta_difference >= -0.65:
                return min(1.35, self.actor_base_speed + 0.28)
            return self.actor_base_speed
        # Hesitant actors amplify repeated high commitment but recover when
        # the ego yields.  This creates delayed consequences without exposing
        # the policy label in the observation.
        phase = (self.step_count // 3) % 2
        if chosen_target_speed >= 0.75 and abs(eta_difference) <= 1.05:
            if phase == 0:
                self.actor_yield_memory_s = max(
                    self.actor_yield_memory_s, self.hesitant_yield_memory_s
                )
            if self.actor_yield_memory_s > 0.0:
                self.actor_yield_memory_s = max(0.0, self.actor_yield_memory_s - self.dt)
                return 0.18
            return min(1.30, self.actor_base_speed + 0.12)
        if chosen_target_speed <= 0.41:
            return min(1.30, self.actor_base_speed + 0.18)
        return self.actor_base_speed

    def _observation(self):
        normalized_previous_action = self.previous_action / float(len(ACTION_NAMES) - 1)
        globals_ = np.asarray(
            [
                self.step_count / float(self.max_steps),
                (self.ego_x - self.start_x) / (self.goal_x - self.start_x),
                self.ego_y / 1.5,
                self.ego_yaw / 0.6,
                self.ego_speed / 1.2,
                self.previous_target_speed / 1.2,
                self.actor_coordinate / 5.0,
                self.actor_speed / 1.5,
                self.actor_acceleration / 1.5,
                self._eta_difference() / 6.0,
                normalized_previous_action,
                min(self.consecutive_stop_count, 12) / 12.0,
            ],
            dtype=np.float32,
        )
        observation = np.concatenate((globals_, self._candidate_features.reshape(-1)))
        return np.clip(observation, -5.0, 5.0).astype(np.float32)

    def _info(self):
        return {
            "action_mask": self.action_masks(),
            "actor_policy_evaluator_only": self.actor_policy,
            "actor_angle_deg": self.actor_angle_deg,
            "minimum_clearance_m": self.minimum_clearance_m,
            "mask_violation_count": self.mask_violation_count,
            "step_count": self.step_count,
        }

    def step(self, action):
        action = int(action)
        if not self.action_space.contains(action):
            raise ValueError("ACTION_OUT_OF_RANGE")
        if not bool(self._action_mask[action]):
            self.mask_violation_count += 1
            raise ValueError("ACTION_REJECTED_BY_HARD_MASK")

        previous_x = self.ego_x
        previous_speed = self.ego_speed
        switched = action != self.previous_action
        if switched:
            self.action_switch_count += 1
        target_speed = float(TARGET_SPEEDS[action])
        state = self._kinematic_step(
            (self.ego_x, self.ego_y, self.ego_yaw, self.ego_speed, self.ego_steering),
            target_speed,
            float(LATERAL_OFFSETS[action]),
            self.dt,
        )
        self.ego_x, self.ego_y, self.ego_yaw, self.ego_speed, self.ego_steering = state

        desired_actor_speed = self._actor_desired_speed(target_speed)
        old_actor_speed = self.actor_speed
        actor_delta = desired_actor_speed - self.actor_speed
        actor_delta = _clip(
            actor_delta,
            -self.maximum_actor_deceleration_mps2 * self.dt,
            self.maximum_actor_acceleration_mps2 * self.dt,
        )
        self.actor_speed += actor_delta
        self.actor_speed = _clip(self.actor_speed, 0.0, 1.55)
        self.actor_acceleration = (self.actor_speed - old_actor_speed) / self.dt
        self.actor_coordinate += 0.5 * (old_actor_speed + self.actor_speed) * self.dt

        actor_x, actor_y = self._actor_world(self.actor_coordinate)
        physical_clearance = math.hypot(self.ego_x - actor_x, self.ego_y - actor_y) - self.combined_radius_m
        self.minimum_clearance_m = min(self.minimum_clearance_m, physical_clearance)

        self.step_count += 1
        self.previous_target_speed = target_speed
        self.consecutive_stop_count = self.consecutive_stop_count + 1 if action == 0 else 0
        self.previous_action = action

        goal_reached = self.ego_x >= self.goal_x and abs(self.ego_y) <= 0.45
        contact = physical_clearance < -1.0e-6
        timed_out = self.step_count >= self.max_steps
        terminated = bool(goal_reached or contact)
        truncated = bool(timed_out and not terminated)

        progress = max(-0.05, self.ego_x - previous_x)
        reward = progress + self.elapsed_reward_per_s * self.dt
        reward -= 0.015 * abs(self.ego_speed - previous_speed)
        reward -= 0.01 if switched else 0.0
        if goal_reached:
            reward += 6.0
        if contact:
            # This is an invariant alarm, not the safety mechanism.
            reward -= 20.0
        elif truncated:
            reward -= 2.0

        self._refresh_candidates()
        info = self._info()
        info.update(
            {
                "goal_reached": goal_reached,
                "contact": contact,
                "restricted_elapsed_s": self.step_count * self.dt,
                "ego_x_m": self.ego_x,
                "ego_y_m": self.ego_y,
                "actor_coordinate_m": self.actor_coordinate,
                "action_name": ACTION_NAMES[action],
            }
        )
        return self._observation(), float(reward), terminated, truncated, info

    def clone(self):
        clone = CounterfactualCrossingEnv(
            {
                "replanning_period_s": self.dt,
                "maximum_episode_decisions": self.max_steps,
                "preview_horizon_s": self.preview_horizon_s,
                "preview_dt_s": self.preview_dt_s,
                "goal_x_m": self.goal_x,
                "start_x_m": self.start_x,
                "wheelbase_m": self.wheelbase_m,
                "maximum_steering_rad": self.maximum_steering_rad,
                "maximum_steering_rate_radps": self.maximum_steering_rate_radps,
                "maximum_acceleration_mps2": self.maximum_acceleration_mps2,
                "maximum_deceleration_mps2": self.maximum_deceleration_mps2,
                "combined_radius_m": self.combined_radius_m,
                "reachable_position_margin_m": self.reachable_position_margin_m,
                "reachable_speed_margin_mps": self.reachable_speed_margin_mps,
                "cautious_yield_memory_s": self.cautious_yield_memory_s,
                "hesitant_yield_memory_s": self.hesitant_yield_memory_s,
                "elapsed_reward_per_s": self.elapsed_reward_per_s,
                "maximum_actor_deceleration_mps2": self.maximum_actor_deceleration_mps2,
                "maximum_actor_acceleration_mps2": self.maximum_actor_acceleration_mps2,
            }
        )
        state = copy.deepcopy(self.__dict__)
        # Gym's generator can be copied directly; spaces remain clone-owned.
        state.pop("action_space", None)
        state.pop("observation_space", None)
        clone.__dict__.update(state)
        return clone
