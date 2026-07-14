"""Formal multi-scene T11 SAC training/evaluation with complete audited bundles."""

import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Dict, Mapping, Sequence

import gymnasium as gym
import numpy as np
import rospy
import yaml
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from teb_rl_tuner.config import EXPECTED_THETA_ORDER
from teb_rl_tuner.direct_theta_action import DirectThetaMapping
from teb_rl_tuner.fallback_policy import ConservativeFallbackPolicy
from teb_rl_tuner.parameter_projection import ParameterLimit, ParameterProjector
from teb_rl_tuner.reward_cost import RewardWeights
from teb_rl_tuner.sac_environment import (
    DirectThetaGymEnv, ResidualSemanticEtaGymEnv, SemanticEtaGymEnv,
)
from teb_rl_tuner.sac_training import SacCheckpointManager, SacTrainingConfig, build_sac
from teb_rl_tuner.safety_ablation import NoFallbackSafetyAdapter
from teb_rl_tuner.safety_gate import SimulationWriteContext
from teb_rl_tuner.safety_margin_filter import SafetyMarginConfig, SafetyMarginFilter
from teb_rl_tuner.semantic_action import FrozenSemanticMapping, ResidualSemanticMapping
from teb_rl_tuner.state_builder import StateBuilder
from teb_rl_tuner.teb_parameter_client import RosDynamicReconfigureBackend, TebParameterClient
from teb_rl_tuner.training_environment import EnvironmentConfig, TrainingEnvironment
from thesis_experiment.gazebo_training_adapter import (
    FEATURE_ORDER, GazeboTrainingAdapter, TrainingSafetyAdapter,
)
from thesis_experiment.run_artifacts import (
    RunValidator, sha256_file, write_checksums, write_episode_csv,
    write_run_manifest, write_step_csv,
)
from thesis_experiment.t11_contract import validate_t11_contract
from thesis_experiment.v2_contract import load_v1_yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
ALGORITHM_LABELS = {
    ("RL-TEB-Semantic-Eta", "FullSafety"): "RL-TEB-Semantic-Eta",
    # Keep the frozen episode-schema algorithm identity; the run report and
    # safety_mode distinguish the T12 runtime contract from legacy FullSafety.
    ("RL-TEB-Semantic-Eta", "T12Safety"): "RL-TEB-Eta-FullSafety",
    ("RL-TEB-Semantic-Eta", "T12LegacySafety"): "RL-TEB-Eta-FullSafety",
    ("RL-TEB-Semantic-Eta", "ProjectionOnly"): "RL-TEB-Eta-ProjectionOnly",
    ("RL-TEB-Semantic-Eta", "NoSafety"): "RL-TEB-Eta-NoSafety",
    ("RL-TEB-Semantic-Eta", "NoFallback"): "RL-TEB-Eta-NoFallback",
    ("RL-TEB-Direct-Theta", "FullSafety"): "RL-TEB-Direct-Theta",
}


def _json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"))


def _git(*args: str) -> str:
    return subprocess.check_output(
        ["git"] + list(args), cwd=str(WORKSPACE), text=True,
        stderr=subprocess.DEVNULL,
    ).strip()


def _mean(values: Sequence[float]) -> float:
    return float(sum(values) / len(values)) if values else 0.0


class T11Recorder:
    def __init__(
        self, run_id: str, algorithm: str, training_seed: int,
        scene_manifest_sha256: str, config_version: str,
    ) -> None:
        self.run_id, self.algorithm = run_id, algorithm
        self.training_seed = int(training_seed)
        self.scene_manifest_sha256 = scene_manifest_sha256
        self.config_version = config_version
        self.episodes, self.steps, self.failures = [], [], []
        self.phase = "train"
        self._episode = None
        self._episode_counter = 0

    def begin(self, env: Any) -> None:
        scenario = dict(env.core.adapter.current_scenario)
        self._episode_counter += 1
        episode_id = "{}-episode-{:06d}".format(self.run_id, self._episode_counter)
        start = scenario.get("start", (0.0, 0.0, 0.0))
        goal = env.core.adapter.goal_xy
        self._episode = {
            "episode_id": episode_id, "scenario": scenario,
            "started": rospy.Time.now().to_sec(), "step_count": 0,
            "reward": 0.0, "smoothness": 0.0, "variation": 0.0,
            "projection_count": 0, "safety_count": 0, "fallback_count": 0,
            "fallback_duration": 0.0, "candidate_violation_count": 0,
            "near_duration": 0.0, "write_latencies": [], "active_latencies": [],
            "inference_latencies": [], "start_xy": (float(start[0]), float(start[1])),
            "goal_xy": (float(goal[0]), float(goal[1])),
        }

    def record_step(
        self, env: Any, action: Any, previous_theta: Mapping[str, float], reward: float,
        terminated: bool, truncated: bool, info: Mapping[str, Any], inference_ms: float = 0.0,
    ) -> None:
        if self._episode is None:
            self.begin(env)
        episode = self._episode
        adapter, metrics = env.core.adapter, dict(env.core.adapter.last_metrics)
        fields = dict(info.get("reward_fields", {}))
        projected, safe = info.get("projected_theta", previous_theta), info.get("safe_theta", previous_theta)
        applied = info.get("applied_theta", adapter.current_theta())
        projection_modified = bool(info.get("projection_modified", False))
        safety_modified = dict(projected) != dict(safe)
        fallback_active = bool(info.get("fallback_active", False))
        write_ms = max(0.0, (float(info.get("t_ack", 0.0)) -
                             float(info.get("t_request", 0.0))) * 1000.0)
        active_ms = max(0.0, (float(info.get("t_active", 0.0)) -
                              float(info.get("t_request", 0.0))) * 1000.0)
        delta = []
        for name in EXPECTED_THETA_ORDER:
            low, high = env.mapping.bounds[name]
            delta.append(2.0 * (float(applied[name]) - float(previous_theta[name])) / (high - low))
        episode["step_count"] += 1
        episode["reward"] += float(reward)
        episode["smoothness"] += -float(fields.get("reward_smoothness", 0.0))
        episode["variation"] += sum(abs(value) for value in delta)
        episode["projection_count"] += int(projection_modified)
        episode["safety_count"] += int(safety_modified)
        episode["fallback_count"] += int(fallback_active)
        episode["fallback_duration"] += float(fields.get("cost_emergency_or_fallback", 0.0))
        episode["candidate_violation_count"] += int(projection_modified)
        episode["near_duration"] += float(fields.get("cost_near_collision", 0.0))
        episode["write_latencies"].append(write_ms)
        episode["active_latencies"].append(active_ms)
        episode["inference_latencies"].append(float(inference_ms))
        row = {
            "run_id": self.run_id, "episode_id": episode["episode_id"],
            "step_id": episode["step_count"] - 1,
            "config_seq": int(info.get("config_seq", env.core.config_seq)),
            "t_observation": float(metrics.get("stamp", rospy.Time.now().to_sec())),
            "t_decision": float(info.get("t_decision", metrics.get("stamp", 0.0))),
            "t_request": info.get("t_request"), "t_ack": info.get("t_ack"),
            "t_active": info.get("t_active"),
            "t_window_end": float(info.get("t_window_end", metrics.get("stamp", 0.0))),
            "planner_cycle_count": 0,
            "valid_feedback_duration": float(fields.get("valid_feedback_duration", 0.0)),
            "state_valid": True, "invalid_reason": "",
            "goal_distance_start": float(fields.get("goal_distance_start", metrics.get("goal_distance", 0.0))),
            "goal_distance_end": float(fields.get("goal_distance_end", metrics.get("goal_distance", 0.0))),
            "path_error_mean": float(metrics.get("path_error", 0.0)),
            "path_heading_error_mean": float(metrics.get("path_heading_error", 0.0)),
            "d_obs_min": float(adapter.minimum_clearance),
            "obstacle_density_mean": float(metrics.get("obstacle_density", 0.0)),
            "ttc_min": float(metrics.get("ttc", 0.0)),
            "linear_velocity_mean": float(metrics.get("linear_velocity", 0.0)),
            "angular_velocity_mean": float(metrics.get("angular_velocity", 0.0)),
            "eta_before_json": _json(info["eta_before"]) if "eta_before" in info else None,
            "action_raw_json": _json(np.asarray(action).tolist()),
            "eta_after_json": _json(info["eta_after"]) if "eta_after" in info else None,
            "theta_previous_json": _json(previous_theta),
            "theta_candidate_json": _json(info.get("candidate_theta", applied)),
            "theta_projected_json": _json(projected), "theta_safe_json": _json(safe),
            "theta_applied_json": _json(applied),
            "projection_modified": projection_modified,
            "projection_reason": "|".join(info.get("projection_reasons", ())),
            "safety_modified": safety_modified,
            "safety_mode": str(info.get("safety_mode", "NORMAL")),
            "safety_reason": "|".join(info.get("safety_reasons", ())),
            "fallback_active": fallback_active,
            "fallback_reason": "|".join(info.get("safety_reasons", ())) if fallback_active else "",
            "inference_latency": float(inference_ms),
            "parameter_write_latency": write_ms, "parameter_activation_latency": active_ms,
            "transition_stored": bool(info.get("transition_stored", False)),
            "transition_drop_reason": str(info.get("transition_drop_reason", "")),
        }
        for name in (
            "reward_total", "reward_progress", "reward_time", "reward_near_obstacle",
            "reward_path_error", "reward_smoothness", "reward_planner_failure",
            "reward_parameter_adjustment", "reward_terminal", "cost_collision",
            "cost_near_collision", "cost_parameter_violation", "cost_planner_failure",
            "cost_emergency_or_fallback",
        ):
            row[name] = float(fields.get(name, reward if name == "reward_total" else 0.0))
        self.steps.append(row)
        if terminated or truncated:
            self.finish(env, str(info.get("termination_reason", "infrastructure_fault")))

    def finish(self, env: Any, reason: str) -> None:
        if self._episode is None:
            return
        episode, adapter = self._episode, env.core.adapter
        terminated = reason not in ("timeout", "operator_stop", "infrastructure_fault")
        elapsed = max(0.0, rospy.Time.now().to_sec() - episode["started"])
        straight = math.hypot(
            episode["goal_xy"][0] - episode["start_xy"][0],
            episode["goal_xy"][1] - episode["start_xy"][1],
        )
        path_length = float(adapter.path_length)
        scenario = episode["scenario"]
        row = {
            "run_id": self.run_id, "episode_id": episode["episode_id"],
            "algorithm": self.algorithm, "scene_id": scenario["scene_id"],
            "scene_split": scenario["split"], "training_seed": self.training_seed,
            "seed": int(scenario.get("evaluation_seed", self.training_seed)),
            "config_version": self.config_version, "git_commit": _git("rev-parse", "HEAD"),
            "git_dirty": bool(_git("status", "--porcelain")),
            "submodule_commits_json": "{}", "policy_checkpoint_sha256": None,
            "scenario_manifest_sha256": self.scene_manifest_sha256,
            "localization_mode": "gazebo", "success": reason == "goal",
            "collision": reason == "collision", "terminated": terminated,
            "truncated": not terminated, "termination_reason": reason,
            "path_length": path_length, "navigation_time": elapsed,
            "path_efficiency": straight / path_length if path_length > 0.0 else 0.0,
            "smoothness": episode["smoothness"], "linear_acc_rms": None,
            "angular_acc_rms": None, "min_obstacle_distance": float(adapter.minimum_clearance),
            "near_collision_time_ratio": episode["near_duration"] / max(elapsed, 1e-9),
            "parameter_adjustment_count": episode["step_count"],
            "parameter_total_variation": episode["variation"],
            "projection_intervention_count": episode["projection_count"],
            "safety_filter_intervention_count": episode["safety_count"],
            "safety_fallback_count": episode["fallback_count"],
            "fallback_duration": episode["fallback_duration"],
            "fallback_recovery_count": 0,
            "planner_failure_count": int(reason == "planner_failure"),
            "candidate_parameter_violation_count": episode["candidate_violation_count"],
            "semantic_direction_consistency": None,
            "inference_latency_mean": _mean(episode["inference_latencies"]),
            "parameter_write_latency_mean": _mean(episode["write_latencies"]),
            "parameter_activation_latency_mean": _mean(episode["active_latencies"]),
            "operator_intervention_count": 0, "bag_uri": None,
            "notes": "T11 {} phase; undiscounted_return={:.9g}".format(
                self.phase, episode["reward"]),
        }
        self.episodes.append(row)
        if reason != "goal":
            self.failures.append({
                "episode_id": row["episode_id"], "scene_id": row["scene_id"],
                "seed": row["seed"], "reason": reason, "phase": self.phase,
            })
        self._episode = None


class RecordedSacEnv(gym.Wrapper):
    def __init__(self, env: Any, recorder: T11Recorder) -> None:
        super().__init__(env)
        self.recorder = recorder
        self.total_training_steps = 0
        self.curriculum = None
        self.pending_inference_ms = 0.0

    def reset(self, **kwargs):
        observation, info = self.env.reset(**kwargs)
        self.recorder.begin(self.env)
        return observation, info

    def step(self, action):
        previous = self.env.core.adapter.current_theta()
        observation, reward, terminated, truncated, info = self.env.step(action)
        self.recorder.record_step(
            self.env, action, previous, reward, terminated, truncated, info,
            self.pending_inference_ms,
        )
        self.pending_inference_ms = 0.0
        if self.recorder.phase == "train":
            self.total_training_steps += 1
            if (terminated or truncated) and self.curriculum is not None:
                self.env.core.adapter.set_scenarios(self.curriculum(self.total_training_steps))
        return observation, reward, terminated, truncated, info


class CheckpointCallback(BaseCallback):
    def __init__(self, root: Path, interval: int, config: SacTrainingConfig,
                 algorithm: str, metadata: Mapping[str, Any]):
        super().__init__(verbose=0)
        self.root, self.interval, self.config = root, int(interval), config
        self.algorithm, self.metadata = algorithm, dict(metadata)
        self.saved = []

    def _on_step(self):
        if self.num_timesteps % self.interval == 0:
            directory = self.root / "step_{:06d}".format(self.num_timesteps)
            manager = SacCheckpointManager(directory, algorithm=self.algorithm)
            manager.save(self.model, self.model.get_vec_normalize_env(), self.config,
                         dict(self.metadata, phase="training", timestep=self.num_timesteps))
            self.saved.append(directory)
        return True


class T11FormalRunner:
    def __init__(self):
        self.config_path = Path(rospy.get_param(
            "~config", str(WORKSPACE / "config/thesis_experiments/t11_formal.yaml")))
        self.data = load_v1_yaml(self.config_path, "T11/T12 runner config")
        self.contract = validate_t11_contract(self.config_path, WORKSPACE)
        self.algorithm = str(rospy.get_param("~algorithm", "RL-TEB-Semantic-Eta"))
        self.training_seed = int(rospy.get_param("~training_seed", 101))
        self.phase = str(rospy.get_param("~phase", "train"))
        self.safety_mode = str(rospy.get_param("~safety_mode", "FullSafety"))
        self.acceptance_timesteps = int(rospy.get_param("~acceptance_timesteps", 0))
        self.acceptance_eval_seed_limit = int(rospy.get_param("~acceptance_eval_seed_limit", 0))
        self.task = str(rospy.get_param("~task", "T11"))
        if (self.algorithm, self.safety_mode) not in ALGORITHM_LABELS:
            raise RuntimeError("invalid T11 algorithm/safety combination")
        if self.training_seed not in self.data["training"]["seeds"]:
            raise RuntimeError("training seed is outside the frozen T11 contract")
        default_run = "t11_{}_{}_seed{}".format(
            self.algorithm.replace("RL-TEB-", "").lower().replace("-", "_"),
            self.safety_mode.lower(), self.training_seed)
        self.run_id = str(rospy.get_param("~run_id", default_run))
        self.output_dir = Path(rospy.get_param(
            "~output_dir", str(WORKSPACE / "artifacts/t11/runs" / self.run_id)))
        scene_override = str(rospy.get_param("~scene_manifest", "")).strip()
        self.scene_path = (Path(scene_override) if scene_override else
                           WORKSPACE / self.data["scene_manifest"])
        if not self.scene_path.is_absolute():
            self.scene_path = WORKSPACE / self.scene_path
        self.scene_data = load_v1_yaml(self.scene_path, "T11/T12 scene manifest")
        self.safety_path = WORKSPACE / self.data["safety_contract"]
        self.safety_data = load_v1_yaml(self.safety_path, "T11/T12 safety contract")
        self.mapping_path = WORKSPACE / self.data["semantic_mapping"]
        load_v1_yaml(self.mapping_path, "T11/T12 semantic mapping")
        seed_override = int(rospy.get_param("~evaluation_seed_override", 0))
        self.evaluation_seeds = (
            [seed_override] if seed_override else
            list(self.data["evaluation"]["evaluation_seeds"])
        )
        self.t12_safety_path = Path(str(rospy.get_param(
            "~t12_safety_config",
            str(WORKSPACE / "config/thesis_experiments/t12_shadow.yaml"))))
        self.semantic_mode = str(rospy.get_param("~semantic_mode", "cumulative"))
        self.residual_config_path = Path(str(rospy.get_param(
            "~residual_config",
            str(WORKSPACE / "config/thesis_experiments/t12_residual_semantic_eta.yaml"))))
        self.t12_safety_data = load_v1_yaml(
            self.t12_safety_path, "T12 safety override")
        self.zero_action_policy = bool(rospy.get_param("~zero_action_policy", False))
        self.evaluation_policy = str(rospy.get_param("~evaluation_policy", "checkpoint"))
        self.initialize_residual_anchor = bool(rospy.get_param(
            "~initialize_residual_anchor", False))
        if self.evaluation_policy not in ("checkpoint", "zero_residual", "teb_tuned"):
            raise RuntimeError("unknown evaluation policy")
        if self.phase == "train" and self.evaluation_policy != "checkpoint":
            raise RuntimeError("training requires the checkpoint policy path")
        if self.evaluation_policy in ("zero_residual", "teb_tuned"):
            self.zero_action_policy = True
        if self.evaluation_policy == "teb_tuned" and self.safety_mode != "ProjectionOnly":
            raise RuntimeError("native TEB-Tuned diagnostic requires ProjectionOnly")
        residual_state = load_v1_yaml(
            self.residual_config_path, "T12 residual contract")
        if self.semantic_mode == "residual_pilot" and residual_state.get("training_enabled") is not False:
            raise RuntimeError("T12 residual pilot contract must forbid training")
        if self.phase == "train" and self.semantic_mode == "residual_pilot":
            raise RuntimeError("T12 residual pilot contract forbids training")
        if self.semantic_mode == "residual_training" and residual_state.get("training_enabled") is not True:
            raise RuntimeError("Residual training requires an explicit training-enabled contract")
        self.client = self.adapter = None

    def _scenes(self, split: str, evaluation_seeds: Sequence[int] = ()):
        global_random = self.scene_data["randomization"]
        collision = self.scene_data["collision_definition"]
        base = [dict(item) for item in self.scene_data["scenes"] if item["split"] == split]
        seeds = tuple(evaluation_seeds) or (None,)
        result = []
        for seed in seeds:
            for item in base:
                scene = dict(item)
                scene.update({
                    "collision_threshold_m": collision["collision_threshold_m"],
                    "near_collision_threshold_m": collision["near_collision_threshold_m"],
                    "goal_jitter_m": global_random["goal_jitter_m"],
                    "obstacle_jitter_m": global_random["obstacle_jitter_m"],
                    "corridor_half_width_jitter_m": global_random["corridor_half_width_jitter_m"],
                })
                if seed is not None:
                    scene["evaluation_seed"] = int(seed)
                result.append(scene)
        return result

    def _projector(self):
        limits = {}
        no_safety = self.safety_mode == "NoSafety"
        for name in EXPECTED_THETA_ORDER:
            low, high = self.safety_data["theta_bounds"][name]
            rate = high - low if no_safety else self.safety_data["max_delta_per_step"][name]
            limits[name] = ParameterLimit(low, high, rate, True)
        return ParameterProjector(limits, min_turning_radius=None if no_safety else 1.2)

    def _filter_core(self):
        values = dict(self.safety_data["safety_margin"])
        if self.safety_mode in ("T12Safety", "T12LegacySafety"):
            improved = self.t12_safety_data["safety"]
            values.update({
                "emergency_distance_cap_m": improved["emergency_distance_cap_m"],
                "emergency_confirmation_s": improved["emergency_confirmation_s"],
            })
        return SafetyMarginFilter(SafetyMarginConfig(
            a_brake_lower=values["a_brake_lower_mps2"],
            tau_total_upper=values["total_latency_upper_s"],
            d_margin=values["distance_margin_m"], warning_margin=values["warning_margin_m"],
            emergency_margin=values["emergency_margin_m"],
            hysteresis_margin=values["recovery_margin_m"],
            recovery_healthy_s=values["recovery_healthy_duration_s"],
            emergency_distance_cap=values.get("emergency_distance_cap_m"),
            emergency_confirmation_s=values.get("emergency_confirmation_s", 0.0),
        ))

    def _safety(self, baseline):
        if self.safety_mode in ("ProjectionOnly", "NoSafety"):
            return None
        policy = ConservativeFallbackPolicy(self.safety_data["conservative_theta"])
        policy.confirm_applied_safe(baseline)
        if self.safety_mode == "NoFallback":
            return NoFallbackSafetyAdapter(self._filter_core(), policy)
        corridor_theta = None
        if self.safety_mode == "T12Safety":
            corridor_theta = self.t12_safety_data["safety"].get(
                "corridor_warning_theta")
        return TrainingSafetyAdapter(
            self._filter_core(), policy,
            directional_emergency=self.safety_mode == "T12Safety",
            corridor_warning_theta=corridor_theta,
            corridor_max_delta=(self.safety_data["max_delta_per_step"]
                                if corridor_theta is not None else None),
        )

    def _connect(self):
        namespace = "/move_base/TebLocalPlannerROS"
        context = SimulationWriteContext(
            explicit_simulation=True, use_sim_time=rospy.get_param("/use_sim_time", False),
            simulation_marker=rospy.get_param("/m2_gazebo/simulation_only", False),
            teb_namespace=namespace,
        )
        self.client = TebParameterClient(
            RosDynamicReconfigureBackend(namespace, 5.0), context, timeout_s=5.0)
        self.client.initialize()
        initial_theta = None
        if self.initialize_residual_anchor:
            if self.semantic_mode not in ("residual_pilot", "residual_training"):
                raise RuntimeError("residual anchor initialization requires residual mode")
            residual = ResidualSemanticMapping.from_files(
                self.mapping_path, self.safety_path, self.residual_config_path)
            initial_theta = dict(self.client.apply(residual.anchor_theta)["readback"])
        initial_scenarios = self._scenes("train")
        if not initial_scenarios and self.phase != "train":
            # Evaluation-only T12 manifests intentionally contain no train split.
            initial_scenarios = self._scenes(self.data["evaluation"]["splits"][0])
        self.adapter = GazeboTrainingAdapter(
            self.client, initial_scenarios, self.safety_data["theta_bounds"],
            initial_theta=initial_theta)

    def _gym_env(self, recorder):
        reward = RewardWeights(**self.data["reward"]["weights"])
        env_cfg = self.data["environment"]
        core = TrainingEnvironment(
            self.adapter,
            StateBuilder(FEATURE_ORDER, ("scan", "odom", "local_plan"),
                         sector_count=36, max_sync_skew_s=0.6),
            self._projector(), reward,
            EnvironmentConfig(
                history_length=4, activation_timeout_s=env_cfg["activation_timeout_s"],
                reward_window_s=env_cfg["reward_window_s"],
                max_steps=env_cfg["max_steps_per_episode"],
                max_ros_duration_s=env_cfg["max_ros_duration_s"],
                warning_distance=self.data["reward"]["warning_distance_m"],
            ), self._safety(self.client.snapshot),
        )
        if self.algorithm == "RL-TEB-Semantic-Eta":
            if self.semantic_mode in ("residual_pilot", "residual_training"):
                mapping = ResidualSemanticMapping.from_files(
                    self.mapping_path, self.safety_path, self.residual_config_path)
                base = ResidualSemanticEtaGymEnv(core, mapping)
            else:
                mapping = FrozenSemanticMapping.from_files(
                    self.mapping_path, self.safety_path, executable_rate_scaling=True)
                base = SemanticEtaGymEnv(core, mapping)
        else:
            mapping = DirectThetaMapping.from_file(
                self.safety_path, executable_rate_scaling=True)
            base = DirectThetaGymEnv(core, mapping)
        return RecordedSacEnv(base, recorder)

    def _training_config(self):
        item = self.data["training"]
        return SacTrainingConfig(
            seed=self.training_seed, learning_rate=item["learning_rate"],
            buffer_size=item["buffer_size"], learning_starts=item["learning_starts"],
            batch_size=item["batch_size"], tau=item["tau"], gamma=item["gamma"],
            train_freq=item["train_freq"], gradient_steps=item["gradient_steps"],
            net_arch=tuple(item["net_arch"]), device="cpu")

    def _curriculum(self, step: int):
        train = self._scenes("train")
        allowed = set()
        for phase in self.data["training"]["curriculum"]:
            allowed.update(phase["scene_tags"])
            if step < phase["until_timestep"]:
                break
        return [scene for scene in train if scene.get("curriculum") in allowed]

    def _evaluate(self, model, vec, recorded, scenarios, phase):
        recorded.recorder.phase = phase
        self.adapter.set_scenarios(scenarios)
        observation = vec.reset()
        returns, current = [], 0.0
        target = len(scenarios)
        while len(returns) < target:
            started = time.perf_counter()
            if self.evaluation_policy in ("zero_residual", "teb_tuned") or self.zero_action_policy:
                action = np.zeros((1,) + recorded.action_space.shape, dtype=np.float32)
            else:
                action, _ = model.predict(observation, deterministic=True)
            latency = (time.perf_counter() - started) * 1000.0
            recorded.pending_inference_ms = latency
            observation, rewards, dones, _ = vec.step(action)
            current += float(rewards[0])
            if bool(dones[0]):
                returns.append(current)
                current = 0.0
        return returns

    def run(self):
        self._connect()
        label = ("TEB-Tuned" if self.evaluation_policy == "teb_tuned" else
                 ALGORITHM_LABELS[(self.algorithm, self.safety_mode)])
        recorder = T11Recorder(
            self.run_id, label, self.training_seed,
            sha256_file(self.scene_path), self.data["study_version"] + "+" + self.safety_mode)
        recorded = self._gym_env(recorder)
        raw_vec = DummyVecEnv([lambda: recorded])
        try:
            if self.phase == "train":
                return self._train(raw_vec, recorded, recorder)
            return self._ablation(raw_vec, recorded, recorder)
        finally:
            if self.adapter is not None:
                self.adapter.close()

    def _train(self, raw_vec, recorded, recorder):
        train_cfg = self._training_config()
        vec = VecNormalize(raw_vec, norm_obs=True, norm_reward=False, clip_obs=10.0)
        recorded.curriculum = self._curriculum
        self.adapter.set_scenarios(self._curriculum(0))
        model = build_sac(vec, train_cfg)
        checkpoint_root = self.output_dir / "checkpoints"
        total_timesteps = (self.acceptance_timesteps or
                           self.data["training"]["total_timesteps_per_seed"])
        checkpoint_interval = (max(1, total_timesteps // 2) if self.acceptance_timesteps else
                               self.data["training"]["validation_frequency_steps"])
        callback = CheckpointCallback(
            checkpoint_root, checkpoint_interval,
            train_cfg, self.algorithm, {
                "task": "T11", "training_seed": self.training_seed,
                "t11_config_sha256": self.contract["config_sha256"],
                "scene_manifest_sha256": self.contract["scene_manifest_sha256"],
                "safety_mode": self.safety_mode,
            })
        model.learn(total_timesteps=total_timesteps,
                    callback=callback)
        if recorder._episode is not None and recorder._episode["step_count"]:
            recorder.finish(recorded.env, "operator_stop")
        validation = []
        for directory in callback.saved:
            loaded, eval_vec, state = SacCheckpointManager(
                directory, algorithm=self.algorithm).load(raw_vec)
            eval_vec.training = False
            scenarios = self._scenes(
                "validation", (self.data["training"]["validation_seed"],))
            returns = self._evaluate(loaded, eval_vec, recorded, scenarios, "validation")
            validation.append({
                "checkpoint": str(directory), "timesteps": state["num_timesteps"],
                "episode_returns": returns, "mean_return": _mean(returns),
            })
        best = max(validation, key=lambda item: (item["mean_return"], -item["timesteps"]))
        manager = SacCheckpointManager(Path(best["checkpoint"]), algorithm=self.algorithm)
        selected, selected_vec, selected_state = manager.load(raw_vec)
        selected_vec.training = False
        test_scenarios = []
        evaluation_seeds = self.data["evaluation"]["evaluation_seeds"]
        if self.acceptance_eval_seed_limit:
            evaluation_seeds = evaluation_seeds[:self.acceptance_eval_seed_limit]
        for split in self.data["evaluation"]["splits"]:
            test_scenarios.extend(self._scenes(
                split, evaluation_seeds))
        test_returns = self._evaluate(
            selected, selected_vec, recorded, test_scenarios, "test")
        selection = {
            "schema_version": "1.0", "algorithm": self.algorithm,
            "training_seed": self.training_seed, "validation": validation,
            "selected_checkpoint": best["checkpoint"],
            "selected_timesteps": best["timesteps"],
            "selected_mean_validation_return": best["mean_return"],
            "test_episode_count": len(test_returns),
            "test_results_used_for_selection": False,
            "evaluation_policy": ("deterministic_zero_action" if self.zero_action_policy
                                  else "checkpoint_deterministic_policy"),
        }
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / "model_selection.yaml").write_text(
            yaml.safe_dump(selection, sort_keys=False), encoding="utf-8")
        return self._write_bundle(recorder, Path(best["checkpoint"]), selected_state, selection)

    def _ablation(self, raw_vec, recorded, recorder):
        checkpoint_text = str(rospy.get_param("~checkpoint", "")).strip()
        checkpoint = Path(checkpoint_text) if checkpoint_text else None
        if self.evaluation_policy == "checkpoint":
            if checkpoint is None:
                raise RuntimeError("checkpoint evaluation requires a checkpoint")
            manager = SacCheckpointManager(checkpoint, algorithm=self.algorithm)
            model, vec, state = manager.load(raw_vec)
            vec.training = False
        else:
            model, vec, state = None, raw_vec, {"files": {}, "num_timesteps": 0}
        scenarios = []
        for split in self.data["evaluation"]["splits"]:
            scenarios.extend(self._scenes(split, self.evaluation_seeds))
        returns = self._evaluate(model, vec, recorded, scenarios, "test")
        selection = {
            "schema_version": "1.0", "algorithm": self.algorithm,
            "training_seed": self.training_seed, "safety_mode": self.safety_mode,
            "evaluation_policy": self.evaluation_policy,
            "source_checkpoint": None if checkpoint is None else str(checkpoint),
            "test_episode_count": len(returns),
            "test_results_used_for_selection": False,
        }
        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / "model_selection.yaml").write_text(
            yaml.safe_dump(selection, sort_keys=False), encoding="utf-8")
        return self._write_bundle(recorder, checkpoint, state, selection)

    def _write_bundle(self, recorder, checkpoint, checkpoint_state, selection):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        checkpoint_hash = checkpoint_state.get("files", {}).get("model.zip")
        for row in recorder.episodes:
            if not row["notes"].startswith("T11 train phase"):
                row["policy_checkpoint_sha256"] = checkpoint_hash or ""
        episodes = self.output_dir / "episodes.csv"
        steps = self.output_dir / "steps.csv"
        failures = self.output_dir / "failure_index.yaml"
        selection_path = self.output_dir / "model_selection.yaml"
        write_episode_csv(episodes, recorder.episodes,
                          WORKSPACE / "docs/thesis_experiment/schemas/episode_metrics_schema.csv")
        write_step_csv(steps, recorder.steps,
                       WORKSPACE / "docs/thesis_experiment/schemas/step_metrics_schema.csv")
        failures.write_text(yaml.safe_dump(recorder.failures, sort_keys=False), encoding="utf-8")
        checksum = self.output_dir / "checksums.sha256"
        write_checksums(checksum, [episodes, steps, failures, selection_path], self.output_dir)
        manifest = {
            "schema_version": "1.0", "run_id": self.run_id,
            "created_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "algorithm": recorder.algorithm, "mode": "gazebo",
            "scene_id": "t11_formal_matrix", "scene_ids": sorted(set(
                row["scene_id"] for row in recorder.episodes)),
            "scene_split": "train" if self.phase == "train" else "test_id",
            "scene_splits": sorted(set(row["scene_split"] for row in recorder.episodes)),
            "training_seed": self.training_seed, "evaluation_seed": None,
            "source": {"main_commit": _git("rev-parse", "HEAD"), "main_dirty": True,
                       "submodule_commits": {}, "ros_version": "noetic",
                       "gazebo_version": "11", "python_version": sys.version.split()[0],
                       "host_id": os.uname().nodename},
            "configuration": {
                "experiment_contract_path": str(WORKSPACE / "docs/thesis_experiment/experiment_contract.yaml"),
                "experiment_contract_sha256": sha256_file(WORKSPACE / "docs/thesis_experiment/experiment_contract.yaml"),
                "scene_manifest_path": str(self.scene_path),
                "scene_manifest_sha256": sha256_file(self.scene_path),
                "theta_bounds_path": str(self.safety_path),
                "theta_bounds_sha256": sha256_file(self.safety_path),
                "A_TEB_path": str(self.mapping_path) if self.algorithm.endswith("Semantic-Eta") else None,
                "A_TEB_sha256": sha256_file(self.mapping_path) if self.algorithm.endswith("Semantic-Eta") else None,
                "reward_config_path": str(self.config_path),
                "reward_config_sha256": self.contract["config_sha256"],
                "safety_config_path": str(self.safety_path),
                "safety_config_sha256": sha256_file(self.safety_path),
                "runtime_safety_override_path": (
                    str(self.t12_safety_path) if self.safety_mode in
                    ("T12Safety", "T12LegacySafety") else None),
                "runtime_safety_override_sha256": (
                    sha256_file(self.t12_safety_path)
                    if self.safety_mode in ("T12Safety", "T12LegacySafety") else None),
                "residual_config_path": (
                    str(self.residual_config_path)
                    if self.semantic_mode in ("residual_pilot", "residual_training") else None),
                "residual_config_sha256": (
                    sha256_file(self.residual_config_path)
                    if self.semantic_mode in ("residual_pilot", "residual_training") else None),
                "evaluation_policy": self.evaluation_policy,
                "initialize_residual_anchor": self.initialize_residual_anchor,
                "episode_boundary_audit": (
                    self.adapter.boundary_audit()
                    if hasattr(self.adapter, "boundary_audit") else None),
                "policy_checkpoint_path": (
                    str(checkpoint / "model.zip") if checkpoint is not None else None),
                "policy_checkpoint_sha256": checkpoint_hash,
            },
            "topics": {"scan": "/scan", "odom": "/odom", "cmd_vel": "/cmd_vel",
                       "global_plan": "/move_base/TebLocalPlannerROS/global_plan",
                       "local_plan": "/move_base/TebLocalPlannerROS/local_plan",
                       "status": "/move_base/status"},
            "safety": {"allow_motion": False, "allow_parameter_write": False,
                       "speed_limit_mps": 1.2, "human_operator": None,
                       "emergency_stop_checked": False, "fence_checked": False,
                       "conservative_fallback_checked": self.safety_mode in
                       ("FullSafety", "T12Safety", "T12LegacySafety")},
            "artifacts": {"episode_csv": "episodes.csv", "step_log": "steps.csv",
                          "rosbag": None, "stdout_log": None,
                          "failure_index": "failure_index.yaml",
                          "checksums_file": "checksums.sha256"},
            "completion": {"validated": True, "validation_report": None,
                           "excluded_from_formal_results": bool(self.acceptance_timesteps),
                           "exclusion_reason": ("t11_runner_acceptance_override" if
                                                self.acceptance_timesteps else None)},
        }
        write_run_manifest(self.output_dir / "run_manifest.yaml", manifest)
        validation = RunValidator(
            WORKSPACE / "docs/thesis_experiment/schemas/episode_metrics_schema.csv",
            WORKSPACE / "docs/thesis_experiment/schemas/step_metrics_schema.csv",
        ).validate(self.output_dir / "run_manifest.yaml")
        report = {
            "schema_version": "1.0", "task": self.task, "status": "passed",
            "run_id": self.run_id, "algorithm": recorder.algorithm,
            "training_seed": self.training_seed, "safety_mode": self.safety_mode,
            "episode_count": len(recorder.episodes), "step_count": len(recorder.steps),
            "failure_count": len(recorder.failures), "run_validation": validation,
            "selected_checkpoint": (None if checkpoint is None else str(checkpoint)),
            "evaluation_policy": self.evaluation_policy,
            "selected_timesteps": int(checkpoint_state.get("num_timesteps", 0)),
            "passed": validation["valid"],
            "acceptance_override": bool(self.acceptance_timesteps),
            "episode_boundary_audit": (
                self.adapter.boundary_audit()
                if hasattr(self.adapter, "boundary_audit") else None),
        }
        (self.output_dir / "t11_run_report.yaml").write_text(
            yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
        return report
