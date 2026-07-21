"""Shared Gazebo SAC smoke runner for the fair T09/T10 action-space pair."""

import time
from pathlib import Path

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
from teb_rl_tuner.sac_environment import DirectThetaGymEnv, SemanticEtaGymEnv
from teb_rl_tuner.sac_training import (
    SacCheckpointManager, SacTrainingConfig, build_sac, sha256_file,
)
from teb_rl_tuner.safety_gate import SimulationWriteContext
from teb_rl_tuner.safety_margin_filter import SafetyMarginConfig, SafetyMarginFilter
from teb_rl_tuner.semantic_action import FrozenSemanticMapping
from teb_rl_tuner.state_builder import StateBuilder
from teb_rl_tuner.teb_parameter_client import RosDynamicReconfigureBackend, TebParameterClient
from teb_rl_tuner.training_environment import EnvironmentConfig, TrainingEnvironment
from thesis_experiment.gazebo_training_adapter import (
    FEATURE_ORDER, GazeboTrainingAdapter, TrainingSafetyAdapter,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")


class AuditCallback(BaseCallback):
    def __init__(self, action_audit_key):
        super().__init__(verbose=0)
        self.action_audit_key = action_audit_key
        self.terminal_reasons = []
        self.policy_steps = 0
        self.stored_transitions = 0

    def _on_step(self):
        for info, done in zip(self.locals.get("infos", []), self.locals.get("dones", [])):
            if info.get(self.action_audit_key):
                self.policy_steps += 1
            if info.get("transition_stored"):
                self.stored_transitions += 1
            if done:
                self.terminal_reasons.append(info.get("termination_reason", ""))
        return True


class GazeboSacSmoke:
    """Use one implementation for both policies; only the action adapter changes."""

    MODES = {
        "T09": {
            "algorithm": "RL-TEB-Semantic-Eta",
            "action_semantics": "delta_eta",
            "action_dimension": 5,
            "audit_key": "mapping_sha256",
            "config": "config/thesis_experiments/t09_sac.yaml",
            "output": "artifacts/t09/gazebo_sac_smoke",
            "report": "t09_gazebo_sac_smoke.yaml",
        },
        "T10": {
            "algorithm": "RL-TEB-Direct-Theta",
            "action_semantics": "delta_normalized_theta",
            "action_dimension": 9,
            "audit_key": "direct_theta_contract_sha256",
            "config": "config/thesis_experiments/t10_direct_theta_sac.yaml",
            "output": "artifacts/t10/gazebo_sac_smoke",
            "report": "t10_gazebo_sac_smoke.yaml",
        },
    }

    def __init__(self, task):
        if task not in self.MODES:
            raise RuntimeError("unsupported SAC smoke task")
        self.task = task
        self.mode = self.MODES[task]
        self.config_path = Path(rospy.get_param(
            "~config", str(WORKSPACE / self.mode["config"])
        ))
        self.output_dir = Path(rospy.get_param(
            "~output_dir", str(WORKSPACE / self.mode["output"])
        ))
        self.data = yaml.safe_load(self.config_path.read_text(encoding="utf-8"))
        if (self.data.get("real_vehicle_use_forbidden") is not True or
                self.data.get("simulation_only") is not True):
            raise RuntimeError("SAC smoke must remain simulation-only")
        if self.data.get("algorithm") != self.mode["algorithm"]:
            raise RuntimeError("SAC smoke algorithm/config mismatch")
        if self.data.get("action", {}).get("action_semantics") != self.mode["action_semantics"]:
            raise RuntimeError("SAC smoke action/config mismatch")
        self.safety_path = WORKSPACE / self.data["safety"]["source"]
        self.safety_data = yaml.safe_load(self.safety_path.read_text(encoding="utf-8"))
        self.client = None
        self.adapter = None

    def _projector(self):
        return ParameterProjector({
            name: ParameterLimit(
                self.safety_data["theta_bounds"][name][0],
                self.safety_data["theta_bounds"][name][1],
                self.safety_data["max_delta_per_step"][name], True,
            ) for name in EXPECTED_THETA_ORDER
        }, min_turning_radius=1.2)

    def _safety(self, baseline):
        values = self.safety_data["safety_margin"]
        result = TrainingSafetyAdapter(
            SafetyMarginFilter(SafetyMarginConfig(
                a_brake_lower=values["a_brake_lower_mps2"],
                tau_total_upper=values["total_latency_upper_s"],
                d_margin=values["distance_margin_m"],
                warning_margin=values["warning_margin_m"],
                emergency_margin=values["emergency_margin_m"],
                hysteresis_margin=values["recovery_margin_m"],
                recovery_healthy_s=values["recovery_healthy_duration_s"],
            )), ConservativeFallbackPolicy(self.safety_data["conservative_theta"]),
        )
        result.fallback_policy.confirm_applied_safe(baseline)
        return result

    def _environment(self):
        namespace = "/move_base/TebLocalPlannerROS"
        context = SimulationWriteContext(
            explicit_simulation=True,
            use_sim_time=rospy.get_param("/use_sim_time", False),
            simulation_marker=rospy.get_param("/m2_gazebo/simulation_only", False),
            teb_namespace=namespace,
        )
        self.client = TebParameterClient(
            RosDynamicReconfigureBackend(namespace, 5.0), context, timeout_s=5.0
        )
        self.client.initialize()
        scenarios = [
            {"scene_id": "paired-sac-smoke-straight", "layout": "clear",
             "goal": [1.5, 0.0, 0.0]},
            {"scene_id": "paired-sac-smoke-offset", "layout": "clear",
             "goal": [1.3, 0.35, 0.15]},
        ]
        self.adapter = GazeboTrainingAdapter(
            self.client, scenarios, self.safety_data["theta_bounds"]
        )
        core = TrainingEnvironment(
            self.adapter,
            StateBuilder(FEATURE_ORDER, ("scan", "odom", "local_plan"),
                         sector_count=36, max_sync_skew_s=0.6),
            self._projector(), RewardWeights(),
            EnvironmentConfig(history_length=4, activation_timeout_s=2.0,
                              reward_window_s=0.35, max_steps=12,
                              max_ros_duration_s=15.0, warning_distance=1.0),
            self._safety(self.client.snapshot),
        )
        if self.task == "T09":
            mapping_path = WORKSPACE / self.data["action"]["mapping_path"]
            mapping = FrozenSemanticMapping.from_files(mapping_path, self.safety_path)
            return SemanticEtaGymEnv(core, mapping), mapping.mapping_sha256
        mapping = DirectThetaMapping.from_file(self.safety_path)
        expected_hash = self.data["action"]["bounds_sha256"]
        if mapping.contract_sha256 != expected_hash:
            raise RuntimeError("T10 direct-theta bounds hash mismatch")
        return DirectThetaGymEnv(core, mapping), mapping.contract_sha256

    @staticmethod
    def _percentile(values, percentile):
        return float(np.percentile(np.asarray(values, dtype=np.float64), percentile))

    def run(self):
        gym_env, action_contract_sha256 = self._environment()
        raw_vec = DummyVecEnv([lambda: gym_env])
        vec = VecNormalize(raw_vec, norm_obs=True, norm_reward=False, clip_obs=10.0)
        smoke, training = self.data["smoke_override"], self.data["training"]
        config = SacTrainingConfig(
            seed=int(training["seed"]), learning_rate=float(training["learning_rate"]),
            buffer_size=int(smoke["buffer_size"]),
            learning_starts=int(smoke["learning_starts"]),
            batch_size=int(smoke["batch_size"]), tau=float(training["tau"]),
            gamma=float(training["gamma"]), train_freq=int(training["train_freq"]),
            gradient_steps=int(training["gradient_steps"]),
            net_arch=tuple(smoke["net_arch"]), device="cpu",
        )
        callback = AuditCallback(self.mode["audit_key"])
        model = build_sac(vec, config)
        actor_parameter_count = sum(parameter.numel() for parameter in model.actor.parameters())
        trainable_parameter_count = sum(
            parameter.numel() for parameter in model.policy.parameters() if parameter.requires_grad
        )
        before = [parameter.detach().cpu().clone() for parameter in model.actor.parameters()]
        checkpoint = SacCheckpointManager(
            self.output_dir / "checkpoint", algorithm=self.mode["algorithm"]
        )
        try:
            model.learn(total_timesteps=int(smoke["total_timesteps_before_resume"]),
                        callback=callback)
            actor_change = sum(float((after.detach().cpu() - initial).abs().sum())
                               for after, initial in zip(model.actor.parameters(), before))
            checkpoint.save(model, vec, config, {
                "phase": "pre_resume", "config_sha256": sha256_file(self.config_path),
                "action_contract_sha256": action_contract_sha256,
                "observation_contract": self.data["observation"]["action_context"],
            })
            resumed, resumed_vec, state = checkpoint.load(raw_vec)
            resumed.learn(total_timesteps=int(smoke["total_timesteps_after_resume"]),
                          reset_num_timesteps=False, callback=callback)
            checkpoint.save(resumed, resumed_vec, config, {
                "phase": "post_resume", "resumed_from_timesteps": state["num_timesteps"],
                "config_sha256": sha256_file(self.config_path),
                "action_contract_sha256": action_contract_sha256,
                "observation_contract": self.data["observation"]["action_context"],
            })
            checkpoint_state = checkpoint.validate()
            resumed_vec.training = False
            observation = resumed_vec.reset()
            evaluation_reasons, inference_latencies_ms = [], []
            evaluation_steps = 0
            while (len(evaluation_reasons) < int(smoke["evaluation_episodes"]) and
                   evaluation_steps < 40):
                started = time.perf_counter()
                action, _ = resumed.predict(observation, deterministic=True)
                inference_latencies_ms.append((time.perf_counter() - started) * 1000.0)
                observation, _, dones, infos = resumed_vec.step(action)
                evaluation_steps += 1
                if bool(dones[0]):
                    evaluation_reasons.append(infos[0].get("termination_reason", ""))
            report = {
                "schema_version": "1.0", "task": self.task, "status": "passed",
                "algorithm": self.mode["algorithm"], "simulation_only": True,
                "formal_experiment": False, "real_vehicle_use_forbidden": True,
                "training_timesteps": int(resumed.num_timesteps),
                "pre_resume_timesteps": int(state["num_timesteps"]),
                "resume_delta_timesteps": int(smoke["total_timesteps_after_resume"]),
                "replay_buffer_size": int(resumed.replay_buffer.size()),
                "actor_parameter_l1_change": actor_change,
                "policy_steps": callback.policy_steps,
                "stored_transitions": callback.stored_transitions,
                "training_terminal_reasons": callback.terminal_reasons,
                "evaluation_episode_count": len(evaluation_reasons),
                "evaluation_terminal_reasons": evaluation_reasons,
                "observation_dimension": int(gym_env.observation_space.shape[0]),
                "action_dimension": int(gym_env.action_space.shape[0]),
                "observation_action_context": self.data["observation"]["action_context"],
                "actor_parameter_count": int(actor_parameter_count),
                "trainable_parameter_count": int(trainable_parameter_count),
                "deterministic_inference_latency_ms": {
                    "sample_count": len(inference_latencies_ms),
                    "mean": float(np.mean(inference_latencies_ms)),
                    "p95": self._percentile(inference_latencies_ms, 95.0),
                    "max": max(inference_latencies_ms),
                },
                "checkpoint_manifest": str(checkpoint.directory / "checkpoint_manifest.yaml"),
                "checkpoint_files": checkpoint_state["files"],
                "config_sha256": sha256_file(self.config_path),
                "action_contract_sha256": action_contract_sha256,
                "snapshot_restored_on_close": True,
            }
            report["passed"] = bool(
                actor_change > 0.0 and resumed.replay_buffer.size() > 0 and
                callback.policy_steps > 0 and callback.stored_transitions > 0 and
                len(evaluation_reasons) == int(smoke["evaluation_episodes"]) and
                report["observation_dimension"] == 254 and
                report["action_dimension"] == self.mode["action_dimension"]
            )
            report["status"] = "passed" if report["passed"] else "failed"
            self.output_dir.mkdir(parents=True, exist_ok=True)
            destination = self.output_dir / self.mode["report"]
            destination.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
            return report
        finally:
            if self.adapter is not None:
                self.adapter.close()
