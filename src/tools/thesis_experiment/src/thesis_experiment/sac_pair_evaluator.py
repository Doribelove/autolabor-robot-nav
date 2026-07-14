"""Evaluate the T09/T10 smoke pair without making formal performance claims."""

from pathlib import Path
import hashlib
from typing import Any, Dict

import yaml

from teb_rl_tuner.sac_fairness import load_and_validate_sac_pair


class SacPairEvaluationError(ValueError):
    pass


COMMON_REPORT_FIELDS = (
    "simulation_only", "formal_experiment", "real_vehicle_use_forbidden",
    "training_timesteps", "pre_resume_timesteps", "resume_delta_timesteps",
    "replay_buffer_size", "evaluation_episode_count", "observation_dimension",
    "observation_action_context",
)


def _load(path: Any) -> Dict[str, Any]:
    source = Path(path)
    value = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise SacPairEvaluationError("report must be a mapping: {}".format(source))
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_checkpoint(directory: Path, algorithm: str) -> Dict[str, Any]:
    state = _load(directory / "checkpoint_manifest.yaml")
    if state.get("status") != "complete" or state.get("algorithm") != algorithm:
        raise SacPairEvaluationError("checkpoint identity/status mismatch: {}".format(directory))
    expected = state.get("files", {})
    for name in ("model.zip", "replay_buffer.pkl", "vecnormalize.pkl"):
        path = directory / name
        if not path.is_file() or _sha256(path) != expected.get(name):
            raise SacPairEvaluationError("checkpoint hash mismatch: {}".format(path))
    return state


def evaluate_sac_pair(
    semantic_config: Any, direct_config: Any, semantic_report: Any, direct_report: Any,
    validate_checkpoints: bool = True,
) -> Dict[str, Any]:
    fairness = load_and_validate_sac_pair(semantic_config, direct_config)
    semantic, direct = _load(semantic_report), _load(direct_report)
    if semantic.get("algorithm") != "RL-TEB-Semantic-Eta":
        raise SacPairEvaluationError("semantic report algorithm mismatch")
    if direct.get("algorithm") != "RL-TEB-Direct-Theta":
        raise SacPairEvaluationError("direct report algorithm mismatch")
    if semantic.get("passed") is not True or direct.get("passed") is not True:
        raise SacPairEvaluationError("both smoke reports must pass")
    for field in COMMON_REPORT_FIELDS:
        if semantic.get(field) != direct.get(field):
            raise SacPairEvaluationError("paired report field differs: {}".format(field))
    if semantic.get("action_dimension") != 5 or direct.get("action_dimension") != 9:
        raise SacPairEvaluationError("paired action dimensions are invalid")
    if semantic.get("config_sha256") != fairness["semantic_config_sha256"]:
        raise SacPairEvaluationError("semantic report config hash is stale")
    if direct.get("config_sha256") != fairness["direct_config_sha256"]:
        raise SacPairEvaluationError("direct report config hash is stale")
    checkpoint_states = {}
    if validate_checkpoints:
        for name, report, algorithm in (
            ("semantic_eta", semantic, "RL-TEB-Semantic-Eta"),
            ("direct_theta", direct, "RL-TEB-Direct-Theta"),
        ):
            directory = Path(report["checkpoint_manifest"]).parent
            state = _validate_checkpoint(directory, algorithm)
            if state.get("num_timesteps") != report.get("training_timesteps"):
                raise SacPairEvaluationError("{} checkpoint timestep mismatch".format(name))
            checkpoint_states[name] = {
                "manifest": str(directory / "checkpoint_manifest.yaml"),
                "num_timesteps": state["num_timesteps"], "files": state["files"],
            }
    semantic_latency = semantic["deterministic_inference_latency_ms"]
    direct_latency = direct["deterministic_inference_latency_ms"]
    return {
        "schema_version": "1.0", "task": "T10", "status": "passed", "passed": True,
        "simulation_only": True, "formal_experiment": False,
        "purpose": "pipeline_fairness_acceptance_not_performance_comparison",
        "fairness_contract": fairness,
        "identical_report_fields": list(COMMON_REPORT_FIELDS),
        "semantic_eta": {
            "action_dimension": 5,
            "actor_parameter_count": semantic["actor_parameter_count"],
            "trainable_parameter_count": semantic["trainable_parameter_count"],
            "inference_latency_ms": semantic_latency,
            "training_terminal_reasons": semantic["training_terminal_reasons"],
            "evaluation_terminal_reasons": semantic["evaluation_terminal_reasons"],
        },
        "direct_theta": {
            "action_dimension": 9,
            "actor_parameter_count": direct["actor_parameter_count"],
            "trainable_parameter_count": direct["trainable_parameter_count"],
            "inference_latency_ms": direct_latency,
            "training_terminal_reasons": direct["training_terminal_reasons"],
            "evaluation_terminal_reasons": direct["evaluation_terminal_reasons"],
        },
        "derived": {
            "actor_parameter_count_delta_direct_minus_semantic": (
                direct["actor_parameter_count"] - semantic["actor_parameter_count"]
            ),
            "trainable_parameter_count_delta_direct_minus_semantic": (
                direct["trainable_parameter_count"] - semantic["trainable_parameter_count"]
            ),
            "mean_inference_latency_ratio_direct_over_semantic": (
                direct_latency["mean"] / semantic_latency["mean"]
            ),
        },
        "checkpoint_validation": checkpoint_states,
        "limitations": [
            "twenty_step_smoke_is_not_converged_training",
            "latency_sample_count_is_too_small_for_performance_claims",
            "formal_paired_training_and_evaluation_remain_T11",
        ],
    }
