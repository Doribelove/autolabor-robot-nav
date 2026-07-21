"""Machine-verifiable T09/T10 fairness contract."""

import hashlib
from pathlib import Path
from typing import Any, Dict, Mapping

import yaml


class SacFairnessError(ValueError):
    pass


PAIRED_FIELDS = (
    "runtime", "observation", "reward", "safety", "training",
    "smoke_override", "checkpoint", "acceptance",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_and_validate_sac_pair(semantic_path: Any, direct_path: Any) -> Dict[str, Any]:
    semantic_source, direct_source = Path(semantic_path), Path(direct_path)
    semantic = yaml.safe_load(semantic_source.read_text(encoding="utf-8"))
    direct = yaml.safe_load(direct_source.read_text(encoding="utf-8"))
    if not isinstance(semantic, dict) or not isinstance(direct, dict):
        raise SacFairnessError("paired SAC configs must be mappings")
    if semantic.get("algorithm") != "RL-TEB-Semantic-Eta":
        raise SacFairnessError("semantic reference algorithm mismatch")
    if direct.get("algorithm") != "RL-TEB-Direct-Theta":
        raise SacFairnessError("direct control algorithm mismatch")
    if semantic.get("action", {}).get("action_semantics") != "delta_eta":
        raise SacFairnessError("semantic action contract mismatch")
    if direct.get("action", {}).get("action_semantics") != "delta_normalized_theta":
        raise SacFairnessError("direct action contract mismatch")
    if len(semantic.get("action", {}).get("eta_order", ())) != 5:
        raise SacFairnessError("semantic action must have five dimensions")
    if len(direct.get("action", {}).get("theta_order", ())) != 9:
        raise SacFairnessError("direct action must have nine dimensions")
    for flag in ("simulation_only", "formal_experiment", "real_vehicle_use_forbidden"):
        if semantic.get(flag) != direct.get(flag):
            raise SacFairnessError("paired simulation boundary differs: {}".format(flag))
    for field in PAIRED_FIELDS:
        if semantic.get(field) != direct.get(field):
            raise SacFairnessError("paired field differs: {}".format(field))
    observation = semantic.get("observation", {})
    if observation.get("total_dimension") != 254:
        raise SacFairnessError("paired observation must remain 254-dimensional")
    if observation.get("action_context") != (
            "previous_applied_delta_normalized_theta_9_plus_l1"):
        raise SacFairnessError("paired observation action context is not action-neutral")
    return {
        "status": "valid", "paired_fields": list(PAIRED_FIELDS),
        "semantic_config_sha256": _sha256(semantic_source),
        "direct_config_sha256": _sha256(direct_source),
        "observation_dimension": 254,
        "semantic_action_dimension": 5, "direct_action_dimension": 9,
    }
