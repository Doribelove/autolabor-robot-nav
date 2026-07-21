"""YAML loading and structural validation for the thesis configuration files."""

from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Sequence

import yaml


EXPECTED_ETA_ORDER = (
    "speed",
    "obstacle_conservatism",
    "clearance",
    "path_tracking",
    "smoothness",
)

EXPECTED_THETA_ORDER = (
    "max_vel_x",
    "max_vel_theta",
    "acc_lim_x",
    "acc_lim_theta",
    "min_obstacle_dist",
    "inflation_dist",
    "weight_obstacle",
    "weight_viapoint",
    "weight_optimaltime",
)


class ConfigValidationError(ValueError):
    """Raised when a thesis configuration violates its structural contract."""


def load_yaml_mapping(path: Any) -> Dict[str, Any]:
    """Load a YAML file and require a mapping at its document root."""

    source = Path(path)
    try:
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigValidationError("Cannot load YAML {}: {}".format(source, exc))
    if not isinstance(data, dict):
        raise ConfigValidationError("YAML root must be a mapping: {}".format(source))
    return data


def _require_mapping(data: Mapping[str, Any], key: str, context: str) -> Mapping[str, Any]:
    value = data.get(key)
    if not isinstance(value, dict):
        raise ConfigValidationError("{}.{} must be a mapping".format(context, key))
    return value


def _require_sequence(data: Mapping[str, Any], key: str, context: str) -> Sequence[Any]:
    value = data.get(key)
    if not isinstance(value, list):
        raise ConfigValidationError("{}.{} must be a list".format(context, key))
    return value


def _require_keys(data: Mapping[str, Any], keys: Iterable[str], context: str) -> None:
    missing = [key for key in keys if key not in data]
    if missing:
        raise ConfigValidationError(
            "{} missing required keys: {}".format(context, ", ".join(sorted(missing)))
        )


def _require_exact_order(actual: Sequence[Any], expected: Sequence[str], context: str) -> None:
    if tuple(actual) != tuple(expected):
        raise ConfigValidationError(
            "{} must equal {}; got {}".format(context, list(expected), list(actual))
        )


def validate_experiment_contract(data: Mapping[str, Any]) -> Mapping[str, Any]:
    """Validate stable structure and invariants of experiment_contract.yaml."""

    _require_keys(
        data,
        (
            "schema_version",
            "paper",
            "environment",
            "confirmed_real_interfaces",
            "timing",
            "state",
            "action",
            "theta_candidates",
            "safety",
            "real_deployment",
        ),
        "contract",
    )
    if str(data["schema_version"]) != "1.0":
        raise ConfigValidationError("contract.schema_version must be 1.0")

    environment = _require_mapping(data, "environment", "contract")
    _require_keys(environment, ("os", "ros", "simulator", "workspace"), "environment")
    if environment["workspace"] != "/home/robot/robot_ws_base_rl":
        raise ConfigValidationError("environment.workspace must select the thesis workspace")

    action = _require_mapping(data, "action", "contract")
    semantic = _require_mapping(action, "semantic_eta", "action")
    _require_exact_order(semantic.get("dimensions", []), EXPECTED_ETA_ORDER, "semantic_eta.dimensions")

    candidates = _require_sequence(data, "theta_candidates", "contract")
    names = []
    for index, candidate in enumerate(candidates):
        if not isinstance(candidate, dict) or not isinstance(candidate.get("name"), str):
            raise ConfigValidationError("theta_candidates[{}] must have a string name".format(index))
        names.append(candidate["name"])
    _require_exact_order(names, EXPECTED_THETA_ORDER, "theta_candidates names")

    safety = _require_mapping(data, "safety", "contract")
    _require_exact_order(
        safety.get("modes", []),
        ("NORMAL", "WARNING", "EMERGENCY", "FAULT"),
        "safety.modes",
    )
    deployment = _require_mapping(data, "real_deployment", "contract")
    if deployment.get("default_mode") != "shadow":
        raise ConfigValidationError("real_deployment.default_mode must remain shadow")
    return data


def validate_runtime_config(data: Mapping[str, Any]) -> Mapping[str, Any]:
    """Validate the non-operational T01 runtime-defaults skeleton."""

    _require_keys(
        data,
        (
            "schema_version",
            "mode",
            "teb_namespace",
            "decision_frequency_hz",
            "theta_order",
            "safety",
        ),
        "runtime_config",
    )
    if str(data["schema_version"]) != "1.0":
        raise ConfigValidationError("runtime_config.schema_version must be 1.0")
    if data["mode"] not in ("disabled", "shadow"):
        raise ConfigValidationError("runtime_config.mode must be disabled or shadow in T01")
    if data["teb_namespace"] != "/move_base/TebLocalPlannerROS":
        raise ConfigValidationError("runtime_config.teb_namespace is unexpected")
    frequency = data["decision_frequency_hz"]
    if isinstance(frequency, bool) or not isinstance(frequency, (int, float)) or frequency <= 0:
        raise ConfigValidationError("decision_frequency_hz must be positive")
    _require_exact_order(data["theta_order"], EXPECTED_THETA_ORDER, "runtime_config.theta_order")
    safety = _require_mapping(data, "safety", "runtime_config")
    if safety.get("allow_motion") is not False or safety.get("allow_parameter_write") is not False:
        raise ConfigValidationError("T01 defaults must disable motion and parameter writes")
    return data


def validate_a_teb(data: Mapping[str, Any], require_frozen: bool = False) -> Mapping[str, Any]:
    """Validate dimensions and freeze invariants of an A_TEB mapping file."""

    _require_keys(
        data,
        ("schema_version", "eta_order", "theta_order", "matrix", "frozen", "sha256"),
        "A_TEB",
    )
    if str(data["schema_version"]) != "1.0":
        raise ConfigValidationError("A_TEB.schema_version must be 1.0")
    _require_exact_order(data["eta_order"], EXPECTED_ETA_ORDER, "A_TEB.eta_order")
    _require_exact_order(data["theta_order"], EXPECTED_THETA_ORDER, "A_TEB.theta_order")
    matrix = data["matrix"]
    if not isinstance(matrix, list) or len(matrix) != len(EXPECTED_THETA_ORDER):
        raise ConfigValidationError("A_TEB.matrix must have nine theta rows")
    for index, row in enumerate(matrix):
        if not isinstance(row, list) or len(row) != len(EXPECTED_ETA_ORDER):
            raise ConfigValidationError("A_TEB.matrix row {} must have five eta columns".format(index))
        for value in row:
            if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float))):
                raise ConfigValidationError("A_TEB.matrix entries must be numeric or null")
    if require_frozen or data["frozen"]:
        if any(value is None for row in matrix for value in row):
            raise ConfigValidationError("A frozen A_TEB matrix cannot contain null")
        digest = data.get("sha256")
        if not isinstance(digest, str) or len(digest) != 64:
            raise ConfigValidationError("A frozen A_TEB requires a 64-character sha256")
    return data
