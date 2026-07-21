"""Fail-closed validation and V1 isolation for the FAM-TEB V2 foundation."""

import hashlib
import math
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

import yaml


class V2ContractError(ValueError):
    """Raised when a V2 contract or the V1/V2 boundary is invalid."""


GEOMETRY_MODES = {
    "BALANCED": 0,
    "CRUISE": 1,
    "STATIC_DENSE": 2,
    "CORRIDOR": 3,
    "MANEUVER": 4,
}
DYNAMIC_OVERLAYS = {
    "NONE": 0,
    "CROSSING": 1,
    "HEAD_ON": 2,
    "FOLLOW": 3,
    "OVERTAKE_OR_YIELD": 4,
}
TRANSITION_STATES = {
    "STABLE": 0,
    "ENTERING": 1,
    "EXITING": 2,
    "HOLDING": 3,
    "SAFE_OVERRIDE": 4,
    "FAULTED": 5,
}
ACTION_STAGES = ("commanded", "feasible", "safe", "executed")
SAFETY_MODES = ("NORMAL", "WARNING", "EMERGENCY", "FAULT")

FAST_PARAMETER_NAMES = (
    "max_vel_x",
    "max_vel_theta",
    "acc_lim_x",
    "acc_lim_theta",
    "min_obstacle_dist",
    "inflation_dist",
    "weight_obstacle",
    "weight_viapoint",
    "weight_optimaltime",
    "dynamic_obstacle_inflation_dist",
    "weight_dynamic_obstacle",
    "weight_dynamic_obstacle_inflation",
    "weight_velocity_obstacle_ratio",
)
SLOW_PARAMETER_TYPES = {
    "max_vel_x_backwards": "double",
    "max_global_plan_lookahead_dist": "double",
    "global_plan_viapoint_sep": "double",
    "include_dynamic_obstacles": "bool",
    "max_number_classes": "int",
    "selection_cost_hysteresis": "double",
    "switching_blocking_period": "double",
}
STARTUP_PARAMETER_TYPES = {
    "planner_backend_objects": "object_set",
    "costmap_converter_plugin": "plugin_name",
    "footprint": "polygon",
    "wheelbase": "double",
    "min_turning_radius": "double",
    "frames_and_sensor_sources": "interface_set",
}

MODE_CONDITION_KEYS = {
    "BALANCED": {
        "enter": ("mode_confidence_max",),
        "exit": ("mode_confidence_min",),
    },
    "CRUISE": {
        "enter": (
            "obstacle_density_max",
            "forward_clearance_min_m",
            "predicted_ttc_min_s",
            "path_curvature_abs_max",
            "goal_direction_variance_max",
        ),
        "exit": (
            "obstacle_density_max",
            "forward_clearance_min_m",
            "predicted_ttc_min_s",
            "path_curvature_abs_max",
        ),
    },
    "STATIC_DENSE": {
        "enter": ("static_density_min", "forward_path_blocked_score_min"),
        "exit": ("static_density_max", "forward_path_blocked_score_max"),
    },
    "CORRIDOR": {
        "enter": (
            "corridor_confidence_min",
            "corridor_width_max_m",
            "front_clearance_min_m",
        ),
        "exit": ("corridor_confidence_max", "corridor_width_min_m"),
    },
    "MANEUVER": {
        "enter": (
            "normal_backend_infeasible_duration_min_s",
            "dead_end_score_min",
            "progress_rate_max_mps",
        ),
        "exit": (
            "normal_backend_feasible_duration_min_s",
            "dead_end_score_max",
        ),
    },
}
DYNAMIC_THRESHOLD_KEYS = {
    "NONE": ("release_confirmation_s",),
    "CROSSING": ("crossing_probability_min", "predicted_ttc_max_s"),
    "HEAD_ON": ("closing_speed_min_mps", "predicted_ttc_max_s"),
    "FOLLOW": ("same_direction_probability_min", "time_headway_max_s"),
    "OVERTAKE_OR_YIELD": ("path_block_probability_min", "decision_horizon_s"),
}


def load_yaml(path: Any) -> Dict[str, Any]:
    """Load a YAML mapping without ROS side effects."""

    source = Path(path)
    try:
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise V2ContractError("cannot load YAML {}: {}".format(source, exc))
    if not isinstance(data, dict):
        raise V2ContractError("YAML root must be a mapping: {}".format(source))
    return data


def _exact_keys(data: Mapping[str, Any], expected: Iterable[str], context: str) -> None:
    expected_set = set(expected)
    missing = sorted(expected_set - set(data))
    extra = sorted(set(data) - expected_set)
    if missing or extra:
        raise V2ContractError(
            "{} keys differ; missing={}, extra={}".format(context, missing, extra)
        )


def _mapping(data: Mapping[str, Any], key: str, context: str) -> Mapping[str, Any]:
    value = data.get(key)
    if not isinstance(value, dict):
        raise V2ContractError("{}.{} must be a mapping".format(context, key))
    return value


def _list(data: Mapping[str, Any], key: str, context: str) -> Sequence[Any]:
    value = data.get(key)
    if not isinstance(value, list):
        raise V2ContractError("{}.{} must be a list".format(context, key))
    return value


def _bool(value: Any, context: str) -> bool:
    if not isinstance(value, bool):
        raise V2ContractError("{} must be bool".format(context))
    return value


def _number_or_none(
    value: Any, context: str, positive: bool = False, non_negative: bool = False
) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise V2ContractError("{} must be numeric or null".format(context))
    number = float(value)
    if not math.isfinite(number):
        raise V2ContractError("{} must be finite".format(context))
    if positive and number <= 0.0:
        raise V2ContractError("{} must be positive".format(context))
    if non_negative and number < 0.0:
        raise V2ContractError("{} must be non-negative".format(context))
    return number


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_architecture_contract(data: Mapping[str, Any]) -> Mapping[str, Any]:
    """Validate the immutable V2 architecture and execution semantics."""

    _exact_keys(
        data,
        (
            "schema_version",
            "architecture_generation",
            "contract_id",
            "status",
            "simulation_only",
            "real_vehicle_use_forbidden",
            "v1_boundary",
            "paths",
            "enums",
            "action_execution",
            "modules",
            "rates_hz",
            "safety",
            "runtime",
        ),
        "architecture_contract",
    )
    if str(data["schema_version"]) != "2.0":
        raise V2ContractError("architecture_contract.schema_version must be 2.0")
    if data["architecture_generation"] != "v2":
        raise V2ContractError("architecture_generation must be v2")
    if data["status"] != "implementation_skeleton":
        raise V2ContractError("architecture status must be implementation_skeleton")
    if data["simulation_only"] is not True or data["real_vehicle_use_forbidden"] is not True:
        raise V2ContractError("the V2 skeleton must remain simulation-only")

    boundary = _mapping(data, "v1_boundary", "architecture_contract")
    _exact_keys(
        boundary,
        (
            "contract_path",
            "baseline_snapshot",
            "preserve_v1_runner",
            "v1_runner_must_reject_v2",
            "historical_pilots_restart_forbidden",
        ),
        "v1_boundary",
    )
    if boundary["contract_path"] != "docs/thesis_experiment/experiment_contract.yaml":
        raise V2ContractError("V1 contract path drifted")
    if boundary["baseline_snapshot"] != (
        "config/thesis_experiments/v2/v1_frozen_baseline.yaml"
    ):
        raise V2ContractError("V1 baseline snapshot path drifted")
    for key in (
        "preserve_v1_runner",
        "v1_runner_must_reject_v2",
        "historical_pilots_restart_forbidden",
    ):
        if boundary[key] is not True:
            raise V2ContractError("v1_boundary.{} must remain true".format(key))

    paths = _mapping(data, "paths", "architecture_contract")
    _exact_keys(paths, ("config_root", "manifest_root", "artifact_root"), "paths")
    expected_paths = {
        "config_root": "config/thesis_experiments/v2",
        "manifest_root": "experiments/manifests/v2",
        "artifact_root": "artifacts/v2",
    }
    if dict(paths) != expected_paths:
        raise V2ContractError("V2 roots must remain isolated")

    enums = _mapping(data, "enums", "architecture_contract")
    _exact_keys(enums, ("geometry_mode", "dynamic_overlay", "transition_state"), "enums")
    if enums["geometry_mode"] != GEOMETRY_MODES:
        raise V2ContractError("GeometryMode mapping drifted")
    if enums["dynamic_overlay"] != DYNAMIC_OVERLAYS:
        raise V2ContractError("DynamicOverlay mapping drifted")
    if enums["transition_state"] != TRANSITION_STATES:
        raise V2ContractError("TransitionState mapping drifted")

    action = _mapping(data, "action_execution", "architecture_contract")
    _exact_keys(
        action,
        (
            "stages",
            "critic_action",
            "record_all_stages",
            "hidden_execution_state_forbidden",
        ),
        "action_execution",
    )
    if tuple(action["stages"]) != ACTION_STAGES or action["critic_action"] != "executed":
        raise V2ContractError("four-stage action semantics drifted")
    if action["record_all_stages"] is not True:
        raise V2ContractError("all action stages must be recorded")
    if action["hidden_execution_state_forbidden"] is not True:
        raise V2ContractError("hidden execution state must remain forbidden")

    modules = _mapping(data, "modules", "architecture_contract")
    _exact_keys(
        modules,
        (
            "required_packages",
            "planned_packages",
            "planner_backends",
            "v2_learning_implemented",
        ),
        "modules",
    )
    if tuple(modules["required_packages"]) != ("nav_world_model", "teb_mode_manager"):
        raise V2ContractError("required V2 package order drifted")
    if tuple(modules["planned_packages"]) != (
        "m2_hybrid_local_planner",
        "m2_maneuver_planner",
    ):
        raise V2ContractError("planned V2 package order drifted")
    if tuple(modules["planner_backends"]) != (
        "single_topology_teb",
        "topology_locked_hcp",
        "ackermann_maneuver",
    ):
        raise V2ContractError("planner backend identities drifted")
    if modules["v2_learning_implemented"] is not False:
        raise V2ContractError("the foundation contract cannot claim V2 learning")

    rates = _mapping(data, "rates_hz", "architecture_contract")
    _exact_keys(
        rates,
        (
            "tracking",
            "prediction",
            "mode_decision",
            "policy_decision",
            "planner_expected",
            "shield_minimum",
        ),
        "rates_hz",
    )
    for name in ("tracking", "prediction", "mode_decision", "policy_decision"):
        band = rates[name]
        if not isinstance(band, dict):
            raise V2ContractError("rates_hz.{} must be a mapping".format(name))
        _exact_keys(band, ("minimum", "maximum"), "rates_hz.{}".format(name))
        lower = _number_or_none(band["minimum"], "{}.minimum".format(name), positive=True)
        upper = _number_or_none(band["maximum"], "{}.maximum".format(name), positive=True)
        if lower is None or upper is None or lower > upper:
            raise V2ContractError("rates_hz.{} range is invalid".format(name))
    _number_or_none(rates["planner_expected"], "planner_expected", positive=True)
    _number_or_none(rates["shield_minimum"], "shield_minimum", positive=True)

    safety = _mapping(data, "safety", "architecture_contract")
    _exact_keys(safety, ("modes", "strict_cbf_claim", "invariants"), "safety")
    if tuple(safety["modes"]) != SAFETY_MODES or safety["strict_cbf_claim"] is not False:
        raise V2ContractError("V2 safety claim or modes drifted")
    expected_invariants = (
        "no_direct_rl_cmd_vel",
        "health_fault_stop_has_priority",
        "ackermann_constraints_cannot_be_disabled",
        "collision_and_stopping_constraints_cannot_be_disabled",
        "reverse_requires_rear_coverage",
        "parameter_activation_required_for_attribution",
        "real_motion_requires_on_site_approval",
    )
    if tuple(safety["invariants"]) != expected_invariants:
        raise V2ContractError("V2 safety invariants drifted")

    runtime = _mapping(data, "runtime", "architecture_contract")
    _exact_keys(
        runtime,
        (
            "default_mode",
            "skeleton_nodes_publish_valid_data",
            "allow_training",
            "allow_motion",
            "allow_parameter_write",
        ),
        "runtime",
    )
    if runtime["default_mode"] != "disabled":
        raise V2ContractError("V2 skeleton default mode must be disabled")
    for key in (
        "skeleton_nodes_publish_valid_data",
        "allow_training",
        "allow_motion",
        "allow_parameter_write",
    ):
        if runtime[key] is not False:
            raise V2ContractError("runtime.{} must remain false".format(key))
    return data


def validate_parameter_registry(
    data: Mapping[str, Any], require_runtime_ready: bool = False
) -> Mapping[str, Any]:
    """Validate typed V2 parameter lifecycles and unresolved calibration fields."""

    _exact_keys(
        data,
        (
            "schema_version",
            "architecture_generation",
            "registry_id",
            "status",
            "runtime_ready",
            "minimum_turning_radius_source",
            "fast_continuous",
            "slow_mode_profile",
            "startup_structural",
        ),
        "parameter_registry",
    )
    if str(data["schema_version"]) != "2.0" or data["architecture_generation"] != "v2":
        raise V2ContractError("parameter registry must be V2 schema 2.0")
    runtime_ready = _bool(data["runtime_ready"], "parameter_registry.runtime_ready")
    if require_runtime_ready and not runtime_ready:
        raise V2ContractError("parameter registry is not runtime-ready")
    if data["minimum_turning_radius_source"] != "calibration_then_fixed":
        raise V2ContractError("turning radius must remain calibration_then_fixed")

    fast = _list(data, "fast_continuous", "parameter_registry")
    if tuple(item.get("name") for item in fast if isinstance(item, dict)) != FAST_PARAMETER_NAMES:
        raise V2ContractError("fast parameter names/order drifted")
    seen = set()
    for index, item in enumerate(fast):
        if not isinstance(item, dict):
            raise V2ContractError("fast_continuous[{}] must be a mapping".format(index))
        _exact_keys(
            item,
            (
                "name",
                "type",
                "lifecycle",
                "owner",
                "online_support",
                "learned_candidate",
                "transform",
                "bounds",
                "max_delta_per_decision",
            ),
            "fast_continuous[{}]".format(index),
        )
        name = item["name"]
        if name in seen:
            raise V2ContractError("duplicate parameter {}".format(name))
        seen.add(name)
        if (
            item["type"] != "double"
            or item["lifecycle"] != "fast_continuous"
            or item["owner"] != "feasible_action_decoder"
            or item["online_support"] is not True
            or not isinstance(item["learned_candidate"], bool)
            or not isinstance(item["transform"], str)
            or not item["transform"]
        ):
            raise V2ContractError("fast parameter {} metadata is invalid".format(name))
        bounds = item["bounds"]
        if not isinstance(bounds, dict):
            raise V2ContractError("{}.bounds must be a mapping".format(name))
        _exact_keys(
            bounds,
            ("simulation_min", "simulation_max", "real_min", "real_max"),
            "{}.bounds".format(name),
        )
        sim_min = _number_or_none(bounds["simulation_min"], "{}.simulation_min".format(name))
        sim_max = _number_or_none(bounds["simulation_max"], "{}.simulation_max".format(name))
        real_min = _number_or_none(bounds["real_min"], "{}.real_min".format(name))
        real_max = _number_or_none(bounds["real_max"], "{}.real_max".format(name))
        delta = _number_or_none(
            item["max_delta_per_decision"],
            "{}.max_delta_per_decision".format(name),
            positive=True,
        )
        if (sim_min is None) != (sim_max is None) or (
            sim_min is not None and sim_min >= sim_max
        ):
            raise V2ContractError("{} simulation bounds are invalid".format(name))
        if (real_min is None) != (real_max is None) or (
            real_min is not None and real_min >= real_max
        ):
            raise V2ContractError("{} real bounds are invalid".format(name))
        if require_runtime_ready and (sim_min is None or sim_max is None or delta is None):
            raise V2ContractError("{} lacks runtime simulation bounds/rate".format(name))

    slow = _list(data, "slow_mode_profile", "parameter_registry")
    if tuple(item.get("name") for item in slow if isinstance(item, dict)) != tuple(
        SLOW_PARAMETER_TYPES
    ):
        raise V2ContractError("slow parameter names/order drifted")
    for index, item in enumerate(slow):
        if not isinstance(item, dict):
            raise V2ContractError("slow_mode_profile[{}] must be a mapping".format(index))
        _exact_keys(
            item,
            (
                "name",
                "type",
                "lifecycle",
                "owner",
                "online_support",
                "learned_candidate",
                "default",
                "status",
            ),
            "slow_mode_profile[{}]".format(index),
        )
        name = item["name"]
        if name in seen:
            raise V2ContractError("duplicate parameter {}".format(name))
        seen.add(name)
        if (
            item["type"] != SLOW_PARAMETER_TYPES[name]
            or item["lifecycle"] != "slow_mode_profile"
            or item["owner"] != "mode_manager"
            or item["online_support"] is not True
            or item["learned_candidate"] is not False
            or not isinstance(item["status"], str)
        ):
            raise V2ContractError("slow parameter {} metadata is invalid".format(name))
        if require_runtime_ready and item["default"] is None:
            raise V2ContractError("{} lacks a frozen mode-profile default".format(name))

    startup = _list(data, "startup_structural", "parameter_registry")
    if tuple(item.get("name") for item in startup if isinstance(item, dict)) != tuple(
        STARTUP_PARAMETER_TYPES
    ):
        raise V2ContractError("startup parameter names/order drifted")
    for index, item in enumerate(startup):
        if not isinstance(item, dict):
            raise V2ContractError("startup_structural[{}] must be a mapping".format(index))
        _exact_keys(
            item,
            (
                "name",
                "type",
                "lifecycle",
                "owner",
                "online_support",
                "learned_candidate",
                "status",
            ),
            "startup_structural[{}]".format(index),
        )
        name = item["name"]
        if name in seen:
            raise V2ContractError("duplicate parameter {}".format(name))
        seen.add(name)
        if (
            item["type"] != STARTUP_PARAMETER_TYPES[name]
            or item["lifecycle"] != "startup_structural"
            or item["owner"] != "process_startup"
            or item["online_support"] is not False
            or item["learned_candidate"] is not False
        ):
            raise V2ContractError("startup parameter {} metadata is invalid".format(name))
    return data


def _validate_threshold_mapping(
    data: Mapping[str, Any],
    expected_keys: Sequence[str],
    context: str,
    require_runtime_ready: bool,
) -> None:
    _exact_keys(data, expected_keys, context)
    for key in expected_keys:
        value = _number_or_none(data[key], "{}.{}".format(context, key), non_negative=True)
        if require_runtime_ready and value is None:
            raise V2ContractError("{}.{} is not frozen".format(context, key))


def validate_mode_thresholds(
    data: Mapping[str, Any], require_runtime_ready: bool = False
) -> Mapping[str, Any]:
    """Validate factorized mode/overlay threshold structure without inventing values."""

    _exact_keys(
        data,
        (
            "schema_version",
            "architecture_generation",
            "threshold_set_id",
            "status",
            "runtime_ready",
            "modes",
            "dynamic_overlays",
            "transition",
            "runtime",
        ),
        "mode_thresholds",
    )
    if str(data["schema_version"]) != "2.0" or data["architecture_generation"] != "v2":
        raise V2ContractError("mode thresholds must be V2 schema 2.0")
    runtime_ready = _bool(data["runtime_ready"], "mode_thresholds.runtime_ready")
    if require_runtime_ready and not runtime_ready:
        raise V2ContractError("mode thresholds are not runtime-ready")

    modes = _mapping(data, "modes", "mode_thresholds")
    _exact_keys(modes, GEOMETRY_MODES, "modes")
    for mode, condition_keys in MODE_CONDITION_KEYS.items():
        state = modes[mode]
        if not isinstance(state, dict):
            raise V2ContractError("modes.{} must be a mapping".format(mode))
        _exact_keys(
            state,
            (
                "minimum_dwell_s",
                "enter_confirmation_s",
                "exit_confirmation_s",
                "enter",
                "exit",
            ),
            "modes.{}".format(mode),
        )
        for name in ("minimum_dwell_s", "enter_confirmation_s", "exit_confirmation_s"):
            value = _number_or_none(
                state[name], "modes.{}.{}".format(mode, name), non_negative=True
            )
            if require_runtime_ready and value is None:
                raise V2ContractError("modes.{}.{} is not frozen".format(mode, name))
        for edge in ("enter", "exit"):
            conditions = state[edge]
            if not isinstance(conditions, dict):
                raise V2ContractError("modes.{}.{} must be a mapping".format(mode, edge))
            _validate_threshold_mapping(
                conditions,
                condition_keys[edge],
                "modes.{}.{}".format(mode, edge),
                require_runtime_ready,
            )

    overlays = _mapping(data, "dynamic_overlays", "mode_thresholds")
    _exact_keys(overlays, DYNAMIC_OVERLAYS, "dynamic_overlays")
    for overlay, keys in DYNAMIC_THRESHOLD_KEYS.items():
        state = overlays[overlay]
        if not isinstance(state, dict):
            raise V2ContractError("dynamic_overlays.{} must be a mapping".format(overlay))
        _validate_threshold_mapping(
            state,
            keys,
            "dynamic_overlays.{}".format(overlay),
            require_runtime_ready,
        )

    transition = _mapping(data, "transition", "mode_thresholds")
    _validate_threshold_mapping(
        transition,
        (
            "anchor_blend_duration_s",
            "max_parameter_total_variation_per_s",
            "safe_release_time_constant_s",
            "minimum_mode_confidence",
        ),
        "transition",
        require_runtime_ready,
    )
    runtime = _mapping(data, "runtime", "mode_thresholds")
    _exact_keys(
        runtime,
        (
            "enabled",
            "parameter_write_enabled",
            "default_geometry_mode",
            "default_dynamic_overlay",
        ),
        "runtime",
    )
    if (
        runtime["enabled"] is not False
        or runtime["parameter_write_enabled"] is not False
        or runtime["default_geometry_mode"] != "BALANCED"
        or runtime["default_dynamic_overlay"] != "NONE"
    ):
        raise V2ContractError("V2 threshold skeleton runtime must stay disabled/BALANCED/NONE")
    return data


def validate_state_contract(
    data: Mapping[str, Any], require_runtime_ready: bool = False
) -> Mapping[str, Any]:
    """Validate angular scan metadata, rear coverage and action context requirements."""

    _exact_keys(
        data,
        (
            "schema_version",
            "architecture_generation",
            "state_contract_id",
            "status",
            "runtime_ready",
            "laser_scan",
            "observation_groups",
            "action_context",
            "v1_compatibility",
        ),
        "state_contract",
    )
    if str(data["schema_version"]) != "2.0" or data["architecture_generation"] != "v2":
        raise V2ContractError("state contract must be V2 schema 2.0")
    runtime_ready = _bool(data["runtime_ready"], "state_contract.runtime_ready")
    if require_runtime_ready and not runtime_ready:
        raise V2ContractError("state contract is not runtime-ready")

    scan = _mapping(data, "laser_scan", "state_contract")
    _exact_keys(
        scan,
        (
            "metadata_required",
            "angle_order",
            "reject_inconsistent_ray_count",
            "reject_non_finite_angles",
            "directional_coverage",
        ),
        "laser_scan",
    )
    expected_metadata = (
        "stamp",
        "frame_id",
        "angle_min",
        "angle_max",
        "angle_increment",
        "range_min",
        "range_max",
        "ray_count",
    )
    if tuple(scan["metadata_required"]) != expected_metadata:
        raise V2ContractError("LaserScan metadata requirements drifted")
    if (
        scan["angle_order"] != "increasing"
        or scan["reject_inconsistent_ray_count"] is not True
        or scan["reject_non_finite_angles"] is not True
    ):
        raise V2ContractError("LaserScan fail-closed semantics drifted")

    coverage = _mapping(scan, "directional_coverage", "laser_scan")
    _exact_keys(
        coverage,
        (
            "directions",
            "front_center_rad",
            "left_center_rad",
            "right_center_rad",
            "rear_center_rad",
            "coverage_probe_half_width_rad",
            "rear_required_for_reverse",
            "minimum_rear_coverage_rad",
            "maximum_scan_age_s",
        ),
        "directional_coverage",
    )
    if tuple(coverage["directions"]) != ("front", "left", "right", "rear"):
        raise V2ContractError("directional coverage names/order drifted")
    expected_centers = {
        "front_center_rad": 0.0,
        "left_center_rad": math.pi / 2.0,
        "right_center_rad": -math.pi / 2.0,
        "rear_center_rad": math.pi,
    }
    for key, expected in expected_centers.items():
        value = _number_or_none(coverage[key], "directional_coverage.{}".format(key))
        if value is None or abs(value - expected) > 1e-12:
            raise V2ContractError("{} drifted".format(key))
    _number_or_none(
        coverage["coverage_probe_half_width_rad"],
        "coverage_probe_half_width_rad",
        positive=True,
    )
    if coverage["rear_required_for_reverse"] is not True:
        raise V2ContractError("rear coverage must be required for reverse")
    rear = _number_or_none(
        coverage["minimum_rear_coverage_rad"],
        "minimum_rear_coverage_rad",
        positive=True,
    )
    age = _number_or_none(
        coverage["maximum_scan_age_s"], "maximum_scan_age_s", positive=True
    )
    if require_runtime_ready and (rear is None or age is None):
        raise V2ContractError("rear coverage and scan age must be frozen for runtime")

    if tuple(data["observation_groups"]) != (
        "geometry_and_motion",
        "dynamic_risk",
        "mode_and_transition",
        "action_execution_context",
        "health_and_safety",
    ):
        raise V2ContractError("V2 observation groups drifted")
    action = _mapping(data, "action_context", "state_contract")
    _exact_keys(
        action,
        (
            "stages",
            "projection_reason_mask_required",
            "safety_reason_mask_required",
            "hidden_ema_hold_state_forbidden",
        ),
        "action_context",
    )
    if tuple(action["stages"]) != ACTION_STAGES:
        raise V2ContractError("state action stages drifted")
    for key in (
        "projection_reason_mask_required",
        "safety_reason_mask_required",
        "hidden_ema_hold_state_forbidden",
    ):
        if action[key] is not True:
            raise V2ContractError("action_context.{} must be true".format(key))

    compatibility = _mapping(data, "v1_compatibility", "state_contract")
    _exact_keys(
        compatibility,
        (
            "v1_observation_dimension_unchanged",
            "v1_sector_count_unchanged",
            "v2_metadata_is_additive",
        ),
        "v1_compatibility",
    )
    if (
        compatibility["v1_observation_dimension_unchanged"] is not True
        or compatibility["v1_sector_count_unchanged"] != 36
        or compatibility["v2_metadata_is_additive"] is not True
    ):
        raise V2ContractError("V1 state compatibility boundary drifted")
    return data


def _required_number(
    data: Mapping[str, Any], key: str, context: str, positive: bool = False,
    non_negative: bool = False,
) -> float:
    value = _number_or_none(
        data.get(key), "{}.{}".format(context, key),
        positive=positive, non_negative=non_negative,
    )
    if value is None:
        raise V2ContractError("{}.{} cannot be null".format(context, key))
    return value


def validate_simulation_contract(data: Mapping[str, Any]) -> Mapping[str, Any]:
    """Validate the V2-02 simulation-only actuator and sensor profile."""

    _exact_keys(
        data,
        (
            "schema_version", "architecture_generation", "contract_id", "status",
            "simulation_only", "formal_experiment", "runtime_ready",
            "real_vehicle_use_forbidden", "profile", "actuator",
            "command_transport", "laser_transport", "contact_collision",
            "determinism", "regression_gates", "claims",
        ),
        "simulation_contract",
    )
    if str(data["schema_version"]) != "2.0" or data["architecture_generation"] != "v2":
        raise V2ContractError("simulation contract must be V2 schema 2.0")
    if data["status"] != "component_candidate_frozen":
        raise V2ContractError("simulation candidate status drifted")
    if (
        data["simulation_only"] is not True
        or data["formal_experiment"] is not False
        or data["runtime_ready"] is not False
        or data["real_vehicle_use_forbidden"] is not True
    ):
        raise V2ContractError("V2-02 must stay non-formal, simulation-only, runtime-disabled")

    profile = _mapping(data, "profile", "simulation_contract")
    _exact_keys(
        profile,
        (
            "name", "plugin", "legacy_v1_default_preserved",
            "planar_kinematic_pose_integration", "full_tire_force_model_claimed",
        ),
        "simulation_contract.profile",
    )
    if (
        profile["name"] != "v2_02_dynamics"
        or profile["plugin"] != "m2_ackermann_plugin"
        or profile["legacy_v1_default_preserved"] is not True
        or profile["planar_kinematic_pose_integration"] is not True
        or profile["full_tire_force_model_claimed"] is not False
    ):
        raise V2ContractError("simulation profile identity/claim drifted")

    actuator = _mapping(data, "actuator", "simulation_contract")
    actuator_keys = (
        "update_rate_hz", "speed_time_constant_s", "steering_time_constant_s",
        "max_acceleration_mps2", "max_deceleration_mps2",
        "max_brake_deceleration_mps2", "max_emergency_deceleration_mps2",
        "max_steering_rate_radps", "reverse_requires_zero_crossing",
    )
    _exact_keys(actuator, actuator_keys, "simulation_contract.actuator")
    values = {
        key: _required_number(actuator, key, "actuator", positive=True)
        for key in actuator_keys[:-1]
    }
    if not (
        values["max_deceleration_mps2"]
        <= values["max_brake_deceleration_mps2"]
        <= values["max_emergency_deceleration_mps2"]
    ):
        raise V2ContractError("service/brake/emergency deceleration ordering is invalid")
    if actuator["reverse_requires_zero_crossing"] is not True:
        raise V2ContractError("reverse must require a zero-speed crossing")

    command = _mapping(data, "command_transport", "simulation_contract")
    _exact_keys(
        command,
        ("delay_s", "jitter_s", "timeout_s", "ordered_activation",
         "deterministic_seed", "queue_limit"),
        "simulation_contract.command_transport",
    )
    delay = _required_number(command, "delay_s", "command_transport", positive=True)
    jitter = _required_number(command, "jitter_s", "command_transport", non_negative=True)
    _required_number(command, "timeout_s", "command_transport", positive=True)
    if jitter > delay or command["ordered_activation"] is not True:
        raise V2ContractError("command delay/jitter ordering is invalid")
    if (
        isinstance(command["deterministic_seed"], bool)
        or not isinstance(command["deterministic_seed"], int)
        or command["deterministic_seed"] < 0
        or isinstance(command["queue_limit"], bool)
        or not isinstance(command["queue_limit"], int)
        or command["queue_limit"] < 2
    ):
        raise V2ContractError("command seed/queue limit is invalid")

    laser = _mapping(data, "laser_transport", "simulation_contract")
    _exact_keys(
        laser,
        (
            "raw_topic", "output_topic", "update_rate_hz", "delay_s", "jitter_s",
            "range_noise_stddev_m", "angle_min_rad", "angle_max_rad", "ray_count",
            "preserve_acquisition_stamp",
        ),
        "simulation_contract.laser_transport",
    )
    if laser["raw_topic"] != "/v2/scan_raw" or laser["output_topic"] != "/scan":
        raise V2ContractError("V2 scan transport topics drifted")
    _required_number(laser, "update_rate_hz", "laser_transport", positive=True)
    laser_delay = _required_number(laser, "delay_s", "laser_transport", positive=True)
    laser_jitter = _required_number(laser, "jitter_s", "laser_transport", non_negative=True)
    _required_number(laser, "range_noise_stddev_m", "laser_transport", non_negative=True)
    angle_min = _required_number(laser, "angle_min_rad", "laser_transport")
    angle_max = _required_number(laser, "angle_max_rad", "laser_transport")
    if laser_jitter > laser_delay or angle_min >= angle_max:
        raise V2ContractError("laser angular or delay range is invalid")
    if laser["ray_count"] != 720 or laser["preserve_acquisition_stamp"] is not True:
        raise V2ContractError("laser sample count/stamp semantics drifted")

    contact = _mapping(data, "contact_collision", "simulation_contract")
    _exact_keys(
        contact,
        ("enabled", "topic", "message_type", "chassis_collision", "evaluator_only"),
        "simulation_contract.contact_collision",
    )
    if contact != {
        "enabled": True,
        "topic": "/m2_gazebo/contacts",
        "message_type": "gazebo_msgs/ContactsState",
        "chassis_collision": "chassis_collision",
        "evaluator_only": True,
    }:
        raise V2ContractError("contact collision contract drifted")

    determinism = _mapping(data, "determinism", "simulation_contract")
    _exact_keys(
        determinism,
        (
            "gazebo_seed_required", "reset_clears_actuator_state",
            "reset_clears_delay_queue", "same_seed_reproducible",
            "common_random_numbers_supported",
        ),
        "simulation_contract.determinism",
    )
    if any(value is not True for value in determinism.values()):
        raise V2ContractError("all V2-02 determinism/reset invariants must be true")

    gates = _mapping(data, "regression_gates", "simulation_contract")
    _exact_keys(gates, ("straight", "circle", "braking", "reverse", "delay"),
                "simulation_contract.regression_gates")
    gate_keys = {
        "straight": ("command_speed_mps", "duration_s", "minimum_steady_speed_mps",
                     "maximum_lateral_error_m"),
        "circle": ("command_speed_mps", "command_yaw_rate_radps",
                   "maximum_steady_yaw_rate_error_radps", "maximum_radius_error_m"),
        "braking": ("initial_speed_mps", "maximum_stop_time_s",
                    "minimum_stopping_distance_m", "maximum_stopping_distance_m"),
        "reverse": ("command_speed_mps", "minimum_steady_speed_mps",
                    "maximum_lateral_error_m"),
        "delay": ("commanded_delay_s", "jitter_s",
                  "scheduler_resolution_tolerance_s", "minimum_observed_s",
                  "maximum_observed_s"),
    }
    for name, keys in gate_keys.items():
        gate = gates[name]
        if not isinstance(gate, dict):
            raise V2ContractError("regression_gates.{} must be a mapping".format(name))
        _exact_keys(gate, keys, "regression_gates.{}".format(name))
        for key in keys:
            _required_number(gate, key, "regression_gates.{}".format(name))
    braking = gates["braking"]
    if not 0.0 < braking["minimum_stopping_distance_m"] < braking["maximum_stopping_distance_m"]:
        raise V2ContractError("braking distance gate must be nonzero and ordered")
    delay_gate = gates["delay"]
    if not 0.0 <= delay_gate["minimum_observed_s"] < delay_gate["maximum_observed_s"]:
        raise V2ContractError("delay observation gate is invalid")
    if delay_gate["scheduler_resolution_tolerance_s"] < 0.0:
        raise V2ContractError("delay scheduler-resolution tolerance cannot be negative")
    theoretical_maximum = (
        delay_gate["commanded_delay_s"] + delay_gate["jitter_s"]
        + delay_gate["scheduler_resolution_tolerance_s"]
    )
    if delay_gate["maximum_observed_s"] + 1.0e-12 < theoretical_maximum:
        raise V2ContractError("delay observation gate excludes its declared tolerance")

    claims = _mapping(data, "claims", "simulation_contract")
    _exact_keys(
        claims,
        (
            "real_vehicle_calibrated", "real_vehicle_safety_claim_allowed",
            "nonzero_stopping_distance_required", "training_allowed",
        ),
        "simulation_contract.claims",
    )
    if claims != {
        "real_vehicle_calibrated": False,
        "real_vehicle_safety_claim_allowed": False,
        "nonzero_stopping_distance_required": True,
        "training_allowed": False,
    }:
        raise V2ContractError("V2-02 claim boundary drifted")
    return data


def validate_evaluation_contract(data: Mapping[str, Any]) -> Mapping[str, Any]:
    """Validate the unified five-family V2-02 evaluator contract."""

    _exact_keys(
        data,
        (
            "schema_version", "architecture_generation", "contract_id", "status",
            "simulation_only", "formal_experiment", "runtime_ready", "trace_columns",
            "semantics", "metrics", "output", "policy_boundary",
        ),
        "evaluation_contract",
    )
    if str(data["schema_version"]) != "2.0" or data["architecture_generation"] != "v2":
        raise V2ContractError("evaluation contract must be V2 schema 2.0")
    if data["status"] != "component_candidate_frozen":
        raise V2ContractError("evaluation contract status drifted")
    if not (
        data["simulation_only"] is True
        and data["formal_experiment"] is False
        and data["runtime_ready"] is False
    ):
        raise V2ContractError("V2-02 evaluator must stay non-formal/runtime-disabled")
    columns = _mapping(data, "trace_columns", "evaluation_contract")
    _exact_keys(columns, ("required", "nullable", "gear_values"), "trace_columns")
    required = (
        "stamp_s", "x_m", "y_m", "yaw_rad", "linear_velocity_mps",
        "angular_velocity_radps", "commanded_speed_mps", "clearance_m",
        "goal_distance_m", "collision", "goal_reached", "contact_count",
        "topology_id", "global_replan_count", "recovery_count", "gear",
    )
    if tuple(columns["required"]) != required or tuple(columns["nullable"]) != ("predicted_ttc_s",):
        raise V2ContractError("V2 trace columns/order drifted")
    if tuple(columns["gear_values"]) != ("REVERSE", "NEUTRAL", "FORWARD"):
        raise V2ContractError("V2 trace gear values drifted")
    semantics = _mapping(data, "semantics", "evaluation_contract")
    _exact_keys(
        semantics,
        (
            "timestamp_clock", "timestamps_strictly_increasing", "terminal_precedence",
            "success_requires_goal_and_stop", "contact_is_collision_source",
            "episode_is_statistical_unit",
        ),
        "evaluation_contract.semantics",
    )
    if semantics["timestamp_clock"] != "gazebo_sim_time":
        raise V2ContractError("evaluator clock must be Gazebo simulation time")
    if tuple(semantics["terminal_precedence"]) != (
        "COLLISION", "SUCCESS", "TIMEOUT", "ABORTED"
    ):
        raise V2ContractError("terminal precedence drifted")
    for key in (
        "timestamps_strictly_increasing", "success_requires_goal_and_stop",
        "contact_is_collision_source", "episode_is_statistical_unit",
    ):
        if semantics[key] is not True:
            raise V2ContractError("semantics.{} must be true".format(key))
    metrics = _mapping(data, "metrics", "evaluation_contract")
    _exact_keys(metrics, ("common", "family"), "evaluation_contract.metrics")
    expected_common = (
        "success", "collision", "navigation_time_s", "path_length_m",
        "mean_abs_speed_mps", "minimum_clearance_m", "stop_count",
        "reverse_distance_m",
    )
    if tuple(metrics["common"]) != expected_common:
        raise V2ContractError("common evaluator metrics drifted")
    family = metrics["family"]
    if not isinstance(family, dict):
        raise V2ContractError("metrics.family must be a mapping")
    expected_family = {
        "CRUISE": ("lateral_rms_m", "heading_oscillation_rad",
                   "unnecessary_deceleration_count"),
        "DYNAMIC": ("minimum_predicted_ttc_s", "stop_count",
                    "interaction_clearance_m"),
        "STATIC_DENSE": ("topology_switch_count", "global_replan_count",
                         "recovery_count"),
        "CORRIDOR": ("lateral_rms_m", "heading_oscillation_rad",
                     "emergency_contact_count"),
        "MANEUVER": ("gear_switch_count", "gear_switch_while_moving_count",
                     "reverse_distance_m"),
    }
    _exact_keys(family, expected_family, "metrics.family")
    if any(tuple(family[name]) != values for name, values in expected_family.items()):
        raise V2ContractError("family evaluator metrics drifted")
    output = _mapping(data, "output", "evaluation_contract")
    _exact_keys(
        output,
        (
            "formal_result", "preserve_failures", "family_results_separate",
            "macro_average_primary_forbidden", "raw_trace_sha256_required",
        ),
        "evaluation_contract.output",
    )
    if output["formal_result"] is not False or any(
        output[key] is not True for key in (
            "preserve_failures", "family_results_separate",
            "macro_average_primary_forbidden", "raw_trace_sha256_required",
        )
    ):
        raise V2ContractError("evaluator output evidence policy drifted")
    boundary = _mapping(data, "policy_boundary", "evaluation_contract")
    _exact_keys(
        boundary,
        (
            "evaluator_reads_manifest_labels", "runtime_policy_reads_manifest_labels",
            "evaluator_output_is_policy_input",
        ),
        "evaluation_contract.policy_boundary",
    )
    if boundary != {
        "evaluator_reads_manifest_labels": True,
        "runtime_policy_reads_manifest_labels": False,
        "evaluator_output_is_policy_input": False,
    }:
        raise V2ContractError("evaluator/runtime policy boundary drifted")
    return data


def validate_world_model_contract(
    data: Mapping[str, Any], workspace: Optional[Any] = None, verify_profiles: bool = False
) -> Mapping[str, Any]:
    """Validate the V2-03 world-model and label-free rule-supervisor contract."""

    _exact_keys(
        data,
        (
            "schema_version", "architecture_generation", "contract_id", "status",
            "simulation_only", "formal_experiment", "runtime_ready", "training_allowed",
            "real_vehicle_use_forbidden", "profiles", "runtime_inputs", "world_model",
            "rule_supervisor", "health", "acceptance_gates", "claims",
        ),
        "world_model_contract",
    )
    if str(data["schema_version"]) != "2.0" or data["architecture_generation"] != "v2":
        raise V2ContractError("world_model_contract schema/generation drifted")
    if data["status"] != "component_candidate_frozen":
        raise V2ContractError("world_model_contract status drifted")
    expected_flags = {
        "simulation_only": True,
        "formal_experiment": False,
        "runtime_ready": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
    }
    for key, expected in expected_flags.items():
        if data[key] is not expected:
            raise V2ContractError("world_model_contract.{} safety boundary drifted".format(key))

    profiles = _mapping(data, "profiles", "world_model_contract")
    _exact_keys(profiles, ("world_model", "rule_supervisor", "deployment_mode_thresholds"),
                "world_model_contract.profiles")
    for name in ("world_model", "rule_supervisor"):
        profile = profiles[name]
        if not isinstance(profile, dict):
            raise V2ContractError("profiles.{} must be a mapping".format(name))
        _exact_keys(profile, ("path", "sha256"), "profiles.{}".format(name))
        if not isinstance(profile["path"], str) or not profile["path"]:
            raise V2ContractError("profiles.{}.path must be nonempty".format(name))
        if not isinstance(profile["sha256"], str) or len(profile["sha256"]) != 64:
            raise V2ContractError("profiles.{}.sha256 must be SHA256".format(name))
    deployment = profiles["deployment_mode_thresholds"]
    if not isinstance(deployment, dict):
        raise V2ContractError("deployment_mode_thresholds must be a mapping")
    _exact_keys(deployment, ("path", "runtime_ready_must_remain"),
                "profiles.deployment_mode_thresholds")
    if deployment["runtime_ready_must_remain"] is not False:
        raise V2ContractError("deployment mode thresholds must remain runtime_ready=false")

    inputs = _mapping(data, "runtime_inputs", "world_model_contract")
    _exact_keys(inputs, ("allowed", "evaluator_only", "forbidden_fields",
                         "runtime_manifest_access", "truth_used_by_policy"),
                "world_model_contract.runtime_inputs")
    if inputs["allowed"] != [
        "/scan", "/odom", "/move_base/NavfnROS/plan",
        "/move_base/local_costmap/costmap",
    ]:
        raise V2ContractError("world-model runtime input allowlist drifted")
    if inputs["evaluator_only"] != [
        "/gazebo/model_states", "/pedsim_simulator/simulated_agents"
    ]:
        raise V2ContractError("truth evaluator-only topics drifted")
    if inputs["forbidden_fields"] != ["family", "split", "evaluator_only", "scene_id"]:
        raise V2ContractError("runtime forbidden fields drifted")
    if inputs["runtime_manifest_access"] is not False or inputs["truth_used_by_policy"] is not False:
        raise V2ContractError("runtime label/truth leakage is forbidden")

    world_model = _mapping(data, "world_model", "world_model_contract")
    _exact_keys(
        world_model,
        (
            "fixed_frame", "robot_frame", "scan_validation", "geometry_source",
            "tracker", "prediction", "tracker_reset_on_time_regression",
            "all_four_scan_directions_required", "optional_costmap_health_reported",
        ),
        "world_model_contract.world_model",
    )
    if world_model != {
        "fixed_frame": "odom",
        "robot_frame": "base_link",
        "scan_validation": "fail_closed",
        "geometry_source": "laser_and_optional_runtime_path",
        "tracker": "gated_nearest_neighbor_alpha_beta",
        "prediction": "constant_velocity_with_growing_covariance",
        "tracker_reset_on_time_regression": True,
        "all_four_scan_directions_required": True,
        "optional_costmap_health_reported": True,
    }:
        raise V2ContractError("world-model algorithm/health semantics drifted")

    supervisor = _mapping(data, "rule_supervisor", "world_model_contract")
    _exact_keys(
        supervisor,
        (
            "implementation", "geometry_modes", "dynamic_overlays",
            "minimum_dwell_and_confirmation", "balanced_on_low_confidence",
            "fault_forces_balanced_none", "publishes_velocity_commands",
            "publishes_parameter_transactions",
        ),
        "world_model_contract.rule_supervisor",
    )
    if supervisor["implementation"] != "deterministic_observation_rules":
        raise V2ContractError("rule supervisor must remain deterministic rules")
    if supervisor["geometry_modes"] != list(GEOMETRY_MODES):
        raise V2ContractError("rule supervisor geometry enums drifted")
    if supervisor["dynamic_overlays"] != list(DYNAMIC_OVERLAYS):
        raise V2ContractError("rule supervisor dynamic enums drifted")
    for key in ("minimum_dwell_and_confirmation", "balanced_on_low_confidence",
                "fault_forces_balanced_none"):
        if supervisor[key] is not True:
            raise V2ContractError("rule supervisor invariant {} drifted".format(key))
    for key in ("publishes_velocity_commands", "publishes_parameter_transactions"):
        if supervisor[key] is not False:
            raise V2ContractError("V2-03 output authority expanded")

    health = _mapping(data, "health", "world_model_contract")
    _exact_keys(health, ("required_checks", "stale_is_invalid",
                         "sequence_mismatch_is_invalid", "fault_priority_over_mode"),
                "world_model_contract.health")
    if health["required_checks"] != [
        "scan_metadata", "scan_age", "tf", "localization_age", "tracker_age",
        "directional_coverage",
    ]:
        raise V2ContractError("world-model health checks drifted")
    if any(health[key] is not True for key in
           ("stale_is_invalid", "sequence_mismatch_is_invalid", "fault_priority_over_mode")):
        raise V2ContractError("world-model health must fail closed")

    gates = _mapping(data, "acceptance_gates", "world_model_contract")
    gate_keys = (
        "synthetic_tracking_position_rmse_max_m", "synthetic_prediction_rmse_max_m",
        "synthetic_id_switches_max", "synthetic_mode_macro_recall_min",
        "health_fault_cases_required", "gazebo_dynamic_track_samples_min",
        "gazebo_dynamic_position_rmse_max_m", "gazebo_dynamic_id_switches_max",
        "gazebo_crossing_overlay_required",
    )
    _exact_keys(gates, gate_keys, "world_model_contract.acceptance_gates")
    for key in gate_keys[:-1]:
        _required_number(gates, key, "acceptance_gates", non_negative=True)
    if not 0.0 < gates["synthetic_mode_macro_recall_min"] <= 1.0:
        raise V2ContractError("mode macro recall gate must be in (0,1]")
    if gates["gazebo_crossing_overlay_required"] is not True:
        raise V2ContractError("Gazebo crossing overlay is required")

    claims = _mapping(data, "claims", "world_model_contract")
    expected_claims = {
        "tracker_real_vehicle_calibrated": False,
        "planner_performance_claim_allowed": False,
        "learning_claim_allowed": False,
        "training_started": False,
        "real_vehicle_started": False,
    }
    _exact_keys(claims, expected_claims, "world_model_contract.claims")
    if claims != expected_claims:
        raise V2ContractError("V2-03 claim boundary drifted")

    if verify_profiles:
        if workspace is None:
            raise V2ContractError("workspace is required to verify V2-03 profiles")
        root = Path(workspace).resolve()
        for name in ("world_model", "rule_supervisor"):
            profile = profiles[name]
            path = (root / profile["path"]).resolve()
            if root not in path.parents or not path.is_file():
                raise V2ContractError("profile path is missing or outside workspace: {}".format(path))
            if _sha256(path) != profile["sha256"]:
                raise V2ContractError("profile hash mismatch: {}".format(name))
            profile_data = load_yaml(path)
            if not (
                profile_data.get("simulation_only") is True
                and profile_data.get("runtime_ready") is False
                and profile_data.get("training_allowed") is False
                and profile_data.get("real_vehicle_use_forbidden") is True
            ):
                raise V2ContractError("profile safety boundary drifted: {}".format(name))
        threshold_path = (root / deployment["path"]).resolve()
        thresholds = load_yaml(threshold_path)
        if thresholds.get("runtime_ready") is not False:
            raise V2ContractError("deployment mode_thresholds unexpectedly became runtime ready")
    return data


def validate_action_pipeline_contract(
    data: Mapping[str, Any],
    workspace: Optional[Any] = None,
    verify_profiles: bool = False,
) -> Mapping[str, Any]:
    """Validate the V2-04 no-training Anchor/action transaction boundary."""

    _exact_keys(
        data,
        (
            "schema_version", "architecture_generation", "contract_id", "status",
            "simulation_only", "runtime_ready", "training_allowed",
            "real_vehicle_use_forbidden", "profiles", "action_decoder",
            "parameter_transaction", "rule_closed_loop", "acceptance_gates", "claims",
        ),
        "action_pipeline_contract",
    )
    if str(data["schema_version"]) != "2.0" or data["architecture_generation"] != "v2":
        raise V2ContractError("action pipeline schema/generation drifted")
    if data["status"] != "uncalibrated_simulation_candidate":
        raise V2ContractError("action pipeline status drifted")
    expected_boundary = {
        "simulation_only": True,
        "runtime_ready": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
    }
    for key, expected in expected_boundary.items():
        if data[key] is not expected:
            raise V2ContractError("action pipeline {} safety boundary drifted".format(key))

    profiles = _mapping(data, "profiles", "action_pipeline_contract")
    _exact_keys(
        profiles,
        (
            "parameter_registry", "anchor_bank", "required_anchor_ids",
            "required_overlay_ids", "calibration_split_only",
            "formal_test_selection_forbidden",
        ),
        "action_pipeline_contract.profiles",
    )
    if profiles["parameter_registry"] != "config/thesis_experiments/v2/parameter_registry.yaml":
        raise V2ContractError("V2-04 parameter registry path drifted")
    if profiles["anchor_bank"] != (
        "src/application/teb_mode_manager/config/v2_04_anchor_bank_candidate.yaml"
    ):
        raise V2ContractError("V2-04 anchor bank path drifted")
    expected_anchors = (
        "anchor_balanced", "anchor_cruise", "anchor_static_dense", "anchor_corridor",
        "anchor_maneuver_forward", "anchor_maneuver_reverse",
    )
    if tuple(profiles["required_anchor_ids"]) != expected_anchors:
        raise V2ContractError("required V2-04 Anchor identities drifted")
    if tuple(profiles["required_overlay_ids"]) != tuple(DYNAMIC_OVERLAYS):
        raise V2ContractError("required V2-04 overlays drifted")
    if profiles["calibration_split_only"] is not True:
        raise V2ContractError("Anchor calibration must use calibration split only")
    if profiles["formal_test_selection_forbidden"] is not True:
        raise V2ContractError("formal test selection must remain forbidden")

    decoder = _mapping(data, "action_decoder", "action_pipeline_contract")
    _exact_keys(
        decoder,
        (
            "parameter_order_source", "normalized_residual_domain",
            "zero_residual_rule_pilot", "constraints", "projection_reason_bits",
        ),
        "action_pipeline_contract.action_decoder",
    )
    if decoder["parameter_order_source"] != "anchor_bank.parameters":
        raise V2ContractError("action parameter order source drifted")
    if tuple(decoder["normalized_residual_domain"]) != (-1.0, 1.0):
        raise V2ContractError("normalized residual domain drifted")
    if decoder["zero_residual_rule_pilot"] is not True:
        raise V2ContractError("V2-04 must remain a zero-residual rule pilot")
    constraints = _mapping(decoder, "constraints", "action_decoder")
    _exact_keys(
        constraints,
        (
            "finite_exact_schema", "bounded_positive_parameters", "positive_weights",
            "minimum_turning_radius_m", "yaw_coupling", "minimum_inflation_gap_m",
            "minimum_dynamic_inflation_gap_m",
        ),
        "action_decoder.constraints",
    )
    for key in ("finite_exact_schema", "bounded_positive_parameters", "positive_weights"):
        if constraints[key] is not True:
            raise V2ContractError("decoder constraint {} must remain enabled".format(key))
    if constraints["yaw_coupling"] != (
        "max_vel_theta_lte_max_vel_x_over_min_turning_radius"
    ):
        raise V2ContractError("Ackermann decoder coupling drifted")
    for key in (
        "minimum_turning_radius_m", "minimum_inflation_gap_m",
        "minimum_dynamic_inflation_gap_m",
    ):
        _number_or_none(constraints[key], "constraints." + key, positive=True)
    reason_bits = _mapping(decoder, "projection_reason_bits", "action_decoder")
    expected_reason_bits = {
        "physical_bound_audit": 1,
        "ackermann_coupling_audit": 2,
        "inflation_gap_audit": 4,
        "dynamic_inflation_gap_audit": 8,
        "positive_weight_audit": 16,
        "speed_envelope_audit": 32,
    }
    if dict(reason_bits) != expected_reason_bits:
        raise V2ContractError("projection reason masks drifted")

    transaction = _mapping(data, "parameter_transaction", "action_pipeline_contract")
    _exact_keys(
        transaction,
        (
            "stages", "transition_origin", "execution_feedback_required", "ack_required",
            "readback_required", "activation_required", "atomic_failure_behavior",
            "continuous_rate_source", "discrete_commit", "slow_profile_types",
            "default_backend", "dynamic_reconfigure_backend_enabled",
            "real_parameter_write_enabled",
        ),
        "action_pipeline_contract.parameter_transaction",
    )
    if tuple(transaction["stages"]) != ACTION_STAGES:
        raise V2ContractError("V2-04 four-stage action order drifted")
    if transaction["transition_origin"] != "previous_executed":
        raise V2ContractError("V2-04 transitions must begin at previous executed")
    for key in ("execution_feedback_required", "ack_required", "readback_required", "activation_required"):
        if transaction[key] is not True:
            raise V2ContractError("transaction {} must remain true".format(key))
    if transaction["atomic_failure_behavior"] != "hold_previous_executed":
        raise V2ContractError("transaction atomic failure behavior drifted")
    if transaction["discrete_commit"] != "after_continuous_convergence":
        raise V2ContractError("typed profile commit rule drifted")
    if tuple(transaction["slow_profile_types"]) != ("double", "int", "bool"):
        raise V2ContractError("typed slow profile types drifted")
    if transaction["default_backend"] != "deterministic_shadow":
        raise V2ContractError("V2-04 must use deterministic shadow backend")
    if transaction["dynamic_reconfigure_backend_enabled"] is not False:
        raise V2ContractError("V2-04 dynamic_reconfigure backend must remain disabled")
    if transaction["real_parameter_write_enabled"] is not False:
        raise V2ContractError("real parameter writes must remain disabled")

    rule = _mapping(data, "rule_closed_loop", "action_pipeline_contract")
    _exact_keys(
        rule,
        (
            "policy_source", "learned_policy_loaded", "checkpoint_access",
            "runtime_scene_labels_allowed", "runtime_manifest_access",
            "forbidden_runtime_topics", "input_topic", "output_topic",
            "decision_frequency_hz", "context_maximum_age_s", "invalid_context_behavior",
        ),
        "action_pipeline_contract.rule_closed_loop",
    )
    if rule["policy_source"] != "rule_supervisor_zero_residual":
        raise V2ContractError("V2-04 rule policy identity drifted")
    for key in (
        "learned_policy_loaded", "checkpoint_access", "runtime_scene_labels_allowed",
        "runtime_manifest_access",
    ):
        if rule[key] is not False:
            raise V2ContractError("rule loop {} boundary drifted".format(key))
    forbidden = set(rule["forbidden_runtime_topics"])
    if not {"/gazebo/model_states", "/pedsim_simulator/simulated_agents"}.issubset(forbidden):
        raise V2ContractError("rule loop truth-topic boundary drifted")
    _number_or_none(rule["decision_frequency_hz"], "decision_frequency_hz", positive=True)
    _number_or_none(rule["context_maximum_age_s"], "context_maximum_age_s", positive=True)
    if rule["invalid_context_behavior"] != "publish_invalid_hold_previous_executed":
        raise V2ContractError("invalid context behavior drifted")

    gates = _mapping(data, "acceptance_gates", "action_pipeline_contract")
    _exact_keys(
        gates,
        (
            "normal_projection_rate_max", "continuous_jump_tolerance",
            "complete_trace_reconstruction_required", "fault_atomicity_required",
            "all_profile_types_exercised",
        ),
        "action_pipeline_contract.acceptance_gates",
    )
    projection_max = _number_or_none(
        gates["normal_projection_rate_max"], "normal_projection_rate_max", non_negative=True
    )
    if projection_max is None or projection_max > 0.10:
        raise V2ContractError("normal projection gate must be at most 10 percent")
    _number_or_none(
        gates["continuous_jump_tolerance"], "continuous_jump_tolerance", non_negative=True
    )
    for key in (
        "complete_trace_reconstruction_required", "fault_atomicity_required",
        "all_profile_types_exercised",
    ):
        if gates[key] is not True:
            raise V2ContractError("acceptance gate {} must remain true".format(key))
    claims = _mapping(data, "claims", "action_pipeline_contract")
    _exact_keys(
        claims,
        (
            "anchor_values_formally_calibrated", "navigation_performance_improved",
            "v2_learning_implemented", "gazebo_navigation_closed_loop_completed",
            "real_vehicle_validated",
        ),
        "action_pipeline_contract.claims",
    )
    if any(value is not False for value in claims.values()):
        raise V2ContractError("V2-04 cannot claim unperformed calibration/performance/learning")

    if verify_profiles:
        if workspace is None:
            raise V2ContractError("workspace is required to verify V2-04 profiles")
        root = Path(workspace).resolve()
        registry = load_yaml(root / profiles["parameter_registry"])
        validate_parameter_registry(registry)
        if registry["runtime_ready"] is not False:
            raise V2ContractError("deployment parameter registry must remain runtime_ready=false")
        try:
            from teb_mode_manager.action_pipeline import AnchorBank, ActionPipelineError
            bank = AnchorBank.from_file(root / profiles["anchor_bank"])
        except (ImportError, ActionPipelineError) as exc:
            raise V2ContractError("V2-04 anchor bank validation failed: {}".format(exc))
        if tuple(bank.anchors) != expected_anchors:
            raise V2ContractError("checked Anchor Bank order/identity drifted")
        expected_parameters = FAST_PARAMETER_NAMES + tuple(SLOW_PARAMETER_TYPES)
        if bank.parameter_names != expected_parameters:
            raise V2ContractError("Anchor Bank parameter order differs from registry")
        registry_types = {
            row["name"]: row["type"]
            for row in registry["fast_continuous"] + registry["slow_mode_profile"]
        }
        registry_lifecycles = {
            row["name"]: row["lifecycle"]
            for row in registry["fast_continuous"] + registry["slow_mode_profile"]
        }
        for name, definition in bank.definitions.items():
            if definition.parameter_type != registry_types[name]:
                raise V2ContractError("Anchor Bank type differs for {}".format(name))
            if definition.lifecycle != registry_lifecycles[name]:
                raise V2ContractError("Anchor Bank lifecycle differs for {}".format(name))
        if bank.minimum_turning_radius_m != float(constraints["minimum_turning_radius_m"]):
            raise V2ContractError("decoder/Anchor Bank turning-radius constraints differ")
    return data


def validate_typed_calibration_contract(
    data: Mapping[str, Any], workspace: Optional[Any] = None,
    verify_resources: bool = False,
) -> Mapping[str, Any]:
    """Validate the V2-04B Gazebo typed-write and calibration-only boundary."""

    _exact_keys(
        data,
        (
            "schema_version", "architecture_generation", "contract_id", "status",
            "simulation_only", "formal_experiment", "runtime_ready",
            "training_allowed", "real_vehicle_use_forbidden", "resources",
            "write_gate", "typed_transaction", "calibration", "acceptance_gates",
            "claims",
        ),
        "typed_calibration_contract",
    )
    if str(data["schema_version"]) != "2.0" or data["architecture_generation"] != "v2":
        raise V2ContractError("typed calibration schema/generation drifted")
    if data["contract_id"] != "fam_teb_v2_04b_typed_transaction_calibration_1":
        raise V2ContractError("typed calibration contract identity drifted")
    if data["status"] != "calibration_started":
        raise V2ContractError("typed calibration status drifted")
    expected_boundary = {
        "simulation_only": True,
        "formal_experiment": False,
        "runtime_ready": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
    }
    for key, expected in expected_boundary.items():
        if data[key] is not expected:
            raise V2ContractError("typed calibration {} boundary drifted".format(key))

    resources = _mapping(data, "resources", "typed_calibration_contract")
    _exact_keys(
        resources,
        ("anchor_bank", "calibration_scene_manifest", "gazebo_startup_profile"),
        "typed_calibration_contract.resources",
    )
    expected_paths = {
        "anchor_bank": "src/application/teb_mode_manager/config/v2_04_anchor_bank_candidate.yaml",
        "calibration_scene_manifest": (
            "experiments/manifests/v2/calibration/v2_04b_anchor_calibration_scenes.yaml"
        ),
        "gazebo_startup_profile": "src/simulation/m2_gazebo/config/v2_04b_calibration_teb.yaml",
    }
    for key, expected_path in expected_paths.items():
        item = _mapping(resources, key, "typed_calibration_contract.resources")
        _exact_keys(item, ("path", "sha256"), "resources." + key)
        if item["path"] != expected_path:
            raise V2ContractError("{} resource path drifted".format(key))
        if not isinstance(item["sha256"], str) or len(item["sha256"]) != 64:
            raise V2ContractError("{} resource hash is invalid".format(key))

    gate = _mapping(data, "write_gate", "typed_calibration_contract")
    _exact_keys(
        gate,
        (
            "explicit_opt_in_required", "use_sim_time_required",
            "simulation_marker_required", "active_gazebo_clock_required",
            "exact_teb_namespace", "generic_runtime_write_enabled",
            "real_vehicle_write_enabled",
        ),
        "typed_calibration_contract.write_gate",
    )
    for key in (
        "explicit_opt_in_required", "use_sim_time_required",
        "active_gazebo_clock_required",
    ):
        if gate[key] is not True:
            raise V2ContractError("typed simulation gate {} drifted".format(key))
    if gate["simulation_marker_required"] != "/m2_gazebo/simulation_only":
        raise V2ContractError("typed simulation marker drifted")
    if gate["exact_teb_namespace"] != "/move_base/TebLocalPlannerROS":
        raise V2ContractError("typed TEB namespace drifted")
    if gate["generic_runtime_write_enabled"] is not False or gate["real_vehicle_write_enabled"] is not False:
        raise V2ContractError("non-Gazebo parameter write must remain disabled")

    transaction = _mapping(data, "typed_transaction", "typed_calibration_contract")
    _exact_keys(
        transaction,
        (
            "parameter_count", "type_counts", "complete_profile_single_request",
            "live_description_validation_required", "live_type_validation_required",
            "live_range_validation_required", "acknowledgement_required",
            "callback_readback_required", "activation_barrier",
            "failure_restore_target", "shutdown_restore_target", "timeout_s",
            "equality_tolerance",
        ),
        "typed_calibration_contract.typed_transaction",
    )
    if transaction["parameter_count"] != 20 or transaction["type_counts"] != {
        "double": 18, "int": 1, "bool": 1
    }:
        raise V2ContractError("typed transaction schema drifted")
    for key in (
        "complete_profile_single_request", "live_description_validation_required",
        "live_type_validation_required", "live_range_validation_required",
        "acknowledgement_required", "callback_readback_required",
    ):
        if transaction[key] is not True:
            raise V2ContractError("typed transaction {} must remain true".format(key))
    if transaction["activation_barrier"] != "teb_reconfigure_callback_before_service_reply":
        raise V2ContractError("typed activation barrier drifted")
    if transaction["failure_restore_target"] != "previous_executed":
        raise V2ContractError("fault rollback must target previous executed")
    if transaction["shutdown_restore_target"] != "startup_snapshot":
        raise V2ContractError("shutdown rollback must target startup snapshot")
    _number_or_none(transaction["timeout_s"], "typed timeout", positive=True)
    _number_or_none(transaction["equality_tolerance"], "typed tolerance", non_negative=True)

    calibration = _mapping(data, "calibration", "typed_calibration_contract")
    _exact_keys(
        calibration,
        (
            "selection_split", "permitted_manifest_splits", "forbidden_selection_splits",
            "strategy", "selection_order", "coordinate_step_fraction_of_domain",
            "coordinate_levels", "candidate_budget_per_anchor", "common_random_numbers",
            "runtime_scene_labels_allowed", "evaluator_output_is_policy_input",
            "anchor_scene_families", "dynamic_overlay_by_family",
            "screening_coordinates", "hard_gates", "family_objectives",
        ),
        "typed_calibration_contract.calibration",
    )
    if calibration["selection_split"] != "calibration" or calibration["permitted_manifest_splits"] != ["calibration"]:
        raise V2ContractError("Anchor selection must be calibration-only")
    if calibration["forbidden_selection_splits"] != ["validation", "test_id", "test_ood"]:
        raise V2ContractError("forbidden selection splits drifted")
    if calibration["strategy"] != "deterministic_one_factor_screen_then_bounded_refinement":
        raise V2ContractError("calibration strategy drifted")
    if calibration["selection_order"] != "safety_feasibility_then_family_objective_lexicographic":
        raise V2ContractError("calibration selection order drifted")
    step = _number_or_none(
        calibration["coordinate_step_fraction_of_domain"], "coordinate step", positive=True
    )
    if step is None or step > 0.25 or calibration["coordinate_levels"] != [-1, 0, 1]:
        raise V2ContractError("calibration coordinate screen drifted")
    if calibration["candidate_budget_per_anchor"] != 9:
        raise V2ContractError("candidate budget drifted")
    if calibration["common_random_numbers"] is not True:
        raise V2ContractError("paired calibration seeds are required")
    if calibration["runtime_scene_labels_allowed"] is not False:
        raise V2ContractError("runtime scene labels remain forbidden")
    if calibration["evaluator_output_is_policy_input"] is not False:
        raise V2ContractError("evaluator output cannot enter runtime policy")
    expected_anchors = (
        "anchor_balanced", "anchor_cruise", "anchor_static_dense", "anchor_corridor",
        "anchor_maneuver_forward", "anchor_maneuver_reverse",
    )
    anchor_families = _mapping(calibration, "anchor_scene_families", "calibration")
    coordinates = _mapping(calibration, "screening_coordinates", "calibration")
    _exact_keys(anchor_families, expected_anchors, "anchor_scene_families")
    _exact_keys(coordinates, expected_anchors, "screening_coordinates")
    family_names = ("CRUISE", "DYNAMIC", "STATIC_DENSE", "CORRIDOR", "MANEUVER")
    if anchor_families["anchor_balanced"] != list(family_names):
        raise V2ContractError("Balanced Anchor must screen all calibration families")
    for anchor_id in expected_anchors:
        if not isinstance(coordinates[anchor_id], list) or len(coordinates[anchor_id]) != 4:
            raise V2ContractError("{} must screen four coordinates".format(anchor_id))
        if len(set(coordinates[anchor_id])) != 4:
            raise V2ContractError("{} coordinates must be unique".format(anchor_id))
    overlays = _mapping(calibration, "dynamic_overlay_by_family", "calibration")
    _exact_keys(overlays, family_names, "dynamic_overlay_by_family")
    if overlays != {
        "CRUISE": "NONE", "DYNAMIC": "CROSSING", "STATIC_DENSE": "NONE",
        "CORRIDOR": "NONE", "MANEUVER": "NONE",
    }:
        raise V2ContractError("calibration overlay mapping drifted")
    hard_gates = _mapping(calibration, "hard_gates", "calibration")
    _exact_keys(
        hard_gates,
        (
            "collision_count_max", "interface_failure_count_max",
            "minimum_clearance_m_min", "successful_episode_required",
        ),
        "calibration.hard_gates",
    )
    if hard_gates["collision_count_max"] != 0 or hard_gates["interface_failure_count_max"] != 0:
        raise V2ContractError("calibration cannot tolerate collision/interface failures")
    _number_or_none(hard_gates["minimum_clearance_m_min"], "minimum clearance", positive=True)
    if hard_gates["successful_episode_required"] is not True:
        raise V2ContractError("successful calibration episode is required")
    objectives = _mapping(calibration, "family_objectives", "calibration")
    _exact_keys(objectives, family_names, "family_objectives")
    for family in family_names:
        rows = objectives[family]
        if not isinstance(rows, list) or len(rows) != 4:
            raise V2ContractError("family objectives must contain four ordered entries")
        metrics = []
        for index, row in enumerate(rows):
            _exact_keys(row, ("metric", "direction"), "family_objectives.{}[{}]".format(family, index))
            if not isinstance(row["metric"], str) or not row["metric"]:
                raise V2ContractError("family objective metric is invalid")
            if row["direction"] not in ("minimize", "maximize"):
                raise V2ContractError("family objective direction is invalid")
            metrics.append(row["metric"])
        if len(set(metrics)) != len(metrics):
            raise V2ContractError("family objective metrics must be unique")

    gates = _mapping(data, "acceptance_gates", "typed_calibration_contract")
    _exact_keys(
        gates,
        (
            "actual_gazebo_transaction_probe_required", "startup_snapshot_restore_required",
            "all_typed_stages_reconstructible", "calibration_scene_count",
            "generated_candidate_count", "planned_calibration_episode_count",
            "navigation_episode_count_min_before_freeze",
        ),
        "typed_calibration_contract.acceptance_gates",
    )
    for key in (
        "actual_gazebo_transaction_probe_required", "startup_snapshot_restore_required",
        "all_typed_stages_reconstructible",
    ):
        if gates[key] is not True:
            raise V2ContractError("acceptance gate {} drifted".format(key))
    if (
        gates["calibration_scene_count"] != 5
        or gates["generated_candidate_count"] != 54
        or gates["planned_calibration_episode_count"] != 90
        or gates["navigation_episode_count_min_before_freeze"] < 30
    ):
        raise V2ContractError("calibration acceptance budgets drifted")
    claims = _mapping(data, "claims", "typed_calibration_contract")
    _exact_keys(
        claims,
        (
            "typed_teb_transaction_implemented", "anchor_calibration_started",
            "anchor_calibration_complete", "anchor_values_frozen",
            "navigation_performance_improved", "sac_training_started",
            "real_vehicle_validated",
        ),
        "typed_calibration_contract.claims",
    )
    if claims["typed_teb_transaction_implemented"] is not True or claims["anchor_calibration_started"] is not True:
        raise V2ContractError("implemented/started claims drifted")
    if any(claims[key] is not False for key in (
        "anchor_calibration_complete", "anchor_values_frozen",
        "navigation_performance_improved", "sac_training_started", "real_vehicle_validated",
    )):
        raise V2ContractError("typed calibration overclaims unfinished work")

    if verify_resources:
        if workspace is None:
            raise V2ContractError("workspace is required to verify calibration resources")
        root = Path(workspace).resolve()
        resolved = {}
        for key, item in resources.items():
            path = (root / item["path"]).resolve()
            try:
                path.relative_to(root)
            except ValueError as exc:
                raise V2ContractError("{} escapes workspace".format(key)) from exc
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
                raise V2ContractError("{} resource hash mismatch".format(key))
            resolved[key] = path
        try:
            from teb_mode_manager.action_pipeline import AnchorBank, FeasibleActionDecoder
            from .v2_scene import load_v2_scene_manifest
            bank = AnchorBank.from_file(resolved["anchor_bank"])
            manifest = load_v2_scene_manifest(resolved["calibration_scene_manifest"], root)
        except Exception as exc:
            raise V2ContractError("calibration resource validation failed: {}".format(exc))
        if tuple(bank.anchors) != expected_anchors:
            raise V2ContractError("calibration Anchor identities drifted")
        for anchor_id, names in coordinates.items():
            for name in names:
                if name not in bank.definitions or bank.definitions[name].parameter_type != "double":
                    raise V2ContractError("{} coordinate {} must be a known double".format(anchor_id, name))
        if len(manifest["scenes"]) != 5 or any(
            scene["split"] != "calibration" for scene in manifest["scenes"]
        ):
            raise V2ContractError("calibration manifest split/count drifted")
        startup = load_yaml(resolved["gazebo_startup_profile"])
        if set(startup) != {"TebLocalPlannerROS"}:
            raise V2ContractError("Gazebo startup profile namespace drifted")
        startup_values = startup["TebLocalPlannerROS"]
        balanced = bank.anchors["anchor_balanced"].values
        if bank.validate_values(startup_values, "gazebo startup profile") != balanced:
            raise V2ContractError("Gazebo startup profile must equal Balanced Anchor")
        # Constructing every candidate also proves the one-factor screen stays
        # inside the decoder's feasible typed domain.
        decoder = FeasibleActionDecoder(bank)
        for anchor_id, names in coordinates.items():
            for name in names:
                definition = bank.definitions[name]
                for direction in (-1.0, 1.0):
                    values = dict(bank.anchors[anchor_id].values)
                    values[name] = min(
                        definition.upper,
                        max(definition.lower, values[name] + direction * step * (
                            definition.upper - definition.lower
                        )),
                    )
                    decoder._intrinsic_feasible(values, None)
    return data


def require_v1_resource(
    path: Any, data: Mapping[str, Any], label: str = "V1 resource"
) -> Mapping[str, Any]:
    """Reject a V2 path or marker before a V1 runner consumes the resource."""

    source = Path(path)
    if any(part.lower() == "v2" for part in source.parts):
        raise V2ContractError("{} resolves under the V2 namespace: {}".format(label, source))
    if data.get("architecture_generation") == "v2" or str(data.get("schema_version")) == "2.0":
        raise V2ContractError("{} contains a V2 marker: {}".format(label, source))
    return data


def load_v1_yaml(path: Any, label: str = "V1 resource") -> Mapping[str, Any]:
    """Load YAML and enforce the V1/V2 namespace boundary."""

    data = load_yaml(path)
    return require_v1_resource(path, data, label)


def validate_v1_baseline_snapshot(
    data: Mapping[str, Any], workspace: Any, verify_evidence: bool = True
) -> Mapping[str, Any]:
    """Validate the historical dirty-state record and frozen T11/T12 hashes."""

    _exact_keys(
        data,
        (
            "schema_version",
            "architecture_generation",
            "snapshot_id",
            "status",
            "captured_at",
            "workspace",
            "main_repository",
            "submodules",
            "frozen_evidence",
            "v1_runner_resources",
            "invariants",
        ),
        "v1_baseline_snapshot",
    )
    if (
        str(data["schema_version"]) != "2.0"
        or data["architecture_generation"] != "v1"
        or data["status"] != "frozen_historical_snapshot"
    ):
        raise V2ContractError("V1 baseline snapshot identity is invalid")
    root = Path(workspace)
    if data["workspace"] != str(root):
        raise V2ContractError("V1 baseline workspace drifted")

    main = _mapping(data, "main_repository", "v1_baseline_snapshot")
    _exact_keys(
        main,
        (
            "branch",
            "head",
            "upstream",
            "remote",
            "dirty",
            "tracked_modified_count",
            "tracked_deleted_count",
            "untracked_file_count",
            "other_status_count",
            "status_porcelain_v1_uall_sha256",
            "tracked_diff_binary_sha256",
            "status_hash_is_historical_not_runtime_gate",
        ),
        "main_repository",
    )
    if (
        main["branch"] != "base_on_rl"
        or not isinstance(main["head"], str)
        or len(main["head"]) != 40
        or main["dirty"] is not True
        or main["status_hash_is_historical_not_runtime_gate"] is not True
    ):
        raise V2ContractError("main repository snapshot is invalid")
    for key in ("status_porcelain_v1_uall_sha256", "tracked_diff_binary_sha256"):
        if not isinstance(main[key], str) or len(main[key]) != 64:
            raise V2ContractError("main_repository.{} must be sha256".format(key))

    submodules = _list(data, "submodules", "v1_baseline_snapshot")
    seen_paths = set()
    for index, item in enumerate(submodules):
        if not isinstance(item, dict):
            raise V2ContractError("submodules[{}] must be a mapping".format(index))
        _exact_keys(
            item,
            ("path", "commit", "dirty_entry_count", "dirty_status_sha256"),
            "submodules[{}]".format(index),
        )
        path = item["path"]
        if path in seen_paths:
            raise V2ContractError("duplicate submodule path {}".format(path))
        seen_paths.add(path)
        if (
            not isinstance(item["commit"], str)
            or len(item["commit"]) != 40
            or isinstance(item["dirty_entry_count"], bool)
            or not isinstance(item["dirty_entry_count"], int)
            or item["dirty_entry_count"] < 0
            or not isinstance(item["dirty_status_sha256"], str)
            or len(item["dirty_status_sha256"]) != 64
        ):
            raise V2ContractError("submodule {} snapshot is invalid".format(path))

    evidence = _list(data, "frozen_evidence", "v1_baseline_snapshot")
    seen_evidence = set()
    for index, item in enumerate(evidence):
        if not isinstance(item, dict):
            raise V2ContractError("frozen_evidence[{}] must be a mapping".format(index))
        _exact_keys(item, ("path", "sha256"), "frozen_evidence[{}]".format(index))
        relative = item["path"]
        if relative in seen_evidence or any(
            part.lower() == "v2" for part in Path(relative).parts
        ):
            raise V2ContractError("frozen evidence path is duplicate or V2: {}".format(relative))
        seen_evidence.add(relative)
        if not isinstance(item["sha256"], str) or len(item["sha256"]) != 64:
            raise V2ContractError("frozen evidence hash is invalid: {}".format(relative))
        if verify_evidence:
            source = root / relative
            if not source.is_file():
                raise V2ContractError("frozen evidence is missing: {}".format(relative))
            actual = _sha256(source)
            if actual != item["sha256"]:
                raise V2ContractError(
                    "frozen evidence hash drifted: {} expected {} got {}".format(
                        relative, item["sha256"], actual
                    )
                )

    resources = _list(data, "v1_runner_resources", "v1_baseline_snapshot")
    for relative in resources:
        source = root / relative
        load_v1_yaml(source, "V1 runner resource")

    invariants = _mapping(data, "invariants", "v1_baseline_snapshot")
    _exact_keys(
        invariants,
        (
            "preserve_all_frozen_evidence_hashes",
            "v1_resources_must_not_resolve_under_v2",
            "v2_must_use_independent_artifact_root",
            "generated_build_outputs_in_git_forbidden",
            "bag_files_in_git_forbidden",
        ),
        "invariants",
    )
    if not all(value is True for value in invariants.values()):
        raise V2ContractError("all V1 isolation invariants must remain true")
    return data
