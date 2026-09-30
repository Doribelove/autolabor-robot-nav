"""Deterministic pure-offline builder for V2-04G-P2-D2.

This module deliberately does not import ROS and does not materialize an active
runtime profile.  It validates the frozen P2-D1 evidence, derives the proposed
single-field speed amendment, and emits the finite budget/statistics design that
a later independently authorized simulation stage would have to follow.
"""

from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path
import stat
from statistics import NormalDist
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import yaml


WORKSPACE_DEFAULT = Path("/home/robot/robot_ws_base_rl")
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_d2_offline_single_factor_design_contract.yaml"
)
ARTIFACT_ROOT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_d2_single_factor_design"
)
AMENDMENT_RELATIVE = ARTIFACT_ROOT_RELATIVE / "v2_04g_p2_d2_candidate_bank_amendment.yaml"
REPORT_RELATIVE = ARTIFACT_ROOT_RELATIVE / "v2_04g_p2_d2_design_report.yaml"

EXPECTED_TEB_DEFAULT = {
    "max_vel_x": 1.2,
    "max_vel_theta": 1.0,
    "acc_lim_x": 0.8,
    "acc_lim_theta": 0.8,
    "min_obstacle_dist": 0.25,
    "inflation_dist": 0.55,
    "weight_obstacle": 50.0,
    "weight_viapoint": 1.0,
    "weight_optimaltime": 1.0,
}
EXPECTED_TEB_TUNED_V1 = {
    "max_vel_x": 0.9,
    "max_vel_theta": 0.75,
    "acc_lim_x": 0.6,
    "acc_lim_theta": 0.6,
    "min_obstacle_dist": 0.375,
    "inflation_dist": 0.725,
    "weight_obstacle": 75.0,
    "weight_viapoint": 2.5,
    "weight_optimaltime": 0.75,
}


class DesignError(ValueError):
    """Raised when a frozen P2-D2 design invariant fails."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise DesignError(message)


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def safe_relative(value: str | Path) -> Path:
    relative = value if isinstance(value, Path) else Path(value)
    require(
        not relative.is_absolute() and bool(relative.parts) and ".." not in relative.parts,
        "unsafe relative path: {}".format(value),
    )
    return relative


def stable_read(workspace: Path, relative: str | Path) -> bytes:
    relative = safe_relative(relative)
    current = workspace
    for component in relative.parts:
        current = current / component
        require(not current.is_symlink(), "symbolic link forbidden: {}".format(relative))
    descriptor = os.open(
        str(workspace / relative),
        os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0),
    )
    try:
        before = os.fstat(descriptor)
        require(stat.S_ISREG(before.st_mode), "not a regular file: {}".format(relative))
        chunks: List[bytes] = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
        require(
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
            "file changed during read: {}".format(relative),
        )
        payload = b"".join(chunks)
        require(len(payload) == before.st_size, "short read: {}".format(relative))
        return payload
    finally:
        os.close(descriptor)


def load_yaml(payload: bytes, label: str) -> Dict[str, Any]:
    try:
        value = yaml.safe_load(payload.decode("utf-8"))
    except Exception as exc:
        raise DesignError("{} parse failed: {}".format(label, exc))
    require(type(value) is dict, "{} must be a mapping".format(label))
    return value


def finite(value: Any, label: str, *, positive: bool = False) -> float:
    require(type(value) in (int, float) and not isinstance(value, bool), "{} must be numeric".format(label))
    number = float(value)
    require(math.isfinite(number), "{} must be finite".format(label))
    if positive:
        require(number > 0.0, "{} must be positive".format(label))
    return number


def close(left: float, right: float, tolerance: float = 1e-12) -> bool:
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=tolerance)


def rounded(value: float) -> float:
    return round(float(value), 12)


def verify_binding(
    workspace: Path, binding: Mapping[str, Any], label: str
) -> Tuple[Path, bytes]:
    require(type(binding) is dict, "{} binding must be a mapping".format(label))
    require(set(binding) == {"path", "sha256"}, "{} binding keys differ".format(label))
    relative = safe_relative(binding["path"])
    expected = binding["sha256"]
    require(type(expected) is str and len(expected) == 64, "{} SHA differs".format(label))
    payload = stable_read(workspace, relative)
    require(sha256_bytes(payload) == expected, "{} target SHA differs".format(label))
    return relative, payload


def _parameter(bank: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    rows = bank.get("parameters")
    require(type(rows) is list, "anchor bank parameters differ")
    matches = [row for row in rows if type(row) is dict and row.get("name") == name]
    require(len(matches) == 1, "anchor bank parameter {} not unique".format(name))
    return matches[0]


def _algorithm(registry: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    rows = registry.get("algorithms")
    require(type(rows) is list, "T08 algorithm registry differs")
    matches = [row for row in rows if type(row) is dict and row.get("name") == name]
    require(len(matches) == 1, "T08 algorithm {} not unique".format(name))
    return matches[0]


def _numeric_mapping_equal(
    actual: Mapping[str, Any], expected: Mapping[str, float], label: str
) -> None:
    require(type(actual) is dict, "{} must be a mapping".format(label))
    require(set(expected).issubset(actual), "{} fields missing".format(label))
    for key, expected_value in expected.items():
        require(
            close(finite(actual[key], "{}.{}".format(label, key)), expected_value),
            "{}.{} differs".format(label, key),
        )


def _seed_range(row: Mapping[str, Any], label: str) -> List[int]:
    require(type(row) is dict, "{} must be a mapping".format(label))
    require(set(row) == {"first", "last", "count"}, "{} keys differ".format(label))
    first = row["first"]
    last = row["last"]
    count = row["count"]
    require(
        type(first) is int and type(last) is int and type(count) is int,
        "{} values must be integers".format(label),
    )
    values = list(range(first, last + 1))
    require(len(values) == count, "{} count differs".format(label))
    return values


def _all_unique(values: Iterable[int], label: str) -> None:
    values = list(values)
    require(len(values) == len(set(values)), "{} overlap".format(label))


def stopping_envelope(
    speed_mps: float,
    latency_s: float,
    service_deceleration_mps2: float,
    margin_m: float,
) -> Dict[str, float]:
    speed = finite(speed_mps, "speed", positive=True)
    latency = finite(latency_s, "latency", positive=True)
    deceleration = finite(
        service_deceleration_mps2, "service deceleration", positive=True
    )
    margin = finite(margin_m, "distance margin")
    require(margin >= 0.0, "distance margin must be nonnegative")
    latency_distance = speed * latency
    braking_distance = speed * speed / (2.0 * deceleration)
    return {
        "speed_mps": rounded(speed),
        "latency_s": rounded(latency),
        "latency_distance_m": rounded(latency_distance),
        "service_braking_distance_m": rounded(braking_distance),
        "additional_margin_m": rounded(margin),
        "required_forward_free_path_m": rounded(
            latency_distance + braking_distance + margin
        ),
    }


def approximate_required_pairs(
    alpha: float, power: float, paired_log_sd: float, reduction_fraction: float
) -> int:
    alpha = finite(alpha, "alpha", positive=True)
    power = finite(power, "power", positive=True)
    paired_log_sd = finite(paired_log_sd, "paired log SD", positive=True)
    reduction_fraction = finite(
        reduction_fraction, "reduction fraction", positive=True
    )
    require(alpha < 0.5, "alpha outside supported range")
    require(0.5 < power < 1.0, "power outside supported range")
    require(reduction_fraction < 1.0, "reduction fraction outside supported range")
    z_alpha = NormalDist().inv_cdf(1.0 - alpha)
    z_power = NormalDist().inv_cdf(power)
    log_effect = -math.log(1.0 - reduction_fraction)
    return int(math.ceil(((z_alpha + z_power) * paired_log_sd / log_effect) ** 2))


def validate_contract(contract: Mapping[str, Any]) -> None:
    require(contract.get("stage") == "V2-04G-P2-D2", "contract stage differs")
    require(
        contract.get("status") == "offline_single_factor_design_contract_frozen",
        "contract status differs",
    )
    require(contract.get("offline_only") is True, "contract is not offline-only")
    require(contract.get("formal_result") is False, "contract formal result differs")
    require(contract.get("runtime_ready") is False, "contract runtime readiness differs")
    require(contract.get("execution_ready") is False, "contract execution readiness differs")
    require(
        contract.get("execution_authorized") is False,
        "contract authorizes execution",
    )
    require(
        contract.get("evidence_budget_authorized") == 0,
        "contract evidence authorization differs",
    )
    require(
        contract.get("training_budget_steps_authorized") == 0,
        "contract training authorization differs",
    )
    require(
        contract.get("real_vehicle_use_forbidden") is True,
        "contract real-vehicle boundary differs",
    )
    bindings = contract.get("source_bindings")
    require(type(bindings) is dict and len(bindings) == 10, "source binding roster differs")

    factor = contract.get("single_executable_factor", {})
    require(factor.get("behavioral_field_count") == 1, "behavioral field count differs")
    require(
        factor.get("ros_runtime_field") == "TebLocalPlannerROS.max_vel_x",
        "runtime factor differs",
    )
    require(factor.get("typed_registry_field") == "max_vel_x", "typed factor differs")
    require(
        factor.get("calibration_grid_mps") == [1.2, 1.25, 1.3, 1.35, 1.4],
        "factor grid differs",
    )
    require(
        factor.get("all_other_behavioral_fields_byte_frozen_at_materialization") is True,
        "nonfactor freeze differs",
    )
    require(
        factor.get("runtime_scene_label_or_expected_status_access") is False,
        "runtime label firewall differs",
    )
    require(
        factor.get("overlay_bypass_or_safety_gate_disable_allowed") is False,
        "safety bypass boundary differs",
    )

    global_budget = contract.get("global_research_budget", {})
    rounds = global_budget.get("rounds")
    require(type(rounds) is list and len(rounds) == 3, "global rounds differ")
    require(
        sum(row["simulation_evidence_units_max"] for row in rounds)
        == global_budget.get("total_new_simulation_evidence_units_max")
        == 2200,
        "global evidence budget sum differs",
    )
    require(
        sum(row["training_environment_steps_max"] for row in rounds)
        == global_budget.get("total_new_training_environment_steps_max")
        == 120000,
        "global training budget sum differs",
    )
    alpha_sum = sum(finite(row["one_sided_alpha"], "round alpha") for row in rounds)
    require(close(alpha_sum, 0.05), "global alpha spend differs")
    require(close(global_budget.get("alpha_spend_sum"), 0.05), "declared alpha sum differs")
    require(
        global_budget.get("budget_expansion_or_transfer_between_rounds_allowed") is False,
        "budget transfer boundary differs",
    )
    require(
        global_budget.get("unused_alpha_or_budget_recycling_allowed") is False,
        "budget recycling boundary differs",
    )

    partitions = contract.get("round_1_partitions", {})
    calibration = partitions.get("calibration", {})
    require(calibration.get("block_count") == 30, "calibration block count differs")
    require(calibration.get("unique_method_count") == 11, "calibration method count differs")
    require(calibration.get("planned_evidence_units") == 330, "calibration budget differs")
    qualification = partitions.get("qualification", {})
    require(qualification.get("planned_evidence_units") == 12, "qualification budget differs")
    held_out = partitions.get("held_out_test", {})
    require(held_out.get("block_count") == 90, "held-out block count differs")
    require(held_out.get("maximum_planned_evidence_units") == 450, "held-out budget differs")
    require(
        330 + 12 + 450 == partitions.get("round_1_maximum_evidence_units") == 792,
        "round-one budget differs",
    )
    require(
        held_out.get("each_frozen_identity_attempt_limit") == 1,
        "held-out attempt limit differs",
    )
    require(
        held_out.get("retry_resume_replacement_or_seed_substitution_allowed") is False,
        "held-out retry firewall differs",
    )

    calibration_seeds: List[int] = []
    for role, row in calibration.get("scene_seed_blocks", {}).items():
        calibration_seeds.extend(_seed_range(row, "calibration {}".format(role)))
    qualification_seeds = _seed_range(qualification.get("seeds", {}), "qualification")
    compile_seeds = partitions.get("compile_support_only", {}).get("seeds")
    require(
        type(compile_seeds) is list
        and all(type(seed) is int for seed in compile_seeds),
        "compile seeds differ",
    )
    test_seeds: List[int] = []
    for role, row in held_out.get("scene_seed_blocks", {}).items():
        test_seeds.extend(_seed_range(row, "held-out {}".format(role)))
    _all_unique(
        calibration_seeds + qualification_seeds + compile_seeds + test_seeds,
        "round-one seed partitions",
    )
    require(calibration_seeds == list(range(5301, 5331)), "calibration seed range differs")
    require(qualification_seeds == list(range(5331, 5343)), "qualification seed range differs")
    require(test_seeds == list(range(5401, 5491)), "held-out seed range differs")

    statistics = contract.get("held_out_statistics", {})
    require(
        close(statistics.get("uncertainty", {}).get("one_sided_alpha"), 0.025),
        "round-one inferential alpha differs",
    )
    minimum_effect = statistics.get("minimum_effect", {})
    require(
        close(minimum_effect.get("point_reduction_fraction_at_least"), 0.10)
        and close(
            minimum_effect.get("one_sided_confidence_lower_fraction_at_least"),
            0.05,
        ),
        "minimum effect gate differs",
    )
    require(
        minimum_effect.get("required_for_both_primary_and_secondary_estimands") is True,
        "dual effect gate differs",
    )
    sample_size = statistics.get("sample_size", {})
    computed_pairs = approximate_required_pairs(
        statistics["uncertainty"]["one_sided_alpha"],
        sample_size["target_power"],
        sample_size["assumed_sd_paired_log_time"],
        sample_size["target_reduction_fraction"],
    )
    require(
        computed_pairs <= sample_size.get("approximate_required_pairs_max") == 22,
        "sample size rationale differs",
    )
    require(sample_size.get("paired_blocks_per_role") == 30, "paired blocks differ")

    positive = contract.get("positive_decision_rule", {})
    require(positive.get("all_conditions_required") is True, "positive conjunction differs")
    require(
        positive.get("unexplained_crash_or_integrity_failure_count_max") == 0,
        "integrity gate differs",
    )
    require(
        close(positive.get("clearance_difference_one_sided_lower_min_m"), -0.05),
        "clearance gate differs",
    )
    require(
        close(positive.get("normal_projection_rate_strictly_below"), 0.10),
        "projection gate differs",
    )
    require(
        positive.get("decision_if_any_condition_fails")
        == "obvious_performance_improvement_not_demonstrated",
        "negative decision wording differs",
    )
    forbidden = contract.get("forbidden_side_effects")
    require(
        type(forbidden) is dict and forbidden and all(value is True for value in forbidden.values()),
        "forbidden side-effect boundary differs",
    )


def build_outputs(
    workspace: Path,
) -> Tuple[Dict[str, Any], Dict[str, Any], List[Dict[str, str]]]:
    workspace = workspace.resolve()
    contract_payload = stable_read(workspace, CONTRACT_RELATIVE)
    contract = load_yaml(contract_payload, "P2-D2 contract")
    validate_contract(contract)

    source_documents: Dict[str, Dict[str, Any]] = {}
    source_entries: List[Dict[str, str]] = []
    for label, binding in contract["source_bindings"].items():
        relative, payload = verify_binding(workspace, binding, label)
        source_entries.append({"path": relative.as_posix(), "sha256": sha256_bytes(payload)})
        if relative.suffix in {".yaml", ".yml"}:
            source_documents[label] = load_yaml(payload, label)
    require(
        len(source_entries) == len({row["path"] for row in source_entries}) == 10,
        "source closure uniqueness differs",
    )
    source_entries.sort(key=lambda row: row["path"])

    d1 = source_documents["p2_d1_report"]
    require(
        d1.get("status") == "offline_trace_attribution_physical_room_pass",
        "P2-D1 status differs",
    )
    require(d1.get("formal_result") is False, "P2-D1 formal boundary differs")
    require(d1.get("execution_authorized") is False, "P2-D1 authorization differs")
    require(
        d1.get("decision", {}).get("factor_hypothesis_retained_for_design") is True,
        "P2-D1 factor decision differs",
    )
    require(
        d1.get("decision", {}).get("specific_executable_runtime_factor_qualified") is False,
        "P2-D1 runtime qualification boundary differs",
    )
    d1_target = finite(
        d1["target_closure"]["10percent"]["first_order_implied_max_vel_x_mps"],
        "P2-D1 ten-percent implied speed",
        positive=True,
    )
    d1_envelope_lower = finite(
        d1["same_path_longitudinal_envelopes"]["frozen_prefix_post_cruise_primary"][
            "paired_bootstrap_one_sided_95_percent_lower"
        ],
        "P2-D1 envelope lower",
    )
    require(d1_envelope_lower > 0.10, "P2-D1 physical-room lower bound differs")

    bank = source_documents["circle_contact_anchor_bank"]
    require(bank.get("runtime_ready") is False, "source bank runtime readiness differs")
    require(bank.get("training_allowed") is False, "source bank training boundary differs")
    require(bank.get("parameter_write_enabled") is False, "source bank write boundary differs")
    max_vel_parameter = _parameter(bank, "max_vel_x")
    require(max_vel_parameter.get("type") == "double", "max_vel_x type differs")
    require(max_vel_parameter.get("lifecycle") == "fast_continuous", "max_vel_x lifecycle differs")
    require(max_vel_parameter.get("bounds") == [0.1, 1.2], "current max_vel_x bounds differ")
    max_vel_rate = finite(max_vel_parameter.get("max_rate_per_s"), "max_vel_x rate", positive=True)
    require(close(max_vel_rate, 0.5), "max_vel_x rate differs")
    cruise = bank.get("anchors", {}).get("anchor_cruise", {}).get("values")
    require(type(cruise) is dict, "anchor_cruise values differ")
    current_speed = finite(cruise.get("max_vel_x"), "current cruise speed", positive=True)
    cruise_yaw = finite(cruise.get("max_vel_theta"), "cruise yaw", positive=True)
    cruise_acceleration = finite(cruise.get("acc_lim_x"), "cruise acceleration", positive=True)
    require(close(current_speed, 1.2), "current cruise speed differs")
    require(close(cruise_yaw, 0.65), "current cruise yaw differs")
    require(close(cruise_acceleration, 0.8), "current cruise acceleration differs")
    turning_radius = finite(
        bank.get("vehicle", {}).get("minimum_turning_radius_m"),
        "minimum turning radius",
        positive=True,
    )
    require(close(turning_radius, 1.2), "minimum turning radius differs")
    transaction = bank.get("transaction", {})
    decision_frequency = finite(
        transaction.get("decision_frequency_hz"),
        "decision frequency",
        positive=True,
    )
    require(close(decision_frequency, 5.0), "decision frequency differs")
    transaction_latency = sum(
        finite(transaction.get(field), field)
        for field in ("ack_delay_s", "readback_delay_s", "activation_delay_s")
    )

    simulation = source_documents["simulation_contract"]
    require(simulation.get("simulation_only") is True, "simulation-only boundary differs")
    require(simulation.get("runtime_ready") is False, "simulation runtime boundary differs")
    actuator = simulation.get("actuator", {})
    command = simulation.get("command_transport", {})
    laser = simulation.get("laser_transport", {})
    service_deceleration = finite(
        actuator.get("max_deceleration_mps2"),
        "service deceleration",
        positive=True,
    )
    actuator_tau = finite(
        actuator.get("speed_time_constant_s"), "actuator speed time constant", positive=True
    )
    require(
        finite(actuator.get("max_acceleration_mps2"), "sim acceleration", positive=True)
        >= cruise_acceleration,
        "frozen acceleration exceeds simulator contract",
    )
    command_normal_delay = finite(command.get("delay_s"), "command delay") + finite(
        command.get("jitter_s"), "command jitter"
    )
    command_timeout = finite(command.get("timeout_s"), "command timeout", positive=True)
    laser_normal_delay = finite(laser.get("delay_s"), "laser delay") + finite(
        laser.get("jitter_s"), "laser jitter"
    )

    action_contract = source_documents["action_pipeline_contract"]
    require(
        action_contract.get("rule_closed_loop", {}).get("runtime_scene_labels_allowed") is False,
        "action pipeline label firewall differs",
    )
    require(
        action_contract.get("action_decoder", {})
        .get("constraints", {})
        .get("yaw_coupling")
        == "max_vel_theta_lte_max_vel_x_over_min_turning_radius",
        "Ackermann coupling differs",
    )
    require(
        close(
            action_contract.get("acceptance_gates", {}).get("normal_projection_rate_max"),
            0.10,
        ),
        "action projection gate differs",
    )

    t08 = source_documents["t08_baseline_registry"]
    teb_default = _algorithm(t08, "TEB-Default")
    teb_tuned = _algorithm(t08, "TEB-Tuned")
    _numeric_mapping_equal(teb_default.get("theta"), EXPECTED_TEB_DEFAULT, "TEB-Default")
    _numeric_mapping_equal(teb_tuned.get("theta"), EXPECTED_TEB_TUNED_V1, "TEB-Tuned-V1")
    require(
        teb_tuned.get("derivation")
        == "t07_calibration_midpoint_engineering_baseline_not_claimed_optimal",
        "TEB-Tuned-V1 provenance differs",
    )
    fixed = source_documents["fixed_teb_reference"].get("TebLocalPlannerROS")
    _numeric_mapping_equal(fixed, EXPECTED_TEB_DEFAULT, "fixed TEB reference")

    factor = contract["single_executable_factor"]
    design_upper = finite(
        factor["proposed_design_upper_bound_mps"], "design upper speed", positive=True
    )
    margin = finite(
        contract["offline_safety_envelope"]["additional_distance_margin_m"],
        "distance margin",
    )
    transition_latency = (design_upper - current_speed) / max_vel_rate
    decision_period = 1.0 / decision_frequency
    normal_latency = (
        laser_normal_delay
        + decision_period
        + command_normal_delay
        + actuator_tau
        + transaction_latency
        + transition_latency
    )
    fault_latency = normal_latency + command_timeout
    normal_envelope = stopping_envelope(
        design_upper, normal_latency, service_deceleration, margin
    )
    fault_envelope = stopping_envelope(
        design_upper, fault_latency, service_deceleration, margin
    )

    grid = [finite(value, "candidate speed", positive=True) for value in factor["calibration_grid_mps"]]
    circle_ackermann = [
        {
            "speed_mps": rounded(speed),
            "maximum_yaw_from_minimum_radius_radps": rounded(speed / turning_radius),
            "frozen_cruise_max_vel_theta_radps": rounded(cruise_yaw),
            "feasible": cruise_yaw <= speed / turning_radius + 1e-12,
        }
        for speed in grid
    ]
    fixed_yaw = finite(fixed["max_vel_theta"], "fixed TEB max yaw", positive=True)
    fixed_ackermann = [
        {
            "speed_mps": rounded(speed),
            "maximum_yaw_from_minimum_radius_radps": rounded(speed / turning_radius),
            "fixed_max_vel_theta_radps": rounded(fixed_yaw),
            "feasible": fixed_yaw <= speed / turning_radius + 1e-12,
        }
        for speed in grid
    ]
    require(all(row["feasible"] for row in circle_ackermann), "circle candidate Ackermann gate fails")
    require(all(row["feasible"] for row in fixed_ackermann), "fixed candidate Ackermann gate fails")
    require(d1_target <= design_upper, "design upper speed does not contain P2-D1 target")

    statistics = contract["held_out_statistics"]
    sample_size = statistics["sample_size"]
    required_pairs = approximate_required_pairs(
        statistics["uncertainty"]["one_sided_alpha"],
        sample_size["target_power"],
        sample_size["assumed_sd_paired_log_time"],
        sample_size["target_reduction_fraction"],
    )

    amendment: Dict[str, Any] = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": "V2-04G-P2-D2",
        "amendment_id": "p2_cruise_forward_velocity_ceiling_v1",
        "status": "offline_proposed_single_field_amendment_not_materialized",
        "simulation_only": True,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "parameter_write_enabled": False,
        "training_allowed": False,
        "real_vehicle_use_forbidden": True,
        "source_bank": contract["source_bindings"]["circle_contact_anchor_bank"],
        "behavioral_diff_contract": {
            "behavioral_field_count": 1,
            "runtime_field": "TebLocalPlannerROS.max_vel_x",
            "typed_registry_field": "max_vel_x",
            "fam_teb_field": "anchors.anchor_cruise.values.max_vel_x",
            "current_control_value_mps": rounded(current_speed),
            "candidate_values_mps": [rounded(value) for value in grid[1:]],
            "proposed_design_bound_mps": [0.1, rounded(design_upper)],
            "registry_bound_change_is_nonbehavioral_metadata": True,
            "all_nonfactor_values_must_match_source_bank": True,
            "runtime_scene_labels_forbidden": True,
            "frozen_overlay_scaling_applies": True,
        },
        "materialization_diff_allowlist": [
            {
                "path": "parameters[name=max_vel_x].bounds[1]",
                "from": 1.2,
                "to": rounded(design_upper),
                "kind": "registry_metadata",
            },
            {
                "path": "anchors.anchor_cruise.values.max_vel_x",
                "from": 1.2,
                "to": "one_frozen_value_from_[1.25,1.30,1.35,1.40]",
                "kind": "sole_behavioral_field",
            },
        ],
        "frozen_execution_constraints": {
            "max_rate_per_s": rounded(max_vel_rate),
            "acc_lim_x_mps2": rounded(cruise_acceleration),
            "minimum_turning_radius_m": rounded(turning_radius),
            "normal_projection_rate_strictly_below": 0.10,
            "transaction_origin": "previous_executed",
            "atomic_failure_behavior": "hold_previous_executed",
            "safety_gate_bypass_allowed": False,
        },
        "offline_derived_envelopes": {
            "normal_transition": normal_envelope,
            "command_timeout_fault": fault_envelope,
            "circle_ackermann_grid": circle_ackermann,
            "fixed_teb_ackermann_grid": fixed_ackermann,
        },
        "qualification_boundary": {
            "new_bound_safety_qualified": False,
            "future_component_probes_required": contract["offline_safety_envelope"][
                "future_qualification_requirements"
            ],
            "active_runtime_profile_modified": False,
        },
    }
    amendment_payload = canonical_yaml_bytes(amendment)

    rounds = contract["global_research_budget"]["rounds"]
    report: Dict[str, Any] = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": "V2-04G-P2-D2",
        "report_id": "fam_teb_v2_04g_p2_d2_offline_single_factor_design_1",
        "status": "offline_single_factor_design_closure_pass",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "training_budget_steps_authorized": 0,
        "real_vehicle_used": False,
        "contract": {
            "path": CONTRACT_RELATIVE.as_posix(),
            "sha256": sha256_bytes(contract_payload),
        },
        "source_integrity": {
            "bound_paths_rehashed": len(source_entries),
            "unique_bound_paths": len({row["path"] for row in source_entries}),
            "integrity_failures": [],
        },
        "p2_d1_entry": {
            "frozen_status": d1["status"],
            "ten_percent_implied_max_vel_x_mps": d1_target,
            "same_path_envelope_one_sided_lower_fraction": d1_envelope_lower,
            "physical_room_only_not_achieved_gain": True,
            "p1_negative_result_preserved": True,
        },
        "selected_design_factor": {
            "factor_id": factor["factor_id"],
            "runtime_field": factor["ros_runtime_field"],
            "behavioral_field_count": 1,
            "control_value_mps": rounded(current_speed),
            "candidate_grid_mps": [rounded(value) for value in grid[1:]],
            "design_upper_bound_mps": rounded(design_upper),
            "all_other_behavioral_fields_frozen": True,
            "runtime_label_access": False,
            "selection_uses_calibration_only": True,
            "held_out_test_used_for_selection": False,
        },
        "offline_mechanical_screen": {
            "status": "design_admissible_future_qualification_required",
            "normal_transition_envelope": normal_envelope,
            "command_timeout_fault_envelope": fault_envelope,
            "circle_ackermann_all_grid_levels_feasible": True,
            "fixed_teb_ackermann_all_grid_levels_feasible": True,
            "simulator_acceleration_supports_frozen_limit": True,
            "p2_d1_ten_percent_implied_speed_inside_design_bound": True,
            "new_bound_safety_qualified": False,
        },
        "fair_baseline_firewall": {
            "transparent_reference": {
                "method_id": "TEB-Default",
                "theta": EXPECTED_TEB_DEFAULT,
                "primary_baseline_eligible": False,
            },
            "required_fixed_baseline_candidates": {
                "TEB-Tuned-V1": {
                    "theta": EXPECTED_TEB_TUNED_V1,
                    "provenance": teb_tuned["derivation"],
                },
                "independently_tuned_fixed_teb": {
                    "source_config": contract["source_bindings"]["fixed_teb_reference"],
                    "only_tuned_field": "TebLocalPlannerROS.max_vel_x",
                    "grid_mps": [rounded(value) for value in grid],
                },
            },
            "primary_baseline_rule": contract["factor_selection"]["primary_fair_baseline"],
            "baseline_weakening_or_test_selection_allowed": False,
        },
        "finite_program": {
            "maximum_factor_rounds": 3,
            "global_simulation_evidence_units_max": sum(
                row["simulation_evidence_units_max"] for row in rounds
            ),
            "global_training_environment_steps_max": sum(
                row["training_environment_steps_max"] for row in rounds
            ),
            "global_one_sided_alpha_spend": rounded(
                sum(row["one_sided_alpha"] for row in rounds)
            ),
            "round_1": {
                "training_steps": 0,
                "calibration_evidence_units": 330,
                "qualification_evidence_units": 12,
                "held_out_test_evidence_units_max": 450,
                "total_evidence_units_max": 792,
                "one_sided_alpha": 0.025,
                "calibration_seeds": [5301, 5330],
                "qualification_seeds": [5331, 5342],
                "held_out_test_seeds": [5401, 5490],
                "test_attempts_per_identity": 1,
            },
            "unused_budget_or_alpha_recycled": False,
            "stop_after_first_full_positive_gate": True,
        },
        "held_out_claim_gate": {
            "primary_comparator": "calibration_frozen_strongest_safe_of_TEB-Tuned-V1_and_independently_tuned_fixed_TEB",
            "primary_point_reduction_min": 0.10,
            "primary_one_sided_lower_reduction_min": 0.05,
            "circle_v120_causal_point_reduction_min": 0.10,
            "circle_v120_causal_one_sided_lower_reduction_min": 0.05,
            "one_sided_alpha": 0.025,
            "approximate_required_pairs_per_role": required_pairs,
            "selected_pairs_per_role": sample_size["paired_blocks_per_role"],
            "success_collision_emergency_noninferiority_required": True,
            "clearance_one_sided_lower_min_m": -0.05,
            "normal_projection_rate_strictly_below": 0.10,
            "unexplained_crash_or_integrity_failure_count_max": 0,
            "all_gates_conjunctive": True,
            "threshold_relaxation_after_outcome_allowed": False,
        },
        "decision": {
            "exactly_one_executable_factor_selected": True,
            "fair_primary_baseline_rule_frozen": True,
            "finite_global_budget_and_alpha_spend_frozen": True,
            "fresh_split_and_one_time_test_firewall_frozen": True,
            "candidate_bank_materialized": False,
            "new_speed_bound_safety_qualified": False,
            "simulation_release_exists": False,
            "simulation_authorized": False,
            "performance_improvement_demonstrated": False,
            "reinforcement_learning_improvement_demonstrated": False,
            "next_safe_stage": "independent_future_round1_bank_runner_scene_and_exact_schedule_materialization_review",
        },
        "claim_boundary": contract["claim_boundary"],
        "side_effect_audit": {
            "ros_gazebo_move_base_started": False,
            "simulation_or_training_started": False,
            "execution_release_schedule_journal_or_consumption_created": False,
            "active_runtime_profile_or_online_parameter_modified": False,
            "p1_i5_t12_or_stable_workspace_modified": False,
            "held_out_test_accessed": False,
            "real_vehicle_can_or_hardware_connected": False,
        },
        "artifacts": {
            "candidate_bank_amendment": {
                "path": AMENDMENT_RELATIVE.as_posix(),
                "sha256": sha256_bytes(amendment_payload),
            }
        },
    }
    return report, amendment, source_entries


def canonical_yaml_bytes(document: Mapping[str, Any]) -> bytes:
    return yaml.safe_dump(
        dict(document), sort_keys=False, allow_unicode=True, width=100
    ).encode("utf-8")


def report_bytes(report: Mapping[str, Any]) -> bytes:
    return canonical_yaml_bytes(report)


def amendment_bytes(amendment: Mapping[str, Any]) -> bytes:
    return canonical_yaml_bytes(amendment)
