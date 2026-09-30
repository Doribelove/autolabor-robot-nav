#!/usr/bin/env python3
"""Independent pure-offline review of the V2-04G-P2-D2 design closure."""

import argparse
import hashlib
import importlib.util
import math
import os
from pathlib import Path
import re
import stat
from statistics import NormalDist
import sys
import tempfile

import yaml


WORKSPACE_DEFAULT = Path("/home/robot/robot_ws_base_rl")
MODULE_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p2_d2.py"
)
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_d2_offline_single_factor_design_contract.yaml"
)
ARTIFACT_ROOT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_d2_single_factor_design"
)
AMENDMENT_RELATIVE = ARTIFACT_ROOT_RELATIVE / "v2_04g_p2_d2_candidate_bank_amendment.yaml"
REPORT_RELATIVE = ARTIFACT_ROOT_RELATIVE / "v2_04g_p2_d2_design_report.yaml"
SOURCE_FREEZE_RELATIVE = ARTIFACT_ROOT_RELATIVE / "p2_d2_source_evidence_freeze.sha256"
REVIEW_RELATIVE = ARTIFACT_ROOT_RELATIVE / "v2_04g_p2_d2_offline_review.yaml"
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")

EXPECTED_DEFAULT = {
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
EXPECTED_TUNED = {
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


class ReviewError(ValueError):
    """Raised when the independent P2-D2 review fails closed."""


def require(condition, message):
    if not condition:
        raise ReviewError(message)


def safe_relative(value):
    relative = value if isinstance(value, Path) else Path(value)
    require(
        not relative.is_absolute() and relative.parts and ".." not in relative.parts,
        "unsafe relative path",
    )
    return relative


def stable_read(workspace, relative):
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
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
        require(
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
            "file changed during review: {}".format(relative),
        )
        payload = b"".join(chunks)
        require(len(payload) == before.st_size, "short read: {}".format(relative))
        return payload
    finally:
        os.close(descriptor)


def sha256(payload):
    return hashlib.sha256(payload).hexdigest()


def load_yaml(payload, label):
    try:
        value = yaml.safe_load(payload.decode("utf-8"))
    except Exception as exc:
        raise ReviewError("{} parse failed: {}".format(label, exc))
    require(type(value) is dict, "{} is not a mapping".format(label))
    return value


def finite(value, label):
    require(type(value) in (int, float) and not isinstance(value, bool), "{} is not numeric".format(label))
    number = float(value)
    require(math.isfinite(number), "{} is not finite".format(label))
    return number


def close(left, right, tolerance=1e-12):
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=tolerance)


def load_design_module(workspace):
    path = workspace / MODULE_RELATIVE
    spec = importlib.util.spec_from_file_location("p2_d2_review_rebuild", path)
    require(spec is not None and spec.loader is not None, "cannot load P2-D2 module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def verify_source_freeze(workspace, payload):
    entries = []
    for line in payload.decode("utf-8").splitlines():
        if not line.strip():
            continue
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        require(match is not None, "source freeze line differs")
        expected, path_text = match.groups()
        require(HEX_SHA256.fullmatch(expected) is not None, "source freeze SHA differs")
        relative = safe_relative(path_text)
        require(
            sha256(stable_read(workspace, relative)) == expected,
            "source freeze target differs: {}".format(relative),
        )
        entries.append({"path": relative.as_posix(), "sha256": expected})
    require(len(entries) == 11, "source freeze count differs")
    require(len({entry["path"] for entry in entries}) == 11, "source freeze duplicate")
    require(
        entries == sorted(entries, key=lambda entry: entry["path"]),
        "source freeze order differs",
    )
    return entries


def algorithm(registry, name):
    matches = [
        row
        for row in registry.get("algorithms", [])
        if type(row) is dict and row.get("name") == name
    ]
    require(len(matches) == 1, "baseline {} not unique".format(name))
    return matches[0]


def numeric_subset(actual, expected, label):
    require(type(actual) is dict, "{} is not a mapping".format(label))
    for key, expected_value in expected.items():
        require(key in actual, "{}.{} missing".format(label, key))
        require(
            close(finite(actual[key], "{}.{}".format(label, key)), expected_value),
            "{}.{} differs".format(label, key),
        )


def bank_parameter(bank, name):
    matches = [
        row
        for row in bank.get("parameters", [])
        if type(row) is dict and row.get("name") == name
    ]
    require(len(matches) == 1, "bank parameter {} not unique".format(name))
    return matches[0]


def independent_review(workspace):
    contract_payload = stable_read(workspace, CONTRACT_RELATIVE)
    amendment_payload = stable_read(workspace, AMENDMENT_RELATIVE)
    report_payload = stable_read(workspace, REPORT_RELATIVE)
    freeze_payload = stable_read(workspace, SOURCE_FREEZE_RELATIVE)
    contract = load_yaml(contract_payload, "contract")
    amendment = load_yaml(amendment_payload, "amendment")
    report = load_yaml(report_payload, "report")
    source_entries = verify_source_freeze(workspace, freeze_payload)

    require(contract.get("stage") == "V2-04G-P2-D2", "contract stage differs")
    require(contract.get("offline_only") is True, "contract is not offline-only")
    require(contract.get("execution_authorized") is False, "contract authorizes execution")
    require(contract.get("evidence_budget_authorized") == 0, "contract evidence authority differs")
    require(
        contract.get("training_budget_steps_authorized") == 0,
        "contract training authority differs",
    )
    require(
        all(contract.get("forbidden_side_effects", {}).values()),
        "contract side-effect boundary differs",
    )
    require(
        report.get("status") == "offline_single_factor_design_closure_pass",
        "report status differs",
    )
    require(
        report.get("contract")
        == {"path": CONTRACT_RELATIVE.as_posix(), "sha256": sha256(contract_payload)},
        "report contract binding differs",
    )
    require(
        report.get("artifacts", {}).get("candidate_bank_amendment")
        == {"path": AMENDMENT_RELATIVE.as_posix(), "sha256": sha256(amendment_payload)},
        "report amendment binding differs",
    )
    require(
        amendment.get("status")
        == "offline_proposed_single_field_amendment_not_materialized",
        "amendment status differs",
    )
    diff = amendment.get("behavioral_diff_contract", {})
    require(diff.get("behavioral_field_count") == 1, "behavioral field count differs")
    require(
        diff.get("runtime_field") == "TebLocalPlannerROS.max_vel_x",
        "runtime field differs",
    )
    require(
        diff.get("candidate_values_mps") == [1.25, 1.3, 1.35, 1.4],
        "candidate values differ",
    )
    require(
        diff.get("all_nonfactor_values_must_match_source_bank") is True,
        "nonfactor freeze differs",
    )
    allowlist = amendment.get("materialization_diff_allowlist")
    require(type(allowlist) is list and len(allowlist) == 2, "diff allowlist differs")
    require(
        [row.get("kind") for row in allowlist]
        == ["registry_metadata", "sole_behavioral_field"],
        "diff allowlist kinds differ",
    )
    require(amendment.get("parameter_write_enabled") is False, "amendment enables writes")
    require(amendment.get("execution_authorized") is False, "amendment authorizes execution")

    bindings = contract["source_bindings"]
    documents = {}
    for label in (
        "p2_d1_report",
        "circle_contact_anchor_bank",
        "simulation_contract",
        "action_pipeline_contract",
        "t08_baseline_registry",
        "fixed_teb_reference",
    ):
        binding = bindings[label]
        payload = stable_read(workspace, binding["path"])
        require(sha256(payload) == binding["sha256"], "{} binding differs".format(label))
        documents[label] = load_yaml(payload, label)

    d1 = documents["p2_d1_report"]
    require(
        d1.get("status") == "offline_trace_attribution_physical_room_pass",
        "P2-D1 entry differs",
    )
    implied_speed = finite(
        d1["target_closure"]["10percent"]["first_order_implied_max_vel_x_mps"],
        "P2-D1 implied speed",
    )
    require(implied_speed <= 1.4, "P2-D1 implied speed outside amendment")
    require(
        d1.get("decision", {}).get("causal_performance_improvement_demonstrated")
        is False,
        "P2-D1 causal boundary differs",
    )

    bank = documents["circle_contact_anchor_bank"]
    max_vel = bank_parameter(bank, "max_vel_x")
    require(max_vel.get("bounds") == [0.1, 1.2], "source bound differs")
    rate = finite(max_vel.get("max_rate_per_s"), "max_vel_x rate")
    require(close(rate, 0.5), "max_vel_x rate differs")
    cruise = bank["anchors"]["anchor_cruise"]["values"]
    current_speed = finite(cruise["max_vel_x"], "current speed")
    acceleration = finite(cruise["acc_lim_x"], "current acceleration")
    circle_yaw = finite(cruise["max_vel_theta"], "circle yaw")
    turning_radius = finite(bank["vehicle"]["minimum_turning_radius_m"], "turning radius")
    transaction = bank["transaction"]
    decision_period = 1.0 / finite(
        transaction["decision_frequency_hz"], "decision frequency"
    )
    transaction_latency = sum(
        finite(transaction[name], name)
        for name in ("ack_delay_s", "readback_delay_s", "activation_delay_s")
    )

    simulation = documents["simulation_contract"]
    actuator = simulation["actuator"]
    command = simulation["command_transport"]
    laser = simulation["laser_transport"]
    require(
        finite(actuator["max_acceleration_mps2"], "sim acceleration") >= acceleration,
        "sim acceleration support differs",
    )
    service_deceleration = finite(
        actuator["max_deceleration_mps2"], "service deceleration"
    )
    transition_latency = (1.4 - current_speed) / rate
    normal_latency = (
        finite(laser["delay_s"], "laser delay")
        + finite(laser["jitter_s"], "laser jitter")
        + decision_period
        + finite(command["delay_s"], "command delay")
        + finite(command["jitter_s"], "command jitter")
        + finite(actuator["speed_time_constant_s"], "speed time constant")
        + transaction_latency
        + transition_latency
    )
    fault_latency = normal_latency + finite(command["timeout_s"], "command timeout")
    braking_distance = 1.4 * 1.4 / (2.0 * service_deceleration)
    normal_required = 1.4 * normal_latency + braking_distance + 0.25
    fault_required = 1.4 * fault_latency + braking_distance + 0.25
    mechanical = report["offline_mechanical_screen"]
    require(close(normal_latency, 0.99), "normal latency differs")
    require(close(fault_latency, 1.49), "fault latency differs")
    require(
        close(
            mechanical["normal_transition_envelope"]["required_forward_free_path_m"],
            normal_required,
        ),
        "normal stopping envelope differs",
    )
    require(
        close(
            mechanical["command_timeout_fault_envelope"][
                "required_forward_free_path_m"
            ],
            fault_required,
        ),
        "fault stopping envelope differs",
    )
    require(mechanical.get("new_bound_safety_qualified") is False, "safety overclaim")
    for speed in (1.2, 1.25, 1.3, 1.35, 1.4):
        require(
            circle_yaw <= speed / turning_radius + 1e-12,
            "circle Ackermann check fails",
        )

    action = documents["action_pipeline_contract"]
    require(
        action["rule_closed_loop"]["runtime_scene_labels_allowed"] is False,
        "runtime label firewall differs",
    )
    require(
        action["action_decoder"]["constraints"]["yaw_coupling"]
        == "max_vel_theta_lte_max_vel_x_over_min_turning_radius",
        "action Ackermann coupling differs",
    )
    require(
        close(action["acceptance_gates"]["normal_projection_rate_max"], 0.10),
        "action projection gate differs",
    )

    registry = documents["t08_baseline_registry"]
    default = algorithm(registry, "TEB-Default")
    tuned = algorithm(registry, "TEB-Tuned")
    numeric_subset(default["theta"], EXPECTED_DEFAULT, "TEB-Default")
    numeric_subset(tuned["theta"], EXPECTED_TUNED, "TEB-Tuned-V1")
    fixed = documents["fixed_teb_reference"]["TebLocalPlannerROS"]
    numeric_subset(fixed, EXPECTED_DEFAULT, "fixed TEB")
    baseline = report["fair_baseline_firewall"]
    require(
        baseline["required_fixed_baseline_candidates"]["TEB-Tuned-V1"]["theta"]
        == EXPECTED_TUNED,
        "TEB-Tuned-V1 report values differ",
    )
    require(
        baseline["baseline_weakening_or_test_selection_allowed"] is False,
        "baseline fairness boundary differs",
    )

    budget = contract["global_research_budget"]
    rounds = budget["rounds"]
    require(
        sum(row["simulation_evidence_units_max"] for row in rounds) == 2200,
        "global simulation budget differs",
    )
    require(
        sum(row["training_environment_steps_max"] for row in rounds) == 120000,
        "global training budget differs",
    )
    require(
        close(sum(row["one_sided_alpha"] for row in rounds), 0.05),
        "global alpha spend differs",
    )
    partitions = contract["round_1_partitions"]
    require(
        partitions["calibration"]["planned_evidence_units"] == 30 * 11 == 330,
        "calibration evidence count differs",
    )
    require(
        partitions["qualification"]["planned_evidence_units"] == 4 * 3 == 12,
        "qualification evidence count differs",
    )
    require(
        partitions["held_out_test"]["maximum_planned_evidence_units"]
        == 90 * 5
        == 450,
        "test evidence count differs",
    )
    require(
        330 + 12 + 450 == partitions["round_1_maximum_evidence_units"] == 792,
        "round-one budget differs",
    )
    require(
        partitions["held_out_test"]["each_frozen_identity_attempt_limit"] == 1,
        "test attempt limit differs",
    )
    require(
        partitions["held_out_test"][
            "retry_resume_replacement_or_seed_substitution_allowed"
        ]
        is False,
        "test retry firewall differs",
    )

    sample = contract["held_out_statistics"]["sample_size"]
    alpha = contract["held_out_statistics"]["uncertainty"]["one_sided_alpha"]
    log_effect = -math.log(1.0 - sample["target_reduction_fraction"])
    required_pairs = math.ceil(
        (
            (
                NormalDist().inv_cdf(1.0 - alpha)
                + NormalDist().inv_cdf(sample["target_power"])
            )
            * sample["assumed_sd_paired_log_time"]
            / log_effect
        )
        ** 2
    )
    require(required_pairs == 22, "independent power calculation differs")
    gate = report["held_out_claim_gate"]
    require(
        gate["primary_point_reduction_min"] == 0.10
        and gate["primary_one_sided_lower_reduction_min"] == 0.05,
        "primary effect gate differs",
    )
    require(
        gate["circle_v120_causal_point_reduction_min"] == 0.10
        and gate["circle_v120_causal_one_sided_lower_reduction_min"] == 0.05,
        "causal effect gate differs",
    )
    require(
        gate["approximate_required_pairs_per_role"] == required_pairs,
        "reported power calculation differs",
    )
    require(gate["all_gates_conjunctive"] is True, "claim gates are not conjunctive")

    module_source = stable_read(workspace, MODULE_RELATIVE).decode("utf-8")
    require("import rospy" not in module_source, "design module imports ROS")
    require("import roslaunch" not in module_source, "design module imports roslaunch")
    require("import subprocess" not in module_source, "design module can spawn processes")
    module = load_design_module(workspace)
    rebuilt_report, rebuilt_amendment, rebuilt_entries = module.build_outputs(workspace)
    require(
        module.report_bytes(rebuilt_report) == report_payload,
        "deterministic report rebuild differs",
    )
    require(
        module.amendment_bytes(rebuilt_amendment) == amendment_payload,
        "deterministic amendment rebuild differs",
    )
    frozen_noncontract_entries = [
        entry
        for entry in source_entries
        if entry["path"] != CONTRACT_RELATIVE.as_posix()
    ]
    require(
        rebuilt_entries == frozen_noncontract_entries,
        "rebuilt source closure differs",
    )
    require(report["side_effect_audit"] == {
        "ros_gazebo_move_base_started": False,
        "simulation_or_training_started": False,
        "execution_release_schedule_journal_or_consumption_created": False,
        "active_runtime_profile_or_online_parameter_modified": False,
        "p1_i5_t12_or_stable_workspace_modified": False,
        "held_out_test_accessed": False,
        "real_vehicle_can_or_hardware_connected": False,
    }, "side-effect audit differs")

    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": "V2-04G-P2-D2",
        "review_id": "fam_teb_v2_04g_p2_d2_independent_offline_review_1",
        "status": "offline_single_factor_design_independent_review_pass",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "training_budget_steps_authorized": 0,
        "integrity_review": {
            "source_paths_rehashed": len(source_entries),
            "source_freeze_sha256": sha256(freeze_payload),
            "contract_sha256": sha256(contract_payload),
            "amendment_sha256": sha256(amendment_payload),
            "report_sha256": sha256(report_payload),
            "deterministic_report_rebuild_byte_equal": True,
            "deterministic_amendment_rebuild_byte_equal": True,
            "integrity_failures": [],
        },
        "independent_result": {
            "behavioral_field_count": 1,
            "runtime_field": "TebLocalPlannerROS.max_vel_x",
            "candidate_grid_mps": [1.25, 1.3, 1.35, 1.4],
            "normal_transition_required_forward_free_path_m": round(normal_required, 12),
            "command_timeout_fault_required_forward_free_path_m": round(
                fault_required, 12
            ),
            "ackermann_grid_feasible": True,
            "p2_d1_implied_ten_percent_speed_inside_design_bound": True,
            "global_simulation_evidence_units_max": 2200,
            "global_training_environment_steps_max": 120000,
            "global_alpha_spend": 0.05,
            "round_1_evidence_units_max": 792,
            "required_pairs_per_role": required_pairs,
        },
        "fairness_review": {
            "teb_default_transparent_reference_present": True,
            "teb_tuned_v1_exact_values_verified": True,
            "independent_fixed_teb_calibration_grid_present": True,
            "strongest_safe_baseline_selected_on_calibration_only": True,
            "held_out_test_baseline_selection_forbidden": True,
            "intentional_baseline_weakening_detected": False,
        },
        "claim_review": {
            "offline_design_closure_only": True,
            "new_speed_bound_safety_qualified": False,
            "performance_improvement_demonstrated": False,
            "reinforcement_learning_improvement_demonstrated": False,
            "simulation_release_exists": False,
            "simulation_authorized": False,
        },
        "side_effect_audit": report["side_effect_audit"],
    }


def review_bytes(review):
    return yaml.safe_dump(
        review, sort_keys=False, allow_unicode=True, width=100
    ).encode("utf-8")


def atomic_write(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=".{}-".format(path.name), dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE_DEFAULT)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check-only", action="store_true")
    arguments = parser.parse_args()
    workspace = arguments.workspace.resolve()
    review = independent_review(workspace)
    payload = review_bytes(review)
    if arguments.write:
        atomic_write(workspace / REVIEW_RELATIVE, payload)
    else:
        require(
            stable_read(workspace, REVIEW_RELATIVE) == payload,
            "review report is not byte-equal to deterministic rebuild",
        )
    print(review["status"])
    print(
        "source_paths={}".format(
            review["integrity_review"]["source_paths_rehashed"]
        )
    )
    print("review_sha256={}".format(sha256(payload)))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ReviewError, RuntimeError, ValueError) as exc:
        print("p2_d2_offline_review_failed: {}".format(exc), file=sys.stderr)
        raise SystemExit(1)
