#!/usr/bin/env python3
"""Build the pure-offline R16-D1 execution failure attribution."""

import argparse
import hashlib
import math
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r16_d1_execution_diagnosis_contract.yaml"
)
R16_CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r16_actual_teb_paired_preflight_contract.yaml"
)
ASSESSMENT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r16_assessment/"
    "paired_preflight_assessment.yaml"
)
FREEZE = Path(
    "artifacts/v3/multiactor/v3_13_q1_r16_assessment/"
    "evidence_freeze.yaml"
)
R16_REVIEW = Path(
    "artifacts/v3/multiactor/v3_13_q1_r16_assessment/"
    "independent_result_review.yaml"
)
REPORT = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r16_actual_teb_preflight/episodes/"
    "002_01_7666_v3_13_analytic/episode_report.yaml"
)
RECEIPT = REPORT.parent / "matrix_receipt.yaml"
EXECUTION_CORE = Path(
    "src/application/v3_scope_planner/src/v3_scope_planner/"
    "execution_core.py"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r16_d1_diagnosis"
)
DIAGNOSIS = ROOT / "execution_failure_attribution.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r16_d1_execution_diagnosis.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r16_d1_execution_diagnosis.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/"
    "test_v3_13_q1_r16_d1_execution_diagnosis.py"
)
RESOURCES = [
    CONTRACT,
    R16_CONTRACT,
    ASSESSMENT,
    FREEZE,
    R16_REVIEW,
    REPORT,
    RECEIPT,
    EXECUTION_CORE,
    BUILDER,
    REVIEWER,
    TEST,
]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(path):
    target = W / path
    require(target.is_file(), "required file absent: {}".format(path))
    return target.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load(path):
    return yaml.safe_load(read(path))


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def write_or_check(path, payload, check):
    target = W / path
    if check:
        require(target.is_file(), "output absent")
        require(target.read_bytes() == payload, "output drift")
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    require(not target.exists(), "refusing overwrite")
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_bytes(payload)
    os.replace(str(temporary), str(target))


def distance(left, right):
    return math.hypot(
        float(left["x_m"]) - float(right["x_m"]),
        float(left["y_m"]) - float(right["y_m"]),
    )


def build_outputs():
    contract = load(CONTRACT)
    upstream = contract["upstream"]
    for key, path in (
        ("R16_contract_sha256", R16_CONTRACT),
        ("R16_assessment_sha256", ASSESSMENT),
        ("R16_evidence_freeze_sha256", FREEZE),
        ("R16_independent_review_sha256", R16_REVIEW),
        ("R16_v3_report_sha256", REPORT),
        ("R16_v3_receipt_sha256", RECEIPT),
        ("execution_core_sha256", EXECUTION_CORE),
    ):
        require(sha(read(path)) == upstream[key], "{} differs".format(path))
    report = load(REPORT)
    result = report["result"]
    decision = result["decisions"][-1]
    require(
        decision["selected_candidate_id"]
        == "recovery_evasive_right_v120_r2",
        "terminal decision differs",
    )
    points = decision["selected_candidate"]["points"]
    terminal = points[-1]
    goal = {"x_m": 5.2, "y_m": 0.0}
    trace = result["trace"]
    candidate_goal_distances = [
        distance(point, goal) for point in points
    ]
    trace_goal_distances = [
        distance(sample["ego"], goal) for sample in trace
    ]
    trace_terminal_distances = [
        distance(sample["ego"], terminal) for sample in trace
    ]
    closest_terminal_index = min(
        range(len(trace_terminal_distances)),
        key=trace_terminal_distances.__getitem__,
    )
    thresholds = contract["diagnostic_thresholds"]
    core_text = read(EXECUTION_CORE).decode("utf-8")
    checks = {
        "candidate_path_misses_common_goal_tolerance": (
            min(candidate_goal_distances)
            > float(thresholds["common_goal_position_tolerance_m"])
        ),
        "robot_trace_misses_common_goal_tolerance": (
            min(trace_goal_distances)
            > float(thresholds["common_goal_position_tolerance_m"])
        ),
        "robot_reaches_candidate_terminal_neighborhood": (
            min(trace_terminal_distances)
            <= float(thresholds["candidate_terminal_neighborhood_m"])
        ),
        "candidate_terminal_speed_nonzero": (
            float(terminal["longitudinal_velocity_mps"]) > 0.0
        ),
        "execution_helper_has_no_terminal_completion_output": (
            '"path_complete"' not in core_text
            and '"terminal_reached"' not in core_text
        ),
        "tracking_error_gate_failed": (
            float(result["tracking_error_max_m"])
            > float(thresholds["tracking_error_gate_m"])
        ),
        "safety_clearance_remained_nonnegative": (
            float(result["minimum_conservative_clearance_m"])
            >= float(thresholds["safe_clearance_min_m"])
            and int(result["contact_state_count"]) == 0
        ),
        "eventual_progress_observed": bool(result["eventual_progress"]),
        "goal_not_reached": not bool(result["goal_reached"]),
    }
    confirmed = all(checks.values())
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R16-D1",
        "status": (
            "offline_goal_and_terminal_execution_attribution_pass"
            if confirmed
            else "offline_failure_attribution_inconclusive"
        ),
        "diagnostic_checks": checks,
        "candidate_terminal": terminal,
        "minimum_candidate_path_goal_distance_m": min(
            candidate_goal_distances
        ),
        "candidate_terminal_goal_distance_m": distance(terminal, goal),
        "minimum_robot_trace_goal_distance_m": min(
            trace_goal_distances
        ),
        "minimum_robot_trace_terminal_distance_m": min(
            trace_terminal_distances
        ),
        "closest_terminal_trace_elapsed_s": trace[
            closest_terminal_index
        ]["elapsed_s"],
        "closest_terminal_trace_pose": trace[
            closest_terminal_index
        ]["ego"],
        "final_pose": result["final_pose"],
        "final_goal_distance_m": result["final_position_error_m"],
        "maximum_tracking_error_m": result["tracking_error_max_m"],
        "minimum_conservative_clearance_m": result[
            "minimum_conservative_clearance_m"
        ],
        "root_cause": (
            "recovery_candidate_endpoint_misses_common_goal_and_"
            "pure_pursuit_has_no_fail_closed_terminal_completion"
        ),
        "attribution_confirmed": confirmed,
        "single_repair_design_authorized": confirmed,
        "repair_requirements": {
            "goal_aligned_recovery_candidate": True,
            "terminal_completion_guard": True,
            "goal_timeout_or_safety_threshold_relaxation": False,
            "fresh_candidate_support_and_prediction_qualification": True,
        },
        "R16_result_changed": False,
        "new_simulation_units_used": 0,
        "actual_TEB_qualification_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    return {DIAGNOSIS: yaml_bytes(value)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        write_or_check(path, payload, args.check)
    resources = RESOURCES + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in resources
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, args.check)
    value = yaml.safe_load(outputs[DIAGNOSIS])
    print(
        "status={} diagnosis_sha256={} manifest_sha256={}".format(
            value["status"], sha(outputs[DIAGNOSIS]), sha(manifest)
        )
    )


if __name__ == "__main__":
    main()
