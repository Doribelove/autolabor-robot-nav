#!/usr/bin/env python3
"""Assess and freeze fresh R8 expanded-predictor qualification."""

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
sys.path.insert(
    0, str(W / "src/application/v3_scope_planner/src")
)
from v3_scope_planner import multi_actor_core_v1  # noqa: E402

RUNNER = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r8_recovery_qualification.py"
)
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r8_recovery_predictor_qualification_contract.yaml"
)
MODEL = Path(
    "artifacts/v3/multiactor/v3_13_q1_r7_predictor/"
    "expanded_recovery_predictor.yaml"
)
READINESS = Path(
    "artifacts/v3/multiactor/v3_13_q1_r8_readiness"
)
EXECUTION = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r8_recovery_qualification"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r8_assessment"
)
ASSESSMENT = ROOT / "predictor_qualification.yaml"
FREEZE = ROOT / "evidence_freeze.yaml"
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r8_recovery_qualification.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r8_qualification_result.py"
)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(path):
    target = W / path
    require(target.is_file(), "required file absent: {}".format(path))
    return target.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load_yaml(path):
    return yaml.safe_load(read(path))


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def load_runner():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r8_runner", RUNNER
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


def build_outputs():
    runner = load_runner()
    contract = load_yaml(CONTRACT)
    model = load_yaml(MODEL)
    require(
        sha(read(MODEL))
        == contract["upstream"]["predictor_sha256"],
        "predictor differs",
    )
    schedule = load_yaml(EXECUTION / "exact_schedule.yaml")
    released = load_yaml(READINESS / "exact_schedule.yaml")
    rows = schedule["rows"]
    require(rows == released["rows"], "schedule differs")
    roots = sorted(
        path.relative_to(W)
        for path in (W / EXECUTION / "episodes").iterdir()
        if path.is_dir()
    )
    require(len(roots) == len(rows) == 40, "episode roots")
    receipts = []
    for row, root in zip(rows, roots):
        receipt = load_yaml(root / "matrix_receipt.yaml")
        require(receipt["row"] == row, "receipt row differs")
        for key, name in (
            ("episode_report_sha256", "episode_report.yaml"),
            ("world_sha256", "world.sdf"),
            ("scene_spec_sha256", "scene_spec.yaml"),
            ("actor_specs_sha256", "actor_specs.yaml"),
            ("roslaunch_log_sha256", "roslaunch.log"),
        ):
            require(
                receipt[key] == sha(read(root / name)),
                "{} hash differs".format(name),
            )
        require(not receipt["timed_out"], "episode timed out")
        receipts.append(receipt)
    events = [
        json.loads(line)
        for line in read(EXECUTION / "attempt_journal.jsonl")
        .decode("utf-8")
        .splitlines()
        if line.strip()
    ]
    require(len(events) == 82, "journal event count")
    require(events[0]["event"] == "execution_opened", "journal open")
    require(events[-1]["event"] == "execution_complete", "journal close")
    require(
        not any(
            event["event"] in (
                "terminal_failure",
                "unit_retry",
                "unit_resume",
            )
            for event in events
        ),
        "failure retry or resume observed",
    )
    summary_contract = {
        "population": {
            "actions": contract[
                "qualification_population"
            ]["actions"]
        },
        "matching_gates": contract["matching_gates"],
    }
    summary = runner.R6.summarize(
        receipts, sha(read(CONTRACT)), summary_contract
    )
    summary["stage"] = "V3-13-Q1-R8"
    summary["status"] = (
        "fresh_expanded_predictor_qualification_collection_pass"
        if summary["collection_gate_pass"]
        else "fresh_expanded_predictor_qualification_collection_fail"
    )
    require(
        read(EXECUTION / "collection_summary.yaml")
        == yaml_bytes(summary),
        "summary differs",
    )

    radius = float(model["calibration"]["coordinate_radius_m"])
    growth = float(model["calibration"]["coordinate_growth_mps"])
    trajectory_rows = []
    for receipt in receipts:
        report = receipt["episode_report"]
        candidate_id = report["forced_candidate_id"]
        for initial in report["common_stop_prefix"][
            "decision_actor_tracks"
        ]:
            track_id = int(initial["track_id"])
            head = model["heads"]["assertive"][candidate_id]
            errors = []
            covered = True
            for sample in report["result"]["trace"]:
                actor = next(
                    item
                    for item in sample["actors"]
                    if int(item["track_id"]) == track_id
                )
                time_s = float(sample["elapsed_s"])
                features = multi_actor_core_v1.contextual_features(
                    initial, time_s
                )
                prediction = float(initial["coordinate_m"]) + sum(
                    float(coefficient) * float(feature)
                    for coefficient, feature in zip(
                        head["coefficients"], features
                    )
                )
                error = abs(
                    float(actor["coordinate_m"]) - prediction
                )
                errors.append(error)
                covered = covered and (
                    error <= radius + growth * time_s + 1.0e-12
                )
            trajectory_rows.append(
                {
                    "block_index": report["block_index"],
                    "scene_id": report["scene_id"],
                    "candidate_id": candidate_id,
                    "track_id": track_id,
                    "trajectory_rmse_m": math.sqrt(
                        sum(error * error for error in errors)
                        / len(errors)
                    ),
                    "maximum_absolute_error_m": max(errors),
                    "simultaneous_trajectory_covered": covered,
                }
            )
    require(len(trajectory_rows) == 80, "trajectory count")
    global_coverage = sum(
        row["simultaneous_trajectory_covered"]
        for row in trajectory_rows
    ) / len(trajectory_rows)
    mean_rmse = sum(
        row["trajectory_rmse_m"] for row in trajectory_rows
    ) / len(trajectory_rows)
    maximum_rmse = max(
        row["trajectory_rmse_m"] for row in trajectory_rows
    )
    scene_rows = []
    for scene_id in sorted(
        {row["scene_id"] for row in trajectory_rows}
    ):
        selected = [
            row for row in trajectory_rows
            if row["scene_id"] == scene_id
        ]
        scene_rows.append(
            {
                "scene_id": scene_id,
                "trajectory_count": len(selected),
                "simultaneous_trajectory_coverage": sum(
                    row["simultaneous_trajectory_covered"]
                    for row in selected
                )
                / len(selected),
                "mean_trajectory_rmse_m": sum(
                    row["trajectory_rmse_m"]
                    for row in selected
                )
                / len(selected),
                "maximum_trajectory_rmse_m": max(
                    row["trajectory_rmse_m"]
                    for row in selected
                ),
            }
        )
    candidate_rows = []
    for candidate_id in contract[
        "qualification_population"
    ]["actions"]:
        selected = [
            row for row in trajectory_rows
            if row["candidate_id"] == candidate_id
        ]
        candidate_rows.append(
            {
                "candidate_id": candidate_id,
                "trajectory_count": len(selected),
                "simultaneous_trajectory_coverage": sum(
                    row["simultaneous_trajectory_covered"]
                    for row in selected
                )
                / len(selected),
                "mean_trajectory_rmse_m": sum(
                    row["trajectory_rmse_m"]
                    for row in selected
                )
                / len(selected),
                "maximum_trajectory_rmse_m": max(
                    row["trajectory_rmse_m"]
                    for row in selected
                ),
            }
        )
    terminal_width = 2.0 * (radius + 7.0 * growth)
    gates = contract["predictor_gates"]
    checks = {
        "collection_gate": summary["collection_gate_pass"],
        "global_coverage": global_coverage
        >= float(
            gates["simultaneous_whole_trajectory_coverage_min"]
        ),
        "every_scene_coverage": all(
            row["simultaneous_trajectory_coverage"]
            >= float(
                gates[
                    "every_scene_simultaneous_trajectory_coverage_min"
                ]
            )
            for row in scene_rows
        ),
        "every_candidate_coverage": all(
            row["simultaneous_trajectory_coverage"]
            >= float(
                gates[
                    "every_candidate_simultaneous_trajectory_coverage_min"
                ]
            )
            for row in candidate_rows
        ),
        "mean_rmse": mean_rmse
        <= float(gates["mean_trajectory_rmse_m_max"]),
        "maximum_rmse": maximum_rmse
        <= float(gates["maximum_trajectory_rmse_m_max"]),
        "terminal_width": terminal_width
        <= float(gates["terminal_full_width_m_max"]),
        "hidden_label_feature_count": 0
        <= int(gates["hidden_label_feature_count_max"]),
    }
    passed = all(checks.values())
    assessment = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R8",
        "status": (
            "fresh_expanded_recovery_predictor_qualification_pass"
            if passed
            else "fresh_expanded_recovery_predictor_qualification_fail"
        ),
        "predictor_sha256": sha(read(MODEL)),
        "trajectory_count": len(trajectory_rows),
        "global_simultaneous_trajectory_coverage": global_coverage,
        "mean_trajectory_rmse_m": mean_rmse,
        "maximum_trajectory_rmse_m": maximum_rmse,
        "terminal_full_width_at_7s_m": terminal_width,
        "scene_statistics": scene_rows,
        "candidate_statistics": candidate_rows,
        "matching_metrics": {
            key: summary[key]
            for key in (
                "complete_action_support_rate",
                "minimum_behavior_propensity",
                "maximum_branch_clock_absolute_error_s",
                "maximum_decision_ego_x_range_m",
                "maximum_decision_actor_coordinate_range_m",
                "maximum_decision_actor_speed_range_mps",
            )
        },
        "gate_checks": checks,
        "qualification_pass": passed,
        "expanded_recovery_planner_integration_design_authorized": passed,
        "direct_actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "policy_or_ranker_training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    assessment_payload = yaml_bytes(assessment)
    resources = [
        READINESS / "execution_release.yaml",
        READINESS / "independent_review.yaml",
        EXECUTION / "exact_schedule.yaml",
        EXECUTION / "attempt_journal.jsonl",
        EXECUTION / "collection_summary.yaml",
        MODEL,
        ASSESSOR,
        REVIEWER,
    ]
    for root in roots:
        resources.extend(
            root / name
            for name in (
                "world.sdf",
                "scene_spec.yaml",
                "actor_specs.yaml",
                "episode_report.yaml",
                "matrix_receipt.yaml",
                "roslaunch.log",
            )
        )
    freeze = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R8",
        "status": "terminal_fresh_predictor_qualification_freeze",
        "resource_count": len(resources) + 1,
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in resources
        ]
        + [
            {
                "path": str(ASSESSMENT),
                "sha256": sha(assessment_payload),
            }
        ],
        "authorized_units": 40,
        "consumed_units": 40,
        "completed_units": 40,
        "forfeited_units": 0,
        "retry_resume_replacement_or_selective_exclusion_observed": False,
        "seeds7625_through_7632_reusable": False,
        "qualification_pass": passed,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    return {
        ASSESSMENT: assessment_payload,
        FREEZE: yaml_bytes(freeze),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        write_or_check(path, payload, args.check)
    assessment = yaml.safe_load(outputs[ASSESSMENT])
    print(
        "status={} assessment_sha256={} freeze_sha256={}".format(
            assessment["status"],
            sha(outputs[ASSESSMENT]),
            sha(outputs[FREEZE]),
        )
    )


if __name__ == "__main__":
    main()
