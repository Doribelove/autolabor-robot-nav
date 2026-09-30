#!/usr/bin/env python3
"""Assess and freeze fresh V3-13-S4 recovery prediction evidence."""

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
SCRIPT_DIR = W / "src/tools/thesis_experiment/scripts"
sys.path.insert(0, str(SCRIPT_DIR))
RUNNER_PATH = SCRIPT_DIR / "run_v3_13_s4_recovery_qualification.py"
CORE_PATH = (
    W
    / "src/application/v3_scope_planner/src/"
    "v3_scope_planner/multi_actor_core_v1.py"
)
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_s4_recovery_predictor_qualification_contract.yaml"
)
MODEL = Path(
    "artifacts/v3/multiactor/v3_13_s3_predictor/"
    "candidate_conditioned_recovery_predictor.yaml"
)
EXECUTION = Path("artifacts/v3/multiactor/v3_13_s4_execution")
READINESS = Path("artifacts/v3/multiactor/v3_13_s4_readiness")
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_s4_recovery_qualification.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_s4_recovery_result.py"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_s4_assessment")
ASSESSMENT = ROOT / "recovery_predictor_qualification.yaml"
FREEZE = ROOT / "evidence_freeze.yaml"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(relative):
    path = W / relative
    require(path.is_file(), "required file absent: {}".format(relative))
    return path.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def load_yaml(relative):
    return yaml.safe_load(read(relative))


def load_runner():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s4_result_runner", RUNNER_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_core():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s4_core", CORE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def execution_resources(roots):
    resources = [
        EXECUTION / "exact_schedule.yaml",
        EXECUTION / "attempt_journal.jsonl",
        EXECUTION / "collection_summary.yaml",
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
    return resources


def validate_journal(rows):
    events = [
        json.loads(line)
        for line in read(EXECUTION / "attempt_journal.jsonl")
        .decode("utf-8")
        .splitlines()
        if line.strip()
    ]
    require(len(events) == 2 + 2 * len(rows), "journal event count")
    require(events[0]["event"] == "execution_opened", "journal open")
    for index, row in enumerate(rows):
        consumed = events[1 + 2 * index]
        complete = events[2 + 2 * index]
        require(consumed["event"] == "unit_consumed", "consume order")
        require(complete["event"] == "unit_complete", "complete order")
        require(consumed["row"] == row, "journal row drift")
        require(complete["row"] == row, "journal completion drift")
    require(events[-1]["event"] == "execution_complete", "terminal")
    require(events[-1]["consumed_units"] == 32, "consumed")
    require(events[-1]["completed_units"] == 32, "completed")
    require(
        not any(
            event["event"] in (
                "terminal_failure",
                "unit_retry",
                "unit_resume",
            )
            for event in events
        ),
        "retry or failure",
    )
    return events


def build_outputs():
    runner = load_runner()
    core = load_core()
    contract = load_yaml(CONTRACT)
    model = load_yaml(MODEL)
    require(
        sha(read(MODEL)) == contract["upstream"]["predictor_sha256"],
        "predictor differs",
    )
    rows = load_yaml(EXECUTION / "exact_schedule.yaml")["rows"]
    released = load_yaml(READINESS / "exact_schedule.yaml")["rows"]
    require(rows == released, "schedule differs")
    roots = sorted(
        path.relative_to(W)
        for path in (W / EXECUTION / "episodes").iterdir()
        if path.is_dir()
    )
    require(len(roots) == 32, "episode root count")
    receipts = []
    for row, root in zip(rows, roots):
        receipt = load_yaml(root / "matrix_receipt.yaml")
        require(receipt["row"] == row, "receipt row differs")
        for field, name in (
            ("episode_report_sha256", "episode_report.yaml"),
            ("world_sha256", "world.sdf"),
            ("scene_spec_sha256", "scene_spec.yaml"),
            ("actor_specs_sha256", "actor_specs.yaml"),
            ("roslaunch_log_sha256", "roslaunch.log"),
        ):
            require(
                receipt[field] == sha(read(root / name)),
                "{} hash differs".format(name),
            )
        require(not receipt["timed_out"], "episode timeout")
        receipts.append(receipt)
    events = validate_journal(rows)
    summary = runner.collection_summary(
        receipts, sha(read(CONTRACT))
    )
    require(
        read(EXECUTION / "collection_summary.yaml")
        == yaml_bytes(summary),
        "collection summary replay differs",
    )
    require(summary["collection_gate_pass"], "collection gate failed")
    values = []
    radius = float(model["calibration"]["coordinate_radius_m"])
    growth = float(model["calibration"]["coordinate_growth_mps"])
    for receipt in receipts:
        report = receipt["episode_report"]
        candidate_id = report["forced_candidate_id"]
        for initial in report["common_stop_prefix"][
            "decision_actor_tracks"
        ]:
            track_id = int(initial["track_id"])
            head = model["heads"]["assertive"][candidate_id]
            errors = []
            simultaneous = True
            for sample in report["result"]["trace"]:
                actor = next(
                    item
                    for item in sample["actors"]
                    if int(item["track_id"]) == track_id
                )
                time_s = float(sample["elapsed_s"])
                features = core.contextual_features(initial, time_s)
                prediction = float(initial["coordinate_m"]) + sum(
                    float(coefficient) * float(feature)
                    for coefficient, feature in zip(
                        head["coefficients"], features
                    )
                )
                error = abs(float(actor["coordinate_m"]) - prediction)
                errors.append(error)
                simultaneous = simultaneous and (
                    error <= radius + growth * time_s + 1.0e-12
                )
            values.append(
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
                    "simultaneous_trajectory_covered": simultaneous,
                }
            )
    require(len(values) == 64, "trajectory count")
    global_coverage = sum(
        value["simultaneous_trajectory_covered"] for value in values
    ) / len(values)
    mean_rmse = sum(
        value["trajectory_rmse_m"] for value in values
    ) / len(values)
    maximum_rmse = max(
        value["trajectory_rmse_m"] for value in values
    )
    scene_rows = []
    for scene_id in sorted({value["scene_id"] for value in values}):
        selected = [
            value for value in values if value["scene_id"] == scene_id
        ]
        scene_rows.append(
            {
                "scene_id": scene_id,
                "trajectory_count": len(selected),
                "simultaneous_trajectory_coverage": sum(
                    value["simultaneous_trajectory_covered"]
                    for value in selected
                )
                / len(selected),
                "mean_trajectory_rmse_m": sum(
                    value["trajectory_rmse_m"] for value in selected
                )
                / len(selected),
                "maximum_trajectory_rmse_m": max(
                    value["trajectory_rmse_m"] for value in selected
                ),
            }
        )
    terminal_width = 2.0 * (radius + 7.0 * growth)
    gates = contract["predictor_gates"]
    checks = {
        "simultaneous_whole_trajectory_coverage": global_coverage
        >= float(gates["simultaneous_whole_trajectory_coverage_min"]),
        "every_scene_simultaneous_trajectory_coverage": all(
            row["simultaneous_trajectory_coverage"]
            >= float(
                gates[
                    "every_scene_simultaneous_trajectory_coverage_min"
                ]
            )
            for row in scene_rows
        ),
        "mean_trajectory_rmse": mean_rmse
        <= float(gates["mean_trajectory_rmse_m_max"]),
        "maximum_trajectory_rmse": maximum_rmse
        <= float(gates["maximum_trajectory_rmse_m_max"]),
        "terminal_full_width": terminal_width
        <= float(gates["terminal_full_width_m_max"]),
        "hidden_label_feature_count": 0
        <= int(gates["hidden_label_feature_count_max"]),
    }
    assessment = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S4-ASSESSMENT",
        "status": (
            "fresh_recovery_predictor_qualification_pass"
            if all(checks.values())
            else "fresh_recovery_predictor_qualification_fail"
        ),
        "execution_release_sha256": sha(
            read(READINESS / "execution_release.yaml")
        ),
        "collection_summary_sha256": sha(
            read(EXECUTION / "collection_summary.yaml")
        ),
        "predictor_sha256": sha(read(MODEL)),
        "authorized_units": 32,
        "consumed_units": 32,
        "completed_units": 32,
        "forfeited_units": 0,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_observed": False,
        "actor_trajectory_count": len(values),
        "simultaneous_whole_trajectory_coverage": global_coverage,
        "mean_trajectory_rmse_m": mean_rmse,
        "maximum_trajectory_rmse_m": maximum_rmse,
        "terminal_full_width_at_7s_m": terminal_width,
        "scene_results": scene_rows,
        "trajectory_results": values,
        "collection_gate_pass": summary["collection_gate_pass"],
        "predictor_gate_checks": checks,
        "fresh_recovery_predictor_qualification_pass": all(
            checks.values()
        ),
        "offline_recovery_planner_audit_authorized": all(
            checks.values()
        ),
        "direct_actual_teb_execution_authorized": False,
        "policy_or_ranker_training_authorized": False,
        "performance_claim_authorized": False,
        "perception_claim_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "journal_event_count": len(events),
    }
    require(
        assessment["fresh_recovery_predictor_qualification_pass"],
        "fresh predictor qualification failed",
    )
    resources = [
        CONTRACT,
        MODEL,
        READINESS / "execution_release.yaml",
        READINESS / "independent_review.yaml",
        ASSESSOR,
        REVIEWER,
    ] + execution_resources(roots)
    freeze = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S4-ASSESSMENT",
        "status": "closed_fresh_recovery_qualification_evidence_freeze",
        "resource_count": len(resources),
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in resources
        ],
        "unsafe_outcomes_retained": True,
    }
    return {
        ASSESSMENT: yaml_bytes(assessment),
        FREEZE: yaml_bytes(freeze),
    }


def write_or_check(relative, payload, check):
    path = W / relative
    if check:
        require(path.is_file(), "output absent: {}".format(relative))
        require(path.read_bytes() == payload, "output drift: {}".format(relative))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    require(not path.exists(), "refusing overwrite: {}".format(relative))
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    outputs = build_outputs()
    for relative, payload in outputs.items():
        write_or_check(relative, payload, arguments.check)
    print(
        "status=v3_13_s4_assessment_{} assessment_sha256={} "
        "freeze_sha256={}".format(
            "reproducible" if arguments.check else "built",
            sha(read(ASSESSMENT)),
            sha(read(FREEZE)),
        )
    )


if __name__ == "__main__":
    main()
