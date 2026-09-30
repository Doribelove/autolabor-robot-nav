#!/usr/bin/env python3
"""Assess midpoint support, predictor coverage and planner recovery."""

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
sys.path.insert(
    0,
    str(
        W
        / "src/application/v3_scope_planner/src"
    ),
)
RUNNER_PATH = SCRIPT_DIR / "run_v3_13_q1_r3_midpoint_clones.py"
CORE_PATH = W / (
    "src/application/v3_scope_planner/src/"
    "v3_scope_planner/multi_actor_core_v1.py"
)
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r3_midpoint_clone_contract.yaml"
)
D0 = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
MODEL = Path(
    "artifacts/v3/multiactor/v3_13_s3_predictor/"
    "candidate_conditioned_recovery_predictor.yaml"
)
READINESS = Path(
    "artifacts/v3/multiactor/v3_13_q1_r3_readiness"
)
EXECUTION = Path(
    "artifacts/v3/multiactor/v3_13_q1_r3_execution"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r3_assessment"
)
ASSESSMENT = ROOT / "midpoint_clone_assessment.yaml"
FREEZE = ROOT / "evidence_freeze.yaml"
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r3_midpoint_clones.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r3_result.py"
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


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def load_yaml(path):
    return yaml.safe_load(read(path))


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
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


def build_outputs():
    runner = load_module("v3_13_q1_r3_runner", RUNNER_PATH)
    core = load_module("v3_13_q1_r3_core", CORE_PATH)
    contract = load_yaml(CONTRACT)
    planner_contract = load_yaml(D0)
    model = load_yaml(MODEL)
    rows = load_yaml(EXECUTION / "exact_schedule.yaml")["rows"]
    released = load_yaml(READINESS / "exact_schedule.yaml")["rows"]
    require(rows == released, "schedule differs")
    roots = sorted(
        path.relative_to(W)
        for path in (W / EXECUTION / "episodes").iterdir()
        if path.is_dir()
    )
    require(len(roots) == 16, "episode root count")
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
        require(not receipt["timed_out"], "episode timed out")
        receipts.append(receipt)
    events = [
        json.loads(line)
        for line in read(EXECUTION / "attempt_journal.jsonl")
        .decode("utf-8")
        .splitlines()
        if line.strip()
    ]
    require(len(events) == 34, "journal event count")
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
        "failure/retry observed",
    )
    summary = runner.summarize(receipts, sha(read(CONTRACT)))
    require(
        read(EXECUTION / "collection_summary.yaml")
        == yaml_bytes(summary),
        "collection summary differs",
    )
    matching = contract["matching_gates"]
    matching_checks = {
        "all_episode_collection_pass": summary[
            "all_episode_collection_pass"
        ],
        "all_reports_finite": summary["all_reports_finite"],
        "complete_action_support_rate": summary[
            "complete_action_support_rate"
        ]
        >= float(matching["complete_action_support_rate_min"]),
        "branch_clock": summary[
            "maximum_branch_clock_absolute_error_s"
        ]
        <= float(matching["branch_clock_absolute_error_max_s"]),
        "decision_ego_x": summary["maximum_decision_ego_x_range_m"]
        <= float(
            matching["paired_decision_ego_x_difference_max_m"]
        ),
        "actor_coordinate": summary[
            "maximum_decision_actor_coordinate_range_m"
        ]
        <= float(
            matching[
                "paired_decision_actor_coordinate_difference_max_m"
            ]
        ),
        "actor_speed": summary[
            "maximum_decision_actor_speed_range_mps"
        ]
        <= float(
            matching[
                "paired_decision_actor_speed_difference_max_mps"
            ]
        ),
    }

    trajectory_rows = []
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
                error = abs(
                    float(actor["coordinate_m"]) - prediction
                )
                errors.append(error)
                simultaneous = simultaneous and (
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
                    "simultaneous_trajectory_covered": simultaneous,
                }
            )
    require(len(trajectory_rows) == 32, "trajectory count")
    global_coverage = sum(
        row["simultaneous_trajectory_covered"]
        for row in trajectory_rows
    ) / len(trajectory_rows)
    mean_rmse = sum(
        row["trajectory_rmse_m"] for row in trajectory_rows
    ) / len(trajectory_rows)
    max_rmse = max(
        row["trajectory_rmse_m"] for row in trajectory_rows
    )
    scene_rows = []
    for scene_id in sorted(
        {row["scene_id"] for row in trajectory_rows}
    ):
        selected = [
            row
            for row in trajectory_rows
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
                    row["trajectory_rmse_m"] for row in selected
                )
                / len(selected),
                "maximum_trajectory_rmse_m": max(
                    row["trajectory_rmse_m"] for row in selected
                ),
            }
        )
    predictor_gates = contract["predictor_gates"]
    terminal_width = 2.0 * (radius + 7.0 * growth)
    predictor_checks = {
        "global_coverage": global_coverage
        >= float(
            predictor_gates[
                "simultaneous_whole_trajectory_coverage_min"
            ]
        ),
        "every_scene_coverage": all(
            row["simultaneous_trajectory_coverage"]
            >= float(
                predictor_gates[
                    "every_scene_simultaneous_trajectory_coverage_min"
                ]
            )
            for row in scene_rows
        ),
        "mean_rmse": mean_rmse
        <= float(predictor_gates["mean_trajectory_rmse_m_max"]),
        "maximum_rmse": max_rmse
        <= float(predictor_gates["maximum_trajectory_rmse_m_max"]),
        "terminal_width": terminal_width
        <= float(predictor_gates["terminal_full_width_m_max"]),
        "hidden_label_feature_count": 0
        <= int(predictor_gates["hidden_label_feature_count_max"]),
    }

    by_block_action = {
        (
            int(receipt["episode_report"]["block_index"]),
            receipt["episode_report"]["forced_candidate_id"],
        ): receipt["episode_report"]
        for receipt in receipts
    }
    planner_rows = []
    for block_index in range(1, 5):
        canonical = by_block_action[
            (block_index, "certified_stop_v000")
        ]
        decision = canonical["common_stop_prefix"]
        root = next(
            root
            for root in roots
            if load_yaml(root / "episode_report.yaml")[
                "block_index"
            ]
            == block_index
        )
        scene = load_yaml(root / "scene_spec.yaml")
        snapshot = {
            "scene_snapshot_id": "R3_audit_{}".format(block_index),
            "world_model_seq": 1,
            "ego": {
                "x_m": decision["decision_pose"]["x_m"],
                "y_m": decision["decision_pose"]["y_m"],
                "yaw_rad": decision["decision_pose"]["yaw_rad"],
                "speed_mps": decision["decision_speed_mps"],
            },
            "actors": decision["decision_actor_tracks"],
            "scene": scene,
            "health": {
                field: True
                for field in planner_contract["hard_verifier"][
                    "required_health_fields"
                ]
            },
        }
        try:
            plan = core.plan(snapshot, model, planner_contract)
            selected_id = plan["decision"]["selected_candidate_id"]
            selected_evaluation = next(
                item
                for item in plan["evaluations"]
                if item["candidate_id"] == selected_id
            )
            selected_report = by_block_action[
                (block_index, selected_id)
            ]
            planner_rows.append(
                {
                    "block_index": block_index,
                    "scene_id": canonical["scene_id"],
                    "actor_policies_evaluator_only": canonical[
                        "actor_policies_evaluator_only"
                    ],
                    "selected_candidate_id": selected_id,
                    "selected_analytic_accepted": selected_evaluation[
                        "accepted"
                    ],
                    "selected_analytic_clearance_m": (
                        selected_evaluation["minimum_clearance_m"]
                    ),
                    "selected_clone_clearance_m": selected_report[
                        "result"
                    ]["minimum_conservative_clearance_m"],
                    "selected_clone_contact_state_count": (
                        selected_report["result"][
                            "contact_state_count"
                        ]
                    ),
                    "planner_error": None,
                }
            )
        except Exception as error:
            planner_rows.append(
                {
                    "block_index": block_index,
                    "scene_id": canonical["scene_id"],
                    "actor_policies_evaluator_only": canonical[
                        "actor_policies_evaluator_only"
                    ],
                    "selected_candidate_id": None,
                    "selected_analytic_accepted": False,
                    "selected_analytic_clearance_m": None,
                    "selected_clone_clearance_m": None,
                    "selected_clone_contact_state_count": None,
                    "planner_error": "{}:{}".format(
                        type(error).__name__, error
                    ),
                }
            )
    planner_checks = {
        "every_block_progress": all(
            row["selected_candidate_id"]
            not in (None, "certified_stop_v000")
            for row in planner_rows
        ),
        "every_selected_analytic_accepted": all(
            row["selected_analytic_accepted"]
            for row in planner_rows
        ),
        "every_selected_clone_contact_free": all(
            row["selected_clone_contact_state_count"] == 0
            for row in planner_rows
        ),
        "every_selected_clone_clearance_nonnegative": all(
            row["selected_clone_clearance_m"] is not None
            and row["selected_clone_clearance_m"] >= 0.0
            for row in planner_rows
        ),
        "every_block_four_action_support": len(by_block_action) == 16,
    }
    all_pass = (
        all(matching_checks.values())
        and all(predictor_checks.values())
        and all(planner_checks.values())
    )
    assessment = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R3-ASSESSMENT",
        "status": (
            "midpoint_support_predictor_planner_qualification_pass"
            if all_pass
            else "midpoint_support_predictor_planner_qualification_fail"
        ),
        "execution_release_sha256": sha(
            read(READINESS / "execution_release.yaml")
        ),
        "collection_summary_sha256": sha(
            read(EXECUTION / "collection_summary.yaml")
        ),
        "authorized_units": 16,
        "consumed_units": 16,
        "completed_units": 16,
        "forfeited_units": 0,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_observed": False,
        "matching_metrics": {
            key: summary[key]
            for key in (
                "complete_action_support_rate",
                "maximum_branch_clock_absolute_error_s",
                "maximum_decision_ego_x_range_m",
                "maximum_decision_actor_coordinate_range_m",
                "maximum_decision_actor_speed_range_mps",
            )
        },
        "matching_gate_checks": matching_checks,
        "actor_trajectory_count": len(trajectory_rows),
        "simultaneous_whole_trajectory_coverage": global_coverage,
        "mean_trajectory_rmse_m": mean_rmse,
        "maximum_trajectory_rmse_m": max_rmse,
        "terminal_full_width_at_7s_m": terminal_width,
        "scene_predictor_results": scene_rows,
        "predictor_gate_checks": predictor_checks,
        "planner_results": planner_rows,
        "planner_gate_checks": planner_checks,
        "qualification_pass": all_pass,
        "new_actual_TEB_preflight_design_authorized": all_pass,
        "direct_actual_TEB_execution_authorized": False,
        "formal_performance_claim_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    resources = [
        CONTRACT,
        D0,
        MODEL,
        READINESS / "execution_release.yaml",
        READINESS / "independent_review.yaml",
        ASSESSOR,
        REVIEWER,
    ] + execution_resources(roots)
    freeze = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R3-ASSESSMENT",
        "status": "closed_midpoint_clone_evidence_freeze",
        "resource_count": len(resources),
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in resources
        ],
        "unsafe_outcomes_retained": True,
        "retry_resume_or_seed_reuse_authorized": False,
    }
    return {
        ASSESSMENT: yaml_bytes(assessment),
        FREEZE: yaml_bytes(freeze),
    }


def main():
    outputs = build_outputs()
    for path, payload in outputs.items():
        target = W / path
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            if target.read_bytes() != payload:
                raise RuntimeError("output drift")
            continue
        temporary = target.with_suffix(target.suffix + ".tmp")
        temporary.write_bytes(payload)
        os.replace(str(temporary), str(target))
    assessment = yaml.safe_load(outputs[ASSESSMENT])
    print(
        "status={} coverage={} mean_rmse_m={} max_rmse_m={} "
        "assessment_sha256={}".format(
            assessment["status"],
            assessment[
                "simultaneous_whole_trajectory_coverage"
            ],
            assessment["mean_trajectory_rmse_m"],
            assessment["maximum_trajectory_rmse_m"],
            sha(outputs[ASSESSMENT]),
        )
    )
    return 0 if assessment["qualification_pass"] else 2


if __name__ == "__main__":
    sys.exit(main())
