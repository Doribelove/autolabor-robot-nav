#!/usr/bin/env python3
"""Offline root-cause analysis of the frozen Q1-R2 diagnostic."""

import hashlib
import json
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r2_diagnostic"
)
EPISODE = ROOT / "episodes/001_01_7331_v3_13_analytic"
REPORT = EPISODE / "episode_report.yaml"
OUTPUT = ROOT / "offline_root_cause.yaml"
FREEZE = ROOT / "evidence_freeze.yaml"
SCRIPT = Path(
    "src/tools/thesis_experiment/scripts/"
    "analyze_v3_13_q1_r2_diagnostic.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r2_diagnostic_result.py"
)


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def build_outputs():
    report = yaml.safe_load(read(REPORT))
    diagnostics = report["planning_diagnostics"]
    failures = [
        item for item in diagnostics if item["planner_error"]
    ]
    terminal = failures[-1] if failures else diagnostics[-1]
    evaluations = terminal["candidate_evaluations"]
    rejection_counts = {}
    for evaluation in evaluations:
        for code in evaluation["rejection_codes"]:
            rejection_counts[code] = rejection_counts.get(code, 0) + 1
    stop = next(
        item
        for item in evaluations
        if item["candidate_id"] == "certified_stop_v000"
    )
    all_dynamic = all(
        "DYNAMIC_OCCUPANCY_INTERSECTION"
        in evaluation["rejection_codes"]
        for evaluation in evaluations
    )
    root_cause = (
        "all_candidates_rejected_by_dynamic_reachable_occupancy"
        if all_dynamic
        else "mixed_candidate_rejection_requires_followup"
    )
    decision_rows = [
        {
            "label": item["label"],
            "sim_time_s": item["sim_time_s"],
            "selected_candidate_id": item[
                "selected_candidate_id"
            ],
            "planner_error": item["planner_error"],
            "ego": item["snapshot"]["ego"],
            "actors": [
                {
                    "track_id": actor["track_id"],
                    "coordinate_m": actor["coordinate_m"],
                    "coordinate_speed_mps": actor[
                        "coordinate_speed_mps"
                    ],
                    "velocity_estimator": actor[
                        "velocity_estimator"
                    ],
                }
                for actor in item["snapshot"]["actors"]
            ],
            "candidate_clearances": {
                evaluation["candidate_id"]: evaluation[
                    "minimum_dynamic_clearance_m"
                ]
                for evaluation in item["candidate_evaluations"]
            },
        }
        for item in diagnostics
    ]
    analysis = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R2-D1",
        "status": "offline_diagnostic_root_cause_complete",
        "planner_failure_observed": bool(failures),
        "terminal_decision_label": terminal["label"],
        "terminal_sim_time_s": terminal["sim_time_s"],
        "terminal_planner_error": terminal["planner_error"],
        "rejection_code_counts": rejection_counts,
        "all_candidates_dynamic_occupancy_rejected": all_dynamic,
        "certified_stop_minimum_dynamic_clearance_m": stop[
            "minimum_dynamic_clearance_m"
        ],
        "root_cause": root_cause,
        "decision_sequence": decision_rows,
        "interpretation": (
            "the hard verifier failed closed; no action was overridden. "
            "A future repair must change one prospective modeling or "
            "decision-timing factor and obtain new sequential support."
        ),
        "actual_TEB_qualification_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    resources = [
        Path(
            "config/thesis_experiments/v3/"
            "v3_13_q1_r2_diagnostic_contract.yaml"
        ),
        Path(
            "artifacts/v3/multiactor/"
            "v3_13_q1_r2_readiness/execution_release.yaml"
        ),
        Path(
            "artifacts/v3/multiactor/"
            "v3_13_q1_r2_readiness/independent_review.yaml"
        ),
        ROOT / "exact_schedule.yaml",
        ROOT / "attempt_journal.jsonl",
        ROOT / "diagnostic_summary.yaml",
        EPISODE / "world.sdf",
        EPISODE / "scene_spec.yaml",
        EPISODE / "actor_specs.yaml",
        EPISODE / "episode_report.yaml",
        EPISODE / "matrix_receipt.yaml",
        EPISODE / "roslaunch.log",
        SCRIPT,
        REVIEWER,
    ]
    freeze = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R2-D1",
        "status": "closed_diagnostic_evidence_freeze",
        "resource_count": len(resources),
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in resources
        ],
        "retry_resume_or_seed_reuse_authorized": False,
    }
    return {
        OUTPUT: yaml_bytes(analysis),
        FREEZE: yaml_bytes(freeze),
    }


def main():
    outputs = build_outputs()
    for path, payload in outputs.items():
        target = W / path
        if target.exists():
            if target.read_bytes() != payload:
                raise RuntimeError("output drift: {}".format(path))
            continue
        temporary = target.with_suffix(target.suffix + ".tmp")
        temporary.write_bytes(payload)
        os.replace(str(temporary), str(target))
    analysis = yaml.safe_load(outputs[OUTPUT])
    print(
        "status={} root_cause={} stop_clearance_m={} "
        "analysis_sha256={}".format(
            analysis["status"],
            analysis["root_cause"],
            analysis[
                "certified_stop_minimum_dynamic_clearance_m"
            ],
            sha(outputs[OUTPUT]),
        )
    )


if __name__ == "__main__":
    main()
