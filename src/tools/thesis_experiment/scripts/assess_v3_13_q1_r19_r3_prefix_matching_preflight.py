#!/usr/bin/env python3
"""Assess and freeze the R19-R3 five-action prefix preflight."""

import argparse
import hashlib
import json
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r3_prefix_matching_preflight_contract.yaml"
)
READINESS = Path(
    "artifacts/v3/multiactor/v3_13_q1_r19_r3_readiness"
)
EXECUTION = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_r3_prefix_matching_preflight"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r19_r3_assessment"
)
ASSESSMENT = ROOT / "preflight_assessment.yaml"
FREEZE = ROOT / "evidence_freeze.yaml"
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r19_r3_prefix_matching_preflight.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r19_r3_preflight_result.py"
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


def build_outputs():
    contract = load(CONTRACT)
    require(
        load(EXECUTION / "exact_schedule.yaml")["rows"]
        == load(READINESS / "exact_schedule.yaml")["rows"],
        "schedule differs",
    )
    roots = sorted(
        path.relative_to(W)
        for path in (W / EXECUTION / "episodes").iterdir()
        if path.is_dir()
    )
    require(len(roots) == 5, "episode root count differs")
    receipts = []
    resources = []
    for root in roots:
        receipt = load(root / "matrix_receipt.yaml")
        report = load(root / "episode_report.yaml")
        require(receipt["episode_report"] == report, "embedded report differs")
        for key, name in (
            ("episode_report_sha256", "episode_report.yaml"),
            ("world_sha256", "world.sdf"),
            ("scene_spec_sha256", "scene_spec.yaml"),
            ("actor_specs_sha256", "actor_specs.yaml"),
            ("roslaunch_log_sha256", "roslaunch.log"),
        ):
            require(
                receipt[key] == sha(read(root / name)),
                "{} hash differs".format(root / name),
            )
        receipts.append(receipt)
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
    events = [
        json.loads(line)
        for line in read(EXECUTION / "attempt_journal.jsonl")
        .decode("utf-8")
        .splitlines()
        if line.strip()
    ]
    require(
        [event["event"] for event in events]
        == ["execution_opened"]
        + ["unit_consumed", "unit_complete"] * 5
        + ["execution_complete"],
        "journal differs",
    )
    summary = load(EXECUTION / "collection_summary.yaml")
    require(summary["completed_units"] == 5, "completion differs")
    expected = set(contract["population"]["actions"])
    reports = [receipt["episode_report"] for receipt in receipts]
    checks = dict(summary["gate_checks"])
    checks.update(
        {
            "fresh_identity": all(
                report["seed"] == 7686
                and report["stage"] == "V3-13-Q1-R19-R3"
                for report in reports
            ),
            "exact_action_support": {
                report["forced_candidate_id"] for report in reports
            }
            == expected,
            "single_factor_recorded": all(
                report["single_factor"]
                == {
                    "name": (
                        "prebranch_warmup_command_sampling_rate_hz"
                    ),
                    "control": 20,
                    "treatment": 100,
                }
                and report["postbranch_execution_changed"] is False
                for report in reports
            ),
            "closed_runtime": all(
                receipt["credential_environment_scrubbed"]
                and receipt["workspace_overlay_explicit"]
                and not receipt["foreign_workspace_precedence_used"]
                and not receipt["timed_out"]
                and receipt["roslaunch_return_code"] == 0
                for receipt in receipts
            ),
            "unsafe_outcomes_retained": all(
                report["unsafe_outcome_retained"]
                for report in reports
            ),
        }
    )
    passed = all(checks.values())
    assessment = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R3",
        "status": (
            "fresh_prefix_matching_preflight_assessment_pass"
            if passed
            else "fresh_prefix_matching_preflight_assessment_fail"
        ),
        "execution_release_sha256": sha(
            read(READINESS / "execution_release.yaml")
        ),
        "readiness_review_sha256": sha(
            read(READINESS / "independent_review.yaml")
        ),
        "collection_summary_sha256": sha(
            read(EXECUTION / "collection_summary.yaml")
        ),
        "authorized_units": 5,
        "consumed_units": 5,
        "completed_units": 5,
        "forfeited_units": 0,
        "checks": checks,
        "metrics": {
            "maximum_branch_clock_absolute_error_s": summary[
                "maximum_branch_clock_absolute_error_s"
            ],
            "maximum_decision_ego_x_range_m": summary[
                "maximum_decision_ego_x_range_m"
            ],
            "maximum_decision_actor_coordinate_range_m": summary[
                "maximum_decision_actor_coordinate_range_m"
            ],
            "maximum_decision_actor_speed_range_mps": summary[
                "maximum_decision_actor_speed_range_mps"
            ],
            "r3_terminal_guard_execution_rate": summary[
                "r3_terminal_guard_execution_rate"
            ],
            "r3_boundary_atomic_execution_rate": summary[
                "r3_boundary_atomic_execution_rate"
            ],
        },
        "all_preflight_gates_pass": passed,
        "fresh_prefix_aligned_support_design_authorized": passed,
        "predictor_fit_authorized": False,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    assessment_payload = yaml_bytes(assessment)
    freeze_resources = [
        CONTRACT,
        READINESS / "exact_schedule.yaml",
        READINESS / "dependency_freeze.yaml",
        READINESS / "execution_release.yaml",
        READINESS / "evidence_manifest.sha256",
        READINESS / "independent_review.yaml",
        EXECUTION / "exact_schedule.yaml",
        EXECUTION / "attempt_journal.jsonl",
        EXECUTION / "collection_summary.yaml",
        ASSESSOR,
        REVIEWER,
    ] + resources
    freeze = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R3",
        "status": "terminal_five_action_prefix_preflight_freeze",
        "resource_count": len(freeze_resources) + 1,
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in freeze_resources
        ]
        + [{"path": str(ASSESSMENT), "sha256": sha(assessment_payload)}],
        "authorized_units": 5,
        "consumed_units": 5,
        "completed_units": 5,
        "forfeited_units": 0,
        "seed7686_reusable": False,
        "preflight_pass": passed,
        "retry_resume_replacement_or_selective_exclusion_observed": False,
        "predictor_fit_authorized": False,
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
