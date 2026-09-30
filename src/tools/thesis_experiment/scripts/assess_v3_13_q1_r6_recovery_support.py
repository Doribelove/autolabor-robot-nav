#!/usr/bin/env python3
"""Assess and freeze V3-13-Q1-R6 five-action support."""

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
RUNNER = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r6_recovery_support.py"
)
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r6_recovery_bank_support_contract.yaml"
)
READINESS = Path(
    "artifacts/v3/multiactor/v3_13_q1_r6_readiness"
)
EXECUTION = Path(
    "artifacts/v3/multiactor/v3_13_q1_r6_recovery_support"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r6_assessment"
)
ASSESSMENT = ROOT / "support_assessment.yaml"
FREEZE = ROOT / "evidence_freeze.yaml"
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r6_recovery_support.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r6_support_result.py"
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
        "v3_13_q1_r6_runner", RUNNER
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
    schedule = load_yaml(EXECUTION / "exact_schedule.yaml")
    released = load_yaml(READINESS / "exact_schedule.yaml")
    rows = schedule["rows"]
    require(rows == released["rows"], "schedule differs")
    roots = sorted(
        path.relative_to(W)
        for path in (W / EXECUTION / "episodes").iterdir()
        if path.is_dir()
    )
    require(len(roots) == len(rows) == 40, "episode root count")
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
        require(
            receipt["credential_environment_scrubbed"]
            and receipt["workspace_overlay_explicit"]
            and not receipt["foreign_workspace_precedence_used"],
            "environment boundary differs",
        )
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
    for index, row in enumerate(rows):
        require(
            events[1 + 2 * index]["event"] == "unit_consumed"
            and events[2 + 2 * index]["event"] == "unit_complete",
            "journal unit order",
        )
        require(
            events[1 + 2 * index]["row"] == row
            and events[2 + 2 * index]["row"] == row,
            "journal row differs",
        )
    summary = runner.summarize(
        receipts, sha(read(CONTRACT)), contract
    )
    require(
        read(EXECUTION / "collection_summary.yaml")
        == yaml_bytes(summary),
        "collection summary differs",
    )
    reports = [receipt["episode_report"] for receipt in receipts]
    action_statistics = {}
    for action in contract["population"]["actions"]:
        group = [
            report
            for report in reports
            if report["forced_candidate_id"] == action
        ]
        clearances = [
            float(
                report["result"][
                    "minimum_conservative_clearance_m"
                ]
            )
            for report in group
        ]
        action_statistics[action] = {
            "episodes": len(group),
            "actor_trajectories": 2 * len(group),
            "minimum_clearance_m": min(clearances),
            "mean_clearance_m": sum(clearances) / len(clearances),
            "negative_clearance_outcomes": sum(
                value < 0.0 for value in clearances
            ),
            "contact_state_count": sum(
                int(report["result"]["contact_state_count"])
                for report in group
            ),
            "trace_samples": sum(
                int(report["result"]["trace_sample_count"])
                for report in group
            ),
        }
    passed = summary["collection_gate_pass"]
    assessment = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R6",
        "status": (
            "fresh_five_action_support_assessment_pass"
            if passed
            else "fresh_five_action_support_assessment_fail"
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
        "authorized_units": 40,
        "consumed_units": 40,
        "completed_units": 40,
        "forfeited_units": 0,
        "matched_context_blocks": 8,
        "actions_per_block": 5,
        "complete_action_support_rate": summary[
            "complete_action_support_rate"
        ],
        "minimum_behavior_propensity": summary[
            "minimum_behavior_propensity"
        ],
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
        "action_statistics": action_statistics,
        "unsafe_outcomes_retained": True,
        "collection_gate_pass": passed,
        "new_candidate_assertive_head_fit_authorized": passed,
        "fit_scope": (
            "assertive_head_for_"
            "recovery_evasive_right_v120_r1_only"
        ),
        "all_S3_heads_must_remain_byte_equivalent": True,
        "fresh_qualification_required_after_fit": True,
        "actual_TEB_execution_authorized": False,
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
        "stage": "V3-13-Q1-R6",
        "status": "terminal_five_action_support_evidence_freeze",
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
        "seeds7617_through_7624_reusable": False,
        "collection_gate_pass": passed,
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
