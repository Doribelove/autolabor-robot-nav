#!/usr/bin/env python3
"""Assess and freeze passing V3-13-S2 synchronized clone evidence."""

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
SCRIPT_DIR = W / "src/tools/thesis_experiment/scripts"
sys.path.insert(0, str(SCRIPT_DIR))
RUNNER_PATH = SCRIPT_DIR / "run_v3_13_s2_sequential_clones.py"
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_s2_sequential_clones.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_s2_sequential_result.py"
)
EXECUTION = Path("artifacts/v3/multiactor/v3_13_s2_execution")
READINESS = Path("artifacts/v3/multiactor/v3_13_s2_readiness")
ROOT = Path("artifacts/v3/multiactor/v3_13_s2_assessment")
ASSESSMENT = ROOT / "sequential_clone_assessment.yaml"
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
        "v3_13_s2_result_runner", RUNNER_PATH
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
    require(events[-1]["consumed_units"] == 32, "consumed units")
    require(events[-1]["completed_units"] == 32, "completed units")
    require(events[-1]["forfeited_units"] == 0, "forfeited units")
    require(
        not any(
            event["event"] in (
                "terminal_failure",
                "unit_retry",
                "unit_resume",
            )
            for event in events
        ),
        "retry or failure observed",
    )
    return events


def build_outputs():
    runner = load_runner()
    schedule = load_yaml(EXECUTION / "exact_schedule.yaml")
    rows = schedule["rows"]
    released = load_yaml(READINESS / "exact_schedule.yaml")
    require(rows == released["rows"], "schedule differs")
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
        require(
            receipt["credential_environment_scrubbed"],
            "credential scrub absent",
        )
        receipts.append(receipt)
    events = validate_journal(rows)
    contract_payload = read(
        Path(
            "config/thesis_experiments/v3/"
            "v3_13_s2_synchronized_sequential_clone_contract.yaml"
        )
    )
    summary = runner.summarize(
        receipts, yaml.safe_load(contract_payload), sha(contract_payload)
    )
    require(
        read(EXECUTION / "collection_summary.yaml")
        == yaml_bytes(summary),
        "summary replay differs",
    )
    reports = [receipt["episode_report"] for receipt in receipts]
    action_groups = {}
    for report in reports:
        action_groups.setdefault(
            report["forced_candidate_id"], []
        ).append(report)
    action_statistics = {}
    for action, group in sorted(action_groups.items()):
        clearances = [
            float(
                report["result"]["minimum_conservative_clearance_m"]
            )
            for report in group
        ]
        action_statistics[action] = {
            "episodes": len(group),
            "minimum_clearance_m": min(clearances),
            "mean_clearance_m": sum(clearances) / len(clearances),
            "negative_clearance_outcomes": sum(
                value < 0.0 for value in clearances
            ),
            "mean_path_length_m": sum(
                float(report["result"]["path_length_m"])
                for report in group
            )
            / len(group),
        }
    blocks = {}
    for report in reports:
        blocks.setdefault(int(report["block_index"]), []).append(report)
    spreads = []
    for block_index, group in sorted(blocks.items()):
        for track_id in (1, 2):
            endpoints = []
            for report in group:
                actor = next(
                    item
                    for item in report["result"]["trace"][-1]["actors"]
                    if int(item["track_id"]) == track_id
                )
                endpoints.append(float(actor["coordinate_m"]))
            spreads.append(
                {
                    "block_index": block_index,
                    "track_id": track_id,
                    "endpoint_coordinate_spread_m": max(endpoints)
                    - min(endpoints),
                }
            )
    assessment = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S2-ASSESSMENT",
        "status": "v3_13_s2_synchronized_clone_assessment_pass",
        "execution_release_sha256": sha(
            read(READINESS / "execution_release.yaml")
        ),
        "readiness_review_sha256": sha(
            read(READINESS / "independent_review.yaml")
        ),
        "collection_summary_sha256": sha(
            read(EXECUTION / "collection_summary.yaml")
        ),
        "authorized_units": 32,
        "consumed_units": 32,
        "completed_units": 32,
        "forfeited_units": 0,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_observed": False,
        "matched_context_blocks": 8,
        "actions_per_block": 4,
        "complete_action_support_rate": summary[
            "complete_action_support_rate"
        ],
        "minimum_behavior_propensity": min(
            float(report["behavior_propensity"]) for report in reports
        ),
        "total_branch_trace_samples": sum(
            int(report["result"]["trace_sample_count"])
            for report in reports
        ),
        "all_finite_tracks": all(
            runner.finite_report(report) for report in reports
        ),
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
        "contact_state_count_retained": summary[
            "contact_state_count_retained"
        ],
        "action_statistics": action_statistics,
        "mean_actor_endpoint_coordinate_spread_m": sum(
            row["endpoint_coordinate_spread_m"] for row in spreads
        )
        / len(spreads),
        "actor_endpoint_spread_positive_rate": sum(
            row["endpoint_coordinate_spread_m"] > 1.0e-6
            for row in spreads
        )
        / len(spreads),
        "actor_endpoint_spreads": spreads,
        "collection_gate_pass": summary["collection_gate_pass"],
        "assertive_recovery_predictor_fit_authorized": summary[
            "collection_gate_pass"
        ],
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
        "performance_claim_authorized": False,
        "perception_claim_authorized": False,
        "policy_or_ranker_training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "journal_event_count": len(events),
    }
    require(assessment["collection_gate_pass"], "collection gate failed")
    require(assessment["all_finite_tracks"], "nonfinite track")
    require(
        assessment["complete_action_support_rate"] == 1.0,
        "support incomplete",
    )
    resources = [
        READINESS / "execution_release.yaml",
        READINESS / "independent_review.yaml",
        ASSESSOR,
        REVIEWER,
    ] + execution_resources(roots)
    freeze = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S2-ASSESSMENT",
        "status": "closed_raw_synchronized_clone_evidence_freeze",
        "resource_count": len(resources),
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in resources
        ],
        "unsafe_outcomes_retained": True,
        "runtime_home_logs_excluded_from_claim_evidence": True,
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
        "status=v3_13_s2_assessment_{} assessment_sha256={} "
        "freeze_sha256={}".format(
            "reproducible" if arguments.check else "built",
            sha(read(ASSESSMENT)),
            sha(read(FREEZE)),
        )
    )


if __name__ == "__main__":
    main()
