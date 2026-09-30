#!/usr/bin/env python3
"""Assess and freeze the R16 actual-TEB paired preflight."""

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
    "run_v3_13_q1_r16_actual_teb_preflight.py"
)
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r16_actual_teb_paired_preflight_contract.yaml"
)
READINESS = Path(
    "artifacts/v3/multiactor/v3_13_q1_r16_readiness"
)
EXECUTION = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r16_actual_teb_preflight"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r16_assessment"
)
ASSESSMENT = ROOT / "paired_preflight_assessment.yaml"
FREEZE = ROOT / "evidence_freeze.yaml"
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r16_actual_teb_preflight.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r16_actual_teb_preflight_result.py"
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


def load_runner():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r16_runner", RUNNER
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_outputs():
    runner = load_runner()
    contract = load(CONTRACT)
    schedule = load(EXECUTION / "exact_schedule.yaml")
    released_schedule = load(READINESS / "exact_schedule.yaml")
    require(
        schedule["rows"] == released_schedule["rows"],
        "schedule differs",
    )
    roots = sorted(
        path.relative_to(W)
        for path in (W / EXECUTION / "episodes").iterdir()
        if path.is_dir()
    )
    require(len(roots) == 2, "episode root count")
    receipts = []
    for row, root in zip(schedule["rows"], roots):
        receipt = load(root / "matrix_receipt.yaml")
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
            receipt["workspace_overlay_explicit"],
            "workspace overlay not explicit",
        )
        require(
            not receipt["foreign_workspace_precedence_used"],
            "foreign workspace precedence used",
        )
        receipts.append(receipt)
    events = [
        json.loads(line)
        for line in read(EXECUTION / "attempt_journal.jsonl")
        .decode("utf-8")
        .splitlines()
        if line.strip()
    ]
    require(len(events) == 6, "journal event count")
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
    summary = runner.summarize(
        receipts, sha(read(CONTRACT)), contract
    )
    require(
        read(EXECUTION / "collection_summary.yaml")
        == yaml_bytes(summary),
        "summary differs",
    )
    passed = bool(summary["collection_gate_pass"])
    assessment = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R16",
        "status": (
            "fresh_actual_teb_paired_integration_preflight_pass"
            if passed
            else "fresh_actual_teb_paired_integration_preflight_fail"
        ),
        "complete_paired_blocks": summary["complete_paired_blocks"],
        "integration_gate_checks": summary[
            "integration_gate_checks"
        ],
        "matching_metrics": {
            key: summary[key]
            for key in (
                "maximum_paired_gate_ego_x_difference_m",
                "maximum_paired_gate_actor_coordinate_difference_m",
                "maximum_paired_gate_actor_speed_difference_mps",
            )
        },
        "actual_teb_outcome_retained_without_pass_requirement": summary[
            "actual_teb_outcome_retained_without_pass_requirement"
        ],
        "v3_outcome": summary["v3_outcome"],
        "preflight_pass": passed,
        "actual_teb_multiscene_qualification_design_authorized": passed,
        "qualification_execution_authorized": False,
        "performance_claim_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
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
        "stage": "V3-13-Q1-R16",
        "status": "terminal_fresh_actual_teb_paired_preflight_freeze",
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
        "authorized_units": 2,
        "consumed_units": 2,
        "completed_units": 2,
        "forfeited_units": 0,
        "seed7666_reusable": False,
        "retry_resume_replacement_or_selective_exclusion_observed": False,
        "preflight_pass": passed,
        "qualification_execution_authorized": False,
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
