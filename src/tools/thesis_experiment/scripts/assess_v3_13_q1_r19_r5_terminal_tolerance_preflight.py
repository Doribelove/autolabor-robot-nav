#!/usr/bin/env python3
"""Assess and freeze the R19-R5 tolerance preflight."""

import argparse
import hashlib
import json
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r5_terminal_tolerance_preflight_contract.yaml"
)
READINESS = Path(
    "artifacts/v3/multiactor/v3_13_q1_r19_r5_readiness"
)
EXECUTION = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_r5_terminal_tolerance_preflight"
)
EPISODE = EXECUTION / (
    "episodes/001_01_7695_recovery_evasive_right_v120_r3"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r19_r5_assessment"
)
ASSESSMENT = ROOT / "preflight_assessment.yaml"
FREEZE = ROOT / "evidence_freeze.yaml"
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r19_r5_terminal_tolerance_preflight.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r19_r5_preflight_result.py"
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
    receipt = load(EPISODE / "matrix_receipt.yaml")
    report = load(EPISODE / "episode_report.yaml")
    require(receipt["episode_report"] == report, "embedded report differs")
    for key, name in (
        ("episode_report_sha256", "episode_report.yaml"),
        ("world_sha256", "world.sdf"),
        ("scene_spec_sha256", "scene_spec.yaml"),
        ("actor_specs_sha256", "actor_specs.yaml"),
        ("roslaunch_log_sha256", "roslaunch.log"),
    ):
        require(
            receipt[key] == sha(read(EPISODE / name)),
            "{} hash differs".format(name),
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
        == [
            "execution_opened",
            "unit_consumed",
            "unit_complete",
            "execution_complete",
        ],
        "journal differs",
    )
    result = report["result"]
    gates = contract["scientific_gates"]
    checks = {
        "identity": (
            report["stage"] == "V3-13-Q1-R19-R5"
            and report["seed"] == 7695
            and report["forced_candidate_id"]
            == "recovery_evasive_right_v120_r3"
        ),
        "single_factor": (
            report["single_factor"]
            == {
                "name": "r3_terminal_tolerance_m",
                "control": 0.12,
                "treatment": 0.15,
            }
            and report["candidate_geometry_changed"] is False
            and report["legacy_action_execution_changed"] is False
        ),
        "attempt1_no_retry": True,
        "workspace_overlay_and_credentials": (
            receipt["credential_environment_scrubbed"]
            and receipt["workspace_overlay_explicit"]
            and not receipt["foreign_workspace_precedence_used"]
        ),
        "not_timed_out": not receipt["timed_out"],
        "clean_exit": receipt["roslaunch_return_code"] == 0,
        "episode_collection": report["episode_collection_pass"],
        "completed_horizon": result["completed_horizon"],
        "contact_count": result["contact_state_count"]
        <= int(gates["contact_state_count_max"]),
        "dynamic_clearance": result["minimum_conservative_clearance_m"]
        >= float(gates["minimum_conservative_clearance_m_min"]),
        "static_clearance": report["minimum_trace_static_clearance_m"]
        >= float(gates["minimum_trace_static_clearance_m_min"]),
        "tracking_error": result["tracking_error_max_m"]
        <= float(gates["maximum_tracking_error_m_max"]),
        "command_gap": result["maximum_command_gap_s"]
        <= float(gates["command_gap_s_max"]),
        "terminal_guard": report["terminal_guard_activation_count"]
        >= int(gates["terminal_guard_activation_count_min"]),
        "boundary_evaluation_exact": report[
            "boundary_terminal_evaluation_count"
        ]
        == int(gates["boundary_terminal_evaluation_count_exact"]),
        "boundary_trigger_exact": report[
            "boundary_terminal_trigger_count"
        ]
        == int(gates["boundary_terminal_trigger_count_exact"]),
        "assertive_modes": set(
            report["observable_modes_at_branch"].values()
        )
        == {gates["every_actor_observable_mode_at_branch"]},
        "unsafe_outcome_retained": report["unsafe_outcome_retained"],
    }
    passed = all(checks.values())
    assessment = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R5",
        "status": (
            "fresh_terminal_tolerance_preflight_pass"
            if passed
            else "fresh_terminal_tolerance_preflight_fail"
        ),
        "seed": 7695,
        "attempt": 1,
        "checks": checks,
        "metrics": {
            "minimum_conservative_clearance_m": result[
                "minimum_conservative_clearance_m"
            ],
            "minimum_trace_static_clearance_m": report[
                "minimum_trace_static_clearance_m"
            ],
            "tracking_error_max_m": result["tracking_error_max_m"],
            "terminal_guard_activation_count": report[
                "terminal_guard_activation_count"
            ],
            "boundary_terminal_evaluation_count": report[
                "boundary_terminal_evaluation_count"
            ],
            "boundary_terminal_trigger_count": report[
                "boundary_terminal_trigger_count"
            ],
        },
        "all_preflight_gates_pass": passed,
        "fresh_tolerance_repaired_support_design_authorized": passed,
        "predictor_fit_authorized": False,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    assessment_payload = yaml_bytes(assessment)
    resources = [
        CONTRACT,
        READINESS / "exact_schedule.yaml",
        READINESS / "dependency_freeze.yaml",
        READINESS / "execution_release.yaml",
        READINESS / "evidence_manifest.sha256",
        READINESS / "independent_review.yaml",
        EXECUTION / "exact_schedule.yaml",
        EXECUTION / "attempt_journal.jsonl",
        EXECUTION / "collection_summary.yaml",
        EPISODE / "world.sdf",
        EPISODE / "scene_spec.yaml",
        EPISODE / "actor_specs.yaml",
        EPISODE / "episode_report.yaml",
        EPISODE / "matrix_receipt.yaml",
        EPISODE / "roslaunch.log",
        ASSESSOR,
        REVIEWER,
    ]
    freeze = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R5",
        "status": "terminal_one_unit_tolerance_preflight_freeze",
        "resource_count": len(resources) + 1,
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in resources
        ]
        + [{"path": str(ASSESSMENT), "sha256": sha(assessment_payload)}],
        "authorized_units": 1,
        "consumed_units": 1,
        "completed_units": 1,
        "forfeited_units": 0,
        "seed7695_reusable": False,
        "preflight_pass": passed,
        "retry_resume_replacement_or_selective_exclusion_observed": False,
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
