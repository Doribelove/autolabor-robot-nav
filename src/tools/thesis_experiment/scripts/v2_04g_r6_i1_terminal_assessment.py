#!/usr/bin/env python3
"""Offline, non-authorizing assessment of the terminal R6-I1 execution."""

import argparse
import hashlib
import os
from pathlib import Path
import re
import sys
import tempfile

import yaml

from thesis_experiment.v2_04g_r6_i1_execution_integrity import (
    validate_persisted_attempt,
)
from thesis_experiment.v2_04g_r6_integrity import strict_yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R6-I1"
ROOT = WORKSPACE / "artifacts/v2/integration/v2_04g_r6_i1"
STAGE_REPORT = ROOT / "v2_04g_r6_i1_stage_report.yaml"
ASSESSMENT = ROOT / "v2_04g_r6_i1_terminal_assessment.yaml"
PREREGISTRATION = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_execution_preregistration.yaml"
)
AUTHORIZATION = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_bounded_simulation_authorization.yaml"
)
RUNNER = WORKSPACE / (
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_bounded_validation.py"
)
ORIGINAL_ASSESSOR = WORKSPACE / (
    "src/tools/thesis_experiment/scripts/assess_v2_04g_r6_i1.py"
)
EXPECTED_IDENTITY = {
    "stage": STAGE,
    "profile_id": "r6_semantics_legacy_control",
    "scene_id": "v2-04g-r6-i1-dynamic-conflict-single-s5141",
    "seed": 5141,
    "attempt": 1,
}
EXPECTED_FAILURE = (
    "service readiness timed out: "
    "/move_base/TebLocalPlannerROS/set_parameters"
)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _relative(path):
    return str(Path(path).resolve().relative_to(WORKSPACE))


def _atomic_yaml(path, document):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = yaml.safe_dump(
        document, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=target.name + ".tmp.", dir=str(target.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(target))
        directory = os.open(str(target.parent), os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _single_log(work, name):
    matches = sorted({
        path.resolve()
        for path in (work / "ros_logs").glob("*/" + name)
        if path.is_file()
    })
    _require(len(matches) == 1, "expected one {}".format(name))
    return matches[0]


def _bootstrap_evidence(entry):
    attempt_root = WORKSPACE / entry["raw_evidence_root"]
    work = attempt_root.parent / "work"
    base_log = work / "base_launch.log"
    master_log = _single_log(work, "master.log")
    rosout_log = _single_log(work, "rosout.log")
    _require(base_log.is_file(), "base launch log is missing")

    runner_source = RUNNER.read_text(encoding="utf-8")
    paused_index = runner_source.index('"paused:=true"')
    wait_index = runner_source.index(
        '"/move_base/TebLocalPlannerROS/set_parameters"'
    )
    unpause_index = runner_source.index(
        '["rosservice", "call", "/gazebo/unpause_physics"]'
    )
    _require(
        paused_index < wait_index < unpause_index,
        "runner bootstrap ordering no longer matches executed source",
    )

    base_text = base_log.read_text(encoding="utf-8", errors="replace")
    master_text = master_log.read_text(encoding="utf-8", errors="replace")
    rosout_text = rosout_log.read_text(
        encoding="utf-8", errors="replace"
    )
    _require(
        "* /use_sim_time: True" in base_text,
        "execution did not record simulated ROS time",
    )
    _require(
        "+SUB [/clock] /move_base" in master_text,
        "move_base did not subscribe to /clock",
    )
    _require(
        "+SERVICE [/move_base/TebLocalPlannerROS/set_parameters]"
        not in master_text,
        "target service was unexpectedly advertised",
    )
    ros_times = [
        float(match.group(1))
        for match in re.finditer(
            r"(?m)^([0-9]+\.[0-9]+) (?:INFO|WARN|ERROR|FATAL) ",
            rosout_text,
        )
    ]
    _require(ros_times, "rosout contains no timestamped evidence")
    _require(
        max(ros_times) == 0.0,
        "ROS time advanced despite the terminal bootstrap diagnosis",
    )
    return {
        "diagnosis": "paused_sim_time_bootstrap_order_deadlock",
        "confidence": "confirmed",
        "causal_chain": [
            "base_launch_requested_with_paused_true",
            "move_base_subscribed_to_simulated_clock_at_zero",
            "runner_waited_for_teb_dynamic_reconfigure_service",
            "runner_unpause_call_was_ordered_after_service_wait",
            "service_was_not_advertised_before_timeout",
        ],
        "executed_runner": {
            "path": _relative(RUNNER),
            "sha256": sha256(RUNNER),
            "source_order_verified": (
                "paused_true_before_service_wait_before_unpause"
            ),
        },
        "sanitized_observations": {
            "use_sim_time": True,
            "maximum_rosout_time_s": max(ros_times),
            "timestamped_rosout_row_count": len(ros_times),
            "move_base_clock_subscription_observed": True,
            "target_service_advertised": False,
        },
        "supporting_files": {
            "base_launch_log": {
                "path": _relative(base_log),
                "sha256": sha256(base_log),
            },
            "master_log": {
                "path": _relative(master_log),
                "sha256": sha256(master_log),
            },
            "rosout_log": {
                "path": _relative(rosout_log),
                "sha256": sha256(rosout_log),
            },
        },
        "semantic_factor_activated": False,
        "transaction_started": False,
        "runtime_evaluator_alignment_observed": False,
    }


def build_assessment():
    preregistration = strict_yaml(PREREGISTRATION)
    authorization = strict_yaml(AUTHORIZATION)
    stage = strict_yaml(STAGE_REPORT)
    _require(
        preregistration.get("stage") == STAGE
        and preregistration.get("execution_authorized") is False,
        "preregistration boundary drifted",
    )
    _require(
        authorization.get("stage") == STAGE
        and authorization.get("execution_authorized") is True
        and authorization.get("evidence_budget_authorized") == 6
        and authorization.get("retry_or_resume_allowed") is False,
        "authorization boundary drifted",
    )
    ledger = stage.get("attempt_ledger")
    _require(
        stage.get("stage") == STAGE
        and stage.get("status") == "terminal_failure"
        and stage.get("formal_result") is False
        and stage.get("runtime_ready") is False
        and stage.get("training_started") is False
        and stage.get("real_vehicle_used") is False
        and stage.get("evidence_units_consumed") == 1
        and stage.get("unattempted_budget_forfeited") == 5
        and stage.get("retry_count") == 0
        and stage.get("resume_used") is False
        and stage.get("resume_forbidden") is True
        and isinstance(ledger, list)
        and len(ledger) == 1,
        "terminal stage boundary drifted",
    )
    entry = ledger[0]
    _require(
        entry.get("identity") == EXPECTED_IDENTITY
        and entry.get("status") == "terminal_failure"
        and entry.get("seed_consumed") is True
        and entry.get("evidence_units_consumed") == 1
        and entry.get("failure_reason") == EXPECTED_FAILURE,
        "terminal attempt identity or reason drifted",
    )
    replay = validate_persisted_attempt(
        WORKSPACE,
        entry,
        preregistration["readiness_gate"][
            "minimum_message_count_per_stream"
        ],
    )
    _require(
        replay.get("identity") == EXPECTED_IDENTITY
        and replay.get("status") == "terminal_failure"
        and replay.get("integrity_pass") is True,
        "terminal journal replay failed",
    )
    bootstrap = _bootstrap_evidence(entry)
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "assessment_id": (
            "fam_teb_v2_04g_r6_i1_terminal_assessment_1"
        ),
        "assessment_date": "2026-07-19",
        "status": "terminal_execution_failure_preserved",
        "assessment_result": "fail",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "execution_authorization": {
            "path": _relative(AUTHORIZATION),
            "sha256": sha256(AUTHORIZATION),
            "evidence_budget_authorized": 6,
        },
        "stage_report": {
            "path": _relative(STAGE_REPORT),
            "sha256": sha256(STAGE_REPORT),
        },
        "attempted_identity_count": 1,
        "evidence_units_consumed": 1,
        "unattempted_budget_forfeited": 5,
        "retry_count": 0,
        "resume_used": False,
        "resume_forbidden": True,
        "terminal_identity": EXPECTED_IDENTITY,
        "terminal_failure_reason": EXPECTED_FAILURE,
        "terminal_journal_replay": replay,
        "bootstrap_root_cause": bootstrap,
        "review_findings": [
            {
                "id": "R6-I1-POST-01",
                "severity": "blocking",
                "finding": (
                    "integration review did not exercise a positive "
                    "/clock bootstrap barrier before move_base readiness"
                ),
            },
            {
                "id": "R6-I1-POST-02",
                "severity": "blocking_for_future_authorization",
                "finding": (
                    "named external Python and runtime bindings are not "
                    "all path-and-SHA closed"
                ),
            },
            {
                "id": "R6-I1-POST-03",
                "severity": "blocking_for_future_authorization",
                "finding": (
                    "runner does not enforce every authorization binding, "
                    "digest, scope flag, and exact schedule invariant"
                ),
            },
            {
                "id": "R6-I1-POST-04",
                "severity": "blocking_for_future_authorization",
                "finding": (
                    "authorization and selected YAML resources are hashed "
                    "and parsed through separate reads"
                ),
            },
            {
                "id": "R6-I1-POST-05",
                "severity": "offline_tooling",
                "finding": (
                    "the preauthorized assessor raises NameError before "
                    "writing; it is preserved byte-for-byte to retain the "
                    "executed dependency closure"
                ),
                "preserved_assessor": {
                    "path": _relative(ORIGINAL_ASSESSOR),
                    "sha256": sha256(ORIGINAL_ASSESSOR),
                },
            },
        ],
        "d1_integrity_outcome": {
            "terminal_journal_no_resume": "pass",
            "exact_terminal_raw_inventory": "pass",
            "compiled_scene_snapshot_materialized": "pass_pre_spawn_only",
            "readiness_direct_counts": "not_reached",
            "two_phase_teardown_restore": "not_reached",
            "semantic_raw_binding": "not_reached",
            "execution_dependency_closure": (
                "authorized_snapshot_preserved_with_post_review_gaps"
            ),
        },
        "claims": {
            "runtime_evaluator_semantic_alignment_validated": False,
            "safety_performance_or_generalization_claimed": False,
            "winner_ranked_or_frozen": False,
            "downstream_authorized": False,
        },
        "next_stage_requirements": [
            "independent_preregistration_and_authorization_only",
            "fresh_seed_and_fresh_budget_only",
            "positive_clock_barrier_before_move_base_readiness",
            "fully_hashed_external_runtime_dependency_closure",
            "single_open_hash_and_parse_for_authorization_resources",
            "closed_authorization_schema_and_exact_schedule_validation",
            "credential_safe_ros_log_environment",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    report = build_assessment()
    if args.write:
        if ASSESSMENT.exists():
            raise FileExistsError(
                "R6-I1 terminal assessment already exists"
            )
        _atomic_yaml(ASSESSMENT, report)
    sys.stdout.write(
        yaml.safe_dump(report, sort_keys=False, allow_unicode=True)
    )
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
