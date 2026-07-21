#!/usr/bin/env python3
"""Offline assessment of every canonical R6-I1 attempt journal."""

import argparse
import hashlib
import os
from pathlib import Path
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
ASSESSMENT = ROOT / "v2_04g_r6_i1_assessment.yaml"
PREREGISTRATION = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_execution_preregistration.yaml"
)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _atomic_yaml(path, value):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
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


def _expected_identity(row):
    stage_report_input_sha256 = (
        stage.get("assessment", {}).get("source_stage_report_sha256")
        if stage.get("assessment_complete") is True
        else None
    ) or sha256(STAGE_REPORT)
    return {
        "stage": STAGE,
        "profile_id": row["profile_id"],
        "scene_id": row["scene_id"],
        "seed": row["seed"],
        "attempt": row["attempt"],
    }


def build_assessment():
    prereg = strict_yaml(PREREGISTRATION)
    stage = strict_yaml(STAGE_REPORT)
    if not (
        prereg.get("stage") == STAGE
        and stage.get("stage") == STAGE
        and stage.get("formal_result") is False
        and stage.get("runtime_ready") is False
        and stage.get("training_started") is False
        and stage.get("real_vehicle_used") is False
        and stage.get("r5_remaining_units_consumed") == 0
        and stage.get("held_out_5001_5010_accessed") is False
    ):
        raise ValueError("R6-I1 stage safety boundary drifted")
    schedule = prereg["schedule"]
    ledger = stage["attempt_ledger"]
    if len(ledger) > len(schedule):
        raise ValueError("attempt ledger exceeds preregistered schedule")
    expected_prefix = [
        _expected_identity(row) for row in schedule[:len(ledger)]
    ]
    actual = [row.get("identity") for row in ledger]
    if actual != expected_prefix:
        raise ValueError("attempt ledger is not the exact schedule prefix")
    minimum = prereg["readiness_gate"][
        "minimum_message_count_per_stream"
    ]
    attempts = []
    integrity_failures = []
    for entry in ledger:
        try:
            replay = validate_persisted_attempt(
                WORKSPACE, entry, minimum
            )
            replay["sequence"] = entry["sequence"]
            replay["expected_ttc_status"] = entry.get(
                "expected_ttc_status"
            )
            replay["observed_ttc_status"] = entry.get(
                "observed_ttc_status"
            )
            replay["semantic_observation"] = entry.get(
                "semantic_observation"
            )
            attempts.append(replay)
        except Exception as exc:
            integrity_failures.append({
                "sequence": entry.get("sequence"),
                "identity": entry.get("identity"),
                "error": "{}: {}".format(type(exc).__name__, exc),
            })
    complete = (
        len(ledger) == 6
        and len(attempts) == 6
        and not integrity_failures
        and all(row["status"] == "evidence_complete" for row in attempts)
    )
    ttc_match = complete and all(
        row["expected_ttc_status"] == row["observed_ttc_status"]
        for row in attempts
    )
    by_identity = {
        (
            row["identity"]["profile_id"],
            row["identity"]["scene_id"],
        ): row
        for row in attempts
    }
    clear_scene = "v2-04g-r6-i1-dynamic-semantic-clear-s5143"
    legacy_clear = by_identity.get(
        ("r6_semantics_legacy_control", clear_scene), {}
    ).get("semantic_observation") or {}
    aligned_clear = by_identity.get(
        ("r6_semantics_circle_contact", clear_scene), {}
    ).get("semantic_observation") or {}
    semantic_identifiability = bool(
        complete
        and legacy_clear.get("finite_ttc_sample_count") == 0
        and legacy_clear.get("non_none_overlay_count", 0) > 0
        and aligned_clear.get("finite_ttc_sample_count") == 0
        and aligned_clear.get("non_none_overlay_count") == 0
    )
    integration_pass = bool(
        complete and ttc_match and semantic_identifiability
    )
    if integration_pass:
        status = "simulation_integration_validation_pass"
    elif stage.get("status") == "terminal_failure":
        status = "terminal_execution_failure_preserved"
    else:
        status = "simulation_integration_validation_fail"
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "assessment_id": "fam_teb_v2_04g_r6_i1_assessment_1",
        "status": status,
        "assessment_result": "pass" if integration_pass else "fail",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "preregistration": {
            "path": str(PREREGISTRATION.relative_to(WORKSPACE)),
            "sha256": sha256(PREREGISTRATION),
        },
        "stage_report_input_sha256": stage_report_input_sha256,
        "planned_identity_count": 6,
        "attempted_identity_count": len(ledger),
        "evidence_budget_authorized": 6,
        "evidence_units_consumed": stage.get(
            "evidence_units_consumed", 0
        ),
        "r5_remaining_units_consumed": 0,
        "held_out_5001_5010_accessed": False,
        "attempt_limit_per_identity": 1,
        "retry_count": stage.get("retry_count", 0),
        "resume_used": stage.get("resume_used", False),
        "all_attempt_journals_directly_replayed": (
            len(attempts) == len(ledger)
        ),
        "attempt_replays": attempts,
        "integrity_failures": integrity_failures,
        "integrity_protocols": {
            "readiness_direct_counts": not integrity_failures,
            "compiled_scene_snapshot_pre_and_post": not integrity_failures,
            "terminal_journal_no_resume": not integrity_failures,
            "exact_raw_evidence_binding": not integrity_failures,
            "execution_dependency_closure": True,
            "two_phase_teardown_restore": (
                not integrity_failures
                and all(
                    row.get("status") != "evidence_complete"
                    or bool(row.get("startup_profile_sha256"))
                    for row in attempts
                )
            ),
        },
        "ttc_status_matches_preregistration": bool(ttc_match),
        "semantic_clear_pair_identifiable": semantic_identifiability,
        "integration_validation_pass": integration_pass,
        "winner_ranked_or_frozen": False,
        "safety_performance_or_generalization_claimed": False,
        "downstream_authorized": False,
        "claim_limit": (
            "fresh_simulation_runtime_evaluator_semantic_integration_only"
        ),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    report = build_assessment()
    if args.write:
        if ASSESSMENT.exists():
            raise FileExistsError("R6-I1 assessment already exists")
        source_stage_report_sha256 = sha256(STAGE_REPORT)
        _atomic_yaml(ASSESSMENT, report)
        stage = strict_yaml(STAGE_REPORT)
        stage.update({
            "status": (
                "complete"
                if report["integration_validation_pass"]
                else "terminal_failure"
            ),
            "assessment_complete": True,
            "assessment": {
                "path": str(ASSESSMENT.relative_to(WORKSPACE)),
                "sha256": sha256(ASSESSMENT),
                "source_stage_report_sha256": source_stage_report_sha256,
            },
            "formal_result": False,
            "runtime_ready": False,
            "winner_ranked_or_frozen": False,
            "downstream_authorized": False,
        })
        _atomic_yaml(STAGE_REPORT, stage)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0 if report["integration_validation_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
