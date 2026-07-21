#!/usr/bin/env python3
"""Deterministic, ROS-free assessment core for the R6-I2 repair review.

This script cannot create an execution authorization and cannot mutate an
execution stage.  Its public assessment function receives every source value
explicitly, including the stage-report digest that was undefined in the
frozen R6-I1 assessor.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path
from typing import Callable, Mapping, Optional

import yaml


STAGE = "V2-04G-R6-I2"
IDENTITY_FIELDS = ("stage", "profile_id", "scene_id", "seed", "attempt")
OVERLAY_RULES = {
    "non_none",
    "non_none_iff_finite_ttc",
    "legacy_non_none_identifiability",
    "none_iff_no_finite_ttc",
}


class R6I2AssessmentError(ValueError):
    """Raised when offline assessment inputs fail closed."""


def _require(condition, message):
    if not condition:
        raise R6I2AssessmentError(message)


def _hex_digest(value, label):
    _require(
        isinstance(value, str)
        and len(value) == 64
        and value == value.lower()
        and all(character in "0123456789abcdef" for character in value),
        "{} must be a lowercase SHA256".format(label),
    )
    return value


def _exact_equal(actual, expected):
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return (
            set(actual) == set(expected)
            and all(_exact_equal(actual[key], expected[key]) for key in expected)
        )
    if isinstance(expected, list):
        return (
            len(actual) == len(expected)
            and all(
                _exact_equal(left, right)
                for left, right in zip(actual, expected)
            )
        )
    return actual == expected


def _expected_identity(stage, row):
    return {
        "stage": stage,
        "profile_id": row["profile_id"],
        "scene_id": row["scene_id"],
        "seed": row["seed"],
        "attempt": row["attempt"],
    }


def _validate_source_boundary(stage, preregistration, stage_report):
    _require(
        isinstance(preregistration, Mapping)
        and preregistration.get("stage") == stage
        and preregistration.get("execution_authorized") is False,
        "preregistration boundary drifted",
    )
    schedule = preregistration.get("schedule")
    if schedule is None:
        schedule = preregistration.get("execution_schedule")
    _require(isinstance(schedule, list), "preregistration schedule is missing")
    _require(
        isinstance(stage_report, Mapping)
        and stage_report.get("stage") == stage,
        "stage report boundary drifted",
    )
    for key in ("formal_result", "runtime_ready"):
        _require(stage_report.get(key) is False, key + " must remain false")
    _require(
        stage_report.get("training_started") is False
        and stage_report.get("real_vehicle_used") is False,
        "training or real-vehicle boundary drifted",
    )
    _require(
        stage_report.get("r5_remaining_units_consumed") == 0
        and stage_report.get("held_out_5001_5010_accessed") is False,
        "prior or held-out budget boundary drifted",
    )
    _require(
        stage_report.get("retry_count") == 0
        and stage_report.get("resume_used") is False,
        "retry or resume boundary drifted",
    )
    ledger = stage_report.get("attempt_ledger")
    _require(isinstance(ledger, list), "attempt ledger is missing")
    _require(
        len(ledger) <= len(schedule),
        "attempt ledger exceeds preregistered schedule",
    )
    expected_prefix = [
        _expected_identity(stage, row) for row in schedule[:len(ledger)]
    ]
    actual = [row.get("identity") for row in ledger]
    _require(
        _exact_equal(actual, expected_prefix),
        "attempt ledger is not the exact schedule prefix",
    )
    consumed = stage_report.get("evidence_units_consumed")
    _require(
        type(consumed) is int and 0 <= consumed <= len(ledger),
        "evidence consumption is inconsistent with the ledger",
    )
    return schedule, ledger


def _semantic_match(schedule_row, ledger_row):
    expected_status = schedule_row["expected_ttc_status"]
    observed_status = ledger_row.get("observed_ttc_status")
    if observed_status != expected_status:
        return False
    rule = schedule_row["expected_overlay_semantics"]
    _require(rule in OVERLAY_RULES, "unknown overlay semantic rule")
    observation = ledger_row.get("semantic_observation")
    _require(
        isinstance(observation, Mapping),
        "semantic observation is missing",
    )
    finite = observation.get("finite_ttc_sample_count")
    overlay = observation.get("non_none_overlay_count")
    _require(
        type(finite) is int and finite >= 0,
        "finite TTC sample count is invalid",
    )
    _require(
        type(overlay) is int and overlay >= 0,
        "non-none overlay count is invalid",
    )
    if rule == "non_none":
        return overlay > 0
    if rule == "non_none_iff_finite_ttc":
        return (overlay > 0) == (finite > 0)
    if rule == "legacy_non_none_identifiability":
        return finite == 0 and overlay > 0
    return (finite == 0 and overlay == 0) or (
        finite > 0 and overlay > 0
    )


def build_offline_assessment(
    *,
    stage,
    preregistration,
    preregistration_sha256,
    stage_report,
    stage_report_sha256,
    replay_attempt: Optional[Callable[[Mapping], Mapping]] = None
):
    """Assess one immutable input state without globals, writes, ROS, or seeds.

    ``replay_attempt`` is mandatory once a ledger is non-empty.  It must replay
    persisted evidence and return a mapping containing ``identity``,
    ``status`` and ``integrity_pass``.  Empty, pre-execution state can be
    assessed without a replay callback.
    """

    _require(stage == STAGE, "assessment stage drifted")
    prereg_digest = _hex_digest(
        preregistration_sha256, "preregistration SHA256"
    )
    stage_input_digest = _hex_digest(
        stage_report_sha256, "stage report input SHA256"
    )
    schedule, ledger = _validate_source_boundary(
        stage, preregistration, stage_report
    )
    _require(
        not ledger or replay_attempt is not None,
        "non-empty ledger requires direct persisted replay",
    )
    replays = []
    integrity_failures = []
    for entry in ledger:
        try:
            replay = dict(replay_attempt(copy.deepcopy(entry)))
            _require(
                _exact_equal(replay.get("identity"), entry["identity"]),
                "replayed identity mismatched ledger",
            )
            _require(
                replay.get("integrity_pass") is True,
                "persisted replay did not pass",
            )
            replay["sequence"] = entry.get("sequence")
            replays.append(replay)
        except Exception as exc:
            integrity_failures.append({
                "sequence": entry.get("sequence"),
                "identity": copy.deepcopy(entry.get("identity")),
                "error": "{}: {}".format(type(exc).__name__, exc),
            })

    complete = bool(
        schedule
        and len(ledger) == len(schedule)
        and len(replays) == len(schedule)
        and not integrity_failures
        and all(row.get("status") == "evidence_complete" for row in replays)
    )
    semantic_rows_pass = bool(
        complete
        and all(
            _semantic_match(schedule_row, ledger_row)
            for schedule_row, ledger_row in zip(schedule, ledger)
        )
    )
    integration_pass = bool(complete and semantic_rows_pass)
    if integration_pass:
        status = "simulation_integration_validation_pass"
    elif stage_report.get("status") == "terminal_failure":
        status = "terminal_execution_failure_preserved"
    elif not ledger:
        status = "no_execution_state_reviewed"
    else:
        status = "simulation_integration_validation_fail"
    if integration_pass:
        assessment_result = "pass"
    elif ledger:
        assessment_result = "fail"
    else:
        assessment_result = "not_executed"
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": stage,
        "assessment_id": "fam_teb_v2_04g_r6_i2_offline_assessment_1",
        "status": status,
        "assessment_result": assessment_result,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "source_bindings": {
            "preregistration_sha256": prereg_digest,
            "stage_report_input_sha256": stage_input_digest,
        },
        "planned_identity_count": len(schedule),
        "attempted_identity_count": len(ledger),
        "evidence_units_consumed": stage_report[
            "evidence_units_consumed"
        ],
        "retry_count": 0,
        "resume_used": False,
        "all_attempt_journals_directly_replayed": (
            len(replays) == len(ledger)
        ),
        "attempt_replays": replays,
        "integrity_failures": integrity_failures,
        "semantic_schedule_pass": semantic_rows_pass,
        "integration_validation_pass": integration_pass,
        "winner_ranked_or_frozen": False,
        "downstream_authorized": False,
        "claim_limit":
            "fresh_simulation_runtime_evaluator_semantic_integration_only",
    }


def build_repair_review():
    """Return the persisted, zero-authority review for this implementation."""

    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "review_id":
            "fam_teb_v2_04g_r6_i2_authorization_assessment_review_1",
        "status": "repair_component_review_pass_execution_not_authorized",
        "review_result": "pass",
        "offline_only": True,
        "execution_authorized": False,
        "real_authorization_created": False,
        "seed_values": [],
        "evidence_budget_units": 0,
        "ros_or_gazebo_started": False,
        "repairs": {
            "single_open_no_follow_hash_and_parse": True,
            "closed_authorization_top_level_schema": True,
            "closed_nested_authorization_schema": True,
            "exact_preregistration_schedule_binding": True,
            "type_sensitive_schedule_comparison": True,
            "all_bound_resources_single_read_verified": True,
            "assessor_source_digests_explicitly_scoped": True,
            "assessor_has_no_free_stage_document_reference": True,
            "assessor_is_deterministic_and_write_free": True,
        },
        "negative_test_classes": [
            "authorization_symlink",
            "duplicate_yaml_key",
            "caller_hash_mismatch",
            "extra_top_level_key",
            "extra_nested_key",
            "schedule_reorder",
            "schedule_type_confusion",
            "resource_hash_mismatch",
            "independent_logical_closure_digest_mismatch",
            "prior_stage_reuse_flag",
            "nonempty_ledger_without_replay",
            "ledger_not_exact_schedule_prefix",
        ],
        "scope_limit":
            "implementation_and_offline_review_only_no_execution_authority",
    }


def _canonical_sha256(value):
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check-report",
        help="compare deterministic review with an existing YAML report",
    )
    args = parser.parse_args()
    report = build_repair_review()
    if args.check_report:
        persisted = yaml.safe_load(
            Path(args.check_report).read_text(encoding="utf-8")
        )
        if not _exact_equal(persisted, report):
            raise R6I2AssessmentError(
                "persisted repair review is not deterministic"
            )
    print(yaml.safe_dump(report, sort_keys=False, allow_unicode=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
