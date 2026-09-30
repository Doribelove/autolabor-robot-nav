#!/usr/bin/env python3
"""Replay P2-C1 raw journals and freeze calibration-only selections."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P2-C1"
REPLAY_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p2_q1_replay.py"
)
_SPEC = importlib.util.spec_from_file_location("p2_c1_frozen_q1_replay", REPLAY_PATH)
REPLAY = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(REPLAY)

CIRCLE_METHODS = (
    "circle_contact_v120",
    "circle_contact_v125",
    "circle_contact_v130",
    "circle_contact_v135",
    "circle_contact_v140",
)
FIXED_METHODS = (
    "fixed_teb_v120__TEB-Default_alias",
    "fixed_teb_v125",
    "fixed_teb_v130",
    "fixed_teb_v135",
    "fixed_teb_v140",
)
TUNED = "TEB-Tuned-V1"
ROLES = (
    "single_true_conflict",
    "multi_true_conflict",
    "time_separated_semantic_clear",
)
CLEAR_ROLE = "time_separated_semantic_clear"


class C1AssessmentError(ValueError):
    """Raised when C1 raw evidence or selection invariants differ."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise C1AssessmentError(message)


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    _require(type(value) is dict, "YAML root differs: {}".format(path))
    return value


def _verify_receipt(workspace: Path, receipt: Mapping[str, Any]) -> bytes:
    _require(
        type(receipt) is dict
        and set(receipt) == {"path", "sha256", "size_bytes"},
        "raw receipt schema differs",
    )
    relative = Path(receipt["path"])
    _require(
        not relative.is_absolute() and ".." not in relative.parts,
        "raw receipt path is unsafe",
    )
    payload = (workspace / relative).read_bytes()
    _require(_sha(payload) == receipt["sha256"], "raw receipt SHA differs")
    _require(len(payload) == receipt["size_bytes"], "raw receipt size differs")
    return payload


def identity(row: Mapping[str, Any]) -> Dict[str, Any]:
    common = {
        "sequence": row["sequence"],
        "row_type": row["row_type"],
        "scene_id": row["scene_id"],
        "seed": row["seed"],
        "attempt": row["attempt"],
    }
    if row["row_type"] == "calibration_navigation":
        common.update(
            {
                "method_id": row["method_id"],
                "block_id": row["block_id"],
                "scene_role": row["scene_role"],
            }
        )
    else:
        common.update(
            {
                "probe_type": row["probe_type"],
                "candidate_speed_mps": row["candidate_speed_mps"],
            }
        )
    return common


def replay_one(
    workspace: Path,
    row: Mapping[str, Any],
    journal_path: Path,
    release_sha256: str,
    thresholds: Mapping[str, Any],
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    journal_payload = journal_path.read_bytes()
    journal = yaml.safe_load(journal_payload)
    _require(type(journal) is dict, "journal root differs")
    _require(journal.get("stage") == STAGE, "journal stage differs")
    _require(journal.get("release_sha256") == release_sha256, "journal release differs")
    _require(journal.get("identity") == identity(row), "journal identity differs")
    _require(journal.get("status") == "completed", "journal is not complete")
    _require(journal.get("consumed") is True, "journal is not consumed")
    _require(journal.get("integrity_failures") == [], "journal has integrity failure")
    resources = journal.get("raw_resources")
    _require(type(resources) is dict and "raw_trace" in resources, "raw trace receipt absent")
    raw_payload = _verify_receipt(workspace, resources["raw_trace"])
    raw = yaml.safe_load(raw_payload)
    _require(type(raw) is dict and raw.get("stage") == STAGE, "raw trace stage differs")
    expected_raw_identity = {
        key: value
        for key, value in identity(row).items()
        if key != "scene_role"
    }
    _require(raw.get("identity") == expected_raw_identity, "raw trace identity differs")
    if row["row_type"] == "calibration_navigation":
        normalized = REPLAY.normalize_navigation_trace(row, raw.get("samples"))
        normalized["terminal_status"] = "completed"
        normalized["q1_source_sequence"] = row["q1_source_sequence"]
    else:
        normalized = REPLAY.assess_qualification_trace(
            row, raw.get("samples"), thresholds
        )
        normalized["terminal_status"] = "completed"
        normalized["q1_source_sequence"] = row["q1_source_sequence"]
    audit = {
        "sequence": row["sequence"],
        "journal_path": str(journal_path.relative_to(workspace)),
        "journal_sha256": _sha(journal_payload),
        "raw_trace_path": resources["raw_trace"]["path"],
        "raw_trace_sha256": resources["raw_trace"]["sha256"],
        "normalized_sha256": REPLAY.canonical_sha256(normalized),
    }
    return normalized, audit


def _geometric(values: Sequence[float]) -> float:
    _require(values and all(value > 0.0 for value in values), "geometric input differs")
    return math.exp(sum(math.log(value) for value in values) / len(values))


def _by_method(rows: Sequence[Mapping[str, Any]], method: str):
    return [row for row in rows if row.get("method_id") == method]


def _role_counts(rows: Sequence[Mapping[str, Any]], field: str):
    return {
        role: sum(bool(row[field]) for row in rows if row["scene_role"] == role)
        for role in ROLES
    }


def _role_clearance(rows: Sequence[Mapping[str, Any]]):
    return {
        role: sum(
            row["minimum_footprint_clearance_m"]
            for row in rows
            if row["scene_role"] == role
        )
        / float(sum(row["scene_role"] == role for row in rows))
        for role in ROLES
    }


def _equal_role_time(rows: Sequence[Mapping[str, Any]]) -> float:
    return _geometric(
        [
            _geometric(
                [
                    row["restricted_completion_time_s"]
                    for row in rows
                    if row["scene_role"] == role
                ]
            )
            for role in ROLES
        ]
    )


def _semantic_clear_time(rows: Sequence[Mapping[str, Any]]) -> float:
    return _geometric(
        [
            row["restricted_completion_time_s"]
            for row in rows
            if row["scene_role"] == CLEAR_ROLE
        ]
    )


def _speed(method: str) -> float:
    if method == TUNED:
        return 0.9
    token = method.split("_v", 1)[1].split("_", 1)[0]
    return int(token) / 100.0


def _pick_within_one_percent(
    eligible: Sequence[Mapping[str, Any]],
    metric: str,
    *,
    prefer_tuned: bool = False,
) -> Mapping[str, Any] | None:
    if not eligible:
        return None
    best = min(row[metric] for row in eligible)
    tied = [row for row in eligible if row[metric] <= best * 1.01 + 1e-12]
    if prefer_tuned:
        tuned = [row for row in tied if row["method_id"] == TUNED]
        if tuned:
            return tuned[0]
    return min(tied, key=lambda row: (_speed(row["method_id"]), row["method_id"]))


def build_assessment(
    workspace: Path,
    schedule: Mapping[str, Any],
    journal_root: Path,
    skipped_path: Path,
    release_sha256: str,
    thresholds: Mapping[str, Any],
) -> Tuple[list, Dict[str, Any], Dict[str, Any]]:
    rows = schedule["rows"]
    skipped_document = _load_yaml(skipped_path)
    skipped = skipped_document.get("skipped_identities", [])
    skipped_by_sequence = {row["sequence"]: row for row in skipped}
    _require(len(skipped_by_sequence) == len(skipped), "skipped identity duplicate")
    normalized = []
    audits = []
    for row in rows:
        journal = journal_root / "sequence_{:04d}.yaml".format(row["sequence"])
        if row["sequence"] in skipped_by_sequence:
            _require(not journal.exists(), "skipped identity has a journal")
            record = skipped_by_sequence[row["sequence"]]
            _require(record["identity"] == identity(row), "skipped identity differs")
            _require(
                record["reason"] == "qualification_speed_disqualified",
                "skipped reason differs",
            )
            continue
        _require(journal.is_file(), "journal absent for non-skipped identity")
        item, audit = replay_one(
            workspace, row, journal, release_sha256, thresholds
        )
        normalized.append(item)
        audits.append(audit)

    qualifications = [
        row for row in normalized if row["row_type"] == "qualification_probe"
    ]
    calibration = [
        row for row in normalized if row["row_type"] == "calibration_navigation"
    ]
    qualification_by_speed = {}
    first_failed_speed = None
    for speed in (1.25, 1.30, 1.35, 1.40):
        probes = [
            row for row in qualifications if row["candidate_speed_mps"] == speed
        ]
        if first_failed_speed is None:
            _require(len(probes) == 3, "executed qualification triplet differs")
            passed = all(row["pass"] for row in probes)
            qualification_by_speed[speed] = {
                "executed": True,
                "all_three_pass": passed,
                "probe_results": {
                    row["probe_type"]: bool(row["pass"]) for row in probes
                },
            }
            if not passed:
                first_failed_speed = speed
        else:
            _require(probes == [], "higher speed executed after qualification failure")
            qualification_by_speed[speed] = {
                "executed": False,
                "all_three_pass": False,
                "reason": "lower_speed_qualification_failed",
            }
    qualified_speeds = [
        1.2,
        *[
            speed
            for speed, result in qualification_by_speed.items()
            if result["executed"] and result["all_three_pass"]
        ],
    ]
    disqualified = {
        speed for speed in (1.25, 1.30, 1.35, 1.40) if speed not in qualified_speeds
    }
    expected_skipped_methods = {
        method
        for method in CIRCLE_METHODS + FIXED_METHODS
        if _speed(method) in disqualified
    }
    expected_skipped_sequences = {
        row["sequence"]
        for row in rows
        if (
            row["row_type"] == "qualification_probe"
            and first_failed_speed is not None
            and row["candidate_speed_mps"] > first_failed_speed
        )
        or (
            row["row_type"] == "calibration_navigation"
            and row["method_id"] in expected_skipped_methods
        )
    }
    _require(
        set(skipped_by_sequence) == expected_skipped_sequences,
        "skipped sequence roster differs",
    )

    control_circle = _by_method(calibration, "circle_contact_v120")
    tuned_rows = _by_method(calibration, TUNED)
    _require(len(control_circle) == 30 and len(tuned_rows) == 30, "frozen controls incomplete")
    control_success = _role_counts(control_circle, "success")
    control_clearance = _role_clearance(control_circle)
    tuned_success = _role_counts(tuned_rows, "success")
    tuned_clearance = _role_clearance(tuned_rows)

    circle_candidates = []
    for method in CIRCLE_METHODS:
        candidate = _by_method(calibration, method)
        if _speed(method) not in qualified_speeds:
            _require(candidate == [], "disqualified circle candidate executed")
            continue
        _require(len(candidate) == 30, "eligible circle matrix incomplete")
        success = _role_counts(candidate, "success")
        clearance = _role_clearance(candidate)
        eligibility = {
            "complete_matrix": len(candidate) == 30,
            "zero_collision": not any(row["collision"] for row in candidate),
            "zero_emergency": not any(row["emergency"] for row in candidate),
            "zero_invalid_transaction": not any(
                row["invalid_transaction_count"] for row in candidate
            ),
            "role_success_noninferior_to_v120": all(
                success[role] >= control_success[role] for role in ROLES
            ),
            "role_clearance_point_difference_min_minus_0_05": all(
                clearance[role] - control_clearance[role] >= -0.05 - 1e-12
                for role in ROLES
            ),
            "normal_projection_rate_below_0_10": (
                sum(
                    row["normal_projection_rate"] * row["normal_trace_sample_count"]
                    for row in candidate
                )
                / sum(row["normal_trace_sample_count"] for row in candidate)
                < 0.10
            ),
        }
        circle_candidates.append(
            {
                "method_id": method,
                "speed_mps": _speed(method),
                "eligible": all(eligibility.values()),
                "eligibility": eligibility,
                "semantic_clear_geometric_restricted_time_s": _semantic_clear_time(
                    candidate
                ),
                "equal_role_geometric_restricted_time_s": _equal_role_time(candidate),
            }
        )

    fixed_candidates = []
    for method in FIXED_METHODS:
        candidate = _by_method(calibration, method)
        if _speed(method) not in qualified_speeds:
            _require(candidate == [], "disqualified fixed candidate executed")
            continue
        _require(len(candidate) == 30, "eligible fixed matrix incomplete")
        success = _role_counts(candidate, "success")
        clearance = _role_clearance(candidate)
        eligibility = {
            "complete_matrix": len(candidate) == 30,
            "zero_collision": not any(row["collision"] for row in candidate),
            "zero_emergency": not any(row["emergency"] for row in candidate),
            "zero_invalid_transaction": not any(
                row["invalid_transaction_count"] for row in candidate
            ),
            "role_success_noninferior_to_TEB_Tuned_V1": all(
                success[role] >= tuned_success[role] for role in ROLES
            ),
            "role_clearance_point_difference_min_minus_0_05": all(
                clearance[role] - tuned_clearance[role] >= -0.05 - 1e-12
                for role in ROLES
            ),
        }
        fixed_candidates.append(
            {
                "method_id": method,
                "speed_mps": _speed(method),
                "eligible": all(eligibility.values()),
                "equal_role_geometric_restricted_time_s": _equal_role_time(candidate),
            }
        )

    selected_circle = _pick_within_one_percent(
        [row for row in circle_candidates if row["eligible"]],
        "semantic_clear_geometric_restricted_time_s",
    )
    selected_fixed = _pick_within_one_percent(
        [row for row in fixed_candidates if row["eligible"]],
        "equal_role_geometric_restricted_time_s",
    )
    tuned_eligibility = {
        "complete_matrix": len(tuned_rows) == 30,
        "zero_collision": not any(row["collision"] for row in tuned_rows),
        "zero_emergency": not any(row["emergency"] for row in tuned_rows),
        "zero_invalid_transaction": not any(
            row["invalid_transaction_count"] for row in tuned_rows
        ),
    }
    fair_candidates = []
    if all(tuned_eligibility.values()):
        fair_candidates.append(
            {
                "method_id": TUNED,
                "eligible": True,
                "equal_role_geometric_restricted_time_s": _equal_role_time(
                    tuned_rows
                ),
            }
        )
    if selected_fixed is not None:
        fair_candidates.append(dict(selected_fixed))
    primary = _pick_within_one_percent(
        fair_candidates,
        "equal_role_geometric_restricted_time_s",
        prefer_tuned=True,
    )
    integrity_complete = (
        len(normalized) + len(skipped_by_sequence) == 342
        and all(row["terminal_status"] == "completed" for row in normalized)
    )
    can_design_test = bool(
        integrity_complete and selected_circle is not None and primary is not None
    )
    replay_report = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "complete_raw_replay",
        "release_sha256": release_sha256,
        "schedule_rows": 342,
        "consumed_rows_replayed": len(normalized),
        "skipped_rows_verified": len(skipped_by_sequence),
        "journal_audits": audits,
        "all_raw_receipts_verified": True,
        "held_out_test_rows": 0,
    }
    assessment = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": (
            "calibration_selection_frozen_test_design_eligible"
            if can_design_test
            else "calibration_negative_no_test"
        ),
        "formal_result": False,
        "performance_improvement_demonstrated": False,
        "runtime_ready": False,
        "release_sha256": release_sha256,
        "qualification": {
            "by_speed_mps": qualification_by_speed,
            "first_failed_speed_mps": first_failed_speed,
            "qualified_speeds_mps": qualified_speeds,
        },
        "circle_candidates": circle_candidates,
        "fixed_candidates": fixed_candidates,
        "teb_tuned_v1": {
            "method_id": TUNED,
            "eligible": all(tuned_eligibility.values()),
            "eligibility": tuned_eligibility,
            "equal_role_geometric_restricted_time_s": _equal_role_time(tuned_rows),
        },
        "selected_circle_candidate": selected_circle,
        "selected_independent_fixed_teb": selected_fixed,
        "selected_primary_fair_baseline": primary,
        "calibration_only_selection": True,
        "held_out_test_accessed": False,
        "held_out_test_design_eligible": can_design_test,
        "positive_performance_claim_allowed": False,
    }
    return normalized, replay_report, assessment


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--schedule", type=Path, required=True)
    parser.add_argument("--journals", type=Path, required=True)
    parser.add_argument("--skipped", type=Path, required=True)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--thresholds-contract", type=Path, required=True)
    args = parser.parse_args()
    schedule = _load_yaml(args.schedule)
    contract = _load_yaml(args.thresholds_contract)
    normalized, replay, assessment = build_assessment(
        WORKSPACE,
        schedule,
        args.journals,
        args.skipped,
        args.release_sha256,
        contract["qualification_thresholds"],
    )
    print(
        json.dumps(
            {
                "normalized": normalized,
                "replay": replay,
                "assessment": assessment,
            },
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
