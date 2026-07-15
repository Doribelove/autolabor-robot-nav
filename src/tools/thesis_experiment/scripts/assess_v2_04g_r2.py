#!/usr/bin/env python3
"""Fail-closed assessment for V2-04G-R2 calibration-only mechanisms."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml

from thesis_experiment.v2_evaluator import load_v2_trace, trace_sha256


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
LEGACY_ASSESSOR = Path(__file__).with_name("assess_v2_04g.py")
_SPEC = importlib.util.spec_from_file_location("assess_v2_04g_frozen_for_r2", LEGACY_ASSESSOR)
_LEGACY = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_LEGACY)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load_r2(progress_path):
    progress = yaml.safe_load(Path(progress_path).read_text(encoding="utf-8"))
    if not (
        progress.get("stage") == "V2-04G-R2"
        and progress.get("status") == "complete"
        and progress.get("valid_evidence_episode_count") == 60
        and progress.get("interface_failure_count") == 0
    ):
        raise ValueError("V2-04G-R2 navigation evidence is incomplete")
    rows = {}
    for record in progress["episodes"]:
        path = Path(record["evaluation"])
        if _sha256(path) != record["evaluation_sha256"]:
            raise ValueError("V2-04G-R2 evaluation hash drifted")
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
        if value.get("stage") != "V2-04G-R2" or value.get("split") != "calibration":
            raise ValueError("V2-04G-R2 evaluation boundary drifted")
        trace_path = path.parent / "trace.csv"
        load_v2_trace(trace_path)
        if trace_sha256(trace_path) != value["raw_trace_sha256"]:
            raise ValueError("V2-04G-R2 trace hash drifted")
        key = (value["supervisor_profile_id"], value["scene_id"])
        if key in rows:
            raise ValueError("duplicate V2-04G-R2 evidence identity")
        rows[key] = value
    return progress, rows


def _activation_pass(summary_path, preregistration_path, planned_count):
    summary = yaml.safe_load(Path(summary_path).read_text(encoding="utf-8"))
    return bool(
        summary.get("stage") == "V2-04G-R2"
        and summary.get("status") == "complete"
        and summary.get("planned_probe_count") == planned_count
        and summary.get("valid_probe_count") == planned_count
        and summary.get("all_probe_hard_gates_pass") is True
        and summary.get("navigation_authorized") is True
        and summary.get("training_started") is False
        and summary.get("real_vehicle_used") is False
        and summary.get("preregistration", {}).get("sha256")
        == _sha256(preregistration_path)
    )


def assess(progress_path, prereg_path, contract_path, ttc_probe_path, activation_path):
    prereg = yaml.safe_load(Path(prereg_path).read_text(encoding="utf-8"))
    contract = yaml.safe_load(Path(contract_path).read_text(encoding="utf-8"))
    if prereg.get("stage") != "V2-04G-R2" or contract.get("stage") != "V2-04G-R2":
        raise ValueError("V2-04G-R2 assessment boundary drifted")
    candidates = tuple(prereg["candidate_ids"])
    activation_ok = _activation_pass(
        activation_path, prereg_path,
        prereg["activation_readiness_probe"]["planned_probe_count"],
    )
    frozen_load = _LEGACY._load
    frozen_candidates = _LEGACY.CANDIDATES
    _LEGACY._load = _load_r2
    _LEGACY.CANDIDATES = candidates
    try:
        report = _LEGACY.assess(
            progress_path, prereg_path, contract_path, ttc_probe_path
        )
    finally:
        _LEGACY._load = frozen_load
        _LEGACY.CANDIDATES = frozen_candidates
    _, evidence = _load_r2(progress_path)
    join_gate = contract["hard_gates"]["bounded_context_join"]
    allowed_reasons = set(join_gate["allowed_join_reasons"])
    mechanism_gate = contract["hard_gates"]["mechanism_activation"]
    priority_gate = contract["hard_gates"]["priority_family_efficiency"]
    fixed_valid = report["fixed_reference"]["validity_gate_pass"]
    for candidate, summary in report["candidate_summaries"].items():
        rows = [row for (profile, _), row in evidence.items() if profile == candidate]
        fractions = [row.get("mechanism_join_valid_fraction") for row in rows]
        reason_sets = [set(row.get("mechanism_join_reason_counts", {})) for row in rows]
        join_pass = bool(fractions) and all(
            value is not None
            and value >= join_gate["minimum_valid_fraction_per_episode"]
            for value in fractions
        ) and all(reasons.issubset(allowed_reasons) for reasons in reason_sets)
        reason_counts = {}
        for row in rows:
            for reason, count in row.get("mechanism_join_reason_counts", {}).items():
                reason_counts[reason] = reason_counts.get(reason, 0) + count
        maneuver_rows = [row for row in rows if row["family"] == "MANEUVER"]
        reverse_episode_count = sum(
            row.get("mechanism_maneuver_reverse_sample_count", 0) > 0
            for row in maneuver_rows
        )
        reverse_sample_count = sum(
            row.get("mechanism_maneuver_reverse_sample_count", 0)
            for row in maneuver_rows
        )
        reverse_pass = (
            reverse_episode_count >= mechanism_gate["maneuver_reverse_episode_count_min"]
            and reverse_sample_count >= mechanism_gate["maneuver_reverse_sample_count_min"]
        )
        target_changes = {
            family: summary["family_time"][family]["median_relative_change_percent"]
            for family in priority_gate["families"]
        }
        priority_pass = all(
            value <= priority_gate["median_navigation_time_regression_percent_max"]
            for value in target_changes.values()
        )
        summary["bounded_context_join"] = {
            "minimum_valid_fraction_per_episode": min(fractions) if fractions else None,
            "reason_counts": reason_counts, "pass": join_pass,
        }
        summary["priority_mechanism_evidence"] = {
            "maneuver_episode_with_reverse_count": reverse_episode_count,
            "maneuver_reverse_sample_count": reverse_sample_count,
            "maneuver_reverse_observability_pass": reverse_pass,
            "target_family_median_time_changes_percent": target_changes,
            "priority_family_efficiency_pass": priority_pass,
        }
        summary["hard_gates"]["activation_readiness"] = activation_ok
        summary["hard_gates"]["bounded_context_join"] = join_pass
        summary["hard_gates"]["maneuver_reverse_observability"] = reverse_pass
        summary["hard_gates"]["priority_family_efficiency"] = priority_pass
        summary["hard_gates"]["family_non_regression_count"] = (
            summary["family_non_regression_count"]
            >= contract["hard_gates"]["efficiency_vs_fixed"][
                "family_non_regression_count_min"
            ]
        )
        summary["all_hard_gates_pass"] = fixed_valid and all(
            summary["hard_gates"].values()
        )
    passing = [
        candidate for candidate, summary in report["candidate_summaries"].items()
        if summary["all_hard_gates_pass"]
    ]
    passing.sort(key=lambda candidate: (
        report["candidate_summaries"][candidate]["total_navigation_time_ratio_vs_fixed"],
        -report["candidate_summaries"][candidate]["family_non_regression_count"],
        -report["candidate_summaries"][candidate]["minimum_clearance_m"],
        report["candidate_summaries"][candidate]["active_anchor_switch_mean"],
        candidate,
    ))
    winner = passing[0] if passing else None
    report.update({
        "stage": "V2-04G-R2", "activation_readiness_probe_pass": activation_ok,
        "passing_candidate_ids": passing, "winner_candidate_id": winner,
    })
    report["evidence"]["activation_readiness_probe"] = {
        "path": str(activation_path), "sha256": _sha256(activation_path),
    }
    report["decision"].update({
        "freeze_authorized": winner is not None,
        "generate_fresh_held_out_validation_authorized": winner is not None,
        "enter_v2_05_authorized": False, "sac_training_authorized": False,
        "real_vehicle_authorized": False,
    })
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--progress", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r2/v2_04g_r2_progress.yaml")
    parser.add_argument("--preregistration", type=Path, default=WORKSPACE /
        "experiments/manifests/v2/calibration/v2_04g_r2_preregistration.yaml")
    parser.add_argument("--contract", type=Path, default=WORKSPACE /
        "config/thesis_experiments/v2/v2_04g_r2_mechanism_repair_contract.yaml")
    parser.add_argument("--ttc-probe", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r2/v2_04g_r2_ttc_three_state_probe.yaml")
    parser.add_argument("--activation-probe", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r2/activation_probe/activation_probe_summary.yaml")
    parser.add_argument("--output", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r2/v2_04g_r2_assessment.yaml")
    args = parser.parse_args()
    report = assess(
        args.progress, args.preregistration, args.contract,
        args.ttc_probe, args.activation_probe,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    temporary.replace(args.output)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
