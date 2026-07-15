#!/usr/bin/env python3
"""Fail-closed assessment for V2-04G-R1 calibration-only evidence."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml

from thesis_experiment.v2_evaluator import load_v2_trace, trace_sha256


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
LEGACY_ASSESSOR = Path(__file__).with_name("assess_v2_04g.py")
_SPEC = importlib.util.spec_from_file_location("assess_v2_04g_frozen", LEGACY_ASSESSOR)
_LEGACY = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_LEGACY)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _load_r1(progress_path):
    progress = yaml.safe_load(Path(progress_path).read_text(encoding="utf-8"))
    if not (
        progress.get("stage") == "V2-04G-R1"
        and progress.get("status") == "complete"
        and progress.get("valid_evidence_episode_count") == 60
        and progress.get("interface_failure_count") == 0
    ):
        raise ValueError("V2-04G-R1 navigation evidence is incomplete")
    rows = {}
    for record in progress["episodes"]:
        path = Path(record["evaluation"])
        if _sha256(path) != record["evaluation_sha256"]:
            raise ValueError("V2-04G-R1 evaluation hash drifted")
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
        if value.get("stage") != "V2-04G-R1" or value.get("split") != "calibration":
            raise ValueError("V2-04G-R1 evaluation boundary drifted")
        trace_path = path.parent / "trace.csv"
        load_v2_trace(trace_path)
        if trace_sha256(trace_path) != value["raw_trace_sha256"]:
            raise ValueError("V2-04G-R1 trace hash drifted")
        key = (value["supervisor_profile_id"], value["scene_id"])
        if key in rows:
            raise ValueError("duplicate V2-04G-R1 evidence identity")
        rows[key] = value
    return progress, rows


def _activation_pass(summary_path, preregistration_path, planned_count):
    summary = yaml.safe_load(Path(summary_path).read_text(encoding="utf-8"))
    return bool(
        summary.get("stage") == "V2-04G-R1"
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
    if prereg.get("stage") != "V2-04G-R1" or contract.get("stage") != "V2-04G-R1":
        raise ValueError("V2-04G-R1 assessment boundary drifted")
    activation_ok = _activation_pass(
        activation_path, prereg_path,
        prereg["activation_readiness_probe"]["planned_probe_count"],
    )
    frozen_load = _LEGACY._load
    _LEGACY._load = _load_r1
    try:
        report = _LEGACY.assess(
            progress_path, prereg_path, contract_path, ttc_probe_path
        )
    finally:
        _LEGACY._load = frozen_load
    _, evidence = _load_r1(progress_path)
    join_gate = contract["hard_gates"]["bounded_context_join"]
    allowed_reasons = set(join_gate["allowed_join_reasons"])
    fixed_valid = report["fixed_reference"]["validity_gate_pass"]
    for candidate, summary in report["candidate_summaries"].items():
        rows = [row for (profile, _), row in evidence.items() if profile == candidate]
        if candidate == "g0_frozen_control":
            join_pass = all(row.get("mechanism_message_count", 0) == 0 for row in rows)
            join_minimum = None
            join_reasons = {}
        else:
            fractions = [row.get("mechanism_join_valid_fraction") for row in rows]
            reason_sets = [set(row.get("mechanism_join_reason_counts", {})) for row in rows]
            join_pass = bool(fractions) and all(
                value is not None
                and value >= join_gate["minimum_valid_fraction_per_episode"]
                for value in fractions
            ) and all(reasons.issubset(allowed_reasons) for reasons in reason_sets)
            join_minimum = min(fractions) if fractions else None
            join_reasons = {}
            for row in rows:
                for reason, count in row.get("mechanism_join_reason_counts", {}).items():
                    join_reasons[reason] = join_reasons.get(reason, 0) + count
        summary["bounded_context_join"] = {
            "minimum_valid_fraction_per_episode": join_minimum,
            "reason_counts": join_reasons,
            "pass": join_pass,
        }
        summary["hard_gates"]["activation_readiness"] = activation_ok
        summary["hard_gates"]["bounded_context_join"] = join_pass
        summary["all_hard_gates_pass"] = fixed_valid and all(
            summary["hard_gates"].values()
        )
    passing = [
        candidate for candidate, summary in report["candidate_summaries"].items()
        if summary["all_hard_gates_pass"]
    ]
    passing.sort(key=lambda candidate: (
        -report["candidate_summaries"][candidate]["family_non_regression_count"],
        report["candidate_summaries"][candidate]["total_navigation_time_ratio_vs_fixed"],
        -report["candidate_summaries"][candidate]["minimum_clearance_m"],
        report["candidate_summaries"][candidate]["active_anchor_switch_mean"],
        candidate,
    ))
    winner = passing[0] if passing else None
    report.update({
        "stage": "V2-04G-R1",
        "activation_readiness_probe_pass": activation_ok,
        "passing_candidate_ids": passing,
        "winner_candidate_id": winner,
    })
    report["evidence"]["activation_readiness_probe"] = {
        "path": str(activation_path), "sha256": _sha256(activation_path),
    }
    report["decision"].update({
        "freeze_authorized": winner is not None,
        "generate_fresh_held_out_validation_authorized": winner is not None,
        "enter_v2_05_authorized": False,
        "sac_training_authorized": False,
        "real_vehicle_authorized": False,
    })
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--progress", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r1/v2_04g_r1_progress.yaml")
    parser.add_argument("--preregistration", type=Path, default=WORKSPACE /
        "experiments/manifests/v2/calibration/v2_04g_r1_preregistration.yaml")
    parser.add_argument("--contract", type=Path, default=WORKSPACE /
        "config/thesis_experiments/v2/v2_04g_r1_interface_repair_contract.yaml")
    parser.add_argument("--ttc-probe", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r1/v2_04g_r1_ttc_three_state_probe.yaml")
    parser.add_argument("--activation-probe", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r1/activation_probe/activation_probe_summary.yaml")
    parser.add_argument("--output", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r1/v2_04g_r1_assessment.yaml")
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
