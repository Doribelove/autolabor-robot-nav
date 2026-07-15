#!/usr/bin/env python3
"""Fail-closed R4-R1 assessment with Maneuver truth-clearance repair gate."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R4-R1"
R4_ASSESSOR = Path(__file__).with_name("assess_v2_04g_r4.py")
_SPEC = importlib.util.spec_from_file_location("assess_v2_04g_r4_frozen_for_r4_r1", R4_ASSESSOR)
_R4 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R4)
_R4.STAGE = STAGE


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def assess(progress_path, prereg_path, contract_path, ttc_probe_path,
           activation_path, candidate_bank_path):
    prereg = yaml.safe_load(Path(prereg_path).read_text(encoding="utf-8"))
    contract = yaml.safe_load(Path(contract_path).read_text(encoding="utf-8"))
    bank = yaml.safe_load(Path(candidate_bank_path).read_text(encoding="utf-8"))
    if not (prereg.get("stage") == STAGE and contract.get("stage") == STAGE
            and bank.get("stage") == STAGE):
        raise ValueError("R4-R1 assessment boundary drifted")
    if _sha256(candidate_bank_path) != prereg["resources"]["candidate_bank"]["sha256"]:
        raise ValueError("R4-R1 candidate bank hash drifted")
    report = _R4.assess(
        progress_path, prereg_path, contract_path, ttc_probe_path, activation_path)
    _, evidence = _R4._load_r4(progress_path)
    eligibility = {row["candidate_id"]: bool(row["winner_eligible"])
                   for row in bank["candidates"]}
    clearance_gate = contract["hard_gates"]["maneuver_clearance_repair"]
    for candidate, summary in report["candidate_summaries"].items():
        rows = [row for (profile, _), row in evidence.items()
                if profile == candidate and row["family"] == "MANEUVER"]
        scan_values = [row["metrics"]["common"]["minimum_clearance_m"] for row in rows
                       if row["metrics"]["common"]["success"]]
        truth_values = [row.get("clearance_audit", {}).get(
            "minimum_truth_box_clearance_m") for row in rows
            if row["metrics"]["common"]["success"]]
        truth_complete = bool(truth_values) and all(value is not None for value in truth_values)
        scan_pass = bool(scan_values) and min(scan_values) >= clearance_gate[
            "minimum_signed_scan_clearance_m_per_successful_maneuver"]
        truth_pass = truth_complete and min(truth_values) >= clearance_gate[
            "minimum_truth_box_clearance_m_per_successful_maneuver"]
        contact_pass = all(row.get("clearance_audit", {}).get("contact_count", 0) == 0
                           for row in rows)
        repair_eligible = eligibility.get(candidate, False)
        summary["maneuver_clearance_repair"] = {
            "successful_maneuver_episode_count": len(scan_values),
            "minimum_signed_scan_clearance_m": min(scan_values) if scan_values else None,
            "minimum_truth_box_clearance_m": min(truth_values) if truth_complete else None,
            "scan_gate_pass": scan_pass, "truth_gate_pass": truth_pass,
            "zero_contact_pass": contact_pass,
            "repair_candidate_winner_eligible": repair_eligible,
        }
        summary["hard_gates"]["maneuver_scan_clearance"] = scan_pass
        summary["hard_gates"]["maneuver_truth_clearance"] = truth_pass
        summary["hard_gates"]["maneuver_zero_contact"] = contact_pass
        summary["hard_gates"]["repair_candidate_eligible"] = repair_eligible
        summary["all_hard_gates_pass"] = (
            report["fixed_reference"]["validity_gate_pass"]
            and all(summary["hard_gates"].values())
        )
    passing = [candidate for candidate, summary in
               report["candidate_summaries"].items()
               if summary["all_hard_gates_pass"]]
    passing.sort(key=lambda candidate: (
        report["candidate_summaries"][candidate][
            "total_navigation_time_ratio_vs_fixed"],
        -report["candidate_summaries"][candidate]["maneuver_clearance_repair"][
            "minimum_truth_box_clearance_m"],
        -report["candidate_summaries"][candidate]["minimum_clearance_m"],
        report["candidate_summaries"][candidate]["active_anchor_switch_mean"],
        candidate,
    ))
    winner = passing[0] if passing else None
    report.update({
        "stage": STAGE, "passing_candidate_ids": passing,
        "winner_candidate_id": winner, "all_hard_gates_pass": winner is not None,
        "single_changed_factor": "maneuver_anchor_min_obstacle_dist_m",
    })
    report["evidence"]["candidate_bank"] = {
        "path": str(candidate_bank_path), "sha256": _sha256(candidate_bank_path)}
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
        "artifacts/v2/calibration/v2_04g_r4_r1/v2_04g_r4_r1_progress.yaml")
    parser.add_argument("--preregistration", type=Path, default=WORKSPACE /
        "experiments/manifests/v2/calibration/v2_04g_r4_r1_preregistration.yaml")
    parser.add_argument("--contract", type=Path, default=WORKSPACE /
        "config/thesis_experiments/v2/v2_04g_r4_r1_clearance_repair_contract.yaml")
    parser.add_argument("--candidate-bank", type=Path, default=WORKSPACE /
        "experiments/manifests/v2/calibration/v2_04g_r4_r1_clearance_candidates.yaml")
    parser.add_argument("--ttc-probe", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r4_r1/v2_04g_r4_r1_ttc_three_state_probe.yaml")
    parser.add_argument("--activation-probe", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r4_r1/activation_probe/activation_probe_summary.yaml")
    parser.add_argument("--output", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r4_r1/v2_04g_r4_r1_assessment.yaml")
    args = parser.parse_args()
    report = assess(args.progress, args.preregistration, args.contract,
                    args.ttc_probe, args.activation_probe, args.candidate_bank)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    temporary.replace(args.output)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
