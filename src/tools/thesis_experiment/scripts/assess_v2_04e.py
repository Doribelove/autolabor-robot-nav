#!/usr/bin/env python3
"""Assess the frozen V2-04E calibration-only supervisor candidate matrix."""

import argparse
import hashlib
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
EXPECTED_MODE = {
    "CRUISE": "CRUISE",
    "STATIC_DENSE": "STATIC_DENSE",
    "CORRIDOR": "CORRIDOR",
    "MANEUVER": "MANEUVER",
}


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write_yaml(path, data):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", default="V2-04E")
    parser.add_argument("--contract", type=Path, default=WORKSPACE /
        "config/thesis_experiments/v2/v2_04e_rule_supervisor_repair_contract.yaml")
    parser.add_argument("--preregistration", type=Path, default=WORKSPACE /
        "experiments/manifests/v2/calibration/v2_04e_r1_preregistration.yaml")
    parser.add_argument("--episodes-root", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04e/episodes")
    parser.add_argument("--output", type=Path, default=WORKSPACE /
        "artifacts/v2/calibration/v2_04e/v2_04e_assessment.yaml")
    args = parser.parse_args()
    contract = yaml.safe_load(args.contract.read_text(encoding="utf-8"))
    prereg = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    gates = contract["hard_gates_per_candidate"]
    by_candidate = {candidate: [] for candidate in prereg["candidate_ids"]}
    for path in sorted(args.episodes_root.glob("*/evaluation.yaml")):
        evidence = yaml.safe_load(path.read_text(encoding="utf-8"))
        candidate = evidence.get("supervisor_profile_id")
        if candidate not in by_candidate:
            continue
        if evidence.get("stage") != args.stage or evidence.get("split") != "calibration":
            raise ValueError("calibration evidence boundary drifted")
        if evidence.get("seed") not in prereg["data_firewall"]["allowed_seed_set"]:
            raise ValueError("V2-04E evidence seed escaped calibration firewall")
        by_candidate[candidate].append((path, evidence))
    summaries = {}
    passing = []
    for candidate, rows in by_candidate.items():
        family = {evidence["family"]: evidence for _, evidence in rows}
        switch_counts = [evidence["active_anchor_switch_count"] for _, evidence in rows]
        expected_sum = sum(
            family[name]["context_geometry_sample_fractions"].get(mode, 0.0)
            for name, mode in EXPECTED_MODE.items() if name in family
        )
        checks = {
            "valid_episode_count": len(rows) == gates["valid_episode_count_exact"],
            "success_count": sum(bool(e["metrics"]["common"]["success"])
                                 for _, e in rows) == gates["success_count_exact"],
            "collision_count": sum(bool(e["metrics"]["common"]["collision"])
                                   for _, e in rows) <= gates["collision_count_max"],
            "typed_transaction_valid": all(e["typed_transaction_valid"] for _, e in rows),
            "training_unused": all(not e["training_used"] for _, e in rows),
            "cruise_fraction": family.get("CRUISE", {}).get(
                "context_geometry_sample_fractions", {}).get("CRUISE", 0.0)
                >= gates["cruise"]["cruise_valid_context_fraction_min"],
            "cruise_static_absorption": family.get("CRUISE", {}).get(
                "context_geometry_sample_fractions", {}).get("STATIC_DENSE", 1.0)
                <= gates["cruise"]["static_dense_valid_context_fraction_max"],
            "static_fraction": family.get("STATIC_DENSE", {}).get(
                "context_geometry_sample_fractions", {}).get("STATIC_DENSE", 0.0)
                >= gates["static_dense"]["static_dense_valid_context_fraction_min"],
            "corridor_fraction": family.get("CORRIDOR", {}).get(
                "context_geometry_sample_fractions", {}).get("CORRIDOR", 0.0)
                >= gates["corridor"]["corridor_valid_context_fraction_min"],
            "maneuver_fraction": family.get("MANEUVER", {}).get(
                "context_geometry_sample_fractions", {}).get("MANEUVER", 0.0)
                >= gates["maneuver"]["maneuver_valid_context_fraction_min"],
            "switch_max": bool(switch_counts) and max(switch_counts)
                <= gates["chatter"]["active_anchor_switch_count_max_per_episode"],
            "switch_mean": bool(switch_counts) and sum(switch_counts) / len(switch_counts)
                <= gates["chatter"]["active_anchor_switch_count_mean_max"],
        }
        passed = all(checks.values())
        if passed:
            passing.append(candidate)
        summaries[candidate] = {
            "hard_gate_pass": passed,
            "checks": checks,
            "success_count": sum(bool(e["metrics"]["common"]["success"]) for _, e in rows),
            "collision_count": sum(bool(e["metrics"]["common"]["collision"]) for _, e in rows),
            "expected_family_mode_fraction_sum": expected_sum,
            "maneuver_fraction": family.get("MANEUVER", {}).get(
                "context_geometry_sample_fractions", {}).get("MANEUVER", 0.0),
            "cruise_static_dense_fraction": family.get("CRUISE", {}).get(
                "context_geometry_sample_fractions", {}).get("STATIC_DENSE", 0.0),
            "active_anchor_switch_counts": switch_counts,
            "active_anchor_switch_count_total": sum(switch_counts),
            "navigation_time_s_total": sum(e["metrics"]["common"]["navigation_time_s"]
                                           for _, e in rows),
            "evidence": [{"path": str(path), "sha256": _sha256(path)} for path, _ in rows],
        }
    ranked = sorted(passing, key=lambda candidate: (
        -summaries[candidate]["expected_family_mode_fraction_sum"],
        summaries[candidate]["active_anchor_switch_count_total"],
        summaries[candidate]["navigation_time_s_total"], candidate,
    ))
    report = {
        "schema_version": "2.0", "stage": args.stage,
        "status": "calibration_complete_no_freeze" if not ranked else "calibration_winner_selected",
        "formal_result": False, "simulation_only": True, "runtime_ready": False,
        "training_started": False, "real_vehicle_used": False,
        "contract": {"path": str(args.contract), "sha256": _sha256(args.contract)},
        "preregistration": {"path": str(args.preregistration),
                            "sha256": _sha256(args.preregistration)},
        "valid_evidence_episode_count": sum(len(rows) for rows in by_candidate.values()),
        "candidate_summaries": summaries,
        "passing_candidate_ids": ranked,
        "selected_candidate_id": ranked[0] if ranked else None,
        "decision": {
            "freeze_rule_supervisor_authorized": bool(ranked),
            "held_out_validation_authorized": bool(ranked),
            "new_calibration_only_repair_stage_required": not bool(ranked),
            "v2_04d_validation_data_used_for_selection": False,
            "sac_training_authorized": False,
            "real_vehicle_authorized": False,
        },
    }
    _write_yaml(args.output, report)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
