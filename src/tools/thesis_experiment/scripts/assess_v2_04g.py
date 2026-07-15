#!/usr/bin/env python3
"""Fail-closed assessment for preregistered V2-04G calibration evidence."""

import argparse
import hashlib
from pathlib import Path
import statistics

import yaml

from thesis_experiment.v2_evaluator import load_v2_trace, trace_sha256


FAMILIES = ("CRUISE", "DYNAMIC", "STATIC_DENSE", "CORRIDOR", "MANEUVER")
CANDIDATES = (
    "g0_frozen_control", "g1_mechanism_balanced", "g2_mechanism_aggressive",
)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _median(values):
    return float(statistics.median(values))


def _load(progress_path):
    progress = yaml.safe_load(Path(progress_path).read_text(encoding="utf-8"))
    if not (
        progress["stage"] == "V2-04G" and progress["status"] == "complete"
        and progress["valid_evidence_episode_count"] == 60
        and progress["interface_failure_count"] == 0
    ):
        raise ValueError("V2-04G navigation evidence is incomplete")
    rows = {}
    for record in progress["episodes"]:
        path = Path(record["evaluation"])
        if _sha256(path) != record["evaluation_sha256"]:
            raise ValueError("V2-04G evaluation hash drifted")
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
        trace_path = path.parent / "trace.csv"
        load_v2_trace(trace_path)
        if trace_sha256(trace_path) != value["raw_trace_sha256"]:
            raise ValueError("V2-04G trace hash drifted")
        key = (value["supervisor_profile_id"], value["scene_id"])
        if key in rows:
            raise ValueError("duplicate V2-04G evidence identity")
        rows[key] = value
    return progress, rows


def assess(progress_path, prereg_path, contract_path, ttc_probe_path):
    progress, evidence = _load(progress_path)
    prereg = yaml.safe_load(Path(prereg_path).read_text(encoding="utf-8"))
    contract = yaml.safe_load(Path(contract_path).read_text(encoding="utf-8"))
    if _sha256(contract_path) != prereg["resources"]["contract"]["sha256"]:
        raise ValueError("V2-04G contract hash drifted")
    probe = yaml.safe_load(Path(ttc_probe_path).read_text(encoding="utf-8"))
    probe_pass = bool(probe["all_three_states_pass"] and probe["probe_count"] == 3)
    gates = contract["hard_gates"]
    fixed = [row for (profile, _), row in evidence.items() if profile == "fixed_reference"]
    fixed_success = sum(row["metrics"]["common"]["success"] for row in fixed)
    fixed_collision = sum(row["metrics"]["common"]["collision"] for row in fixed)
    fixed_valid = (
        len(fixed) == 15
        and fixed_success >= gates["fixed_reference_validity"]["success_count_min"]
        and fixed_collision <= gates["fixed_reference_validity"]["collision_count_max"]
    )
    fixed_by_scene = {row["scene_id"]: row for row in fixed}
    summaries = {}
    passing = []
    for candidate in CANDIDATES:
        rows = [row for (profile, _), row in evidence.items() if profile == candidate]
        success = sum(row["metrics"]["common"]["success"] for row in rows)
        collision = sum(row["metrics"]["common"]["collision"] for row in rows)
        clearance_values = [
            row["metrics"]["common"]["minimum_clearance_m"]
            for row in rows if row["metrics"]["common"]["success"]
        ]
        typed_invalid = sum(not row["typed_transaction_valid"] for row in rows)
        dynamic = [row for row in rows if row["family"] == "DYNAMIC"]
        ttc_counts = {status: sum(row["ttc_status"] == status for row in dynamic)
                      for status in (
                          "OBSERVED_CONFLICT", "NO_CONFLICT_IN_HORIZON", "TRACKER_INVALID"
                      )}
        switches = [row["active_anchor_switch_count"] for row in rows]
        family_time = {}
        non_regression_count = 0
        family_regression_pass = True
        for family in FAMILIES:
            family_rows = [row for row in rows if row["family"] == family]
            paired_percent = []
            for row in family_rows:
                reference = fixed_by_scene[row["scene_id"]]
                reference_time = reference["metrics"]["common"]["navigation_time_s"]
                candidate_time = row["metrics"]["common"]["navigation_time_s"]
                paired_percent.append(100.0 * (candidate_time - reference_time) / reference_time)
            median_percent = _median(paired_percent)
            family_time[family] = {
                "paired_relative_changes_percent": paired_percent,
                "median_relative_change_percent": median_percent,
            }
            non_regression_count += int(median_percent <= 0.0)
            family_regression_pass &= median_percent <= gates["efficiency_vs_fixed"][
                "family_median_navigation_time_regression_percent_max"
            ]
        total_time = sum(row["metrics"]["common"]["navigation_time_s"] for row in rows)
        fixed_total = sum(row["metrics"]["common"]["navigation_time_s"] for row in fixed)
        total_ratio = total_time / fixed_total
        mechanism_pass = True
        mechanism = {
            "topology_locked_sample_count": sum(
                row.get("mechanism_topology_locked_sample_count", 0) for row in rows
            ),
            "corridor_centerline_sample_count": sum(
                row.get("mechanism_corridor_centerline_sample_count", 0) for row in rows
            ),
            "maneuver_reverse_sample_count": sum(
                row.get("mechanism_maneuver_reverse_sample_count", 0) for row in rows
            ),
            "topology_switch_max": max(
                (row.get("mechanism_topology_switch_count", 0) for row in rows), default=0
            ),
        }
        if candidate != "g0_frozen_control":
            limits = gates["mechanism_activation_for_non_control_candidates"]
            mechanism_pass = (
                mechanism["topology_locked_sample_count"]
                >= limits["topology_locked_sample_count_min"]
                and mechanism["corridor_centerline_sample_count"]
                >= limits["corridor_centerline_sample_count_min"]
                and mechanism["maneuver_reverse_sample_count"]
                >= limits["maneuver_reverse_sample_count_min"]
                and mechanism["topology_switch_max"]
                <= limits["mechanism_topology_switch_count_max_per_episode"]
            )
        candidate_gates = {
            "complete": len(rows) == 15,
            "success_non_degradation": success >= fixed_success,
            "collision": collision == 0,
            "minimum_clearance": bool(clearance_values) and min(clearance_values) >= 0.25,
            "typed_transaction": typed_invalid == 0,
            "boundary": all(
                not row["training_used"] and not row["runtime_policy_manifest_access"]
                for row in rows
            ),
            "ttc": (
                ttc_counts["OBSERVED_CONFLICT"] >= 2
                and ttc_counts["NO_CONFLICT_IN_HORIZON"] >= 1
                and ttc_counts["TRACKER_INVALID"] == 0
                and probe_pass
            ),
            "chatter": max(switches) <= 3 and sum(switches) / len(switches) <= 2.0,
            "mechanism_activation": mechanism_pass,
            "total_time": total_ratio <= 1.05,
            "family_time_regression": family_regression_pass,
            "family_non_regression_count": non_regression_count >= 3,
        }
        all_pass = fixed_valid and all(candidate_gates.values())
        if all_pass:
            passing.append(candidate)
        summaries[candidate] = {
            "episode_count": len(rows), "success_count": success,
            "collision_count": collision,
            "minimum_clearance_m": min(clearance_values) if clearance_values else None,
            "active_anchor_switch_max": max(switches),
            "active_anchor_switch_mean": sum(switches) / len(switches),
            "ttc_counts": ttc_counts, "mechanism": mechanism,
            "total_navigation_time_s": total_time,
            "total_navigation_time_ratio_vs_fixed": total_ratio,
            "family_time": family_time,
            "family_non_regression_count": non_regression_count,
            "hard_gates": candidate_gates, "all_hard_gates_pass": all_pass,
        }
    passing.sort(key=lambda candidate: (
        -summaries[candidate]["family_non_regression_count"],
        summaries[candidate]["total_navigation_time_ratio_vs_fixed"],
        -summaries[candidate]["minimum_clearance_m"],
        summaries[candidate]["active_anchor_switch_mean"], candidate,
    ))
    winner = passing[0] if passing else None
    return {
        "schema_version": "2.0", "stage": "V2-04G",
        "status": "calibration_assessed", "simulation_only": True,
        "runtime_ready": False, "training_started": False,
        "real_vehicle_used": False,
        "evidence": {
            "progress": {"path": str(progress_path), "sha256": _sha256(progress_path)},
            "preregistration": {"path": str(prereg_path), "sha256": _sha256(prereg_path)},
            "contract": {"path": str(contract_path), "sha256": _sha256(contract_path)},
            "ttc_probe": {"path": str(ttc_probe_path), "sha256": _sha256(ttc_probe_path)},
            "navigation_episode_count": len(evidence),
        },
        "fixed_reference": {
            "episode_count": len(fixed), "success_count": fixed_success,
            "collision_count": fixed_collision, "validity_gate_pass": fixed_valid,
        },
        "ttc_three_state_component_probe_pass": probe_pass,
        "candidate_summaries": summaries,
        "passing_candidate_ids": passing,
        "winner_candidate_id": winner,
        "decision": {
            "freeze_authorized": winner is not None,
            "generate_fresh_held_out_validation_authorized": winner is not None,
            "enter_v2_05_authorized": False,
            "sac_training_authorized": False,
            "real_vehicle_authorized": False,
        },
    }


def main():
    workspace = Path("/home/robot/robot_ws_base_rl")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--progress", type=Path, default=workspace /
        "artifacts/v2/calibration/v2_04g/v2_04g_progress.yaml")
    parser.add_argument("--preregistration", type=Path, default=workspace /
        "experiments/manifests/v2/calibration/v2_04g_preregistration.yaml")
    parser.add_argument("--contract", type=Path, default=workspace /
        "config/thesis_experiments/v2/v2_04g_mechanism_repair_contract.yaml")
    parser.add_argument("--ttc-probe", type=Path, default=workspace /
        "artifacts/v2/calibration/v2_04g/v2_04g_ttc_three_state_probe.yaml")
    parser.add_argument("--output", type=Path, default=workspace /
        "artifacts/v2/calibration/v2_04g/v2_04g_assessment.yaml")
    args = parser.parse_args()
    report = assess(args.progress, args.preregistration, args.contract, args.ttc_probe)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    temporary.replace(args.output)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
