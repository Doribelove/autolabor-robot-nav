#!/usr/bin/env python3
"""Fail-closed assessment of fresh V2-04F three-method paired evidence."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml

from thesis_experiment.v2_evaluator import load_v2_trace, trace_sha256


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
METHODS = ("fixed_teb", "balanced_anchor", "rule_multi_anchor")
FAMILIES = ("CRUISE", "DYNAMIC", "STATIC_DENSE", "CORRIDOR", "MANEUVER")
_SPEC = importlib.util.spec_from_file_location(
    "v2_04d_assessor_frozen", Path(__file__).with_name("assess_v2_04d.py")
)
_LEGACY = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_LEGACY)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def load_evidence(progress_path):
    progress = yaml.safe_load(Path(progress_path).read_text(encoding="utf-8"))
    if not (progress["stage"] == "V2-04F" and progress["status"] == "complete"
            and progress["valid_evidence_episode_count"] == 30
            and progress["interface_failure_count"] == 0):
        raise ValueError("V2-04F evidence is incomplete")
    evidence = {}
    for row in progress["episodes"]:
        path = Path(row["evaluation"])
        if _sha256(path) != row["evaluation_sha256"]:
            raise ValueError("V2-04F evaluation hash drifted")
        evaluation = yaml.safe_load(path.read_text(encoding="utf-8"))
        trace_path = path.parent / "trace.csv"
        load_v2_trace(trace_path)
        if trace_sha256(trace_path) != evaluation["raw_trace_sha256"]:
            raise ValueError("V2-04F trace hash drifted")
        identity = (evaluation["method"], evaluation["scene_id"])
        if identity in evidence:
            raise ValueError("duplicate V2-04F evidence identity")
        evidence[identity] = evaluation
    if len(evidence) != 30:
        raise ValueError("V2-04F evidence count drifted")
    return progress, evidence


def assess(progress_path, preregistration_path, contract_path):
    progress, evidence = load_evidence(progress_path)
    prereg = yaml.safe_load(Path(preregistration_path).read_text(encoding="utf-8"))
    contract = yaml.safe_load(Path(contract_path).read_text(encoding="utf-8"))
    if _sha256(contract_path) != prereg["resources"]["contract"]["sha256"]:
        raise ValueError("V2-04F contract hash drifted")
    summaries = {method: _LEGACY._method_summary(evidence, method) for method in METHODS}
    fixed_success = summaries["fixed_teb"]["success_count"]
    success_pass = all(summaries[method]["success_count"] >= fixed_success
                       for method in ("balanced_anchor", "rule_multi_anchor"))
    collision_pass = all(row["metrics"]["common"]["collision"] is False
                         for row in evidence.values())
    clearance_threshold = contract["stage_1_hard_gates"][
        "minimum_clearance_m_min_per_successful_episode"
    ]
    clearance_failures = [
        {"method": row["method"], "scene_id": row["scene_id"],
         "minimum_clearance_m": row["metrics"]["common"]["minimum_clearance_m"]}
        for row in evidence.values()
        if row["metrics"]["common"]["success"]
        and row["metrics"]["common"]["minimum_clearance_m"] < clearance_threshold
    ]
    clearance_pass = not clearance_failures
    transaction_pass = all(
        summaries[method]["typed_transaction_invalid_count"] == 0
        for method in ("balanced_anchor", "rule_multi_anchor")
    )
    boundary_pass = all(not summary["training_used"]
                        and not summary["runtime_policy_manifest_access"]
                        for summary in summaries.values())
    balanced_anchor_pass = summaries["balanced_anchor"]["distinct_active_anchor_ids"] == [
        "anchor_balanced"
    ]
    required_anchors = set(contract["stage_1_hard_gates"]["rule_required_anchor_ids"])
    observed_rule_anchors = set(summaries["rule_multi_anchor"]["distinct_active_anchor_ids"])
    rule_anchor_pass = required_anchors <= observed_rule_anchors
    rule_rows = [row for (method, _), row in evidence.items()
                 if method == "rule_multi_anchor"]
    cruise_rows = [row for row in rule_rows if row["family"] == "CRUISE"]
    maneuver_rows = [row for row in rule_rows if row["family"] == "MANEUVER"]
    cruise_static_max = max(row["context_geometry_sample_fractions"]["STATIC_DENSE"]
                            for row in cruise_rows)
    maneuver_fraction_min = min(row["context_geometry_sample_fractions"]["MANEUVER"]
                                for row in maneuver_rows)
    switch_max = max(row["active_anchor_switch_count"] for row in rule_rows)
    mechanism = contract["mechanism_checks"]
    cruise_absorption_pass = cruise_static_max <= mechanism[
        "rule_cruise_static_dense_fraction_max"
    ]
    maneuver_pass = maneuver_fraction_min >= mechanism["rule_maneuver_fraction_min"]
    chatter_pass = switch_max <= mechanism["rule_active_anchor_switch_count_max_per_episode"]
    base_stage_1_pass = all((success_pass, collision_pass, clearance_pass,
                             transaction_pass, balanced_anchor_pass,
                             rule_anchor_pass, boundary_pass))
    mechanism_pass = all((cruise_absorption_pass, maneuver_pass, chatter_pass))
    all_stage_1_pass = base_stage_1_pass and mechanism_pass

    dynamic = [row for row in evidence.values() if row["family"] == "DYNAMIC"]
    observed_count = sum(row["ttc_status"] == "OBSERVED_CONFLICT" for row in dynamic)
    invalid_count = sum(row["ttc_status"] == "TRACKER_INVALID" for row in dynamic)
    observed_fraction = float(observed_count) / len(dynamic)
    ttc_contract = contract["ttc_evidence_quality"]
    ttc_pass = observed_fraction >= ttc_contract["dynamic_observed_conflict_fraction_min"] \
        and invalid_count <= ttc_contract["tracker_invalid_dynamic_episode_count_max"]

    family_summaries = _LEGACY._family_summaries(evidence)
    descriptive_comparisons = _LEGACY._paired_comparisons(evidence)
    rule_time_regressed_all_families = all(
        descriptive_comparisons["rule_multi_anchor_vs_fixed_teb"][family][
            "navigation_time_s"
        ]["median_target_minus_reference"] > 0.0 for family in FAMILIES
    )
    blockers = []
    if clearance_failures:
        blockers.append("successful_episode_minimum_clearance_below_0_25_m")
    if not chatter_pass:
        blockers.append("held_out_rule_anchor_switch_count_exceeded_preregistered_maximum")
    if not ttc_pass:
        blockers.append("dynamic_observed_conflict_fraction_below_preregistered_threshold")
    if rule_time_regressed_all_families:
        blockers.append("rule_navigation_time_regressed_vs_fixed_in_all_five_families")
    performance_proven = bool(
        all_stage_1_pass and ttc_pass and not rule_time_regressed_all_families
    )
    return {
        "schema_version": "2.0", "stage": "V2-04F",
        "status": "fresh_held_out_paired_validation_assessed",
        "formal_result": False, "simulation_only": True, "runtime_ready": False,
        "training_started": False, "real_vehicle_used": False,
        "evidence": {
            "progress": {"path": str(progress_path), "sha256": _sha256(progress_path)},
            "preregistration": {"path": str(preregistration_path),
                                "sha256": _sha256(preregistration_path)},
            "contract": {"path": str(contract_path), "sha256": _sha256(contract_path)},
            "valid_paired_episode_count": len(evidence),
        },
        "stage_1_hard_gates": {
            "complete_evidence_pass": len(evidence) == 30,
            "success_non_degradation_pass": success_pass,
            "collision_pass": collision_pass,
            "minimum_clearance_pass": clearance_pass,
            "minimum_clearance_failures": clearance_failures,
            "typed_transaction_pass": transaction_pass,
            "balanced_single_anchor_pass": balanced_anchor_pass,
            "rule_required_anchor_coverage_pass": rule_anchor_pass,
            "training_and_label_boundary_pass": boundary_pass,
            "base_safety_and_interface_gates_pass": base_stage_1_pass,
            "all_stage_1_hard_gates_pass": all_stage_1_pass,
        },
        "mechanism_checks": {
            "cruise_static_dense_fraction_max_observed": cruise_static_max,
            "cruise_not_absorbed_by_static_pass": cruise_absorption_pass,
            "maneuver_fraction_min_observed": maneuver_fraction_min,
            "maneuver_activation_pass": maneuver_pass,
            "rule_active_anchor_switch_count_max_observed": switch_max,
            "chatter_pass": chatter_pass,
            "all_mechanism_checks_pass": mechanism_pass,
        },
        "ttc_evidence_quality": {
            "dynamic_episode_count": len(dynamic),
            "observed_conflict_count": observed_count,
            "observed_conflict_fraction": observed_fraction,
            "required_observed_conflict_fraction": ttc_contract[
                "dynamic_observed_conflict_fraction_min"
            ],
            "tracker_invalid_count": invalid_count,
            "cross_method_ttc_coverage_pass": ttc_pass,
        },
        "method_summaries": summaries,
        "family_summaries": family_summaries,
        "paired_comparisons_descriptive_only": descriptive_comparisons,
        "decision": {
            "success_non_degradation_proven": success_pass,
            "stage_2_performance_claim_authorized": all_stage_1_pass,
            "descriptive_paired_metrics_computed": True,
            "performance_effectiveness_proven": performance_proven,
            "enter_v2_05_authorized": performance_proven,
            "validation_may_modify_frozen_supervisor": False,
            "runtime_ready": False,
            "sac_training_authorized": False,
            "real_vehicle_authorized": False,
            "blockers": blockers,
        },
    }


def markdown(report):
    lines = [
        "# V2-04F Fresh Held-Out Three-Method Validation", "",
        "Success non-degradation: **{}**. All hard gates: **{}**. "
        "Performance effectiveness: **{}**.".format(
            report["decision"]["success_non_degradation_proven"],
            report["stage_1_hard_gates"]["all_stage_1_hard_gates_pass"],
            report["decision"]["performance_effectiveness_proven"],
        ), "",
        "| Method | Success | Collision | Minimum clearance (m) | Total time (s) | Anchors |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for method in METHODS:
        row = report["method_summaries"][method]
        lines.append("| {} | {}/{} | {} | {:.3f} | {:.1f} | {} |".format(
            method, row["success_count"], row["episode_count"], row["collision_count"],
            row["minimum_clearance_m"], row["sum_navigation_time_s"],
            ", ".join(row["distinct_active_anchor_ids"]),
        ))
    lines.extend(["", "## Rule paired medians (descriptive only)", "",
                  "Stage-2 claims are not authorized because hard gates failed.", "",
                  "| Family | Time vs Fixed | Time vs Balanced | Path vs Fixed | Clearance vs Fixed (m) |",
                  "|---|---:|---:|---:|---:|"])
    comparisons = report["paired_comparisons_descriptive_only"]
    for family in FAMILIES:
        fixed = comparisons["rule_multi_anchor_vs_fixed_teb"][family]
        balanced = comparisons["rule_multi_anchor_vs_balanced_anchor"][family]
        lines.append("| {} | {:+.1f}% | {:+.1f}% | {:+.1f}% | {:+.3f} |".format(
            family,
            fixed["navigation_time_s"]["median_relative_change_percent"],
            balanced["navigation_time_s"]["median_relative_change_percent"],
            fixed["path_length_m"]["median_relative_change_percent"],
            fixed["minimum_clearance_m"]["median_target_minus_reference"],
        ))
    lines.extend(["", "## Blockers", ""])
    lines.extend("- `{}`".format(value) for value in report["decision"]["blockers"])
    lines.extend(["", "The frozen supervisor and Anchor Bank were not modified. "
                  "SAC training and real-vehicle execution remain unauthorized.", ""])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--progress", type=Path, default=WORKSPACE /
        "artifacts/v2/validation/v2_04f/v2_04f_progress.yaml")
    parser.add_argument("--preregistration", type=Path, default=WORKSPACE /
        "experiments/manifests/v2/validation/v2_04f_preregistration.yaml")
    parser.add_argument("--contract", type=Path, default=WORKSPACE /
        "config/thesis_experiments/v2/v2_04f_fresh_paired_validation_contract.yaml")
    parser.add_argument("--output", type=Path, default=WORKSPACE /
        "artifacts/v2/validation/v2_04f/v2_04f_paired_assessment.yaml")
    parser.add_argument("--markdown", type=Path, default=WORKSPACE /
        "artifacts/v2/validation/v2_04f/V2_04F_PAIRED_VALIDATION_REPORT.md")
    args = parser.parse_args()
    report = assess(args.progress.resolve(), args.preregistration.resolve(),
                    args.contract.resolve())
    _write(args.output, yaml.safe_dump(report, sort_keys=False))
    _write(args.markdown, markdown(report))
    print(yaml.safe_dump(report["decision"], sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
