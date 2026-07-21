#!/usr/bin/env python3
"""Fail-closed offline assessment of completed V2-04D paired evidence."""

import argparse
import hashlib
from pathlib import Path
import statistics

import yaml

from thesis_experiment.v2_evaluator import load_v2_trace, trace_sha256


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
METHODS = ("fixed_teb", "balanced_anchor", "rule_multi_anchor")
FAMILIES = ("CRUISE", "DYNAMIC", "STATIC_DENSE", "CORRIDOR", "MANEUVER")


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _median(values):
    values = [value for value in values if value is not None]
    return statistics.median(values) if values else None


def _write(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def load_evidence(progress_path):
    progress = yaml.safe_load(Path(progress_path).read_text(encoding="utf-8"))
    if not (
        progress["status"] == "paired_validation_complete"
        and progress["valid_evidence_episode_count"] == 30
        and progress["interface_failure_count"] == 0
    ):
        raise ValueError("V2-04D evidence is incomplete")
    evidence = {}
    for row in progress["episodes"]:
        path = Path(row["evaluation"])
        if _sha256(path) != row["evaluation_sha256"]:
            raise ValueError("V2-04D evaluation hash drifted")
        evaluation = yaml.safe_load(path.read_text(encoding="utf-8"))
        trace_path = path.parent / "trace.csv"
        load_v2_trace(trace_path)
        if trace_sha256(trace_path) != evaluation["raw_trace_sha256"]:
            raise ValueError("V2-04D trace hash drifted")
        identity = (evaluation["method"], evaluation["scene_id"])
        if identity in evidence:
            raise ValueError("duplicate V2-04D evidence identity")
        evidence[identity] = evaluation
    if len(evidence) != 30:
        raise ValueError("V2-04D evidence count drifted")
    return progress, evidence


def _method_summary(evidence, method):
    rows = [value for (name, _), value in evidence.items() if name == method]
    anchors = sorted({anchor for row in rows for anchor in row["active_anchor_sequence"]})
    return {
        "episode_count": len(rows),
        "success_count": sum(row["metrics"]["common"]["success"] for row in rows),
        "collision_count": sum(row["metrics"]["common"]["collision"] for row in rows),
        "minimum_clearance_m": min(
            row["metrics"]["common"]["minimum_clearance_m"] for row in rows
        ),
        "sum_navigation_time_s": sum(
            row["metrics"]["common"]["navigation_time_s"] for row in rows
        ),
        "median_navigation_time_s": _median(
            [row["metrics"]["common"]["navigation_time_s"] for row in rows]
        ),
        "median_path_length_m": _median(
            [row["metrics"]["common"]["path_length_m"] for row in rows]
        ),
        "distinct_active_anchor_ids": anchors,
        "distinct_active_anchor_id_count": len(anchors),
        "context_geometry_switch_count_total": sum(
            row["context_geometry_switch_count"] for row in rows
        ),
        "active_anchor_switch_count_total": sum(
            row["active_anchor_switch_count"] for row in rows
        ),
        "training_used": any(row["training_used"] for row in rows),
        "runtime_policy_manifest_access": any(
            row["runtime_policy_manifest_access"] for row in rows
        ),
        "typed_transaction_invalid_count": sum(
            not row["typed_transaction_valid"] for row in rows
        ),
    }


def _family_summaries(evidence):
    result = {}
    for method in METHODS:
        result[method] = {}
        for family in FAMILIES:
            rows = [value for (name, _), value in evidence.items()
                    if name == method and value["family"] == family]
            common_names = tuple(rows[0]["metrics"]["common"])
            family_names = tuple(rows[0]["metrics"]["family"])
            result[method][family] = {
                "replicate_count": len(rows),
                "common_medians": {
                    name: _median([row["metrics"]["common"][name] for row in rows])
                    for name in common_names
                    if name not in ("success", "collision")
                },
                "family_medians": {
                    name: _median([row["metrics"]["family"][name] for row in rows])
                    for name in family_names
                },
            }
    return result


def _paired_comparisons(evidence):
    comparisons = {}
    for target, reference in (
        ("balanced_anchor", "fixed_teb"),
        ("rule_multi_anchor", "fixed_teb"),
        ("rule_multi_anchor", "balanced_anchor"),
    ):
        comparison_id = "{}_vs_{}".format(target, reference)
        comparisons[comparison_id] = {}
        for family in FAMILIES:
            target_rows = {
                value["scene_id"]: value for (name, _), value in evidence.items()
                if name == target and value["family"] == family
            }
            reference_rows = {
                value["scene_id"]: value for (name, _), value in evidence.items()
                if name == reference and value["family"] == family
            }
            if set(target_rows) != set(reference_rows):
                raise ValueError("paired scene identities drifted")
            metrics = {}
            for name, direction in (
                ("navigation_time_s", "minimize"),
                ("path_length_m", "minimize"),
                ("minimum_clearance_m", "maximize"),
                ("stop_count", "minimize"),
                ("reverse_distance_m", "context_dependent"),
            ):
                deltas = []
                relative = []
                for scene_id in sorted(target_rows):
                    target_value = target_rows[scene_id]["metrics"]["common"][name]
                    reference_value = reference_rows[scene_id]["metrics"]["common"][name]
                    deltas.append(target_value - reference_value)
                    if reference_value != 0.0:
                        relative.append(100.0 * (target_value - reference_value) / reference_value)
                metrics[name] = {
                    "direction": direction,
                    "median_target_minus_reference": _median(deltas),
                    "median_relative_change_percent": _median(relative),
                }
            comparisons[comparison_id][family] = metrics
    return comparisons


def assess(progress_path, preregistration_path, contract_path):
    progress, evidence = load_evidence(progress_path)
    prereg = yaml.safe_load(Path(preregistration_path).read_text(encoding="utf-8"))
    contract = yaml.safe_load(Path(contract_path).read_text(encoding="utf-8"))
    if _sha256(contract_path) != prereg["resources"]["contract"]["sha256"]:
        raise ValueError("V2-04D contract hash drifted")
    summaries = {method: _method_summary(evidence, method) for method in METHODS}
    fixed_success = summaries["fixed_teb"]["success_count"]
    clearance_pass = all(
        row["metrics"]["common"]["minimum_clearance_m"] >= 0.25
        for row in evidence.values() if row["metrics"]["common"]["success"]
    )
    balanced_anchors_pass = summaries["balanced_anchor"][
        "distinct_active_anchor_ids"
    ] == ["anchor_balanced"]
    rule_anchor_pass = summaries["rule_multi_anchor"][
        "distinct_active_anchor_id_count"
    ] >= 3
    success_pass = all(
        summaries[method]["success_count"] >= fixed_success
        for method in ("balanced_anchor", "rule_multi_anchor")
    )
    collision_pass = all(summary["collision_count"] == 0 for summary in summaries.values())
    transaction_pass = all(
        summaries[method]["typed_transaction_invalid_count"] == 0
        for method in ("balanced_anchor", "rule_multi_anchor")
    )
    boundary_pass = all(
        not summary["training_used"] and not summary["runtime_policy_manifest_access"]
        for summary in summaries.values()
    )
    stage_1_pass = all((
        success_pass, collision_pass, clearance_pass, transaction_pass,
        balanced_anchors_pass, rule_anchor_pass, boundary_pass,
    ))
    dynamic = [row for row in evidence.values() if row["family"] == "DYNAMIC"]
    observed_count = sum(row["ttc_status"] == "OBSERVED_CONFLICT" for row in dynamic)
    invalid_count = sum(row["ttc_status"] == "TRACKER_INVALID" for row in dynamic)
    observed_fraction = float(observed_count) / len(dynamic)
    ttc_coverage_pass = (
        observed_fraction >= contract["ttc_semantics"][
            "dynamic_observed_conflict_fraction_min"
        ]
        and invalid_count <= contract["ttc_semantics"][
            "tracker_invalid_dynamic_episode_count_max"
        ]
    )
    family_summaries = _family_summaries(evidence)
    comparisons = _paired_comparisons(evidence) if stage_1_pass else {}
    rule_time_regressed_all_families = all(
        comparisons["rule_multi_anchor_vs_fixed_teb"][family]["navigation_time_s"][
            "median_target_minus_reference"
        ] > 0.0 for family in FAMILIES
    )
    rule_maneuver_activated = "anchor_maneuver_forward" in summaries[
        "rule_multi_anchor"
    ]["distinct_active_anchor_ids"]
    performance_effectiveness_proven = bool(
        stage_1_pass and not rule_time_regressed_all_families and rule_maneuver_activated
    )
    blockers = []
    if rule_time_regressed_all_families:
        blockers.append("rule_navigation_time_regressed_in_all_five_families")
    if not rule_maneuver_activated:
        blockers.append("rule_supervisor_never_activated_maneuver_anchor")
    if not ttc_coverage_pass:
        blockers.append("cross_method_dynamic_observed_conflict_fraction_below_preregistered_threshold")
    return {
        "schema_version": "2.0",
        "stage": "V2-04D",
        "status": "paired_validation_assessed",
        "formal_result": False,
        "simulation_only": True,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "evidence": {
            "progress": {"path": str(progress_path), "sha256": _sha256(progress_path)},
            "preregistration": {
                "path": str(preregistration_path), "sha256": _sha256(preregistration_path)
            },
            "contract": {"path": str(contract_path), "sha256": _sha256(contract_path)},
            "valid_paired_episode_count": len(evidence),
        },
        "stage_1_hard_gates": {
            "complete_evidence_pass": len(evidence) == 30,
            "success_non_degradation_pass": success_pass,
            "collision_pass": collision_pass,
            "minimum_clearance_pass": clearance_pass,
            "typed_transaction_pass": transaction_pass,
            "balanced_single_anchor_pass": balanced_anchors_pass,
            "rule_multi_anchor_mechanism_pass": rule_anchor_pass,
            "training_and_label_boundary_pass": boundary_pass,
            "all_stage_1_hard_gates_pass": stage_1_pass,
        },
        "ttc_evidence_quality": {
            "dynamic_episode_count": len(dynamic),
            "observed_conflict_count": observed_count,
            "observed_conflict_fraction": observed_fraction,
            "required_observed_conflict_fraction": contract["ttc_semantics"][
                "dynamic_observed_conflict_fraction_min"
            ],
            "tracker_invalid_count": invalid_count,
            "cross_method_ttc_coverage_pass": ttc_coverage_pass,
            "by_method": {
                method: {
                    status: sum(
                        row["ttc_status"] == status
                        for (name, _), row in evidence.items()
                        if name == method and row["family"] == "DYNAMIC"
                    )
                    for status in (
                        "OBSERVED_CONFLICT", "NO_CONFLICT_IN_HORIZON", "TRACKER_INVALID"
                    )
                } for method in METHODS
            },
        },
        "method_summaries": summaries,
        "family_summaries": family_summaries,
        "paired_comparisons": comparisons,
        "decision": {
            "success_non_degradation_proven": stage_1_pass,
            "stage_2_performance_comparison_completed": stage_1_pass,
            "performance_effectiveness_proven": performance_effectiveness_proven,
            "enter_v2_05_authorized": performance_effectiveness_proven,
            "anchor_bank_remains_frozen": True,
            "runtime_ready": False,
            "sac_training_authorized": False,
            "real_vehicle_authorized": False,
            "blockers": blockers,
        },
    }


def markdown(report):
    lines = [
        "# V2-04D No-Training Paired Validation",
        "",
        "Stage 1 passed: **{}**. Performance effectiveness proven: **{}**. "
        "V2-05 authorized: **{}**.".format(
            report["stage_1_hard_gates"]["all_stage_1_hard_gates_pass"],
            report["decision"]["performance_effectiveness_proven"],
            report["decision"]["enter_v2_05_authorized"],
        ),
        "",
        "| Method | Success | Collision | Minimum clearance (m) | Total time (s) | Anchors |",
        "|---|---:|---:|---:|---:|---|",
    ]
    for method in METHODS:
        item = report["method_summaries"][method]
        lines.append("| {} | {}/{} | {} | {:.3f} | {:.1f} | {} |".format(
            method, item["success_count"], item["episode_count"], item["collision_count"],
            item["minimum_clearance_m"], item["sum_navigation_time_s"],
            ", ".join(item["distinct_active_anchor_ids"]) or "n/a",
        ))
    lines.extend([
        "",
        "## Rule Multi-Anchor paired median change",
        "",
        "Positive time/path percentages are regressions; positive clearance is improvement.",
        "",
        "| Family | Time vs Fixed | Time vs Balanced | Path vs Fixed | Clearance vs Fixed (m) |",
        "|---|---:|---:|---:|---:|",
    ])
    for family in FAMILIES:
        fixed = report["paired_comparisons"]["rule_multi_anchor_vs_fixed_teb"][family]
        balanced = report["paired_comparisons"]["rule_multi_anchor_vs_balanced_anchor"][family]
        lines.append("| {} | {:+.1f}% | {:+.1f}% | {:+.1f}% | {:+.3f} |".format(
            family,
            fixed["navigation_time_s"]["median_relative_change_percent"],
            balanced["navigation_time_s"]["median_relative_change_percent"],
            fixed["path_length_m"]["median_relative_change_percent"],
            fixed["minimum_clearance_m"]["median_target_minus_reference"],
        ))
    lines.extend([
        "",
        "## Blockers",
        "",
    ])
    lines.extend("- `{}`".format(item) for item in report["decision"]["blockers"])
    lines.extend([
        "",
        "The frozen Anchor Bank was not modified. SAC training and real-vehicle execution remain unauthorized.",
        "",
    ])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--progress", type=Path, default=WORKSPACE /
        "artifacts/v2/validation/v2_04d/v2_04d_paired_progress.yaml")
    parser.add_argument("--preregistration", type=Path, default=WORKSPACE /
        "experiments/manifests/v2/validation/v2_04d_preregistration.yaml")
    parser.add_argument("--contract", type=Path, default=WORKSPACE /
        "config/thesis_experiments/v2/v2_04d_paired_validation_contract.yaml")
    parser.add_argument("--output", type=Path, default=WORKSPACE /
        "artifacts/v2/validation/v2_04d/v2_04d_paired_assessment.yaml")
    parser.add_argument("--markdown", type=Path, default=WORKSPACE /
        "artifacts/v2/validation/v2_04d/V2_04D_PAIRED_VALIDATION_REPORT.md")
    args = parser.parse_args()
    report = assess(args.progress.resolve(), args.preregistration.resolve(),
                    args.contract.resolve())
    _write(args.output, yaml.safe_dump(report, sort_keys=False))
    _write(args.markdown, markdown(report))
    print(yaml.safe_dump(report["decision"], sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
