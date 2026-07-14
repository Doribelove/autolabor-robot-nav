"""Fail-closed offline freeze assessment for the completed V2-04B screen."""

import hashlib
from pathlib import Path
from typing import Any, Dict, Mapping

import yaml


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _metric(evaluation: Mapping[str, Any], name: str) -> Any:
    common = evaluation["metrics"]["common"]
    family = evaluation["metrics"]["family"]
    if name in common:
        return common[name]
    if name in family:
        return family[name]
    raise ValueError("objective metric {} is missing".format(name))


def assess_anchor_freeze(
    contract: Mapping[str, Any], plan: Mapping[str, Any],
    progress: Mapping[str, Any], progress_path: Any,
) -> Dict[str, Any]:
    """Assess without mutating the candidate or frozen Anchor banks."""

    planned = int(plan["planned_episode_count"])
    completed = int(progress["valid_evidence_episode_count"])
    if planned != 90 or progress["planned_navigation_episode_count"] != planned:
        raise ValueError("freeze assessment episode budget drifted")
    if progress["training_started"] is not False or progress["runtime_ready"] is not False:
        raise ValueError("freeze assessment boundary drifted")

    evaluations = {}
    for row in progress["episodes"]:
        path = Path(row["evaluation"])
        if _sha256(path) != row["evaluation_sha256"]:
            raise ValueError("evaluation hash drifted for {}".format(row["candidate_id"]))
        evaluation = yaml.safe_load(path.read_text(encoding="utf-8"))
        identity = (row["candidate_id"], row["scene_id"])
        if identity in evaluations:
            raise ValueError("duplicate completed evaluation identity")
        evaluations[identity] = (row, evaluation)

    candidate_summaries = {}
    for candidate in plan["candidates"]:
        entries = []
        for expected in candidate["evaluations"]:
            identity = (candidate["candidate_id"], expected["scene_id"])
            if identity not in evaluations:
                continue
            progress_row, evaluation = evaluations[identity]
            entries.append((expected, progress_row, evaluation))
        candidate_summaries[candidate["candidate_id"]] = {
            "candidate": candidate,
            "entries": entries,
            "complete": len(entries) == len(candidate["evaluations"]),
            "hard_gate_pass": bool(entries) and all(row["hard_gate_pass"]
                                                     for _, row, _ in entries),
        }

    dynamic_entries = [
        evaluation for _, evaluation in evaluations.values()
        if evaluation["family"] == "DYNAMIC"
    ]
    dynamic_ttc_observed = sum(
        item["metrics"]["family"]["minimum_predicted_ttc_s"] is not None
        for item in dynamic_entries
    )
    blockers = []
    if completed != planned or len(evaluations) != planned:
        blockers.append("preregistered_calibration_matrix_incomplete")
    if progress["interface_failure_count"] != 0:
        blockers.append("interface_failures_present")
    if "cross_family_aggregation" not in contract["calibration"]:
        blockers.append("balanced_cross_family_aggregation_not_preregistered")
    if dynamic_ttc_observed != len(dynamic_entries):
        blockers.append("dynamic_primary_ttc_objective_unobserved")
    if (
        contract["calibration"]["strategy"]
        == "deterministic_one_factor_screen_then_bounded_refinement"
        and "refinement" not in plan
    ):
        blockers.append("bounded_refinement_not_preregistered")

    objectives = contract["calibration"]["family_objectives"]
    provisional = {}
    for anchor_id, families in contract["calibration"]["anchor_scene_families"].items():
        if len(families) != 1:
            provisional[anchor_id] = {
                "rankable": False,
                "reason": "multi_family_aggregation_not_preregistered",
            }
            continue
        family = families[0]
        candidates = []
        for summary in candidate_summaries.values():
            candidate = summary["candidate"]
            if candidate["anchor_id"] != anchor_id or not summary["complete"]:
                continue
            if not summary["hard_gate_pass"]:
                continue
            if len(summary["entries"]) != 1:
                raise ValueError("single-family candidate has ambiguous episode count")
            evaluation = summary["entries"][0][2]
            raw_vector = []
            sort_vector = []
            for objective in objectives[family]:
                value = _metric(evaluation, objective["metric"])
                if value is None:
                    raise ValueError("single-family objective is unobserved")
                raw_vector.append(value)
                sort_vector.append(value if objective["direction"] == "minimize" else -value)
            candidates.append((tuple(sort_vector), candidate, raw_vector))
        candidates.sort(key=lambda item: (item[0], item[1]["candidate_id"]))
        if not candidates:
            provisional[anchor_id] = {"rankable": False, "reason": "no_hard_gate_candidate"}
            continue
        _, winner, raw_vector = candidates[0]
        provisional[anchor_id] = {
            "rankable": True,
            "provisional_only": True,
            "candidate_id": winner["candidate_id"],
            "profile_sha256": winner["profile_sha256"],
            "objective_metrics": [item["metric"] for item in objectives[family]],
            "objective_vector": raw_vector,
        }

    progress_source = Path(progress_path)
    return {
        "schema_version": "2.0",
        "stage": "V2-04B",
        "assessment_id": "fam_teb_v2_04b_final_freeze_assessment_1",
        "status": "freeze_blocked" if blockers else "freeze_eligible",
        "formal_result": False,
        "simulation_only": True,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "evidence": {
            "planned_navigation_episode_count": planned,
            "valid_evidence_episode_count": completed,
            "successful_episode_count": progress["successful_episode_count"],
            "hard_gate_pass_episode_count": progress["hard_gate_pass_episode_count"],
            "interface_failure_count": progress["interface_failure_count"],
            "dynamic_episode_count": len(dynamic_entries),
            "dynamic_ttc_observed_episode_count": dynamic_ttc_observed,
            "progress_sha256": _sha256(progress_source),
        },
        "freeze_blockers": blockers,
        "provisional_single_family_screen_winners": provisional,
        "decision": {
            "enter_anchor_freeze": not blockers,
            "anchor_bank_mutation_allowed": not blockers,
            "required_next_stage": "preregister_v2_04c_refinement_and_aggregation"
            if blockers else "freeze_calibrated_anchor_bank",
        },
        "claims": {
            "calibration_screen_complete": completed == planned,
            "anchor_values_frozen": False,
            "performance_improvement_observed": False,
            "sac_training_started": False,
            "real_vehicle_validated": False,
        },
    }


def write_freeze_assessment(report: Mapping[str, Any], path: Any) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        yaml.safe_dump(dict(report), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
