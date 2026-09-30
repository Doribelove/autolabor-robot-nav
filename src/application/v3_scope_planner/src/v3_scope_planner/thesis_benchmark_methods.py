"""Frozen method adapters for the same-population thesis benchmark."""
import copy
import time

from . import core
from . import spatial_core_r1


METHODS = (
    "v3_spatial_safe",
    "actual_teb",
    "fixed_v120_shielded",
    "v3_no_candidate_conditioning",
)


def derive_no_candidate_conditioning_model(model):
    """Replace spatial action heads by same-mode certified-stop heads only."""
    value = copy.deepcopy(model)
    replaced = []
    heads = value["supported_action_heads"]
    for mode in sorted(heads):
        stop = heads[mode].get("certified_stop_v000")
        spatial = heads[mode].get("spatial_left_v120")
        if stop is not None and spatial is not None:
            heads[mode]["spatial_left_v120"] = copy.deepcopy(stop)
            replaced.append(mode)
    if replaced != ["cautious", "hesitant"]:
        raise ValueError("unexpected spatial action-head support: {}".format(replaced))
    value["model_id"] = "v3_19_no_candidate_conditioning_from_{}".format(model["model_id"])
    value["stage"] = "V3-19-A0"
    value["status"] = "frozen_single_factor_ablation_model"
    value["ablation"] = {
        "single_factor": "candidate_conditioning",
        "control": "candidate_specific_spatial_action_head",
        "treatment": "same_mode_certified_stop_head_for_spatial_candidate",
        "replaced_modes": replaced,
        "candidate_generator_changed": False,
        "hard_verifier_changed": False,
        "analytic_ranker_changed": False,
        "training_steps": 0,
    }
    return value


def _normalize(result, snapshot, latency_ms, policy_kind):
    selected_id = result["decision"]["selected_candidate_id"]
    selected = next(item for item in result["candidate_set"]["candidates"] if item["candidate_id"] == selected_id)
    evaluation = next(item for item in result["evaluations"] if item["candidate_id"] == selected_id)
    return {
        "snapshot": snapshot,
        "latency_ms": float(latency_ms),
        "result_digest": spatial_core_r1.digest(result),
        "selected_candidate_id": selected_id,
        "selected_candidate": selected,
        "selected_evaluation": evaluation,
        "safety_fallback_used": bool(result["decision"]["safety_fallback_used"]),
        "policy_kind": policy_kind,
    }


def plan_spatial(snapshot, model, contract, policy_kind="v3_spatial_safe"):
    started = time.perf_counter_ns()
    result = spatial_core_r1.plan(snapshot, model, contract)
    latency_ms = (time.perf_counter_ns() - started) / 1.0e6
    return _normalize(result, snapshot, latency_ms, policy_kind)


def plan_fixed_v120(snapshot, model, contract):
    """Apply fixed feasible-v1.20 selection with the same hard stop shield."""
    started = time.perf_counter_ns()
    result = core.plan(snapshot, model, contract)
    desired = next(item for item in result["evaluations"] if item["candidate_id"] == "feasible_v120")
    stop_id = contract["candidate_generator"]["certified_stop_candidate_id"]
    stop = next(item for item in result["evaluations"] if item["candidate_id"] == stop_id)
    selected = desired if desired["accepted"] else stop
    if not selected["accepted"]:
        raise RuntimeError("fixed-v1.20 comparator lacks an accepted certified stop")
    value = copy.deepcopy(result)
    decision = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "candidate_set_digest": result["candidate_set"]["candidate_set_digest"],
        "selected_candidate_id": selected["candidate_id"],
        "policy_kind": "fixed_v120_shielded",
        "policy_artifact_digest": core.digest({
            "method": "fixed_v120_shielded",
            "desired_candidate_id": "feasible_v120",
            "fallback_candidate_id": stop_id,
        }),
        "safety_fallback_used": not desired["accepted"],
        "fallback_reason": "FIXED_CANDIDATE_REJECTED_CERTIFIED_STOP" if not desired["accepted"] else "",
    }
    decision["decision_digest"] = core.digest(decision)
    value["decision"] = decision
    latency_ms = (time.perf_counter_ns() - started) / 1.0e6
    return _normalize(value, snapshot, latency_ms, "fixed_v120_shielded")

