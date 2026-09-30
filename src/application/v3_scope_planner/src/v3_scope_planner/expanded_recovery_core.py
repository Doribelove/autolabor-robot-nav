"""Five-action V3-13 recovery planner with frozen verifier and ranker.

The original four-action planner remains byte-for-byte unchanged.  This
module is eligible only after a certified-stop prefix and appends the single
R4-R1 candidate, evaluates all five actions with the R7 predictor, then uses
the original deterministic safe-candidate selection rule.
"""

from typing import Any, Dict, List, Mapping

from v3_scope_planner import multi_actor_core as base
from v3_scope_planner import multi_actor_core_v1 as predictor
from v3_scope_planner import recovery_candidate_core as recovery


def _select(
    candidate_set: Mapping[str, Any],
    evaluations: List[Mapping[str, Any]],
    contract: Mapping[str, Any],
) -> Dict[str, Any]:
    candidates = {
        item["candidate_id"]: item
        for item in candidate_set["candidates"]
    }
    accepted = [
        item
        for item in evaluations
        if item["accepted"]
        and not candidates[item["candidate_id"]]["certified_stop"]
    ]
    fallback_used = False
    if accepted:
        selected = sorted(
            accepted,
            key=lambda item: (
                -float(item["analytic_score"]),
                item["candidate_id"],
            ),
        )[0]
    else:
        safe_stops = [
            item
            for item in evaluations
            if item["accepted"]
            and candidates[item["candidate_id"]]["certified_stop"]
        ]
        base.require(
            safe_stops, "NO_SAFE_CANDIDATE_OR_CERTIFIED_STOP"
        )
        selected = sorted(
            safe_stops, key=lambda item: item["candidate_id"]
        )[0]
        fallback_used = True
    decision = {
        "policy_id": contract["analytic_ranker"]["ranker_id"],
        "selected_candidate_id": selected["candidate_id"],
        "selection_propensity": 1.0,
        "safety_fallback_used": fallback_used,
        "candidate_set_digest": candidate_set[
            "candidate_set_digest"
        ],
        "world_model_actor_order_invariant": True,
        "recovery_extension_used": True,
    }
    decision["decision_digest"] = base.digest(decision)
    return decision


def plan(
    snapshot: Mapping[str, Any],
    model: Mapping[str, Any],
    contract: Mapping[str, Any],
    extension: Mapping[str, Any],
    recovery_context: bool,
) -> Dict[str, Any]:
    """Plan only in an explicitly declared post-certified-stop context."""
    base.require(bool(recovery_context), "RECOVERY_CONTEXT_REQUIRED")
    base.require(
        int(contract["minimum_actor_count"])
        <= len(snapshot.get("actors", []))
        <= int(contract["maximum_actor_count"]),
        "ACTOR_COUNT_OUT_OF_SCOPE",
    )
    track_ids = [
        actor["track_id"] for actor in snapshot["actors"]
    ]
    base.require(
        len(track_ids) == len(set(track_ids)),
        "DUPLICATE_TRACK_ID",
    )
    base.require(
        all(
            "actor_policy" not in actor
            and "hidden_type" not in actor
            for actor in snapshot["actors"]
        ),
        "HIDDEN_POLICY_RUNTIME_LEAK",
    )
    candidate_set = recovery.generate_candidate_set(
        snapshot, contract, extension, True
    )
    evaluation_contract = recovery._candidate_contract(
        contract, extension
    )
    evaluations = [
        predictor.evaluate_candidate(
            snapshot, candidate, model, evaluation_contract
        )
        for candidate in candidate_set["candidates"]
    ]
    decision = _select(candidate_set, evaluations, contract)
    return {
        "candidate_set": candidate_set,
        "evaluations": evaluations,
        "decision": decision,
        "evaluation_contract_factor": (
            "R4_R1_recovery_geometry_envelope_only"
        ),
    }
