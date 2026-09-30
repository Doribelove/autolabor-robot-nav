"""Five-action progress-relative V3-13 recovery planner.

This module keeps the frozen D0 verifier and analytic ranker, appends the
single R10 progress-relative recovery candidate, and evaluates it with the
R13 candidate-conditioned predictor.  It is valid only after the explicit
certified-stop recovery prefix.
"""

from typing import Any, Dict, List, Mapping

from v3_scope_planner import expanded_recovery_core as selection
from v3_scope_planner import multi_actor_core as base
from v3_scope_planner import multi_actor_core_v1 as predictor
from v3_scope_planner import progress_recovery_candidate_core as recovery


def _select(
    candidate_set: Mapping[str, Any],
    evaluations: List[Mapping[str, Any]],
    contract: Mapping[str, Any],
) -> Dict[str, Any]:
    return selection._select(candidate_set, evaluations, contract)


def plan(
    snapshot: Mapping[str, Any],
    model: Mapping[str, Any],
    contract: Mapping[str, Any],
    extension: Mapping[str, Any],
    recovery_context: bool,
) -> Dict[str, Any]:
    """Select a hard-verified action in the post-stop recovery context."""
    base.require(bool(recovery_context), "RECOVERY_CONTEXT_REQUIRED")
    base.require(
        int(contract["minimum_actor_count"])
        <= len(snapshot.get("actors", []))
        <= int(contract["maximum_actor_count"]),
        "ACTOR_COUNT_OUT_OF_SCOPE",
    )
    track_ids = [actor["track_id"] for actor in snapshot["actors"]]
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
    evaluation_contract = recovery.evaluation_contract(
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
            "R10_progress_relative_recovery_geometry_only"
        ),
        "predictor_generation": "R13",
        "hard_verifier_and_ranker_generation": "D0_frozen",
    }
