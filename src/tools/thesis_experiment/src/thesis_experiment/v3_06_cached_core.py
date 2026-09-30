"""Semantics-preserving per-plan cache for duplicate candidate predictions."""

import copy

from v3_scope_planner import core as base


digest = base.digest
canonical_bytes = base.canonical_bytes
actor_cleared = base.actor_cleared
_combined_radius = base._combined_radius


def _cache_key(candidate):
    return (
        float(candidate["target_speed_mps"]),
        base.digest(candidate["points"]),
    )


def plan_with_diagnostics(snapshot, model, contract):
    original = base.predict_occupancy_tube
    cache = {}
    calls = {"computed": 0, "reused": 0}

    def cached(snapshot_value, candidate, model_value, contract_value):
        key = _cache_key(candidate)
        if key not in cache:
            value = original(
                snapshot_value, candidate, model_value, contract_value
            )
            cache[key] = copy.deepcopy(value)
            calls["computed"] += 1
            return value
        value = copy.deepcopy(cache[key])
        value["candidate_id"] = candidate["candidate_id"]
        value["features"] = base._features(snapshot_value, candidate)
        value.pop("occupancy_tube_digest", None)
        value["occupancy_tube_digest"] = base.digest(value)
        calls["reused"] += 1
        return value

    base.predict_occupancy_tube = cached
    try:
        result = base.plan(snapshot, model, contract)
    finally:
        base.predict_occupancy_tube = original
    return result, calls


def plan(snapshot, model, contract):
    return plan_with_diagnostics(snapshot, model, contract)[0]
