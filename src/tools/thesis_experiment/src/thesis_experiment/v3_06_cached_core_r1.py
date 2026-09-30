"""Duplicate prediction cache with equivalent exception replay."""

import copy

from v3_scope_planner import core as base

digest = base.digest
canonical_bytes = base.canonical_bytes
actor_cleared = base.actor_cleared
_combined_radius = base._combined_radius


def _cache_key(candidate):
    return (float(candidate["target_speed_mps"]), base.digest(candidate["points"]))


def plan_with_diagnostics(snapshot, model, contract):
    original = base.predict_occupancy_tube
    cache = {}
    calls = {"computed": 0, "reused": 0, "reused_exceptions": 0}

    def cached(snapshot_value, candidate, model_value, contract_value):
        key = _cache_key(candidate)
        if key in cache:
            kind, stored = cache[key]
            calls["reused"] += 1
            if kind == "error":
                calls["reused_exceptions"] += 1
                error_type, error_args = stored
                raise error_type(*error_args)
            value = copy.deepcopy(stored)
            value["candidate_id"] = candidate["candidate_id"]
            value["features"] = base._features(snapshot_value, candidate)
            value.pop("occupancy_tube_digest", None)
            value["occupancy_tube_digest"] = base.digest(value)
            return value
        calls["computed"] += 1
        try:
            value = original(
                snapshot_value, candidate, model_value, contract_value
            )
        except Exception as exc:
            cache[key] = ("error", (type(exc), exc.args))
            raise
        cache[key] = ("value", copy.deepcopy(value))
        return value

    base.predict_occupancy_tube = cached
    try:
        result = base.plan(snapshot, model, contract)
    finally:
        base.predict_occupancy_tube = original
    return result, calls


def plan(snapshot, model, contract):
    return plan_with_diagnostics(snapshot, model, contract)[0]
