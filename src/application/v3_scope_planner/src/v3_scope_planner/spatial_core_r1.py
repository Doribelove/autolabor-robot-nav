"""Finite-diagnostic wrapper for frozen V3 spatial planner semantics."""

import copy
import math

from . import spatial_core as base

PlannerError = base.PlannerError
require = base.require
finite = base.finite
digest = base.digest
infer_mode = base.infer_mode
actor_cleared = base.actor_cleared
generate_candidate_set = base.generate_candidate_set
predict_spatial_tube = base.predict_spatial_tube

FINITE_REJECTED_CLEARANCE_SENTINEL_M = -1.0e9


def normalize_diagnostics(result):
    value = copy.deepcopy(result)
    for evaluation in value["evaluations"]:
        clearance = evaluation.get("minimum_clearance_m")
        if not math.isfinite(float(clearance)):
            evaluation["minimum_clearance_m"] = (
                FINITE_REJECTED_CLEARANCE_SENTINEL_M
            )
    return value


def evaluate_candidate_set(snapshot, candidate_set, model, contract):
    return normalize_diagnostics(
        base.evaluate_candidate_set(snapshot, candidate_set, model, contract)
    )


def plan(snapshot, model, contract):
    return evaluate_candidate_set(
        snapshot, generate_candidate_set(snapshot, contract), model, contract
    )
