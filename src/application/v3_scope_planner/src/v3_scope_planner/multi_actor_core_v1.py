"""V3-13 candidate-conditioned contextual predictor over the frozen D0 core."""

import math

from v3_scope_planner import multi_actor_core as base


MultiActorPlannerError = base.MultiActorPlannerError
digest = base.digest
finite = base.finite
require = base.require
generate_candidate_set = base.generate_candidate_set
validate_candidate = base.validate_candidate
infer_observable_mode = base.infer_observable_mode
validate_cloned_intervention_block = base.validate_cloned_intervention_block


def contextual_features(actor, time_s):
    time_s = float(time_s)
    initial_speed = float(actor["coordinate_speed_mps"])
    initial_coordinate = float(actor["coordinate_m"])
    origin_x = float(actor["path_origin_x_m"])
    return [
        time_s,
        time_s * time_s,
        time_s * time_s * time_s,
        initial_speed * time_s,
        -initial_coordinate * time_s,
        origin_x * time_s,
        origin_x * time_s * time_s,
    ]


def predict_actor_tube(actor, candidate, model, contract):
    required = (
        "track_id",
        "coordinate_m",
        "coordinate_speed_mps",
        "path_angle_deg",
        "path_origin_x_m",
        "path_origin_y_m",
        "age_s",
    )
    require(
        all(base.finite(actor.get(field)) for field in required if field != "track_id"),
        "NONFINITE_ACTOR_TRACK",
    )
    require("actor_policy" not in actor, "HIDDEN_POLICY_RUNTIME_LEAK")
    require("hidden_type" not in actor, "HIDDEN_POLICY_RUNTIME_LEAK")
    require(
        model["model_type"] == "candidate_conditioned_contextual_ridge_v1",
        "UNSUPPORTED_PREDICTOR_TYPE",
    )
    mode = infer_observable_mode(
        float(actor["coordinate_speed_mps"]), contract
    )
    require(mode in model["heads"], "UNSUPPORTED_OBSERVABLE_MODE:" + mode)
    head = model["heads"][mode][candidate["candidate_id"]]
    coefficients = [float(value) for value in head["coefficients"]]
    require(len(coefficients) == 7, "PREDICTOR_COEFFICIENT_COUNT")
    radius = float(model["calibration"]["coordinate_radius_m"])
    growth = float(model["calibration"]["coordinate_growth_mps"])
    initial_coordinate = float(actor["coordinate_m"])
    samples = []
    for point in candidate["points"]:
        time_s = float(point["time_from_start_s"])
        features = contextual_features(actor, time_s)
        displacement = sum(
            coefficient * feature
            for coefficient, feature in zip(coefficients, features)
        )
        center = initial_coordinate + displacement
        half_width = radius + growth * time_s
        samples.append(
            {
                "time_s": time_s,
                "lower_coordinate_m": center - half_width,
                "upper_coordinate_m": center + half_width,
            }
        )
    value = {
        "track_id": actor["track_id"],
        "candidate_id": candidate["candidate_id"],
        "observable_mode": mode,
        "feature_order": list(model["feature_order"]),
        "samples": samples,
    }
    value["tube_digest"] = digest(value)
    return value


def _with_contextual_predictor(function, *arguments):
    original = base.predict_actor_tube
    base.predict_actor_tube = predict_actor_tube
    try:
        return function(*arguments)
    finally:
        base.predict_actor_tube = original


def evaluate_candidate(snapshot, candidate, model, contract):
    return _with_contextual_predictor(
        base.evaluate_candidate, snapshot, candidate, model, contract
    )


def plan(snapshot, model, contract):
    return _with_contextual_predictor(
        base.plan, snapshot, model, contract
    )

