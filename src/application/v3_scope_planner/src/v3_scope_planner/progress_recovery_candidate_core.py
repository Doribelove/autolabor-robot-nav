"""Translation-invariant recovery candidate generation for V3-13.

The candidate steering phase is evaluated against longitudinal progress from
the current recovery pose.  The resulting local Ackermann trajectory is then
rigidly transformed into the world frame.  This avoids the absolute-world-x
phase defect frozen by Q1-R9.
"""

import copy
import math
from typing import Any, Dict, Mapping, Tuple

from v3_scope_planner import multi_actor_core as base


def require(condition: bool, message: str) -> None:
    if not condition:
        raise base.MultiActorPlannerError(message)


def evaluation_contract(
    frozen_contract: Mapping[str, Any],
    extension: Mapping[str, Any],
) -> Dict[str, Any]:
    result = copy.deepcopy(dict(frozen_contract))
    spec = extension["candidate_extension"]
    envelope = extension["hardware_envelope"]
    generator = result["candidate_generator"]
    vehicle = result["vehicle"]
    generator["generator_id"] = spec["generator_id"]
    generator["horizon_s"] = float(spec["horizon_s"])
    generator["sample_period_s"] = float(spec["sample_period_s"])
    generator["outbound_x_range_m"] = list(
        spec["outbound_progress_range_m"]
    )
    generator["return_x_range_m"] = list(
        spec["return_progress_range_m"]
    )
    generator["steering_peak_rad"] = float(spec["steering_peak_rad"])
    vehicle["maximum_acceleration_mps2"] = float(
        spec["longitudinal_acceleration_limit_mps2"]
    )
    vehicle["maximum_steering_angle_rad"] = float(
        envelope["maximum_steering_angle_rad"]
    )
    vehicle["maximum_steering_rate_radps"] = float(
        spec["steering_rate_limit_radps"]
    )
    vehicle["maximum_curvature_inv_m"] = math.tan(
        float(envelope["maximum_steering_angle_rad"])
    ) / float(envelope["wheelbase_m"])
    return result


def _world_transform(
    candidate: Mapping[str, Any],
    snapshot: Mapping[str, Any],
) -> Dict[str, Any]:
    value = copy.deepcopy(dict(candidate))
    origin_x = float(snapshot["ego"]["x_m"])
    origin_y = float(snapshot["ego"]["y_m"])
    origin_yaw = float(snapshot["ego"]["yaw_rad"])
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    for point in value["points"]:
        local_x = float(point["x_m"])
        local_y = float(point["y_m"])
        point["x_m"] = round(
            origin_x + cosine * local_x - sine * local_y, 12
        )
        point["y_m"] = round(
            origin_y + sine * local_x + cosine * local_y, 12
        )
        point["yaw_rad"] = round(
            origin_yaw + float(point["yaw_rad"]), 12
        )
    value["scene_snapshot_id"] = snapshot["scene_snapshot_id"]
    body = copy.deepcopy(value)
    body.pop("candidate_digest", None)
    value["candidate_digest"] = base.digest(body)
    return value


def generate_recovery_candidate(
    snapshot: Mapping[str, Any],
    frozen_contract: Mapping[str, Any],
    extension: Mapping[str, Any],
    recovery_context: bool,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    spec = extension["candidate_extension"]
    require(bool(recovery_context), "RECOVERY_CONTEXT_REQUIRED")
    require(spec["recovery_only"] is True, "RECOVERY_ONLY_FLAG_REQUIRED")
    require(
        spec["eligible_after_certified_stop_prefix_only"] is True,
        "CERTIFIED_STOP_PREFIX_REQUIRED",
    )
    require(
        spec["phase_coordinate"]
        == "longitudinal_progress_from_recovery_pose_m",
        "PROGRESS_PHASE_REQUIRED",
    )
    local_contract = evaluation_contract(
        frozen_contract, extension
    )
    local_snapshot = {
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "ego": {
            "x_m": 0.0,
            "y_m": 0.0,
            "yaw_rad": 0.0,
            "speed_mps": float(snapshot["ego"]["speed_mps"]),
        },
    }
    candidate_spec = {
        "candidate_id": spec["candidate_id"],
        "kind": spec["kind"],
        "direction": int(spec["direction"]),
        "target_speed_mps": float(spec["target_speed_mps"]),
        "certified_stop": bool(spec["certified_stop"]),
    }
    local_candidate = base._generate_candidate(
        local_snapshot, candidate_spec, local_contract
    )
    candidate = _world_transform(local_candidate, snapshot)
    failures = list(
        base.validate_candidate(candidate, local_contract)
    )
    require(
        not failures,
        "PROGRESS_RECOVERY_CANDIDATE_INVALID:"
        + ",".join(failures),
    )
    return candidate, local_contract


def generate_candidate_set(
    snapshot: Mapping[str, Any],
    frozen_contract: Mapping[str, Any],
    extension: Mapping[str, Any],
    recovery_context: bool,
) -> Dict[str, Any]:
    legacy = base.generate_candidate_set(snapshot, frozen_contract)
    candidates = copy.deepcopy(legacy["candidates"])
    extension_count = 0
    if recovery_context:
        candidate, _ = generate_recovery_candidate(
            snapshot, frozen_contract, extension, True
        )
        candidates.append(candidate)
        extension_count = 1
    identifiers = [item["candidate_id"] for item in candidates]
    require(
        len(identifiers) == len(set(identifiers)),
        "DUPLICATE_CANDIDATE_ID",
    )
    value = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "world_model_seq": int(snapshot["world_model_seq"]),
        "generator_id": (
            extension["candidate_extension"]["generator_id"]
            if recovery_context
            else legacy["generator_id"]
        ),
        "recovery_context": bool(recovery_context),
        "phase_coordinate": (
            "longitudinal_progress_from_recovery_pose_m"
            if recovery_context
            else "frozen_legacy"
        ),
        "legacy_candidate_set_digest": legacy[
            "candidate_set_digest"
        ],
        "extension_count": extension_count,
        "candidates": candidates,
    }
    value["candidate_set_digest"] = base.digest(value)
    return value
