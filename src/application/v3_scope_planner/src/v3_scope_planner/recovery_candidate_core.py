"""Pure V3-13 recovery-only candidate-bank extension.

The frozen D0 four-action generator remains untouched.  This module appends
one prospectively specified evasive candidate only when the caller declares a
post-stop recovery context.  It does not add a predictor head or planner
authority.
"""

import copy
import math
from typing import Any, Dict, Mapping, Tuple

from v3_scope_planner import multi_actor_core as base


def require(condition: bool, message: str) -> None:
    if not condition:
        raise base.MultiActorPlannerError(message)


def physical_curvature_limit(extension: Mapping[str, Any]) -> float:
    envelope = extension["hardware_envelope"]
    return math.tan(float(envelope["maximum_steering_angle_rad"])) / float(
        envelope["wheelbase_m"]
    )


def _candidate_contract(
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
    generator["outbound_x_range_m"] = list(spec["outbound_x_range_m"])
    generator["return_x_range_m"] = list(spec["return_x_range_m"])
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
    vehicle["maximum_curvature_inv_m"] = physical_curvature_limit(extension)
    return result


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
    local_contract = _candidate_contract(frozen_contract, extension)
    candidate_spec = {
        "candidate_id": spec["candidate_id"],
        "kind": spec["kind"],
        "direction": int(spec["direction"]),
        "target_speed_mps": float(spec["target_speed_mps"]),
        "certified_stop": bool(spec["certified_stop"]),
    }
    candidate = base._generate_candidate(
        snapshot, candidate_spec, local_contract
    )
    failures = list(base.validate_candidate(candidate, local_contract))
    require(not failures, "RECOVERY_CANDIDATE_INVALID:" + ",".join(failures))
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
    require(len(identifiers) == len(set(identifiers)), "DUPLICATE_CANDIDATE_ID")
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
        "legacy_candidate_set_digest": legacy["candidate_set_digest"],
        "extension_count": extension_count,
        "candidates": candidates,
    }
    value["candidate_set_digest"] = base.digest(value)
    return value
