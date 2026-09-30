"""Fail-closed schedule-to-runtime action binding for V3 multi-action runs."""

from math import isfinite


class ActionBindingError(ValueError):
    """Raised before evidence consumption or inclusion when identity drifts."""


DEFAULT_R3_CANDIDATE = "recovery_evasive_right_v120_r3"
DEFAULT_ALLOWED_CANDIDATES = (
    "spatial_left_v120",
    "spatial_right_v120",
    "straight_v120",
    "certified_stop_v000",
    DEFAULT_R3_CANDIDATE,
)


def _require(condition, message):
    if not condition:
        raise ActionBindingError(message)


def _strict_int(value, label):
    _require(not isinstance(value, bool), "{} must be an integer".format(label))
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        raise ActionBindingError("{} must be an integer".format(label))
    _require(str(parsed) == str(value), "{} must be canonical".format(label))
    return parsed


def _strict_float(value, label):
    _require(not isinstance(value, bool), "{} must be numeric".format(label))
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        raise ActionBindingError("{} must be numeric".format(label))
    _require(isfinite(parsed), "{} must be finite".format(label))
    return parsed


def validate_schedule_row(row, allowed_candidates=DEFAULT_ALLOWED_CANDIDATES):
    """Validate the immutable fields used to construct a launch command."""

    _require(isinstance(row, dict), "schedule row must be a mapping")
    required = (
        "sequence",
        "block_index",
        "seed",
        "scene_id",
        "forced_candidate_id",
        "behavior_propensity",
    )
    for key in required:
        _require(key in row, "schedule row missing {}".format(key))
    _strict_int(row["sequence"], "sequence")
    _strict_int(row["block_index"], "block_index")
    _strict_int(row["seed"], "seed")
    _require(
        isinstance(row["scene_id"], str) and row["scene_id"],
        "scene_id must be a nonempty string",
    )
    candidate = row["forced_candidate_id"]
    _require(isinstance(candidate, str), "forced_candidate_id must be a string")
    _require(candidate in tuple(allowed_candidates), "candidate is outside frozen bank")
    propensity = _strict_float(row["behavior_propensity"], "behavior_propensity")
    _require(0.0 < propensity <= 1.0, "behavior propensity must be in (0, 1]")
    return row


def build_launch_binding_args(
    row,
    allowed_candidates=DEFAULT_ALLOWED_CANDIDATES,
    r3_candidate=DEFAULT_R3_CANDIDATE,
    legacy_horizon_s=7.0,
    r3_horizon_s=7.55,
):
    """Build only row-derived launch overrides; no action may be hardcoded."""

    validate_schedule_row(row, allowed_candidates)
    candidate = row["forced_candidate_id"]
    horizon = r3_horizon_s if candidate == r3_candidate else legacy_horizon_s
    return [
        "seed:={}".format(row["seed"]),
        "scene_id:={}".format(row["scene_id"]),
        "block_index:={}".format(row["block_index"]),
        "forced_candidate_id:={}".format(candidate),
        "behavior_propensity:={:.12g}".format(float(row["behavior_propensity"])),
        "horizon_s:={:.12g}".format(float(horizon)),
    ]


def _single_override(command, name):
    prefix = "{}:=".format(name)
    matches = [token[len(prefix):] for token in command if token.startswith(prefix)]
    _require(len(matches) == 1, "{} override must occur exactly once".format(name))
    _require(matches[0] != "", "{} override must be nonempty".format(name))
    return matches[0]


def verify_preconsumption_binding(
    row,
    command,
    allowed_candidates=DEFAULT_ALLOWED_CANDIDATES,
    r3_candidate=DEFAULT_R3_CANDIDATE,
    legacy_horizon_s=7.0,
    r3_horizon_s=7.55,
):
    """Verify schedule-to-launch identity before writing ``unit_consumed``."""

    validate_schedule_row(row, allowed_candidates)
    _require(isinstance(command, (list, tuple)), "launch command must be a sequence")
    candidate = _single_override(command, "forced_candidate_id")
    _require(candidate == row["forced_candidate_id"], "scheduled/launch candidate mismatch")
    _require(_strict_int(_single_override(command, "seed"), "launch seed") == row["seed"], "scheduled/launch seed mismatch")
    _require(_single_override(command, "scene_id") == row["scene_id"], "scheduled/launch scene mismatch")
    _require(_strict_int(_single_override(command, "block_index"), "launch block") == row["block_index"], "scheduled/launch block mismatch")
    launch_propensity = _strict_float(
        _single_override(command, "behavior_propensity"), "launch propensity"
    )
    _require(
        abs(launch_propensity - float(row["behavior_propensity"])) <= 1e-12,
        "scheduled/launch propensity mismatch",
    )
    expected_horizon = r3_horizon_s if candidate == r3_candidate else legacy_horizon_s
    launch_horizon = _strict_float(_single_override(command, "horizon_s"), "launch horizon")
    _require(abs(launch_horizon - expected_horizon) <= 1e-12, "candidate/horizon mismatch")
    return {
        "sequence": row["sequence"],
        "scheduled_candidate_id": row["forced_candidate_id"],
        "launch_candidate_id": candidate,
        "binding_pass": True,
        "checked_before_unit_consumed": True,
    }


def verify_runtime_report_binding(
    row, report, allowed_candidates=DEFAULT_ALLOWED_CANDIDATES
):
    """Verify the runtime report before marking a unit complete or scientific."""

    validate_schedule_row(row, allowed_candidates)
    _require(isinstance(report, dict), "runtime report must be a mapping")
    _require("forced_candidate_id" in report, "runtime report lacks candidate identity")
    runtime_candidate = report["forced_candidate_id"]
    _require(isinstance(runtime_candidate, str), "runtime candidate must be a string")
    _require(runtime_candidate in tuple(allowed_candidates), "runtime candidate is outside frozen bank")
    _require(
        runtime_candidate == row["forced_candidate_id"],
        "scheduled/runtime candidate mismatch",
    )
    for field in ("seed", "scene_id", "block_index"):
        _require(field in report, "runtime report lacks {}".format(field))
        _require(report[field] == row[field], "scheduled/runtime {} mismatch".format(field))
    return {
        "sequence": row["sequence"],
        "scheduled_candidate_id": row["forced_candidate_id"],
        "runtime_candidate_id": runtime_candidate,
        "binding_pass": True,
        "checked_before_scientific_inclusion": True,
    }
