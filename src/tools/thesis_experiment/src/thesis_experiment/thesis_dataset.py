"""Canonical, non-invasive thesis experiment row schema and validation."""
import hashlib
import json
import math


SCHEMA_VERSION = "1.0"

FIELD_GROUPS = {
    "identity": (
        "stage", "sequence", "block_id", "attempt", "seed", "scene_id",
        "actor_policy", "method", "method_role",
    ),
    "execution": (
        "simulation_only", "completed", "terminal_reason", "retry_used",
        "resume_used", "wall_time_s", "sim_time_s",
    ),
    "task": (
        "goal_success", "restricted_time_s", "path_length_m",
        "final_position_error_m",
    ),
    "safety": (
        "contact_count", "collision", "minimum_clearance_m",
        "emergency_count", "safety_fallback_used",
    ),
    "planner": (
        "decision_count", "selected_candidate_id", "selected_accepted",
        "certified_stop_available", "planner_latency_p99_ms",
        "tracking_error_max_m",
    ),
    "provenance": (
        "contract_sha256", "release_sha256", "schedule_sha256",
        "report_sha256", "raw_manifest_sha256", "source_revision",
    ),
    "claim_scope": (
        "scientific_inclusion", "statistical_unit", "oracle_track_used",
        "training_steps", "hardware_accessed", "real_vehicle_used",
    ),
}

REQUIRED_FIELDS = tuple(
    field for group in FIELD_GROUPS.values() for field in group
)


def canonical_json(value):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False,
    ).encode("utf-8")


def digest(value):
    return hashlib.sha256(canonical_json(value)).hexdigest()


def _finite_or_none(value):
    return value is None or (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def validate_row(row):
    missing = [field for field in REQUIRED_FIELDS if field not in row]
    if missing:
        raise ValueError("missing fields: {}".format(",".join(missing)))
    if row.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("schema_version differs")
    if int(row["sequence"]) < 1 or int(row["attempt"]) < 1:
        raise ValueError("sequence/attempt must be positive")
    if not isinstance(row["scientific_inclusion"], bool):
        raise ValueError("scientific_inclusion must be boolean")
    for field in (
        "wall_time_s", "sim_time_s", "restricted_time_s", "path_length_m",
        "final_position_error_m", "minimum_clearance_m",
        "planner_latency_p99_ms", "tracking_error_max_m",
    ):
        if not _finite_or_none(row[field]):
            raise ValueError("{} must be finite or null".format(field))
    for field in (
        "simulation_only", "completed", "retry_used", "resume_used",
        "goal_success", "collision", "safety_fallback_used",
        "selected_accepted", "certified_stop_available", "oracle_track_used",
        "hardware_accessed", "real_vehicle_used",
    ):
        if row[field] is not None and not isinstance(row[field], bool):
            raise ValueError("{} must be boolean or null".format(field))
    if row["hardware_accessed"] or row["real_vehicle_used"]:
        raise ValueError("thesis simulation row unexpectedly accessed hardware")
    for field in (
        "contract_sha256", "release_sha256", "schedule_sha256",
        "report_sha256", "raw_manifest_sha256",
    ):
        value = row[field]
        if value and (len(value) != 64 or any(c not in "0123456789abcdef" for c in value)):
            raise ValueError("{} is not a lowercase SHA-256".format(field))
    return True


def schema_document():
    return {
        "schema_version": SCHEMA_VERSION,
        "row_unit": "one_executed_episode_or_declared_offline_unit",
        "field_groups": {key: list(value) for key, value in FIELD_GROUPS.items()},
        "missing_value_policy": "use null; never zero-fill an unavailable measurement",
        "failure_policy": "retain every consumed identity and encode terminal_reason",
        "statistical_policy": "block/scene-seed is the independent unit; frames are repeated measures",
        "provenance_policy": "bind contract/release/schedule/report/raw manifest with SHA-256",
        "authority_policy": "simulation/offline by default; hardware rows require separate on-site contract",
    }

