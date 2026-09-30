"""Raw-first replay helpers for the future V2-04G-P2-Q1 execution.

The functions operate on explicit samples and receipts only.  They do not trust
episode summary metrics and do not import ROS.  Q1 uses deterministic synthetic
fixtures to review these rules before any execution release exists.
"""

from __future__ import annotations

import hashlib
import math
import os
from pathlib import Path
import re
import stat
from typing import Any, Dict, List, Mapping, Sequence, Tuple

import yaml


STAGE = "V2-04G-P2-Q1"
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")
NAVIGATION_REQUIRED_SAMPLE_FIELDS = {
    "stamp_s",
    "x_m",
    "y_m",
    "linear_velocity_mps",
    "yaw_rate_radps",
    "clearance_m",
    "predicted_ttc_s",
    "collision",
    "goal_reached",
    "emergency",
    "projection_applied",
    "transaction_valid",
    "tracker_valid",
}
QUALIFICATION_REQUIRED_FIELDS = {
    "stamp_s",
    "x_m",
    "y_m",
    "linear_velocity_mps",
    "yaw_rate_radps",
    "clearance_m",
    "collision",
    "dynamic_overlay",
    "conflict_triggered",
    "emergency",
    "projection_applied",
    "transaction_valid",
}


class Q1ReplayError(ValueError):
    """Raised when a raw Q1 resource cannot be replayed deterministically."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise Q1ReplayError(message)


def finite(value: Any, label: str) -> float:
    require(
        type(value) in (int, float) and not isinstance(value, bool),
        "{} must be numeric".format(label),
    )
    number = float(value)
    require(math.isfinite(number), "{} must be finite".format(label))
    return number


def safe_relative(value: str | Path) -> Path:
    relative = value if isinstance(value, Path) else Path(value)
    require(
        not relative.is_absolute() and relative.parts and ".." not in relative.parts,
        "unsafe relative path",
    )
    return relative


def read_once_no_follow(workspace: Path, relative: str | Path) -> bytes:
    relative = safe_relative(relative)
    current = workspace
    for component in relative.parts:
        current = current / component
        require(not current.is_symlink(), "symbolic link forbidden: {}".format(relative))
    descriptor = os.open(
        str(workspace / relative),
        os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0),
    )
    try:
        before = os.fstat(descriptor)
        require(stat.S_ISREG(before.st_mode), "not a regular file")
        chunks: List[bytes] = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
        require(
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
            "file changed during replay",
        )
        payload = b"".join(chunks)
        require(len(payload) == before.st_size, "short read")
        return payload
    finally:
        os.close(descriptor)


def load_yaml(payload: bytes, label: str) -> Dict[str, Any]:
    try:
        value = yaml.safe_load(payload.decode("utf-8"))
    except Exception as exc:
        raise Q1ReplayError("{} parse failed: {}".format(label, exc))
    require(type(value) is dict, "{} must be a mapping".format(label))
    return value


def canonical_sha256(value: Any) -> str:
    payload = yaml.safe_dump(
        value, sort_keys=True, allow_unicode=True, width=100
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _normalized_samples(
    samples: Sequence[Mapping[str, Any]], required_fields: set[str]
) -> List[Dict[str, Any]]:
    require(type(samples) is list and len(samples) >= 2, "raw sample trace is too short")
    output: List[Dict[str, Any]] = []
    previous_stamp = None
    for index, sample in enumerate(samples):
        require(type(sample) is dict, "sample {} is not a mapping".format(index))
        require(set(sample) == required_fields, "sample {} fields differ".format(index))
        row: Dict[str, Any] = {}
        for field in (
            "stamp_s",
            "x_m",
            "y_m",
            "linear_velocity_mps",
            "yaw_rate_radps",
            "clearance_m",
        ):
            row[field] = finite(sample[field], "sample {} {}".format(index, field))
        require(row["clearance_m"] >= 0.0, "sample clearance is negative")
        if previous_stamp is not None:
            require(row["stamp_s"] > previous_stamp, "sample stamps are not increasing")
        previous_stamp = row["stamp_s"]
        for field in required_fields - set(row):
            value = sample[field]
            if field == "predicted_ttc_s":
                row[field] = None if value is None else finite(
                    value, "sample {} predicted_ttc_s".format(index)
                )
                if row[field] is not None:
                    require(row[field] >= 0.0, "predicted TTC is negative")
            elif field in {
                "collision",
                "goal_reached",
                "emergency",
                "projection_applied",
                "transaction_valid",
                "conflict_triggered",
                "tracker_valid",
            }:
                require(type(value) is bool, "sample {} {} is not bool".format(index, field))
                row[field] = value
            elif field == "dynamic_overlay":
                require(
                    value in {"NONE", "CROSSING", "HEAD_ON", "FOLLOW", "OVERTAKE_OR_YIELD"},
                    "sample dynamic overlay differs",
                )
                row[field] = value
            else:
                raise Q1ReplayError("unhandled sample field {}".format(field))
        output.append(row)
    return output


def _path_length(rows: Sequence[Mapping[str, Any]], end_index: int | None = None) -> float:
    if end_index is None:
        end_index = len(rows) - 1
    require(0 <= end_index < len(rows), "path end index differs")
    return sum(
        math.hypot(
            rows[index]["x_m"] - rows[index - 1]["x_m"],
            rows[index]["y_m"] - rows[index - 1]["y_m"],
        )
        for index in range(1, end_index + 1)
    )


def normalize_navigation_trace(
    planned: Mapping[str, Any], samples: Sequence[Mapping[str, Any]]
) -> Dict[str, Any]:
    require(planned.get("row_type") == "calibration_navigation", "planned row type differs")
    rows = _normalized_samples(samples, NAVIGATION_REQUIRED_SAMPLE_FIELDS)
    collision = any(row["collision"] for row in rows)
    goal_reached = any(row["goal_reached"] for row in rows)
    stopped = abs(rows[-1]["linear_velocity_mps"]) <= 0.05
    success = bool(goal_reached and stopped and not collision)
    duration = rows[-1]["stamp_s"] - rows[0]["stamp_s"]
    timeout = finite(planned.get("timeout_s"), "planned timeout")
    require(duration > 0.0 and duration <= timeout + 0.5, "navigation duration differs")
    restricted = duration if success else timeout
    require(any(row["tracker_valid"] for row in rows), "navigation tracker is never valid")
    finite_ttc = [
        row["predicted_ttc_s"]
        for row in rows
        if row["tracker_valid"] and row["predicted_ttc_s"] is not None
    ]
    normal_rows = [row for row in rows if not row["emergency"]]
    require(normal_rows, "navigation trace has no normal samples")
    projection_rate = sum(
        row["projection_applied"] for row in normal_rows
    ) / float(len(normal_rows))
    invalid_transactions = sum(not row["transaction_valid"] for row in rows)
    return {
        "sequence": planned["sequence"],
        "row_type": planned["row_type"],
        "method_id": planned["method_id"],
        "scene_id": planned["scene_id"],
        "scene_role": planned["scene_role"],
        "seed": planned["seed"],
        "attempt": planned["attempt"],
        "block_id": planned["block_id"],
        "success": success,
        "collision": collision,
        "emergency": any(row["emergency"] for row in rows),
        "navigation_time_s": duration,
        "restricted_completion_time_s": restricted,
        "path_length_m": _path_length(rows),
        "minimum_footprint_clearance_m": min(row["clearance_m"] for row in rows),
        "observed_ttc_status": (
            "OBSERVED_CONFLICT" if finite_ttc else "NO_CONFLICT_IN_HORIZON"
        ),
        "normal_projection_rate": projection_rate,
        "normal_trace_sample_count": len(normal_rows),
        "invalid_transaction_count": invalid_transactions,
        "trace_sample_count": len(rows),
        "raw_trace_sha256": canonical_sha256(list(samples)),
    }


def assess_qualification_trace(
    planned: Mapping[str, Any],
    samples: Sequence[Mapping[str, Any]],
    thresholds: Mapping[str, Any],
) -> Dict[str, Any]:
    require(planned.get("row_type") == "qualification_probe", "planned row type differs")
    rows = _normalized_samples(samples, QUALIFICATION_REQUIRED_FIELDS)
    speed = finite(planned.get("candidate_speed_mps"), "candidate speed")
    probe = planned.get("probe_type")
    require(
        probe
        in {"braking", "high_speed_to_conflict_overlay_transition", "ackermann_circle"},
        "qualification probe differs",
    )
    collision = any(row["collision"] for row in rows)
    emergency = any(row["emergency"] for row in rows)
    invalid_transactions = sum(not row["transaction_valid"] for row in rows)
    base = {
        "sequence": planned["sequence"],
        "row_type": planned["row_type"],
        "probe_type": probe,
        "candidate_speed_mps": speed,
        "scene_id": planned["scene_id"],
        "seed": planned["seed"],
        "attempt": planned["attempt"],
        "collision": collision,
        "emergency": emergency,
        "normal_projection_rate": (
            sum(row["projection_applied"] for row in rows if not row["emergency"])
            / float(sum(not row["emergency"] for row in rows))
            if any(not row["emergency"] for row in rows)
            else 1.0
        ),
        "invalid_transaction_sample_count": invalid_transactions,
        "trace_sample_count": len(rows),
        "raw_trace_sha256": canonical_sha256(list(samples)),
    }
    if probe == "braking":
        threshold = thresholds["braking"]
        start_index = max(
            range(len(rows)), key=lambda index: abs(rows[index]["linear_velocity_mps"])
        )
        initial_speed = abs(rows[start_index]["linear_velocity_mps"])
        stop_index = next(
            (
                index
                for index in range(start_index + 1, len(rows))
                if abs(rows[index]["linear_velocity_mps"]) <= 0.05
            ),
            None,
        )
        require(stop_index is not None, "braking trace never stops")
        stop_time = rows[stop_index]["stamp_s"] - rows[start_index]["stamp_s"]
        stop_distance = _path_length(rows[start_index : stop_index + 1])
        passed = (
            abs(initial_speed - speed)
            <= finite(threshold["candidate_speed_tolerance_mps"], "speed tolerance")
            and stop_time <= finite(threshold["maximum_stop_time_s"], "maximum stop time")
            and stop_distance
            <= finite(threshold["maximum_stopping_distance_m"], "maximum stop distance")
            and not collision
            and not emergency
            and invalid_transactions == 0
        )
        base.update(
            {
                "initial_speed_mps": initial_speed,
                "stop_time_s": stop_time,
                "stopping_distance_m": stop_distance,
                "pass": passed,
            }
        )
    elif probe == "high_speed_to_conflict_overlay_transition":
        threshold = thresholds["high_speed_to_conflict_overlay_transition"]
        speed_ready_index = next(
            (
                index
                for index, row in enumerate(rows)
                if abs(row["linear_velocity_mps"] - speed) <= 0.08
            ),
            None,
        )
        require(speed_ready_index is not None, "overlay trace never reaches candidate speed")
        trigger_index = next(
            (
                index
                for index in range(speed_ready_index, len(rows))
                if rows[index]["conflict_triggered"]
            ),
            None,
        )
        require(trigger_index is not None, "overlay trace has no conflict trigger")
        activation_index = next(
            (
                index
                for index in range(trigger_index, len(rows))
                if rows[index]["dynamic_overlay"] != "NONE"
            ),
            None,
        )
        require(activation_index is not None, "overlay trace never activates")
        latency = rows[activation_index]["stamp_s"] - rows[trigger_index]["stamp_s"]
        speed_at_safe_state = abs(rows[activation_index]["linear_velocity_mps"])
        maximum_speed_before_safe_state = max(
            abs(row["linear_velocity_mps"])
            for row in rows[trigger_index : activation_index + 1]
        )
        minimum_clearance = min(row["clearance_m"] for row in rows)
        passed = (
            latency
            <= finite(threshold["maximum_activation_latency_s"], "activation latency")
            and speed_at_safe_state
            <= finite(threshold["maximum_speed_at_safe_state_mps"], "safe-state speed")
            + 1e-12
            and minimum_clearance
            >= finite(threshold["minimum_clearance_m"], "minimum clearance")
            and not collision
            and not emergency
            and invalid_transactions == 0
        )
        base.update(
            {
                "activation_latency_s": latency,
                "speed_at_safe_state_mps": speed_at_safe_state,
                "maximum_speed_before_safe_state_mps": maximum_speed_before_safe_state,
                "minimum_clearance_m": minimum_clearance,
                "pass": passed,
            }
        )
    else:
        threshold = thresholds["ackermann_circle"]
        require(len(rows) >= 4, "circle trace is too short")
        steady = rows[len(rows) // 3 : max(len(rows) // 3 + 1, 3 * len(rows) // 4)]
        steady_speed = sum(abs(row["linear_velocity_mps"]) for row in steady) / len(steady)
        steady_yaw = sum(abs(row["yaw_rate_radps"]) for row in steady) / len(steady)
        require(steady_yaw > 1e-9, "circle trace yaw rate is zero")
        radius = steady_speed / steady_yaw
        target_radius = 1.2
        radius_error = abs(radius - target_radius)
        passed = (
            abs(steady_speed - speed)
            <= finite(threshold["steady_speed_tolerance_mps"], "steady speed tolerance")
            and radius_error
            <= finite(threshold["maximum_radius_error_m"], "maximum radius error")
            and not collision
            and not emergency
            and invalid_transactions == 0
        )
        base.update(
            {
                "steady_speed_mps": steady_speed,
                "steady_yaw_rate_radps": steady_yaw,
                "radius_m": radius,
                "radius_error_m": radius_error,
                "pass": passed,
            }
        )
    return base


def replay_journal(
    workspace: Path,
    planned: Mapping[str, Any],
    journal_relative: str | Path,
    release_sha256: str,
    thresholds: Mapping[str, Any],
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    require(HEX_SHA256.fullmatch(release_sha256) is not None, "release SHA differs")
    journal_payload = read_once_no_follow(workspace, journal_relative)
    journal = load_yaml(journal_payload, "journal")
    require(
        set(journal)
        == {
            "schema_version",
            "stage",
            "release_sha256",
            "identity",
            "status",
            "consumed",
            "raw_trace",
            "integrity_failures",
        },
        "journal schema differs",
    )
    require(journal["stage"] == STAGE, "journal stage differs")
    require(journal["release_sha256"] == release_sha256, "journal release differs")
    identity_keys = (
        "sequence",
        "row_type",
        "scene_id",
        "seed",
        "attempt",
    )
    expected_identity = {key: planned[key] for key in identity_keys}
    if planned["row_type"] == "calibration_navigation":
        expected_identity["method_id"] = planned["method_id"]
        expected_identity["block_id"] = planned["block_id"]
    else:
        expected_identity["probe_type"] = planned["probe_type"]
        expected_identity["candidate_speed_mps"] = planned["candidate_speed_mps"]
    require(journal["identity"] == expected_identity, "journal identity differs")
    require(journal["status"] == "completed", "journal is not completed")
    require(journal["consumed"] is True, "journal is not consumed")
    require(journal["integrity_failures"] == [], "journal integrity failure")
    receipt = journal["raw_trace"]
    require(
        type(receipt) is dict and set(receipt) == {"path", "sha256", "size_bytes"},
        "raw trace receipt differs",
    )
    raw_payload = read_once_no_follow(workspace, receipt["path"])
    require(sha256_bytes(raw_payload) == receipt["sha256"], "raw trace SHA differs")
    require(len(raw_payload) == receipt["size_bytes"], "raw trace size differs")
    raw = load_yaml(raw_payload, "raw trace")
    require(raw.get("stage") == STAGE, "raw trace stage differs")
    require(raw.get("identity") == expected_identity, "raw trace identity differs")
    samples = raw.get("samples")
    if planned["row_type"] == "calibration_navigation":
        normalized = normalize_navigation_trace(planned, samples)
    else:
        normalized = assess_qualification_trace(planned, samples, thresholds)
    audit = {
        "sequence": planned["sequence"],
        "journal_path": safe_relative(journal_relative).as_posix(),
        "journal_sha256": sha256_bytes(journal_payload),
        "raw_trace_path": safe_relative(receipt["path"]).as_posix(),
        "raw_trace_sha256": receipt["sha256"],
        "normalized_sha256": canonical_sha256(normalized),
    }
    return normalized, audit


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()
