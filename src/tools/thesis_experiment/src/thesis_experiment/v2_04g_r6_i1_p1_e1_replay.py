"""Unified raw-resource replay adapter for the exact V2-04G-P1-E1 matrix."""

from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import stat
from typing import Any, Dict, Mapping, Sequence, Tuple

import yaml

from thesis_experiment import v2_04g_r6_i1_p1_statistics as statistics


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P1-E1"
RAW_LABELS = frozenset({
    "activation", "evaluation", "trace", "clearance",
    "overlay", "teardown_receipt", "process_log",
})
TRACE_COLUMNS = (
    "stamp_s", "x_m", "y_m", "yaw_rad", "linear_velocity_mps",
    "angular_velocity_radps", "commanded_speed_mps", "clearance_m",
    "goal_distance_m", "collision", "goal_reached", "contact_count",
    "topology_id", "global_replan_count", "recovery_count", "gear",
    "predicted_ttc_s",
)
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")
RAW_OVERLAYS = frozenset({
    "NONE", "CROSSING", "HEAD_ON", "FOLLOW", "OVERTAKE_OR_YIELD",
})
MAPPED_OVERLAYS = frozenset({
    "NONE", "LEGACY_DYNAMIC", "CIRCLE_CONTACT_DYNAMIC", "CAUTION", "EMERGENCY",
})


class P1ReplayError(ValueError):
    """Raised when raw evidence cannot deterministically support one row."""


class _UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise P1ReplayError("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P1ReplayError(message)


def _safe_relative(value: Any) -> Path:
    _require(type(value) is str, "resource path must be text")
    path = Path(value)
    _require(
        not path.is_absolute() and bool(path.parts)
        and all(part not in {"", ".", ".."} for part in path.parts),
        "unsafe workspace-relative path",
    )
    return path


def _read_once_no_follow(workspace: Path, relative: Path) -> bytes:
    root = workspace.resolve(strict=True)
    parts = _safe_relative(Path(relative).as_posix()).parts
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags = flags | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptors = []
    try:
        descriptor = os.open(str(root), directory_flags)
        descriptors.append(descriptor)
        for component in parts[:-1]:
            descriptor = os.open(component, directory_flags, dir_fd=descriptor)
            descriptors.append(descriptor)
        file_descriptor = os.open(
            parts[-1], flags | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=descriptors[-1],
        )
        try:
            before = os.fstat(file_descriptor)
            _require(stat.S_ISREG(before.st_mode), "raw resource is not regular")
            _require(before.st_nlink == 1, "hard-linked raw resource forbidden")
            chunks = []
            while True:
                chunk = os.read(file_descriptor, 1024 * 1024)
                if not chunk:
                    break
                chunks.append(chunk)
            after = os.fstat(file_descriptor)
            fields = ("st_dev", "st_ino", "st_mode", "st_nlink", "st_size", "st_mtime_ns", "st_ctime_ns")
            _require(
                all(getattr(before, key) == getattr(after, key) for key in fields),
                "raw resource changed during read",
            )
            payload = b"".join(chunks)
            _require(len(payload) == before.st_size, "short raw-resource read")
            return payload
        finally:
            os.close(file_descriptor)
    except OSError as exc:
        raise P1ReplayError("cannot read {}: {}".format(relative, exc)) from exc
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def _yaml(payload: bytes, label: str) -> Mapping[str, Any]:
    try:
        value = yaml.load(payload.decode("utf-8"), Loader=_UniqueLoader)
    except (UnicodeError, yaml.YAMLError) as exc:
        raise P1ReplayError("cannot parse {}: {}".format(label, exc)) from exc
    _require(type(value) is dict, "{} must be a mapping".format(label))
    return value


def _finite(value: Any, label: str) -> float:
    _require(type(value) in (int, float), "{} must be numeric".format(label))
    result = float(value)
    _require(math.isfinite(result), "{} must be finite".format(label))
    return result


def _float_text(value: str, label: str, *, optional=False):
    if optional and value.strip() == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise P1ReplayError("{} is not numeric".format(label)) from exc
    _require(math.isfinite(result), "{} must be finite".format(label))
    return result


def _bool_text(value: str, label: str) -> bool:
    normalized = value.strip().lower()
    _require(normalized in {"true", "false", "1", "0"}, "{} is not boolean".format(label))
    return normalized in {"true", "1"}


def _trace(payload: bytes) -> Sequence[Dict[str, Any]]:
    try:
        reader = csv.DictReader(io.StringIO(payload.decode("utf-8"), newline=""))
        _require(tuple(reader.fieldnames or ()) == TRACE_COLUMNS, "trace columns/order differ")
        raw_rows = list(reader)
    except UnicodeError as exc:
        raise P1ReplayError("trace is not UTF-8") from exc
    _require(len(raw_rows) >= 2, "trace has fewer than two rows")
    rows = []
    previous = None
    for index, raw in enumerate(raw_rows):
        label = "trace[{}]".format(index)
        row = {
            key: _float_text(raw[key], label + "." + key)
            for key in (
                "stamp_s", "x_m", "y_m", "yaw_rad", "linear_velocity_mps",
                "angular_velocity_radps", "commanded_speed_mps", "clearance_m",
                "goal_distance_m",
            )
        }
        row["predicted_ttc_s"] = _float_text(
            raw["predicted_ttc_s"], label + ".predicted_ttc_s", optional=True
        )
        row["collision"] = _bool_text(raw["collision"], label + ".collision")
        row["goal_reached"] = _bool_text(raw["goal_reached"], label + ".goal_reached")
        for key in ("contact_count", "global_replan_count", "recovery_count"):
            try:
                row[key] = int(raw[key])
            except (TypeError, ValueError) as exc:
                raise P1ReplayError("{} is not integer".format(label + "." + key)) from exc
            _require(row[key] >= 0, "trace count is negative")
        row["topology_id"] = raw["topology_id"]
        row["gear"] = raw["gear"]
        _require(row["gear"] in {"REVERSE", "NEUTRAL", "FORWARD"}, "trace gear differs")
        if previous is not None:
            _require(row["stamp_s"] > previous, "trace time is not strictly increasing")
        previous = row["stamp_s"]
        rows.append(row)
    return rows


def _expected_mapped(method_id: str, raw_state: str) -> str:
    if raw_state == "NONE":
        return "NONE"
    if method_id == statistics.CONTROL:
        return "LEGACY_DYNAMIC"
    if method_id == statistics.TREATMENT:
        return "CIRCLE_CONTACT_DYNAMIC"
    raise P1ReplayError("fixed-TEB raw overlay is not NONE")


def _overlay_intervals(document: Mapping[str, Any], planned: Mapping[str, Any]):
    for key in ("stage", "method_id", "scene_id", "seed", "attempt"):
        expected = STAGE if key == "stage" else planned[key]
        _require(document.get(key) == expected, "overlay identity differs: {}".format(key))
    _require(document.get("behavior_read_only") is True, "overlay instrumentation is not read-only")
    _require(document.get("runtime_policy_received_trace") is False, "overlay trace leaked to policy")
    duration = _finite(document.get("observation_duration_s"), "overlay duration")
    _require(duration > 0.0, "overlay duration must be positive")
    start = _finite(document.get("goal_dispatch_stamp_s"), "goal dispatch stamp")
    end = _finite(document.get("terminal_event_stamp_s"), "terminal event stamp")
    _require(abs((end - start) - duration) <= 1.0e-9, "overlay window duration differs")
    events = document.get("events")
    _require(type(events) is list and events, "overlay event trace is empty")
    normalized = []
    previous_offset = None
    for index, event in enumerate(events):
        _require(type(event) is dict, "overlay event must be a mapping")
        raw_state = event.get("raw_state")
        mapped = event.get("mapped_state")
        _require(raw_state in RAW_OVERLAYS, "unknown raw overlay state")
        _require(mapped in MAPPED_OVERLAYS, "unknown mapped overlay state")
        _require(mapped == _expected_mapped(planned["method_id"], raw_state), "overlay mapping differs")
        offset = _finite(event.get("offset_s"), "overlay offset")
        stamp = _finite(event.get("stamp_s"), "overlay stamp")
        _require(-1.0e-9 <= offset <= duration + 1.0e-9, "overlay event outside window")
        _require(abs((stamp - start) - offset) <= 1.0e-8, "overlay stamp/offset differs")
        if previous_offset is not None:
            _require(offset >= previous_offset - 1.0e-9, "overlay offsets are nonmonotonic")
        if normalized and mapped == normalized[-1][1]:
            continue
        if offset >= duration - 1.0e-9 and normalized:
            continue
        normalized.append((max(0.0, offset), mapped))
        previous_offset = offset
    _require(normalized and abs(normalized[0][0]) <= 1.0e-9, "overlay trace does not start at dispatch")
    intervals = []
    for index, (offset, state) in enumerate(normalized):
        next_offset = normalized[index + 1][0] if index + 1 < len(normalized) else duration
        _require(next_offset > offset + 1.0e-12, "overlay interval is empty or overlaps")
        intervals.append({"start_s": offset, "end_s": next_offset, "state": state})
    _require(abs(intervals[-1]["end_s"] - duration) <= 1.0e-9, "overlay intervals do not tile window")
    return duration, intervals


def _resource_payloads(
    workspace: Path, journal: Mapping[str, Any]
) -> Tuple[Dict[str, bytes], Dict[str, str]]:
    resources = journal.get("raw_resources")
    _require(type(resources) is dict and set(resources) == RAW_LABELS, "raw resource roster differs")
    payloads = {}
    digests = {}
    for label, receipt in resources.items():
        _require(
            type(receipt) is dict and set(receipt) == {"path", "sha256", "size_bytes"},
            "raw receipt schema differs",
        )
        _require(HEX_SHA256.fullmatch(str(receipt["sha256"])) is not None, "raw receipt SHA differs")
        payload = _read_once_no_follow(workspace, _safe_relative(receipt["path"]))
        digest = hashlib.sha256(payload).hexdigest()
        _require(digest == receipt["sha256"], "raw resource hash differs: {}".format(label))
        _require(len(payload) == receipt["size_bytes"], "raw resource size differs: {}".format(label))
        payloads[label] = payload
        digests[label] = digest
    return payloads, digests


def replay_one(
    workspace: Path,
    planned: Mapping[str, Any],
    journal_relative: Path,
    release_sha256: str,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    journal_payload = _read_once_no_follow(workspace, journal_relative)
    journal = _yaml(journal_payload, "attempt journal")
    required = {
        "schema_version", "stage", "release_sha256", "identity", "status",
        "consumed", "consumption_boundary", "raw_resources", "integrity_failures",
        "process_exit_codes", "terminal_recorded_at_utc",
    }
    _require(set(journal) == required, "attempt journal schema differs")
    _require(journal["stage"] == STAGE, "attempt journal stage differs")
    _require(journal["release_sha256"] == release_sha256, "attempt journal release differs")
    expected_identity = {
        key: planned[key]
        for key in ("sequence", "method_id", "scene_id", "seed", "attempt", "block_id", "scene_role")
    }
    _require(journal["identity"] == expected_identity, "attempt journal identity differs")
    _require(journal["status"] == "completed", "attempt journal is not completed")
    _require(journal["consumed"] is True, "scheduled identity was not consumed")
    _require(journal["consumption_boundary"] == "base_roslaunch_spawn_requested", "consumption boundary differs")
    _require(journal["integrity_failures"] == [], "journal contains integrity failures")
    exits = journal["process_exit_codes"]
    _require(type(exits) is dict and exits.get("episode") == 0, "episode process did not exit cleanly")
    payloads, digests = _resource_payloads(workspace, journal)

    evaluation = _yaml(payloads["evaluation"], "evaluation")
    clearance = _yaml(payloads["clearance"], "clearance")
    overlay = _yaml(payloads["overlay"], "overlay trace")
    activation = _yaml(payloads["activation"], "activation")
    teardown = _yaml(payloads["teardown_receipt"], "teardown receipt")
    rows = _trace(payloads["trace"])
    _require(evaluation.get("stage") == STAGE, "evaluation stage differs")
    _require(evaluation.get("scene_id") == planned["scene_id"], "evaluation scene differs")
    _require(evaluation.get("seed") == planned["seed"], "evaluation seed differs")
    _require(evaluation.get("attempt") == 1, "evaluation attempt differs")
    _require(evaluation.get("p1_method_id") == planned["method_id"], "evaluation method differs")
    _require(clearance.get("stage") == STAGE, "clearance stage differs")

    if planned["method_id"] in {statistics.CONTROL, statistics.TREATMENT}:
        _require(activation.get("all_hard_gates_pass") is True, "semantic activation gate failed")
        _require(evaluation.get("typed_transaction_valid") is True, "typed transaction invalid")
        _require(evaluation.get("training_used") is False, "training used during P1")
        _require(teardown.get("service_response_success") is True, "semantic teardown failed")
    else:
        _require(activation.get("record_type") == "fixed_teb_structural_activation", "fixed activation differs")
        _require(activation.get("supervisor_started") is False, "fixed supervisor unexpectedly started")
        _require(activation.get("typed_transaction_started") is False, "fixed transaction unexpectedly started")
        _require(activation.get("dynamic_obstacle_bridge_started") is False, "fixed obstacle bridge unexpectedly started")
        _require(teardown.get("no_write_boundary_unchanged") is True, "fixed TEB changed")
        _require(evaluation.get("typed_transaction_expected") is False, "fixed evaluator expected transaction")

    collision = any(row["collision"] or row["contact_count"] > 0 for row in rows)
    stopped = abs(rows[-1]["linear_velocity_mps"]) <= 0.05
    success = bool(rows[-1]["goal_reached"] and stopped and not collision)
    navigation_time = rows[-1]["stamp_s"] - rows[0]["stamp_s"]
    _require(navigation_time > 0.0 and navigation_time <= 80.5, "replayed navigation time differs")
    minimum_clearance = min(row["clearance_m"] for row in rows)
    finite_ttc = [row["predicted_ttc_s"] for row in rows if row["predicted_ttc_s"] is not None]
    observed_ttc = "OBSERVED_CONFLICT" if finite_ttc else "NO_CONFLICT_IN_HORIZON"
    duration, intervals = _overlay_intervals(overlay, planned)
    algorithmic_emergency = any(item["state"] == "EMERGENCY" for item in intervals)
    runner_fault = str(evaluation.get("runner_fault_reason", ""))
    if success:
        terminal_status = "completed"
    elif collision:
        terminal_status = "collision"
    elif runner_fault == "timeout" or navigation_time >= 79.0:
        terminal_status = "timeout"
    else:
        terminal_status = "planner_failure"
    normalized = {
        "sequence": planned["sequence"],
        "method_id": planned["method_id"],
        "scene_id": planned["scene_id"],
        "seed": planned["seed"],
        "attempt": planned["attempt"],
        "block_id": planned["block_id"],
        "scene_role": planned["scene_role"],
        "consumed": True,
        "terminal_status": terminal_status,
        "success": success,
        "collision": collision,
        "navigation_time_s": navigation_time,
        "timeout_s": 80.0,
        "minimum_footprint_clearance_m": minimum_clearance,
        "observed_ttc_status": observed_ttc,
        "algorithmic_emergency": algorithmic_emergency,
        "interface_failure": False,
        "observation_duration_s": duration,
        "overlay_intervals": intervals,
        "raw_resources_complete": True,
        "integrity_failures": [],
    }
    audit = {
        "sequence": planned["sequence"],
        "journal_path": journal_relative.as_posix(),
        "journal_sha256": hashlib.sha256(journal_payload).hexdigest(),
        "raw_sha256": digests,
        "trace_rows": len(rows),
        "finite_ttc_rows": len(finite_ttc),
        "terminal_status": terminal_status,
        "normalized_sha256": statistics.canonical_sha256(normalized),
    }
    return normalized, audit


def replay_matrix(
    schedule: Mapping[str, Any],
    journal_root: Path,
    release_sha256: str,
    workspace: Path = WORKSPACE,
) -> Tuple[Sequence[Dict[str, Any]], Dict[str, Any]]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    _require(HEX_SHA256.fullmatch(release_sha256) is not None, "release SHA256 differs")
    planned_rows = statistics.validate_schedule(schedule)
    normalized = []
    audits = []
    for planned in planned_rows:
        journal_relative = Path(journal_root) / "sequence_{:04d}.yaml".format(planned["sequence"])
        row, audit = replay_one(workspace, planned, journal_relative, release_sha256)
        normalized.append(row)
        audits.append(audit)
    return normalized, {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "raw_replay_complete",
        "release_sha256": release_sha256,
        "episode_count": len(normalized),
        "raw_resources_rehashed": len(normalized) * len(RAW_LABELS),
        "normalized_matrix_sha256": statistics.canonical_sha256(normalized),
        "episode_audits": audits,
    }


def build_terminal_assessment(
    schedule: Mapping[str, Any],
    journal_root: Path,
    release_sha256: str,
    workspace: Path = WORKSPACE,
) -> Tuple[Sequence[Dict[str, Any]], Dict[str, Any], Dict[str, Any]]:
    normalized, replay = replay_matrix(schedule, journal_root, release_sha256, workspace)
    assessment = statistics.build_assessment(schedule, list(normalized))
    assessment["execution_stage"] = STAGE
    assessment["release_sha256"] = release_sha256
    assessment["raw_replay_report_sha256"] = statistics.canonical_sha256(replay)
    return normalized, replay, assessment
