"""Deterministic, pure-offline P2-D1 trace attribution.

This module never imports ROS.  It verifies the frozen P1 bindings, retains all
30 semantic-clear paired blocks, and measures whether an 8--10 percent aggregate
target lies inside a same-path longitudinal kinematic envelope.  The envelope is
a design-screening bound, not an achieved or causal treatment effect.
"""

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
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import yaml


STAGE = "V2-04G-P2-D1"
CONTROL = "r6_semantics_legacy_control"
TREATMENT = "r6_semantics_circle_contact"
CLEAR_ROLE = "time_separated_semantic_clear"
SEEDS = tuple(range(5261, 5291))
RELEASE_SHA256 = "05a6efe21a4c110be667b0873d8ac1b26ac692feefea8932f40a506ca9b9027b"
CURRENT_MAX_VEL_X_MPS = 1.2
CURRENT_ACC_LIM_X_MPS2 = 0.8
CAP_BOUND_THRESHOLD_MPS = 1.14
MOTION_COMMAND_THRESHOLD_MPS = 0.05
BOOTSTRAP_RESAMPLES = 10000
BOOTSTRAP_SEED = 20260722
ALPHA = 0.05
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")
TRACE_FIELDS = (
    "stamp_s", "x_m", "y_m", "yaw_rad", "linear_velocity_mps",
    "angular_velocity_radps", "commanded_speed_mps", "clearance_m",
    "goal_distance_m", "collision", "goal_reached", "contact_count",
    "topology_id", "global_replan_count", "recovery_count", "gear",
    "predicted_ttc_s",
)
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_d1_offline_trace_attribution_contract.yaml"
)
MATRIX_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/"
    "normalized_terminal_matrix.json"
)
JOURNAL_DIRECTORY = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/journals"
)
BLOCKS_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_d1_trace_attribution/"
    "v2_04g_p2_d1_trace_attribution_blocks.json"
)
REPORT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_d1_trace_attribution/"
    "v2_04g_p2_d1_trace_attribution_report.yaml"
)


class AttributionError(ValueError):
    """Raised when P2-D1 input integrity or a frozen invariant fails."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AttributionError(message)


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def safe_relative(value: str | Path) -> Path:
    relative = value if isinstance(value, Path) else Path(value)
    require(not relative.is_absolute(), "absolute path is forbidden")
    require(".." not in relative.parts and relative.parts, "unsafe relative path")
    return relative


def stable_read(workspace: Path, relative: str | Path) -> bytes:
    relative = safe_relative(relative)
    current = workspace
    for component in relative.parts:
        current = current / component
        require(not current.is_symlink(), "symbolic link forbidden: {}".format(relative))
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(str(workspace / relative), flags)
    try:
        before = os.fstat(descriptor)
        require(stat.S_ISREG(before.st_mode), "not a regular file: {}".format(relative))
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
        require(
            (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns)
            == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns),
            "file changed during read: {}".format(relative),
        )
        payload = b"".join(chunks)
        require(len(payload) == before.st_size, "short read: {}".format(relative))
        return payload
    finally:
        os.close(descriptor)


def load_yaml(payload: bytes, label: str) -> Dict[str, Any]:
    try:
        value = yaml.safe_load(payload.decode("utf-8"))
    except Exception as exc:
        raise AttributionError("{} YAML parse failed: {}".format(label, exc))
    require(type(value) is dict, "{} is not a mapping".format(label))
    return value


def load_json(payload: bytes, label: str) -> Any:
    try:
        return json.loads(payload.decode("utf-8"))
    except Exception as exc:
        raise AttributionError("{} JSON parse failed: {}".format(label, exc))


def finite(value: Any, label: str, *, nonnegative: bool = False) -> float:
    require(type(value) in (int, float), "{} is not numeric".format(label))
    result = float(value)
    require(math.isfinite(result), "{} is not finite".format(label))
    if nonnegative:
        require(result >= 0.0, "{} is negative".format(label))
    return result


def close(left: float, right: float, tolerance: float = 1e-9) -> bool:
    return abs(left - right) <= tolerance


def verify_binding(workspace: Path, binding: Mapping[str, Any], label: str) -> Tuple[Path, bytes]:
    require(type(binding) is dict and set(binding) == {"path", "sha256"}, "{} binding differs".format(label))
    require(type(binding["path"]) is str, "{} path differs".format(label))
    require(type(binding["sha256"]) is str and HEX_SHA256.fullmatch(binding["sha256"]), "{} SHA differs".format(label))
    relative = safe_relative(binding["path"])
    payload = stable_read(workspace, relative)
    require(sha256_bytes(payload) == binding["sha256"], "{} hash differs".format(label))
    return relative, payload


def verify_sha_manifest(workspace: Path, payload: bytes, expected_count: int) -> List[Dict[str, str]]:
    entries = []
    for line in payload.decode("utf-8").splitlines():
        if not line.strip():
            continue
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        require(match is not None, "terminal evidence manifest line differs")
        expected_sha, path_text = match.groups()
        relative = safe_relative(path_text)
        require(sha256_bytes(stable_read(workspace, relative)) == expected_sha, "terminal target differs: {}".format(relative))
        entries.append({"path": relative.as_posix(), "sha256": expected_sha})
    require(len(entries) == expected_count, "terminal evidence manifest count differs")
    require(len({entry["path"] for entry in entries}) == expected_count, "duplicate terminal evidence path")
    return entries


def validate_contract(contract: Mapping[str, Any]) -> None:
    required = {
        "schema_version", "architecture_generation", "stage", "status",
        "offline_only", "formal_result", "runtime_ready", "execution_ready",
        "execution_authorized", "evidence_budget_authorized",
        "training_budget_steps", "real_vehicle_use_forbidden",
        "scientific_question", "source_evidence", "trace_selection",
        "analysis_definition", "interpretation_boundary", "forbidden_side_effects",
    }
    require(set(contract) == required, "contract top-level schema differs")
    require(contract["schema_version"] == "1.0", "contract schema differs")
    require(contract["architecture_generation"] == "v2", "architecture differs")
    require(contract["stage"] == STAGE, "contract stage differs")
    require(contract["status"] == "offline_trace_attribution_contract_frozen", "contract status differs")
    require(contract["offline_only"] is True, "contract is not offline-only")
    for key in ("formal_result", "runtime_ready", "execution_ready", "execution_authorized"):
        require(contract[key] is False, "{} must be false".format(key))
    require(contract["evidence_budget_authorized"] == 0, "evidence budget is not zero")
    require(contract["training_budget_steps"] == 0, "training budget is not zero")
    require(contract["real_vehicle_use_forbidden"] is True, "real vehicle is not forbidden")
    question = contract["scientific_question"]
    require(question["factor_hypothesis_id"] == "semantic_clear_post_cruise_longitudinal_release", "factor hypothesis differs")
    require(question["factor_scope"]["executable_runtime_field_selected"] is False, "runtime factor was prematurely selected")
    selection = contract["trace_selection"]
    require(selection["scene_role"] == CLEAR_ROLE, "trace role differs")
    require(selection["seeds"] == {"start": 5261, "end": 5290, "count": 30}, "seed selection differs")
    require(selection["paired_methods"] == {"control": CONTROL, "treatment": TREATMENT}, "paired methods differ")
    require(selection["treatment_trace_count"] == 30, "trace count differs")
    require(selection["retries_replacements_or_exclusions_allowed"] is False, "sample exclusion was enabled")
    definition = contract["analysis_definition"]
    require(definition["current_longitudinal_constraints"] == {
        "max_vel_x_mps": CURRENT_MAX_VEL_X_MPS,
        "acc_lim_x_mps2": CURRENT_ACC_LIM_X_MPS2,
        "terminal_velocity_mps": 0.0,
    }, "longitudinal constraints differ")
    require(definition["cap_bound_observation_threshold"] == {
        "fraction_of_current_max_vel_x": 0.95,
        "actual_speed_threshold_mps": CAP_BOUND_THRESHOLD_MPS,
    }, "cap threshold differs")
    require(definition["motion_command_threshold_mps"] == MOTION_COMMAND_THRESHOLD_MPS, "motion threshold differs")
    require(definition["phase_distance_fractions"] == [0.1, 0.9], "phase boundaries differ")
    require(definition["uncertainty"] == {
        "method": "paired_seed_bootstrap_on_mean_log_ratio",
        "resamples": BOOTSTRAP_RESAMPLES,
        "seed": BOOTSTRAP_SEED,
        "deterministic_rng": "sha256_counter_p2_d1_v1",
        "one_sided_alpha": ALPHA,
    }, "uncertainty definition differs")
    require(definition["target_reductions"] == [0.08, 0.10], "target reductions differ")
    gate = definition["physical_room_gate"]
    require(gate == {
        "envelope_point_reduction_at_least": 0.10,
        "one_sided_95_percent_lower_at_least": 0.10,
        "all_30_blocks_retained": True,
        "raw_trace_and_evaluation_hashes_match_journals": True,
    }, "physical-room gate differs")
    require(all(value is True for value in contract["forbidden_side_effects"].values()), "a forbidden side effect was enabled")


def geometric_mean(values: Sequence[float]) -> float:
    require(bool(values), "geometric mean is empty")
    require(all(value > 0.0 and math.isfinite(value) for value in values), "geometric mean input differs")
    return math.exp(sum(math.log(value) for value in values) / len(values))


def minimum_time(distance_m: float, start_velocity_mps: float, end_velocity_mps: float, max_velocity_mps: float, acceleration_mps2: float) -> float:
    """Bang-bang same-path time with symmetric acceleration magnitude."""

    for value, label in (
        (distance_m, "distance"), (start_velocity_mps, "start velocity"),
        (end_velocity_mps, "end velocity"), (max_velocity_mps, "max velocity"),
        (acceleration_mps2, "acceleration"),
    ):
        require(math.isfinite(value), "{} is not finite".format(label))
    require(distance_m >= 0.0, "distance is negative")
    require(0.0 <= start_velocity_mps <= max_velocity_mps, "start velocity outside envelope")
    require(0.0 <= end_velocity_mps <= max_velocity_mps, "end velocity outside envelope")
    require(max_velocity_mps > 0.0 and acceleration_mps2 > 0.0, "invalid longitudinal limit")
    accelerate_distance = (max_velocity_mps ** 2 - start_velocity_mps ** 2) / (2.0 * acceleration_mps2)
    decelerate_distance = (max_velocity_mps ** 2 - end_velocity_mps ** 2) / (2.0 * acceleration_mps2)
    if distance_m >= accelerate_distance + decelerate_distance:
        return (
            (max_velocity_mps - start_velocity_mps) / acceleration_mps2
            + (max_velocity_mps - end_velocity_mps) / acceleration_mps2
            + (distance_m - accelerate_distance - decelerate_distance) / max_velocity_mps
        )
    peak_squared = acceleration_mps2 * distance_m + (
        start_velocity_mps ** 2 + end_velocity_mps ** 2
    ) / 2.0
    peak_velocity = math.sqrt(max(peak_squared, 0.0))
    require(peak_velocity + 1e-12 >= max(start_velocity_mps, end_velocity_mps), "distance cannot connect endpoint velocities")
    return (
        (peak_velocity - start_velocity_mps) / acceleration_mps2
        + (peak_velocity - end_velocity_mps) / acceleration_mps2
    )


def time_at_distance(distance_m: float, total_distance_m: float, max_velocity_mps: float, acceleration_mps2: float) -> float:
    """Time at a path coordinate on a rest-to-rest bang-bang trajectory."""

    require(0.0 <= distance_m <= total_distance_m, "path coordinate differs")
    acceleration_distance = max_velocity_mps ** 2 / (2.0 * acceleration_mps2)
    total_time = minimum_time(total_distance_m, 0.0, 0.0, max_velocity_mps, acceleration_mps2)
    if total_distance_m < 2.0 * acceleration_distance:
        peak_velocity = math.sqrt(acceleration_mps2 * total_distance_m)
        acceleration_distance = peak_velocity ** 2 / (2.0 * acceleration_mps2)
        if distance_m <= acceleration_distance:
            return math.sqrt(2.0 * distance_m / acceleration_mps2)
        return total_time - math.sqrt(2.0 * (total_distance_m - distance_m) / acceleration_mps2)
    if distance_m <= acceleration_distance:
        return math.sqrt(2.0 * distance_m / acceleration_mps2)
    if distance_m >= total_distance_m - acceleration_distance:
        return total_time - math.sqrt(2.0 * (total_distance_m - distance_m) / acceleration_mps2)
    return max_velocity_mps / acceleration_mps2 + (
        distance_m - acceleration_distance
    ) / max_velocity_mps


def _counter_index(namespace: str, replicate: int, draw: int, size: int) -> int:
    payload = "p2-d1:{}:{}:{}:{}".format(
        BOOTSTRAP_SEED, namespace, replicate, draw
    ).encode("utf-8")
    return int.from_bytes(hashlib.sha256(payload).digest(), "big") % size


def bootstrap_lower(log_ratios: Sequence[float], namespace: str) -> float:
    effects = []
    for replicate in range(BOOTSTRAP_RESAMPLES):
        mean_log = sum(
            log_ratios[_counter_index(namespace, replicate, draw, len(log_ratios))]
            for draw in range(len(log_ratios))
        ) / len(log_ratios)
        effects.append(1.0 - math.exp(-mean_log))
    effects.sort()
    index = int(math.ceil(ALPHA * len(effects))) - 1
    return effects[max(0, min(index, len(effects) - 1))]


def _path_time_at_fraction(stamps: Sequence[float], distances: Sequence[float], fraction: float) -> float:
    total = sum(distances)
    target = fraction * total
    cumulative = 0.0
    for index, distance in enumerate(distances):
        next_cumulative = cumulative + distance
        if next_cumulative + 1e-15 >= target:
            interpolation = 0.0 if distance == 0.0 else (target - cumulative) / distance
            return stamps[index] + interpolation * (stamps[index + 1] - stamps[index]) - stamps[0]
        cumulative = next_cumulative
    return stamps[-1] - stamps[0]


def _mean(rows: Sequence[Mapping[str, Any]], key: str) -> float:
    return sum(float(row[key]) for row in rows) / len(rows)


def _solve_recovery_fraction(rows: Sequence[Mapping[str, Any]], legacy_geometric_s: float, target: float) -> float:
    low, high = 0.0, 1.0
    for _ in range(100):
        middle = (low + high) / 2.0
        candidates = [
            row["treatment_navigation_time_s"]
            - middle * row["post_cruise_recoverable_time_s"]
            for row in rows
        ]
        reduction = 1.0 - geometric_mean(candidates) / legacy_geometric_s
        if reduction < target:
            low = middle
        else:
            high = middle
    return high


def _solve_cap_multiplier(rows: Sequence[Mapping[str, Any]], legacy_geometric_s: float, target: float) -> float:
    low, high = 1.0, 2.0
    for _ in range(100):
        middle = (low + high) / 2.0
        candidates = [
            row["treatment_navigation_time_s"]
            - row["actual_cap_bound_dwell_s"] * (1.0 - 1.0 / middle)
            for row in rows
        ]
        reduction = 1.0 - geometric_mean(candidates) / legacy_geometric_s
        if reduction < target:
            low = middle
        else:
            high = middle
    return high


def _candidate(plan: Mapping[str, Any], candidate_id: str) -> Mapping[str, Any]:
    matches = [item for item in plan["candidates"] if item.get("candidate_id") == candidate_id]
    require(len(matches) == 1, "historical candidate differs: {}".format(candidate_id))
    return matches[0]


def _evaluation_metrics(document: Mapping[str, Any], candidate_id: str) -> Dict[str, float]:
    require(document.get("candidate_id") == candidate_id, "historical evaluation candidate differs")
    require(document.get("termination") == "SUCCESS", "historical evaluation did not succeed")
    common = document.get("metrics", {}).get("common", {})
    return {
        "navigation_time_s": finite(common.get("navigation_time_s"), "historical navigation time"),
        "path_length_m": finite(common.get("path_length_m"), "historical path length"),
        "minimum_clearance_m": finite(common.get("minimum_clearance_m"), "historical clearance"),
    }


def _historical_check(source_documents: Mapping[str, Mapping[str, Any]]) -> Dict[str, Any]:
    plan = source_documents["historical_screen_plan"]
    identifiers = {
        "speed_low": "anchor_cruise-c01-max_vel_x_low",
        "speed_high": "anchor_cruise-c02-max_vel_x_high",
        "center": "anchor_cruise-c00-center",
        "weight_low": "anchor_cruise-c07-weight_optimaltime_low",
        "weight_high": "anchor_cruise-c08-weight_optimaltime_high",
    }
    evaluations = {
        "speed_low": source_documents["historical_speed_low_evaluation"],
        "speed_high": source_documents["historical_speed_high_evaluation"],
        "center": source_documents["historical_cruise_center_evaluation"],
        "weight_low": source_documents["historical_weight_low_evaluation"],
        "weight_high": source_documents["historical_weight_high_evaluation"],
    }
    rows = {}
    for label, candidate_id in identifiers.items():
        candidate = _candidate(plan, candidate_id)
        metrics = _evaluation_metrics(evaluations[label], candidate_id)
        rows[label] = {
            "candidate_id": candidate_id,
            "max_vel_x_mps": finite(candidate["values"]["max_vel_x"], "historical max_vel_x"),
            "weight_optimaltime": finite(candidate["values"]["weight_optimaltime"], "historical weight_optimaltime"),
            **metrics,
        }
    speed_effect = 1.0 - rows["center"]["navigation_time_s"] / rows["speed_high"]["navigation_time_s"]
    weight_effect = 1.0 - rows["weight_high"]["navigation_time_s"] / rows["weight_low"]["navigation_time_s"]
    return {
        "status": "descriptive_single_seed_direction_check_only",
        "scene_id": "v2-cal-cruise-straight-35m-s4301",
        "seed": 4301,
        "rows": rows,
        "speed_ceiling_high_1_09_to_center_1_20_time_reduction_fraction": speed_effect,
        "speed_ceiling_path_change_m": rows["center"]["path_length_m"] - rows["speed_high"]["path_length_m"],
        "speed_ceiling_clearance_change_m": rows["center"]["minimum_clearance_m"] - rows["speed_high"]["minimum_clearance_m"],
        "weight_optimaltime_1_03_to_1_37_time_reduction_fraction": weight_effect,
        "interpretation": {
            "speed_ceiling_has_target_scale_directional_sensitivity": speed_effect >= 0.08,
            "weight_optimaltime_alone_has_target_scale_evidence": weight_effect >= 0.08,
            "new_above_1_2_speed_bound_qualified": False,
            "causal_generalization_authorized": False,
        },
    }


def _parse_trace(payload: bytes, label: str) -> List[Dict[str, str]]:
    text = payload.decode("utf-8")
    reader = csv.DictReader(io.StringIO(text))
    require(tuple(reader.fieldnames or ()) == TRACE_FIELDS, "{} header differs".format(label))
    rows = list(reader)
    require(len(rows) >= 2, "{} has too few rows".format(label))
    return rows


def _trace_block(
    workspace: Path,
    treatment: Mapping[str, Any],
    control: Mapping[str, Any],
) -> Tuple[Dict[str, Any], List[Dict[str, str]]]:
    sequence = treatment["sequence"]
    journal_relative = JOURNAL_DIRECTORY / "sequence_{:04d}.yaml".format(sequence)
    journal_payload = stable_read(workspace, journal_relative)
    journal = load_yaml(journal_payload, "journal {}".format(sequence))
    identity = journal.get("identity", {})
    for key in ("sequence", "method_id", "scene_id", "seed", "attempt", "block_id", "scene_role"):
        require(identity.get(key) == treatment[key], "journal identity differs: {}".format(key))
    require(journal.get("release_sha256") == RELEASE_SHA256, "journal release differs")
    require(journal.get("status") == "completed" and journal.get("consumed") is True, "journal is not completed")
    require(journal.get("integrity_failures") == [], "journal has integrity failure")
    resources = journal.get("raw_resources", {})
    verified_resources = []
    payloads = {}
    for name in ("trace", "evaluation"):
        binding = resources.get(name)
        require(type(binding) is dict and set(binding) == {"path", "sha256", "size_bytes"}, "journal resource differs")
        relative = safe_relative(binding["path"])
        payload = stable_read(workspace, relative)
        require(len(payload) == binding["size_bytes"], "journal resource size differs")
        require(sha256_bytes(payload) == binding["sha256"], "journal resource hash differs")
        payloads[name] = payload
        verified_resources.append({
            "path": relative.as_posix(),
            "sha256": binding["sha256"],
        })
    evaluation = load_yaml(payloads["evaluation"], "evaluation {}".format(sequence))
    require(evaluation.get("raw_trace_sha256") == resources["trace"]["sha256"], "evaluation trace SHA differs")
    require(evaluation.get("scene_id") == treatment["scene_id"], "evaluation scene differs")
    require(evaluation.get("seed") == treatment["seed"], "evaluation seed differs")
    require(evaluation.get("p1_method_id") == TREATMENT, "evaluation method differs")
    require(evaluation.get("termination") == "SUCCESS", "treatment did not succeed")
    require(evaluation.get("ttc_status") == "NO_CONFLICT_IN_HORIZON", "semantic-clear TTC status differs")
    require(evaluation.get("finite_ttc_sample_count") == 0, "semantic-clear finite TTC differs")
    require(evaluation.get("context_overlay_sample_fractions", {}).get("NONE") == 1.0, "semantic-clear overlay differs")
    require(evaluation.get("active_anchor_sequence") == ["anchor_balanced", "anchor_cruise"], "anchor sequence differs")
    require(evaluation.get("active_anchor_switch_count") == 1, "anchor switch count differs")
    require(evaluation.get("training_used") is False, "training was used")

    raw_rows = _parse_trace(payloads["trace"], "trace {}".format(sequence))
    first_goal_index = next((index for index, row in enumerate(raw_rows) if row["goal_reached"] == "true"), None)
    require(first_goal_index is not None, "trace has no goal-reached sample")
    rows = raw_rows[: first_goal_index + 1]
    stamps = [float(row["stamp_s"]) for row in rows]
    require(all(math.isfinite(value) for value in stamps), "trace stamp is not finite")
    require(all(stamps[index + 1] > stamps[index] for index in range(len(stamps) - 1)), "trace stamps are not increasing")
    require(close(stamps[0], finite(evaluation["goal_dispatch_stamp_s"], "goal dispatch")), "trace start differs")
    require(close(stamps[-1], finite(evaluation["terminal_event_stamp_s"], "terminal event")), "trace terminal differs")
    observation_duration = stamps[-1] - stamps[0]
    require(close(observation_duration, finite(evaluation["observation_duration_s"], "observation duration")), "observation duration differs")
    require(close(observation_duration, finite(treatment["observation_duration_s"], "matrix observation duration")), "matrix observation differs")
    navigation_time = finite(treatment["navigation_time_s"], "treatment navigation time")
    require(close(navigation_time, finite(evaluation["metrics"]["common"]["navigation_time_s"], "evaluation navigation time")), "navigation time differs")
    post_terminal_accounting = navigation_time - observation_duration
    require(post_terminal_accounting >= 0.0, "post-terminal accounting is negative")

    x_values = [float(row["x_m"]) for row in rows]
    y_values = [float(row["y_m"]) for row in rows]
    actual_speeds = [abs(float(row["linear_velocity_mps"])) for row in rows]
    commands = [abs(float(row["commanded_speed_mps"])) for row in rows]
    require(all(math.isfinite(value) for value in x_values + y_values + actual_speeds + commands), "trace numeric value is not finite")
    distances = [
        math.hypot(x_values[index + 1] - x_values[index], y_values[index + 1] - y_values[index])
        for index in range(len(rows) - 1)
    ]
    intervals = [stamps[index + 1] - stamps[index] for index in range(len(rows) - 1)]
    total_path = sum(distances)
    require(total_path > 0.0, "trace path is empty")
    first_command_index = next((index for index, value in enumerate(commands) if value > MOTION_COMMAND_THRESHOLD_MPS), None)
    require(first_command_index is not None, "trace has no movement command")
    dispatch_delay = stamps[first_command_index] - stamps[0]
    first_cruise_index = next((index for index, row in enumerate(rows) if row["topology_id"] == "anchor_cruise"), None)
    require(first_cruise_index is not None and first_cruise_index > 0, "trace has no post-start cruise")
    require(all(row["topology_id"] == "anchor_balanced" for row in rows[:first_cruise_index]), "pre-cruise topology differs")
    require(all(row["topology_id"] == "anchor_cruise" for row in rows[first_cruise_index:]), "post-cruise topology differs")
    cruise_entry_elapsed = stamps[first_cruise_index] - stamps[0]
    cruise_entry_velocity = actual_speeds[first_cruise_index]
    post_cruise_path = sum(distances[first_cruise_index:])
    post_cruise_minimum = minimum_time(
        post_cruise_path,
        cruise_entry_velocity,
        0.0,
        CURRENT_MAX_VEL_X_MPS,
        CURRENT_ACC_LIM_X_MPS2,
    )
    post_cruise_envelope_navigation = (
        cruise_entry_elapsed + post_cruise_minimum + post_terminal_accounting
    )
    whole_minimum = minimum_time(
        total_path, 0.0, 0.0, CURRENT_MAX_VEL_X_MPS, CURRENT_ACC_LIM_X_MPS2
    )
    whole_envelope_navigation = dispatch_delay + whole_minimum + post_terminal_accounting
    require(post_cruise_envelope_navigation <= navigation_time + 1e-12, "post-cruise envelope exceeds observation")

    actual_cap_dwell = sum(
        interval for index, interval in enumerate(intervals)
        if actual_speeds[index] >= CAP_BOUND_THRESHOLD_MPS
    )
    commanded_cap_dwell = sum(
        interval for index, interval in enumerate(intervals)
        if commands[index] >= CAP_BOUND_THRESHOLD_MPS
    )
    cruise_dwell = sum(
        interval for index, interval in enumerate(intervals)
        if rows[index]["topology_id"] == "anchor_cruise"
    )

    time_10 = _path_time_at_fraction(stamps, distances, 0.1)
    time_90 = _path_time_at_fraction(stamps, distances, 0.9)
    ideal_10 = time_at_distance(0.1 * total_path, total_path, CURRENT_MAX_VEL_X_MPS, CURRENT_ACC_LIM_X_MPS2)
    ideal_90 = time_at_distance(0.9 * total_path, total_path, CURRENT_MAX_VEL_X_MPS, CURRENT_ACC_LIM_X_MPS2)
    start_slack = (time_10 - dispatch_delay) - ideal_10
    middle_slack = (time_90 - time_10) - (ideal_90 - ideal_10)
    terminal_slack = (observation_duration - time_90) - (whole_minimum - ideal_90)
    require(close(start_slack + middle_slack + terminal_slack, observation_duration - dispatch_delay - whole_minimum, 1e-7), "phase slack does not close")

    legacy_navigation = finite(control["navigation_time_s"], "legacy navigation time")
    block = {
        "seed": treatment["seed"],
        "block_id": treatment["block_id"],
        "scene_id": treatment["scene_id"],
        "control_sequence": control["sequence"],
        "treatment_sequence": treatment["sequence"],
        "journal_path": journal_relative.as_posix(),
        "journal_sha256": sha256_bytes(journal_payload),
        "trace_path": resources["trace"]["path"],
        "trace_sha256": resources["trace"]["sha256"],
        "evaluation_path": resources["evaluation"]["path"],
        "evaluation_sha256": resources["evaluation"]["sha256"],
        "legacy_navigation_time_s": legacy_navigation,
        "treatment_navigation_time_s": navigation_time,
        "observation_duration_s": observation_duration,
        "post_terminal_accounting_s": post_terminal_accounting,
        "total_path_to_terminal_m": total_path,
        "dispatch_to_first_motion_command_s": dispatch_delay,
        "first_cruise_elapsed_s": cruise_entry_elapsed,
        "first_cruise_velocity_mps": cruise_entry_velocity,
        "post_cruise_path_m": post_cruise_path,
        "cruise_dwell_s": cruise_dwell,
        "actual_cap_bound_dwell_s": actual_cap_dwell,
        "commanded_cap_bound_dwell_s": commanded_cap_dwell,
        "actual_cap_bound_fraction_of_observation": actual_cap_dwell / observation_duration,
        "commanded_cap_bound_fraction_of_observation": commanded_cap_dwell / observation_duration,
        "start_0_to_10_percent_path_slack_s": start_slack,
        "middle_10_to_90_percent_path_slack_s": middle_slack,
        "terminal_90_to_100_percent_path_slack_s": terminal_slack,
        "whole_trace_envelope_navigation_time_s": whole_envelope_navigation,
        "post_cruise_envelope_navigation_time_s": post_cruise_envelope_navigation,
        "post_cruise_recoverable_time_s": navigation_time - post_cruise_envelope_navigation,
        "observed_paired_reduction_fraction": 1.0 - navigation_time / legacy_navigation,
        "whole_trace_envelope_paired_reduction_fraction": 1.0 - whole_envelope_navigation / legacy_navigation,
        "post_cruise_envelope_paired_reduction_fraction": 1.0 - post_cruise_envelope_navigation / legacy_navigation,
    }
    return block, verified_resources


def build_outputs(workspace: Path) -> Tuple[Dict[str, Any], List[Dict[str, Any]], List[Dict[str, str]]]:
    workspace = workspace.resolve()
    contract_payload = stable_read(workspace, CONTRACT_RELATIVE)
    contract = load_yaml(contract_payload, "P2-D1 contract")
    validate_contract(contract)

    source_documents = {}
    verified_source_bindings = []
    for label, binding in contract["source_evidence"].items():
        relative, payload = verify_binding(workspace, binding, label)
        verified_source_bindings.append({"label": label, "path": relative.as_posix(), "sha256": binding["sha256"]})
        if relative.suffix in (".yaml", ".yml"):
            source_documents[label] = load_yaml(payload, label)
        elif relative.suffix == ".json":
            source_documents[label] = load_json(payload, label)
        else:
            source_documents[label] = payload
    terminal_entries = verify_sha_manifest(
        workspace,
        source_documents["p1_terminal_evidence_freeze"],
        10,
    )
    require(source_documents["p1_release"].get("status") == "unique_270_unit_simulation_release", "P1 release status differs")
    replay = source_documents["p1_raw_replay_report"]
    require(replay.get("status") == "raw_replay_complete", "P1 raw replay status differs")
    require(replay.get("release_sha256") == RELEASE_SHA256, "P1 raw replay release differs")
    require(replay.get("episode_count") == 270 and replay.get("raw_resources_rehashed") == 1890, "P1 raw replay count differs")
    assessment = source_documents["p1_four_gate_assessment"]
    require(assessment.get("performance_improvement_demonstrated") is False, "P1 negative conclusion was changed")
    require(assessment.get("four_gate_decision", {}).get("efficiency_superiority") is False, "P1 efficiency conclusion was changed")
    bank = source_documents["treatment_anchor_bank"]
    cruise = bank.get("anchors", {}).get("anchor_cruise", {}).get("values", {})
    require(cruise.get("max_vel_x") == CURRENT_MAX_VEL_X_MPS, "current cruise speed differs")
    require(cruise.get("acc_lim_x") == CURRENT_ACC_LIM_X_MPS2, "current cruise acceleration differs")
    parameter_rows = {item.get("name"): item for item in bank.get("parameters", [])}
    require(parameter_rows.get("max_vel_x", {}).get("bounds") == [0.1, 1.2], "speed bank bound differs")
    require(parameter_rows.get("acc_lim_x", {}).get("bounds") == [0.2, 0.8], "acceleration bank bound differs")

    matrix = source_documents["p1_normalized_matrix"]
    require(type(matrix) is list and len(matrix) == 270, "normalized matrix differs")
    selected = [row for row in matrix if row.get("scene_role") == CLEAR_ROLE]
    require(len(selected) == 90, "semantic-clear matrix count differs")
    by_identity = {(row["seed"], row["method_id"]): row for row in selected}
    require(set(by_identity) == {(seed, method) for seed in SEEDS for method in (CONTROL, TREATMENT, "fixed_teb_reference")}, "semantic-clear identity roster differs")

    blocks = []
    raw_resources = []
    for seed in SEEDS:
        treatment = by_identity[(seed, TREATMENT)]
        control = by_identity[(seed, CONTROL)]
        for row in (treatment, control):
            require(row["terminal_status"] == "completed", "selected row is not completed")
            require(row["success"] is True and row["collision"] is False, "selected row outcome differs")
            require(row["integrity_failures"] == [] and row["raw_resources_complete"] is True, "selected row integrity differs")
        block, verified = _trace_block(workspace, treatment, control)
        blocks.append(block)
        raw_resources.extend(verified)
    require([row["seed"] for row in blocks] == list(SEEDS), "block order differs")
    require(len(raw_resources) == 60 and len({row["path"] for row in raw_resources}) == 60, "raw resource closure differs")

    legacy_geometric = geometric_mean([row["legacy_navigation_time_s"] for row in blocks])
    treatment_geometric = geometric_mean([row["treatment_navigation_time_s"] for row in blocks])
    whole_geometric = geometric_mean([row["whole_trace_envelope_navigation_time_s"] for row in blocks])
    post_cruise_geometric = geometric_mean([row["post_cruise_envelope_navigation_time_s"] for row in blocks])
    observed_reduction = 1.0 - treatment_geometric / legacy_geometric
    whole_reduction = 1.0 - whole_geometric / legacy_geometric
    post_cruise_reduction = 1.0 - post_cruise_geometric / legacy_geometric
    whole_lower = bootstrap_lower(
        [math.log(row["legacy_navigation_time_s"] / row["whole_trace_envelope_navigation_time_s"]) for row in blocks],
        "whole-trace-envelope",
    )
    post_cruise_lower = bootstrap_lower(
        [math.log(row["legacy_navigation_time_s"] / row["post_cruise_envelope_navigation_time_s"]) for row in blocks],
        "post-cruise-envelope",
    )
    target_rows = {}
    for target in (0.08, 0.10):
        target_time = legacy_geometric * (1.0 - target)
        recovery = _solve_recovery_fraction(blocks, legacy_geometric, target)
        multiplier = _solve_cap_multiplier(blocks, legacy_geometric, target)
        target_rows["{}percent".format(int(target * 100))] = {
            "target_reduction_fraction": target,
            "target_treatment_geometric_time_s": target_time,
            "additional_reduction_from_observed_treatment_s": treatment_geometric - target_time,
            "required_fraction_of_post_cruise_recoverable_time": recovery,
            "first_order_actual_cap_dwell_multiplier": multiplier,
            "first_order_implied_max_vel_x_mps": CURRENT_MAX_VEL_X_MPS * multiplier,
            "inside_current_anchor_bank_speed_bound": CURRENT_MAX_VEL_X_MPS * multiplier <= 1.2 + 1e-12,
        }

    historical = _historical_check(source_documents)
    physical_room_pass = post_cruise_reduction >= 0.10 and post_cruise_lower >= 0.10
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "report_id": "fam_teb_v2_04g_p2_d1_offline_trace_attribution_1",
        "status": "offline_trace_attribution_physical_room_pass" if physical_room_pass else "offline_trace_attribution_physical_room_not_demonstrated",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "training_budget_steps": 0,
        "real_vehicle_used": False,
        "contract": {
            "path": CONTRACT_RELATIVE.as_posix(),
            "sha256": sha256_bytes(contract_payload),
        },
        "source_integrity": {
            "contract_source_bindings_verified": len(verified_source_bindings),
            "p1_terminal_manifest_targets_verified": len(terminal_entries),
            "selected_paired_blocks": len(blocks),
            "selected_treatment_journals_verified": len(blocks),
            "selected_raw_trace_and_evaluation_resources_verified": len(raw_resources),
            "missing_replaced_retried_or_excluded_blocks": 0,
            "integrity_failures": [],
        },
        "frozen_p1_result": {
            "legacy_geometric_restricted_time_s": legacy_geometric,
            "treatment_geometric_restricted_time_s": treatment_geometric,
            "observed_reduction_fraction": observed_reduction,
            "efficiency_gate_pass": False,
            "performance_improvement_demonstrated": False,
            "p1_conclusion_changed_by_p2_d1": False,
        },
        "observed_trace_attribution": {
            "current_anchor_cruise_max_vel_x_mps": CURRENT_MAX_VEL_X_MPS,
            "current_anchor_cruise_acc_lim_x_mps2": CURRENT_ACC_LIM_X_MPS2,
            "current_bank_max_vel_x_upper_mps": 1.2,
            "current_bank_acc_lim_x_upper_mps2": 0.8,
            "both_current_longitudinal_values_at_bank_upper_bound": True,
            "mean_total_path_to_terminal_m": _mean(blocks, "total_path_to_terminal_m"),
            "mean_dispatch_to_first_motion_command_s": _mean(blocks, "dispatch_to_first_motion_command_s"),
            "mean_first_cruise_elapsed_s": _mean(blocks, "first_cruise_elapsed_s"),
            "mean_first_cruise_velocity_mps": _mean(blocks, "first_cruise_velocity_mps"),
            "mean_post_cruise_path_m": _mean(blocks, "post_cruise_path_m"),
            "mean_cruise_dwell_s": _mean(blocks, "cruise_dwell_s"),
            "mean_actual_cap_bound_dwell_s": _mean(blocks, "actual_cap_bound_dwell_s"),
            "mean_commanded_cap_bound_dwell_s": _mean(blocks, "commanded_cap_bound_dwell_s"),
            "mean_actual_cap_bound_fraction_of_observation": _mean(blocks, "actual_cap_bound_fraction_of_observation"),
            "mean_commanded_cap_bound_fraction_of_observation": _mean(blocks, "commanded_cap_bound_fraction_of_observation"),
            "mean_start_0_to_10_percent_path_slack_s": _mean(blocks, "start_0_to_10_percent_path_slack_s"),
            "mean_middle_10_to_90_percent_path_slack_s": _mean(blocks, "middle_10_to_90_percent_path_slack_s"),
            "mean_terminal_90_to_100_percent_path_slack_s": _mean(blocks, "terminal_90_to_100_percent_path_slack_s"),
            "mean_post_cruise_recoverable_time_s": _mean(blocks, "post_cruise_recoverable_time_s"),
        },
        "same_path_longitudinal_envelopes": {
            "definition": {
                "path_geometry_replayed_not_changed": True,
                "pre_cruise_prefix_observed_not_retimed": True,
                "post_terminal_accounting_observed_not_retimed": True,
                "max_vel_x_mps": CURRENT_MAX_VEL_X_MPS,
                "symmetric_acceleration_mps2": CURRENT_ACC_LIM_X_MPS2,
                "terminal_velocity_mps": 0.0,
            },
            "whole_trace_diagnostic": {
                "geometric_time_s": whole_geometric,
                "reduction_fraction_vs_paired_legacy": whole_reduction,
                "paired_bootstrap_one_sided_95_percent_lower": whole_lower,
                "blocks_at_or_above_8_percent": sum(row["whole_trace_envelope_paired_reduction_fraction"] >= 0.08 for row in blocks),
                "blocks_at_or_above_10_percent": sum(row["whole_trace_envelope_paired_reduction_fraction"] >= 0.10 for row in blocks),
                "status": "diagnostic_only_less_conservative_than_frozen_prefix",
            },
            "frozen_prefix_post_cruise_primary": {
                "geometric_time_s": post_cruise_geometric,
                "reduction_fraction_vs_paired_legacy": post_cruise_reduction,
                "paired_bootstrap_one_sided_95_percent_lower": post_cruise_lower,
                "margin_above_10_percent_point_fraction": post_cruise_reduction - 0.10,
                "margin_above_10_percent_lower_fraction": post_cruise_lower - 0.10,
                "blocks_at_or_above_8_percent": sum(row["post_cruise_envelope_paired_reduction_fraction"] >= 0.08 for row in blocks),
                "blocks_at_or_above_10_percent": sum(row["post_cruise_envelope_paired_reduction_fraction"] >= 0.10 for row in blocks),
                "physical_room_gate_pass": physical_room_pass,
            },
        },
        "target_closure": target_rows,
        "historical_single_coordinate_check": historical,
        "decision": {
            "aggregate_physical_room_for_8_to_10_percent": physical_room_pass,
            "factor_hypothesis_retained_for_design": physical_room_pass,
            "specific_executable_runtime_factor_qualified": False,
            "counterfactual_gain_achieved": False,
            "causal_performance_improvement_demonstrated": False,
            "new_speed_bound_safety_qualified": False,
            "simulation_authorized": False,
            "decision_reason": (
                "post_cruise_same_path_point_and_one_sided_lower_exceed_ten_percent_but_runtime_mapping_and_causal_gain_remain_unqualified"
                if physical_room_pass else
                "post_cruise_same_path_envelope_did_not_clear_the_frozen_ten_percent_room_gate"
            ),
        },
        "claim_boundary": {
            "allowed": [
                "the_frozen_30_block_trace_set_contains_aggregate_same_path_longitudinal_headroom_for_an_8_to_10_percent_design_target",
                "the_primary_envelope_preserves_the_observed_pre_cruise_prefix_and_uses_current_longitudinal_limits",
                "the_result_is_a_physical_room_screen_not_an_achieved_treatment_effect",
            ],
            "forbidden": [
                "p2_performance_improvement",
                "causal_gain_from_a_named_runtime_parameter",
                "safety_of_speed_above_the_current_bank_bound",
                "p1_positive_reinterpretation",
                "cross_scene_or_real_vehicle_generalization",
                "winner_runtime_ready_or_deployment_ready",
            ],
        },
        "side_effect_audit": {
            "ros_gazebo_move_base_started": False,
            "simulation_or_training_started": False,
            "release_attempt_journal_or_consumption_created": False,
            "seed_evidence_or_training_budget_consumed": False,
            "runtime_profile_or_p1_evidence_modified": False,
            "real_vehicle_connected": False,
        },
    }
    source_closure = verified_source_bindings + [
        {"label": "p1_terminal_manifest_target", **entry} for entry in terminal_entries
    ] + [
        {"label": "selected_journal", "path": row["journal_path"], "sha256": row["journal_sha256"]}
        for row in blocks
    ] + [
        {"label": "selected_raw", **resource} for resource in raw_resources
    ]
    deduplicated = {}
    for item in source_closure:
        previous = deduplicated.get(item["path"])
        require(previous is None or previous["sha256"] == item["sha256"], "source closure SHA conflict")
        deduplicated[item["path"]] = item
    source_closure = [deduplicated[path] for path in sorted(deduplicated)]
    blocks_payload = canonical_blocks_bytes(blocks)
    report["source_integrity"]["unique_closed_source_paths"] = len(source_closure)
    report["artifacts"] = {
        "block_table": {
            "path": BLOCKS_RELATIVE.as_posix(),
            "sha256": sha256_bytes(blocks_payload),
            "rows": len(blocks),
        }
    }
    return report, blocks, source_closure


def canonical_blocks_bytes(blocks: Sequence[Mapping[str, Any]]) -> bytes:
    return (json.dumps(blocks, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n").encode("utf-8")


def report_bytes(report: Mapping[str, Any]) -> bytes:
    return yaml.safe_dump(dict(report), sort_keys=False, allow_unicode=True).encode("utf-8")
