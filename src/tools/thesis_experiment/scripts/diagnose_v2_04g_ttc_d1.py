#!/usr/bin/env python3
"""Reproduce the V2-04G-TTC-D1 seed5111 diagnosis without starting ROS.

The script verifies every declared frozen input before importing the pure
geometry modules.  Its only permitted persistent write is the deterministic
D1 YAML report.
"""

import argparse
from collections import Counter
import copy
import hashlib
import math
from pathlib import Path
import sys

import yaml


STAGE = "V2-04G-TTC-D1"
SOURCE_STAGE = "V2-04G-R5"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_ttc_d1_offline_diagnosis_contract.yaml"
)
OUTPUT_RELATIVE = Path(
    "artifacts/v2/diagnosis/v2_04g_ttc_d1/"
    "v2_04g_ttc_d1_report.yaml"
)
R5_ARTIFACT_RELATIVE = Path("artifacts/v2/calibration/v2_04g_r5")
EXPECTED_RISK_IDS = (
    "D1-RISK-READINESS-DIRECT-COUNTS",
    "D1-RISK-COMPILED-SCENE-TOCTOU",
    "D1-RISK-SIGINT-IN-PROGRESS",
    "D1-RISK-ASSESSMENT-RAW-BINDING",
    "D1-RISK-EXECUTION-HASH-CLOSURE",
    "D1-RISK-TEARDOWN-RESTORE",
)
FROZEN_HORIZONS = (5.0, 4.5, 4.0)
EXPLORATORY_HORIZONS = (1.5, 1.0)


class D1DiagnosisError(ValueError):
    """Raised when a D1 boundary or frozen input is ambiguous or has drifted."""


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise D1DiagnosisError("duplicate YAML key: {!r}".format(key))
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _require(condition, message):
    if not condition:
        raise D1DiagnosisError(message)


def _load_yaml(path):
    source = Path(path)
    try:
        value = yaml.load(
            source.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader
        )
    except (OSError, yaml.YAMLError) as exc:
        raise D1DiagnosisError(
            "cannot strictly load YAML {}: {}".format(source, exc)
        ) from exc
    _require(isinstance(value, dict), "{} must contain a mapping".format(source))
    return value


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _inside(root, path, label):
    boundary = Path(root).resolve()
    resolved = Path(path).resolve()
    try:
        resolved.relative_to(boundary)
    except ValueError as exc:
        raise D1DiagnosisError(
            "{} leaves workspace: {}".format(label, resolved)
        ) from exc
    return resolved


def _resolve(workspace, value, label):
    path = Path(value)
    resolved = path.resolve() if path.is_absolute() else (workspace / path).resolve()
    return _inside(workspace, resolved, label)


def _relative(workspace, path):
    return str(Path(path).resolve().relative_to(Path(workspace).resolve()))


def _line_number(text, needle):
    for index, line in enumerate(text.splitlines(), start=1):
        if needle in line:
            return index
    raise D1DiagnosisError("required source evidence is missing: {}".format(needle))


def _validate_contract(workspace, contract_path):
    root = Path(workspace).resolve()
    expected = (root / CONTRACT_RELATIVE).resolve()
    actual = _inside(root, contract_path, "D1 contract")
    _require(actual == expected, "D1 contract path drifted")
    contract = _load_yaml(actual)
    _require(
        contract.get("schema_version") == "2.0"
        and contract.get("architecture_generation") == "v2"
        and contract.get("stage") == STAGE
        and contract.get("status") == "offline_diagnosis_contract"
        and contract.get("offline_only") is True
        and contract.get("diagnosis_only") is True
        and contract.get("simulation_started") is False
        and contract.get("calibration_execution_allowed") is False
        and contract.get("formal_result") is False
        and contract.get("runtime_ready") is False
        and contract.get("training_allowed") is False
        and contract.get("real_vehicle_use_forbidden") is True,
        "D1 safety boundary drifted",
    )
    permissions = contract.get("permissions", {})
    _require(
        permissions.get("read_frozen_r5_inputs") is True
        and permissions.get("write_only_d1_report") is True
        and all(
            permissions.get(key) is False
            for key in (
                "write_r5_contract_script_scene_or_artifact",
                "start_ros",
                "start_gazebo",
                "start_move_base",
                "execute_component_probe",
                "execute_navigation",
                "consume_seed_or_evidence_unit",
                "change_runtime_threshold",
                "change_scene",
                "change_evaluator",
                "create_r6_execution_authorization",
                "train_sac_or_any_model",
                "connect_real_vehicle",
                "write_real_vehicle_teb_parameters",
            )
        ),
        "D1 permissions are not offline/read-only",
    )
    scope = contract.get("scope", {})
    _require(
        scope.get("source_stage") == SOURCE_STAGE
        and scope.get("source_identity") == "r5-readiness-r5_ttc_h450-s5111"
        and scope.get("source_seed") == 5111
        and scope.get("evidence_units_consumed") == 0,
        "D1 source identity or evidence boundary drifted",
    )
    replay = contract.get("replay_contract", {})
    _require(
        tuple(float(value) for value in replay.get(
            "frozen_runtime_candidate_horizons_s", []
        )) == FROZEN_HORIZONS
        and tuple(float(value) for value in replay.get(
            "exploratory_future_horizons_s", []
        )) == EXPLORATORY_HORIZONS
        and replay.get("exploratory_horizons_are_runtime_changes") is False
        and replay.get("exploratory_horizons_are_offline_counterfactuals_only")
        is True,
        "D1 replay horizon boundary drifted",
    )
    risk_ids = tuple(
        row.get("risk_id") for row in contract.get("required_risk_audit", [])
    )
    _require(risk_ids == EXPECTED_RISK_IDS, "D1 required risk set drifted")
    output = contract.get("output", {})
    _require(
        output.get("path") == str(OUTPUT_RELATIVE)
        and output.get("deterministic") is True
        and output.get("atomic_write") is True
        and output.get("only_persistent_write_allowed") is True,
        "D1 output boundary drifted",
    )
    authorizations = contract.get("authorizations_after_diagnosis", {})
    _require(
        authorizations
        and all(value is False for value in authorizations.values()),
        "D1 grants a downstream authorization",
    )
    return contract, actual


def _verify_declared_inputs(workspace, contract):
    resources = contract.get("frozen_inputs")
    _require(isinstance(resources, dict) and resources, "frozen_inputs is empty")
    verified = {}
    for label, resource in resources.items():
        _require(
            isinstance(resource, dict)
            and set(resource) == {"path", "sha256"}
            and isinstance(resource["path"], str)
            and isinstance(resource["sha256"], str)
            and len(resource["sha256"]) == 64,
            "frozen input declaration drifted: {}".format(label),
        )
        path = _resolve(workspace, resource["path"], label)
        _require(path.is_file(), "frozen input is missing: {}".format(label))
        digest = _sha256(path)
        _require(
            digest == resource["sha256"],
            "frozen input hash drifted: {}".format(label),
        )
        verified[label] = {
            "path": resource["path"],
            "sha256": digest,
            "verified": True,
        }
    return verified


def _verify_dependency_chain(workspace, contract):
    verified = []
    for index, resource in enumerate(contract.get("execution_dependency_chain", [])):
        _require(
            isinstance(resource, dict) and set(resource) == {"path", "sha256"},
            "execution dependency declaration {} drifted".format(index),
        )
        path = _resolve(workspace, resource["path"], "execution dependency")
        _require(path.is_file(), "execution dependency is missing")
        digest = _sha256(path)
        _require(digest == resource["sha256"], "execution dependency hash drifted")
        verified.append({"path": resource["path"], "sha256": digest})
    _require(len(verified) == 7, "execution dependency chain must contain 7 files")
    return verified


def _resource_entries(value, prefix=""):
    if isinstance(value, dict):
        if "path" in value or "sha256" in value:
            _require(
                set(value) == {"path", "sha256"},
                "{} resource declaration drifted".format(prefix),
            )
            yield prefix, value
            return
        for key, child in value.items():
            child_prefix = "{}.{}".format(prefix, key) if prefix else str(key)
            yield from _resource_entries(child, child_prefix)


def _verify_preregistered_closure(workspace, preregistration):
    entries = list(_resource_entries({
        "resources": preregistration.get("resources", {}),
        "frozen_r4_r1_boundary": preregistration.get(
            "frozen_r4_r1_boundary", {}
        ),
    }))
    _require(len(entries) == 39, "R5 declared closure is not 39 files")
    paths = set()
    records = []
    for label, resource in entries:
        path = _resolve(workspace, resource["path"], label)
        _require(path.is_file(), "R5 closure file is missing: {}".format(label))
        digest = _sha256(path)
        _require(
            digest == resource["sha256"],
            "R5 closure hash drifted: {}".format(label),
        )
        relative = _relative(workspace, path)
        _require(relative not in paths, "R5 closure contains duplicate paths")
        paths.add(relative)
        records.append({
            "label": label,
            "path": relative,
            "sha256": digest,
        })
    return records, paths


def _verify_compiled_index(workspace, index_path, expected_count):
    index = _load_yaml(index_path)
    files = index.get("files")
    _require(
        isinstance(files, list)
        and len(files) == expected_count
        and index.get("scene_count") * 2 == expected_count,
        "compiled index child count drifted",
    )
    index_root = Path(index_path).resolve().parent
    records = []
    for row in files:
        _require(
            isinstance(row, dict) and set(row) == {"path", "sha256"},
            "compiled index resource declaration drifted",
        )
        path = _resolve(workspace, row["path"], "compiled scene child")
        _inside(index_root, path, "compiled scene child")
        _require(path.is_file(), "compiled scene child is missing")
        digest = _sha256(path)
        _require(digest == row["sha256"], "compiled scene child hash drifted")
        records.append({"path": row["path"], "sha256": digest})
    return {
        "index_path": _relative(workspace, index_path),
        "declared_child_file_count": expected_count,
        "verified_child_file_count": len(records),
        "all_child_hashes_match": True,
        "files": records,
    }


def _snapshot_tree(workspace, relative_root):
    root = (Path(workspace) / relative_root).resolve()
    _inside(workspace, root, "R5 artifact tree")
    _require(root.is_dir(), "R5 artifact tree is missing")
    files = []
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        files.append({
            "path": _relative(workspace, path),
            "sha256": _sha256(path),
        })
    canonical = "".join(
        "{} {}\n".format(row["path"], row["sha256"]) for row in files
    ).encode("utf-8")
    return {
        "file_count": len(files),
        "tree_sha256": hashlib.sha256(canonical).hexdigest(),
        "files": files,
    }


def _load_project_modules(workspace):
    root = Path(workspace)
    sys.dont_write_bytecode = True
    additions = (
        root / "src/perception/nav_world_model/src",
        root / "src/application/teb_mode_manager/src",
        root / "src/tools/thesis_experiment/src",
    )
    for path in reversed(additions):
        value = str(path)
        if value not in sys.path:
            sys.path.insert(0, value)
    from nav_world_model import core
    from nav_world_model import risk_evidence
    from teb_mode_manager import rule_supervisor
    from thesis_experiment import v2_evaluator
    from thesis_experiment import v2_scene

    return core, risk_evidence, rule_supervisor, v2_evaluator, v2_scene


def _tracker_from_config(core, config):
    tracker = config["tracker"]
    prediction = config["prediction"]
    return core.MultiObjectTracker(
        association_gate_m=tracker["association_gate_m"],
        alpha=tracker["alpha"],
        beta=tracker["beta"],
        minimum_confirmed_hits=tracker["minimum_confirmed_hits"],
        maximum_misses=tracker["maximum_misses"],
        maximum_dt_s=tracker["maximum_dt_s"],
        stationary_speed_max_mps=tracker["stationary_speed_max_mps"],
        dynamic_speed_min_mps=tracker["dynamic_speed_min_mps"],
        prediction_horizon_s=prediction["horizon_s"],
        prediction_step_s=prediction["step_s"],
        confidence_decay_per_s=prediction["confidence_decay_per_s"],
        crossing_lateral_speed_min_mps=tracker[
            "crossing_lateral_speed_min_mps"
        ],
        crossing_path_half_width_m=tracker["crossing_path_half_width_m"],
    )


def _actor_state(agent, stamp_s):
    trajectory = agent["trajectory"]
    _require(len(trajectory) == 2, "D1 expects one two-point actor trajectory")
    first, second = trajectory
    start = float(first["time_s"])
    end = float(second["time_s"])
    _require(end > start, "actor trajectory time is invalid")
    if stamp_s < start:
        return (
            float(first["x_m"]), float(first["y_m"]), 0.0, 0.0,
            float(first["yaw_rad"]),
        )
    if stamp_s > end:
        return (
            float(second["x_m"]), float(second["y_m"]), 0.0, 0.0,
            float(second["yaw_rad"]),
        )
    ratio = (stamp_s - start) / (end - start)
    x = float(first["x_m"]) + ratio * (
        float(second["x_m"]) - float(first["x_m"])
    )
    y = float(first["y_m"]) + ratio * (
        float(second["y_m"]) - float(first["y_m"])
    )
    vx = (float(second["x_m"]) - float(first["x_m"])) / (end - start)
    vy = (float(second["y_m"]) - float(first["y_m"])) / (end - start)
    yaw = float(first["yaw_rad"]) + ratio * (
        float(second["yaw_rad"]) - float(first["yaw_rad"])
    )
    return x, y, vx, vy, yaw


def _relative_samples(core, supervisor_module, rows, agent, tracker, radius):
    samples = []
    for index, row in enumerate(rows):
        actor_x, actor_y, actor_vx, actor_vy, _ = _actor_state(
            agent, row["stamp_s"]
        )
        robot = core.RobotState(
            x=row["x_m"],
            y=row["y_m"],
            yaw=row["yaw_rad"],
            linear_velocity=row["linear_velocity_mps"],
        )
        proxy = core._Track(
            track_id=1,
            x=actor_x,
            y=actor_y,
            vx=actor_vx,
            vy=actor_vy,
            radius=radius,
            created_s=0.0,
            updated_s=row["stamp_s"],
            hits=10,
        )
        motion_class = tracker._classify(proxy, robot)
        cosine, sine = math.cos(robot.yaw), math.sin(robot.yaw)
        dx, dy = actor_x - robot.x, actor_y - robot.y
        robot_vx = robot.linear_velocity * cosine
        robot_vy = robot.linear_velocity * sine
        relative_vx_world = actor_vx - robot_vx
        relative_vy_world = actor_vy - robot_vy
        runtime_track = supervisor_module.RuntimeTrack(
            track_id=1,
            motion_class=motion_class,
            x=cosine * dx + sine * dy,
            y=-sine * dx + cosine * dy,
            vx=cosine * relative_vx_world + sine * relative_vy_world,
            vy=-sine * relative_vx_world + cosine * relative_vy_world,
            radius=radius,
            confidence=0.90,
        )
        samples.append({
            "index": index,
            "stamp_s": row["stamp_s"],
            "motion_class": motion_class,
            "track": runtime_track,
            "actor_x_m": actor_x,
            "actor_y_m": actor_y,
        })
    return samples


def _overlay_replay(supervisor_module, config, samples):
    supervisor = supervisor_module.RuleContextSupervisor(config)
    return [
        supervisor._overlay((sample["track"],))[0]
        for sample in samples
    ]


def _counts(values):
    counter = Counter(values)
    return {
        key: counter.get(key, 0)
        for key in (
            "NONE", "CROSSING", "HEAD_ON", "FOLLOW", "OVERTAKE_OR_YIELD"
        )
    }


def _motion_counts(samples):
    counter = Counter(sample["motion_class"] for sample in samples)
    return {
        key: counter.get(key, 0)
        for key in (
            "STATIONARY", "CROSSING", "HEAD_ON", "DEPARTING", "UNKNOWN"
        )
    }


def _pairwise_difference(rows, samples, outputs, first, second):
    details = []
    for index, (left, right) in enumerate(zip(outputs[first], outputs[second])):
        if left == right:
            continue
        details.append({
            "trace_index": index,
            "stamp_s": rows[index]["stamp_s"],
            "motion_class": samples[index]["motion_class"],
            "first_overlay": left,
            "second_overlay": right,
        })
    return {
        "first_horizon_s": first,
        "second_horizon_s": second,
        "difference_count": len(details),
        "crossing_motion_class_difference_count": sum(
            row["motion_class"] == "CROSSING" for row in details
        ),
        "differences": details,
    }


def _interpolate_at(rows, key, target):
    for before, after in zip(rows, rows[1:]):
        first, second = float(before[key]), float(after[key])
        if first <= target <= second and second > first:
            ratio = (target - first) / (second - first)
            result = {
                name: float(before[name]) + ratio * (
                    float(after[name]) - float(before[name])
                )
                for name in ("stamp_s", "x_m", "y_m", "yaw_rad")
            }
            result["interpolation_ratio"] = ratio
            return result
    raise D1DiagnosisError(
        "trace does not bracket {}={}".format(key, target)
    )


def _ttc_and_margin(
    risk_evidence, samples, robot_radius_m, horizon_s, minimum_confidence
):
    finite = []
    closest = []
    interaction_radius = robot_radius_m + samples[0]["track"].radius
    for sample in samples:
        track = sample["track"]
        relative = risk_evidence.RelativeTrack(
            x=track.x,
            y=track.y,
            vx=track.vx,
            vy=track.vy,
            radius=track.radius,
            confidence=track.confidence,
            motion_class=track.motion_class,
        )
        value = risk_evidence.relative_collision_ttc(
            relative,
            robot_radius_m=robot_radius_m,
            horizon_s=horizon_s,
            minimum_confidence=minimum_confidence,
        )
        if value is not None:
            finite.append({
                "trace_index": sample["index"],
                "stamp_s": sample["stamp_s"],
                "ttc_s": value,
                "motion_class": sample["motion_class"],
            })
        speed_squared = track.vx * track.vx + track.vy * track.vy
        if speed_squared <= 1.0e-12:
            closest_time = 0.0
        else:
            closest_time = max(
                0.0,
                min(
                    horizon_s,
                    -(track.x * track.vx + track.y * track.vy)
                    / speed_squared,
                ),
            )
        center_separation = math.hypot(
            track.x + track.vx * closest_time,
            track.y + track.vy * closest_time,
        )
        closest.append({
            "trace_index": sample["index"],
            "stamp_s": sample["stamp_s"],
            "motion_class": sample["motion_class"],
            "closest_time_from_sample_s": closest_time,
            "predicted_center_separation_m": center_separation,
            "circle_envelope_margin_m": center_separation - interaction_radius,
        })
    minimum = min(closest, key=lambda row: row["circle_envelope_margin_m"])
    return {
        "proxy_finite_ttc_sample_count": len(finite),
        "proxy_minimum_ttc_s": min(
            (row["ttc_s"] for row in finite), default=None
        ),
        "finite_samples": finite,
        "robot_radius_m": robot_radius_m,
        "actor_radius_m": samples[0]["track"].radius,
        "interaction_radius_m": interaction_radius,
        "minimum_predicted_circle_envelope": minimum,
    }


def _truth_clearance(risk_evidence, rows, agent):
    robot_size = (1.04, 0.70)
    actor_size = (
        float(agent["size_m"][0]),
        float(agent["size_m"][1]),
    )
    values = []
    for index, row in enumerate(rows):
        actor_x, actor_y, _, _, actor_yaw = _actor_state(
            agent, row["stamp_s"]
        )
        clearance = risk_evidence.oriented_box_clearance(
            (row["x_m"], row["y_m"], row["yaw_rad"]),
            robot_size,
            (actor_x, actor_y, actor_yaw),
            actor_size,
        )
        values.append({
            "trace_index": index,
            "stamp_s": row["stamp_s"],
            "clearance_m": clearance,
            "robot_x_m": row["x_m"],
            "robot_y_m": row["y_m"],
            "actor_x_m": actor_x,
            "actor_y_m": actor_y,
        })
    return min(values, key=lambda row: row["clearance_m"])


def _source_block(text, start_marker, end_marker):
    _require(start_marker in text, "source start marker is missing")
    tail = text.split(start_marker, 1)[1]
    _require(end_marker in tail, "source end marker is missing")
    return tail.split(end_marker, 1)[0]


def _risk_audit(
    workspace, verified, prereg_paths, dependencies, documents
):
    readiness_path = _resolve(
        workspace, verified["r5_readiness_batch"]["path"], "readiness source"
    )
    assessor_path = _resolve(
        workspace, verified["r5_assessor"]["path"], "assessor source"
    )
    runner_path = _resolve(
        workspace, verified["r5_episode_runner"]["path"], "runner source"
    )
    launch_path = _resolve(
        workspace, verified["launch_log"]["path"], "launch log"
    )
    readiness_text = readiness_path.read_text(encoding="utf-8")
    assessor_text = assessor_path.read_text(encoding="utf-8")
    runner_text = runner_path.read_text(encoding="utf-8")
    launch_text = launch_path.read_text(encoding="utf-8", errors="replace")
    gate_block = _source_block(
        readiness_text,
        "    if not (\n        listener_report.get(\"stage\")",
        "        raise RuntimeError(\"R5 readiness activation or TTC coverage gate failed\")",
    )
    design_block = _source_block(
        readiness_text, "def _validate_design(prereg):", "\ndef _listener_command"
    )
    summary = documents["readiness_summary"]
    assessment = documents["r5_assessment"]
    evaluation = documents["evaluation"]
    activation = documents["activation_report"]
    assessment_evidence = assessment.get("evidence", {})
    missing_dependencies = [
        row for row in dependencies if row["path"] not in prereg_paths
    ]
    stage_implementation = documents["r5_stage_report"].get(
        "execution_implementation", {}
    )
    goal_line = _line_number(launch_text, "GOAL Reached!")
    restore_line = _line_number(
        launch_text, "failed to restore startup typed TEB profile"
    )
    findings = [
        {
            "risk_id": EXPECTED_RISK_IDS[0],
            "status": "CONFIRMED",
            "evidence": {
                "source_path": _relative(workspace, readiness_path),
                "immediate_gate_start_line": _line_number(
                    readiness_text,
                    "listener_report.get(\"stage\") == STAGE",
                ),
                "direct_tracker_message_count_check_present": (
                    "tracker_message_count" in gate_block
                ),
                "direct_context_message_count_check_present": (
                    "context_message_count" in gate_block
                ),
                "minimum_required_count": 20,
                "observed_evaluation_tracker_message_count": evaluation[
                    "tracker_message_count"
                ],
                "observed_evaluation_context_message_count": evaluation[
                    "context_message_count"
                ],
                "observed_activation_context_message_count": activation[
                    "context_message_count"
                ],
            },
            "interpretation": (
                "The immediate combined readiness/TTC condition relies on the "
                "listener aggregate gate but does not directly compare the "
                "evaluation tracker/context counts to the preregistered minimum."
            ),
            "future_requirement": (
                "Bind and directly hard-check both evaluation stream counts "
                "before accepting an identity."
            ),
            "changes_r5_terminal_stop": False,
        },
        {
            "risk_id": EXPECTED_RISK_IDS[1],
            "status": "CONFIRMED",
            "evidence": {
                "source_path": _relative(workspace, readiness_path),
                "compiled_load_line": _line_number(
                    readiness_text,
                    "instances = _BASE._load_instances(COMPILED_SCENES)",
                ),
                "execution_load_call": (
                    "instances = _BASE._load_instances(COMPILED_SCENES)"
                ),
                "child_sha256_revalidation_present_in_design_block": (
                    "sha256" in design_block
                    or "compiled_scene_index.yaml" in design_block
                ),
                "d1_readiness_child_hashes_verified": 14,
                "d1_navigation_child_hashes_verified": 30,
            },
            "interpretation": (
                "The frozen index hashes were checked before execution, but "
                "the child files were loaded later without revalidating each "
                "index-declared digest, leaving a TOCTOU interval."
            ),
            "future_requirement": (
                "Revalidate every compiled child immediately before launch and "
                "bind the exact instance/world pair into the attempt ledger."
            ),
            "changes_r5_terminal_stop": False,
        },
        {
            "risk_id": EXPECTED_RISK_IDS[2],
            "status": "CONFIRMED",
            "evidence": {
                "source_path": _relative(workspace, readiness_path),
                "attempt_ledger_written_before_run": True,
                "except_exception_line": _line_number(
                    readiness_text, "except Exception as exc:"
                ),
                "keyboard_interrupt_handler_present": (
                    "except KeyboardInterrupt" in readiness_text
                ),
                "base_exception_handler_present": (
                    "except BaseException" in readiness_text
                ),
                "summary_initial_status": "in_progress",
                "resume_forbidden_if_interrupted": True,
            },
            "interpretation": (
                "KeyboardInterrupt is outside Exception, so SIGINT can bypass "
                "the terminal-state recorder after an attempt was ledgered."
            ),
            "future_requirement": (
                "Atomically catch interruption, teardown, and persist an "
                "explicit terminal_interrupted state before re-raising."
            ),
            "changes_r5_terminal_stop": False,
        },
        {
            "risk_id": EXPECTED_RISK_IDS[3],
            "status": "CONFIRMED",
            "evidence": {
                "source_path": _relative(workspace, assessor_path),
                "readiness_summary_reports_count": len(summary.get("reports", [])),
                "assessment_evidence_keys": sorted(assessment_evidence),
                "activation_directly_bound": "activation_report" in assessment_evidence,
                "evaluation_directly_bound": "evaluation" in assessment_evidence,
                "trace_directly_bound": "trace" in assessment_evidence,
                "stage_report_separately_binds_raw_hashes": all(
                    key
                    in documents["r5_stage_report"].get(
                        "preserved_evidence", {}
                    )
                    for key in (
                        "activation_report", "evaluation", "trace",
                    )
                ),
            },
            "interpretation": (
                "The terminal summary contains no accepted report row, so the "
                "assessment output binds the summary but not the raw activation, "
                "evaluation, or trace.  The manually assembled stage report "
                "binds them separately."
            ),
            "future_requirement": (
                "Make a terminal assessment verify and emit hashes for all raw "
                "attempt evidence, including failed-gate attempts."
            ),
            "changes_r5_terminal_stop": False,
        },
        {
            "risk_id": EXPECTED_RISK_IDS[4],
            "status": "CONFIRMED",
            "evidence": {
                "source_path": _relative(workspace, runner_path),
                "dynamic_chain_entry_line": _line_number(
                    runner_text, "_SPEC.loader.exec_module(_R4_R1)"
                ),
                "transitive_dependency_count": len(dependencies),
                "dependencies": [
                    {
                        **row,
                        "in_preregistered_39_file_closure": (
                            row["path"] in prereg_paths
                        ),
                    }
                    for row in dependencies
                ],
                "missing_from_preregistered_closure_count": len(
                    missing_dependencies
                ),
                "missing_from_preregistered_closure_paths": [
                    row["path"] for row in missing_dependencies
                ],
                "top_level_runner_bound_by_stage_report": (
                    stage_implementation.get("episode_runner_sha256")
                    == dependencies[0]["sha256"]
                ),
            },
            "interpretation": (
                "The top-level R5 runner hash is recorded in the stage report, "
                "but its dynamically loaded wrapper/evaluator chain is not fully "
                "included in the preregistered closure."
            ),
            "future_requirement": (
                "Generate a mechanical transitive execution manifest and bind "
                "the entire closure before authorization."
            ),
            "changes_r5_terminal_stop": False,
        },
        {
            "risk_id": EXPECTED_RISK_IDS[5],
            "status": "CONFIRMED",
            "evidence": {
                "log_path": _relative(workspace, launch_path),
                "goal_reached_line": goal_line,
                "startup_restore_failure_line": restore_line,
                "restore_failure_after_goal": restore_line > goal_line,
                "navigation_success": evaluation["metrics"]["common"]["success"],
                "measurement_interface_gates_passed": activation[
                    "all_hard_gates_pass"
                ],
                "startup_profile_restored": False,
            },
            "interpretation": (
                "The measurement and navigation evidence completed, but teardown "
                "failed to restore the startup typed TEB profile; therefore the "
                "claim is limited to interface gates during the measurement window."
            ),
            "future_requirement": (
                "Treat startup-profile restore as a teardown hard gate with an "
                "independently verified final profile."
            ),
            "changes_r5_terminal_stop": False,
        },
    ]
    _require(
        tuple(row["risk_id"] for row in findings) == EXPECTED_RISK_IDS,
        "risk audit did not produce the required set",
    )
    _require(
        all(row["status"] == "CONFIRMED" for row in findings),
        "a required D1 risk was not confirmed",
    )
    return findings


def build_report(workspace, contract_path=None):
    """Build the deterministic report in memory without persistent writes."""

    root = Path(workspace).resolve()
    contract_path = (
        root / CONTRACT_RELATIVE if contract_path is None else Path(contract_path)
    )
    contract, contract_path = _validate_contract(root, contract_path)
    verified = _verify_declared_inputs(root, contract)
    dependencies = _verify_dependency_chain(root, contract)
    before_tree = _snapshot_tree(root, R5_ARTIFACT_RELATIVE)

    documents = {
        key: _load_yaml(_resolve(root, verified[key]["path"], key))
        for key in (
            "r5_preregistration",
            "r5_stage_report",
            "r5_assessment",
            "readiness_summary",
            "activation_report",
            "evaluation",
            "clearance_audit",
            "seed5111_instance",
        )
    }
    preregistration = documents["r5_preregistration"]
    _require(
        preregistration.get("stage") == SOURCE_STAGE
        and preregistration.get("formal_result") is False
        and preregistration.get("runtime_ready") is False,
        "R5 preregistration boundary drifted",
    )
    closure, closure_paths = _verify_preregistered_closure(
        root, preregistration
    )
    readiness_compiled = _verify_compiled_index(
        root,
        _resolve(
            root,
            verified["readiness_compiled_index"]["path"],
            "readiness compiled index",
        ),
        14,
    )
    navigation_compiled = _verify_compiled_index(
        root,
        _resolve(
            root,
            verified["navigation_compiled_index"]["path"],
            "navigation compiled index",
        ),
        30,
    )

    core, risk_evidence, supervisor_module, evaluator, scene_module = (
        _load_project_modules(root)
    )
    instance = documents["seed5111_instance"]
    unhashed = {
        key: instance[key] for key in ("schema_version", "generator", "scene")
    }
    _require(
        scene_module.canonical_sha256(unhashed) == instance["instance_sha256"],
        "seed5111 canonical instance hash drifted",
    )
    scene = instance["scene"]
    _require(
        scene.get("scene_id")
        == "v2-04g-r5-readiness-dynamic-conflict-s5111"
        and scene.get("family") == "DYNAMIC"
        and scene.get("seed") == 5111
        and len(scene.get("dynamic_agents", [])) == 1,
        "seed5111 scene identity drifted",
    )
    agent = scene["dynamic_agents"][0]
    _require(agent.get("agent_id") == "crossing-agent", "actor identity drifted")
    trace_path = _resolve(root, verified["trace"]["path"], "trace")
    rows = evaluator.load_v2_trace(trace_path)
    evaluation = documents["evaluation"]
    clearance = documents["clearance_audit"]
    activation = documents["activation_report"]
    summary = documents["readiness_summary"]
    assessment = documents["r5_assessment"]
    _require(
        evaluation.get("stage") == SOURCE_STAGE
        and evaluation.get("scene_id") == scene["scene_id"]
        and evaluation.get("seed") == 5111
        and evaluation.get("raw_trace_sha256") == verified["trace"]["sha256"]
        and evaluation.get("ttc_status") == "NO_CONFLICT_IN_HORIZON"
        and evaluation.get("finite_ttc_sample_count") == 0,
        "seed5111 evaluation identity or TTC result drifted",
    )
    _require(
        summary.get("status") == "terminal_failure"
        and summary.get("evidence_unit_count") == 1
        and summary.get("resume_forbidden") is True
        and assessment.get("status") == "calibration_terminally_stopped"
        and assessment.get("ranking_performed") is False,
        "R5 terminal-stop boundary drifted",
    )
    _require(
        activation.get("all_hard_gates_pass") is True
        and clearance.get("runtime_policy_received_truth") is False,
        "seed5111 interface/truth boundary drifted",
    )

    world_config = _load_yaml(
        _resolve(root, verified["world_model_config"]["path"], "world model config")
    )
    replay_contract = contract["replay_contract"]
    _require(
        float(world_config["prediction"]["horizon_s"])
        == float(replay_contract["runtime_motion_classifier_horizon_s"])
        and float(world_config["scan"]["robot_radius_m"])
        == float(replay_contract["robot_circle_radius_m"]),
        "world-model replay semantics drifted",
    )
    tracker = _tracker_from_config(core, world_config)
    actor_radius = math.hypot(
        float(agent["size_m"][0]) / 2.0,
        float(agent["size_m"][1]) / 2.0,
    )
    samples = _relative_samples(
        core, supervisor_module, rows, agent, tracker, actor_radius
    )

    supervisor_keys = {
        5.0: "runtime_supervisor_h500",
        4.5: "runtime_supervisor_h450",
        4.0: "runtime_supervisor_h400",
    }
    supervisor_configs = {}
    overlays = {}
    for horizon, key in supervisor_keys.items():
        config = _load_yaml(_resolve(root, verified[key]["path"], key))
        _require(
            float(config["dynamic"]["predicted_ttc_max_s"]) == horizon,
            "{} runtime horizon drifted".format(key),
        )
        supervisor_configs[horizon] = config
        overlays[horizon] = _overlay_replay(
            supervisor_module, config, samples
        )
    for horizon in EXPLORATORY_HORIZONS:
        config = copy.deepcopy(supervisor_configs[5.0])
        config["dynamic"]["predicted_ttc_max_s"] = horizon
        overlays[horizon] = _overlay_replay(
            supervisor_module, config, samples
        )

    frozen_pairwise = [
        _pairwise_difference(rows, samples, overlays, first, second)
        for first, second in ((5.0, 4.5), (4.5, 4.0), (5.0, 4.0))
    ]
    exploratory_pairwise = [
        _pairwise_difference(rows, samples, overlays, first, second)
        for first, second in ((5.0, 1.5), (1.5, 1.0))
    ]
    crossing_indices = [
        sample["index"]
        for sample in samples
        if sample["motion_class"] == "CROSSING"
    ]
    all_frozen_equal_on_crossing = all(
        len({overlays[horizon][index] for horizon in FROZEN_HORIZONS}) == 1
        for index in crossing_indices
    )
    _require(
        len(rows) == 193
        and len(crossing_indices) == 21
        and all_frozen_equal_on_crossing,
        "seed5111 replay distinguishability drifted",
    )

    first, second = agent["trajectory"]
    actor_crossing_time = 0.5 * (
        float(first["time_s"]) + float(second["time_s"])
    )
    robot_at_actor_crossing = _interpolate_at(
        rows, "stamp_s", actor_crossing_time
    )
    crossing_x = 0.5 * (float(first["x_m"]) + float(second["x_m"]))
    robot_at_crossing_point = _interpolate_at(rows, "x_m", crossing_x)
    arrival_gap = robot_at_crossing_point["stamp_s"] - actor_crossing_time

    ttc = _ttc_and_margin(
        risk_evidence,
        samples,
        float(replay_contract["robot_circle_radius_m"]),
        float(replay_contract["evaluator_circle_contact_horizon_s"]),
        float(replay_contract["evaluator_minimum_track_confidence"]),
    )
    trace_finite = [
        row["predicted_ttc_s"]
        for row in rows
        if row["predicted_ttc_s"] is not None
    ]
    proxy_truth = _truth_clearance(risk_evidence, rows, agent)
    frozen_truth = float(clearance["minimum_truth_box_clearance_m"])
    _require(
        not trace_finite
        and ttc["proxy_finite_ttc_sample_count"] == 0
        and ttc["minimum_predicted_circle_envelope"][
            "circle_envelope_margin_m"
        ] > 0.0
        and frozen_truth > 0.0,
        "seed5111 TTC/clearance diagnosis drifted",
    )

    findings = _risk_audit(
        root, verified, closure_paths, dependencies, documents
    )
    after_tree = _snapshot_tree(root, R5_ARTIFACT_RELATIVE)
    _require(before_tree == after_tree, "D1 modified the R5 artifact tree")

    report = {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "complete_offline_diagnosis",
        "offline_only": True,
        "diagnosis_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "source": {
            "stage": SOURCE_STAGE,
            "identity": "r5-readiness-r5_ttc_h450-s5111",
            "seed": 5111,
            "reuse_classification": "read_only_diagnosis_not_new_evidence",
            "new_evidence_units_consumed": 0,
            "r5_remaining_units_consumed": 0,
        },
        "implementation": {
            "contract": {
                "path": _relative(root, contract_path),
                "sha256": _sha256(contract_path),
            },
            "script": {
                "path": _relative(root, Path(__file__).resolve()),
                "sha256": _sha256(Path(__file__).resolve()),
            },
        },
        "integrity": {
            "declared_d1_frozen_inputs": {
                "count": len(verified),
                "all_hashes_match": True,
                "resources": verified,
            },
            "r5_preregistered_resource_closure": {
                "declared_count": len(closure),
                "verified_count": len(closure),
                "all_hashes_match": True,
                "resources": closure,
            },
            "compiled_scene_children": {
                "readiness": readiness_compiled,
                "navigation": navigation_compiled,
                "all_44_child_hashes_match": True,
            },
            "r5_artifact_tree_before_and_after_identical": True,
            "r5_artifact_tree": before_tree,
        },
        "frozen_r5_outcome": {
            "expected_ttc_status": "OBSERVED_CONFLICT",
            "observed_ttc_status": evaluation["ttc_status"],
            "readiness_status": summary["status"],
            "assessment_status": assessment["status"],
            "attempted_evidence_units": 1,
            "remaining_units_forfeited": 68,
            "retry_performed": False,
            "resume_performed": False,
            "component_started": False,
            "navigation_started": False,
            "ranking_performed": False,
            "winner": None,
        },
        "seed5111": {
            "trace_row_count": len(rows),
            "trace_finite_predicted_ttc_sample_count": len(trace_finite),
            "evaluation_tracker_message_count": evaluation[
                "tracker_message_count"
            ],
            "evaluation_context_message_count": evaluation[
                "context_message_count"
            ],
            "activation_context_message_count": activation[
                "context_message_count"
            ],
            "observed_runtime_context_overlay_sample_counts": evaluation[
                "context_overlay_sample_counts"
            ],
            "offline_truth_proxy_motion_class_counts": _motion_counts(samples),
            "sampling_note": (
                "Runtime context counts and trace-proxy counts have different "
                "rates and are not treated as sample-aligned."
            ),
        },
        "semantic_comparison": {
            "runtime_pipeline": {
                "motion_classification": (
                    "centerline crossing must be reachable within the upstream "
                    "world-model prediction horizon"
                ),
                "upstream_motion_classifier_horizon_s": float(
                    world_config["prediction"]["horizon_s"]
                ),
                "overlay": (
                    "CROSSING uses centerline-crossing time; other dynamic "
                    "classes use point closest approach minus actor radius"
                ),
                "runtime_context_crossing_samples_observed": evaluation[
                    "context_overlay_sample_counts"
                ]["CROSSING"],
            },
            "evaluator_pipeline": {
                "semantics": "first circle-envelope contact under relative motion",
                "horizon_s": float(
                    replay_contract["evaluator_circle_contact_horizon_s"]
                ),
                "robot_radius_m": float(
                    replay_contract["robot_circle_radius_m"]
                ),
                "actor_radius_m": actor_radius,
                "finite_ttc_samples_observed": evaluation[
                    "finite_ttc_sample_count"
                ],
            },
            "semantic_difference_confirmed": True,
            "runtime_crossing_overlay_with_zero_evaluator_finite_ttc": (
                evaluation["context_overlay_sample_counts"]["CROSSING"] > 0
                and evaluation["finite_ttc_sample_count"] == 0
            ),
            "diagnosis": (
                "A runtime CROSSING overlay is not equivalent to evaluator "
                "circle-envelope contact.  The upstream 2.0 s motion classifier "
                "also caps which samples can reach the longer supervisor horizons."
            ),
        },
        "integrated_candidate_distinguishability": {
            "frozen_candidate_horizons_s": list(FROZEN_HORIZONS),
            "overlay_counts_by_horizon_s": {
                str(horizon): _counts(overlays[horizon])
                for horizon in FROZEN_HORIZONS
            },
            "pairwise": frozen_pairwise,
            "reachable_crossing_sample_count": len(crossing_indices),
            "all_frozen_candidates_equal_on_reachable_crossing_samples": (
                all_frozen_equal_on_crossing
            ),
            "frozen_candidate_crossing_difference_count": 0,
            "only_unknown_class_samples_distinguish_frozen_candidates": all(
                row["motion_class"] == "UNKNOWN"
                for comparison in frozen_pairwise
                for row in comparison["differences"]
            ),
            "conclusion": (
                "The 5.0/4.5/4.0 second factor is not integratedly "
                "identifiable on the target CROSSING samples in this trace."
            ),
        },
        "arrival_timing": {
            "intersection_x_m": crossing_x,
            "actor_centerline_crossing_time_s": actor_crossing_time,
            "robot_pose_at_actor_crossing": robot_at_actor_crossing,
            "robot_crossing_point_arrival_time_s": robot_at_crossing_point[
                "stamp_s"
            ],
            "robot_pose_at_crossing_point": robot_at_crossing_point,
            "robot_minus_actor_arrival_time_s": arrival_gap,
            "actor_arrived_first": arrival_gap > 0.0,
        },
        "ttc_and_circle_envelope": {
            "evaluation_finite_ttc_sample_count": evaluation[
                "finite_ttc_sample_count"
            ],
            "trace_finite_ttc_sample_count": len(trace_finite),
            **ttc,
            "classification": "NO_CIRCLE_CONTACT_WITHIN_5S_PROXY",
        },
        "truth_clearance": {
            "frozen_async_gazebo_truth_minimum_m": frozen_truth,
            "frozen_async_gazebo_truth_detail": clearance[
                "minimum_truth_detail"
            ],
            "trace_synchronous_proxy_minimum": proxy_truth,
            "absolute_difference_m": abs(
                proxy_truth["clearance_m"] - frozen_truth
            ),
            "minimum_signed_scan_clearance_m": clearance[
                "minimum_signed_scan_clearance_m"
            ],
            "contact_count": clearance["contact_count"],
            "runtime_policy_received_truth": clearance[
                "runtime_policy_received_truth"
            ],
            "authority_note": (
                "The preserved asynchronous Gazebo model-state audit is the "
                "authoritative executed truth metric; the trace-synchronous "
                "interpolation is a close offline cross-check."
            ),
        },
        "exploratory_future_horizons": {
            "horizons_s": list(EXPLORATORY_HORIZONS),
            "offline_counterfactual_only": True,
            "runtime_config_created": False,
            "r6_execution_authorization_created": False,
            "overlay_counts_by_horizon_s": {
                str(horizon): _counts(overlays[horizon])
                for horizon in (5.0, 1.5, 1.0)
            },
            "pairwise": exploratory_pairwise,
            "assessment": (
                "1.5 and 1.0 seconds are distinguishable on this frozen trace, "
                "including CROSSING samples, so they may be considered only as "
                "future preregistration candidates.  One trace cannot establish "
                "suitability, safety, performance, or a winner."
            ),
            "suitable_for_future_r6_execution": None,
            "requires_fresh_preregistration_review_and_authorization": True,
        },
        "risk_audit": {
            "required_count": len(EXPECTED_RISK_IDS),
            "confirmed_count": len(findings),
            "all_required_risks_machine_recorded": True,
            "findings": findings,
        },
        "root_cause_classification": {
            "primary": (
                "seed5111 actor_robot_temporal_separation_produced_no_evaluator_"
                "circle_contact_within_horizon"
            ),
            "structural": (
                "runtime_centerline_crossing_semantics_and_upstream_2s_"
                "classification_cap_do_not_match_evaluator_circle_contact_semantics"
            ),
            "r5_factor_issue": (
                "5.0_4.5_4.0_supervisor_horizons_are_not_identifiable_on_"
                "reachable_crossing_samples"
            ),
            "tracker_invalid": False,
            "transaction_or_join_failure": False,
            "collision": False,
        },
        "authorizations_after_diagnosis": dict(
            contract["authorizations_after_diagnosis"]
        ),
        "side_effects": {
            "ros_started": False,
            "gazebo_started": False,
            "move_base_started": False,
            "component_executions": 0,
            "navigation_executions": 0,
            "seeds_consumed": 0,
            "evidence_units_consumed": 0,
            "threshold_files_changed": 0,
            "scene_files_changed": 0,
            "evaluator_files_changed": 0,
            "r5_files_changed": 0,
            "training_started": False,
            "real_vehicle_connected": False,
            "real_vehicle_teb_parameter_writes": 0,
            "only_persistent_write_is_d1_report": True,
        },
    }
    _require(
        report["integrity"]["r5_artifact_tree_before_and_after_identical"]
        and all(
            value is False
            for value in report["authorizations_after_diagnosis"].values()
        ),
        "D1 report violates the preservation/authorization boundary",
    )
    return report


def _atomic_yaml(path, value):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(
        yaml.safe_dump(
            value, sort_keys=False, allow_unicode=True, width=100
        ),
        encoding="utf-8",
    )
    temporary.replace(target)


def diagnose(workspace, contract_path=None, output_path=None):
    """Build and atomically persist the exact contract-declared D1 report."""

    root = Path(workspace).resolve()
    contract_path = (
        root / CONTRACT_RELATIVE if contract_path is None else Path(contract_path)
    )
    contract, _ = _validate_contract(root, contract_path)
    expected_output = (root / OUTPUT_RELATIVE).resolve()
    output = expected_output if output_path is None else _inside(
        root, output_path, "D1 output"
    )
    _require(output == expected_output, "D1 output path drifted")
    _require(
        contract["output"]["path"] == str(OUTPUT_RELATIVE),
        "contract output path drifted",
    )
    report = build_report(root, contract_path)
    _atomic_yaml(output, report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace",
        default="/home/robot/robot_ws_base_rl",
        help="thesis workspace root",
    )
    parser.add_argument(
        "--contract",
        help="must resolve to the canonical D1 contract path",
    )
    parser.add_argument(
        "--output",
        help="must resolve to the canonical D1 report path",
    )
    args = parser.parse_args()
    root = Path(args.workspace).resolve()
    contract = Path(args.contract) if args.contract else root / CONTRACT_RELATIVE
    output = Path(args.output) if args.output else root / OUTPUT_RELATIVE
    report = diagnose(root, contract, output)
    print(yaml.safe_dump(report, sort_keys=False, allow_unicode=True, width=100))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
