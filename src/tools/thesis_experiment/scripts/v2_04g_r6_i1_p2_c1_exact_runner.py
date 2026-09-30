#!/usr/bin/env python3
"""Execute the unique qualification-first P2-C1 Gazebo release.

The runner reuses the terminally proven P1 process/bootstrap primitives, while
binding C1 identities, Q1 raw trace adapters, candidate configs, and the
qualification-first monotone speed gate.  It is serial, non-resumable, and
stops on the first execution-integrity failure.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys
import time
from datetime import datetime, timezone
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SOURCE_ROOT = WORKSPACE / "src/tools/thesis_experiment/src"
DEVEL_PYTHON = WORKSPACE / "devel/lib/python3/dist-packages"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))
if str(DEVEL_PYTHON) not in sys.path:
    sys.path.insert(1, str(DEVEL_PYTHON))

from thesis_experiment.v2_04g_r6_i1_p2_c1_release import (  # noqa: E402
    C1ReleaseError,
    validate_release,
)


STAGE = "V2-04G-P2-C1"
OUTPUT_ROOT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_execution"
)
OUTPUT_ROOT = WORKSPACE / OUTPUT_ROOT_RELATIVE
ATTEMPTS_ROOT = OUTPUT_ROOT / "attempts"
JOURNAL_ROOT = OUTPUT_ROOT / "journals"
CONSUMPTION_ROOT = OUTPUT_ROOT / "consumption"
STAGE_REPORT = OUTPUT_ROOT / "p2_c1_stage_report.yaml"
SKIPPED_LEDGER = OUTPUT_ROOT / "p2_c1_skipped_identities.yaml"
FAILURE_INDEX = OUTPUT_ROOT / "p2_c1_failure_index.yaml"
NORMALIZED_MATRIX = OUTPUT_ROOT / "normalized_calibration_matrix.json"
REPLAY_REPORT = OUTPUT_ROOT / "p2_c1_raw_replay_report.yaml"
ASSESSMENT_REPORT = OUTPUT_ROOT / "p2_c1_calibration_selection.yaml"

METHODS_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_offline_readiness/"
    "v2_04g_p2_q1_method_bindings.yaml"
)
D2_CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_d2_offline_single_factor_design_contract.yaml"
)
Q1_CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_q1_offline_implementation_readiness_contract.yaml"
)
LAUNCH_RELATIVE = Path(
    "src/simulation/m2_gazebo/launch/m2_v2_04g_p2_q1_execution.launch"
)
NAVIGATION_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_navigation_episode.py"
)
QUALIFICATION_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_qualification_probe.py"
)
CONTROL_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_runtime_control.py"
)
LISTENER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_activation_probe_listener.py"
)
FIXED_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_fixed_snapshot.py"
)
ASSESSOR_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_assess.py"
)
P1_RUNNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p1_e1_exact_runner.py"
)

CIRCLE_METHODS = {
    "circle_contact_v120",
    "circle_contact_v125",
    "circle_contact_v130",
    "circle_contact_v135",
    "circle_contact_v140",
}
FIXED_METHODS = {
    "fixed_teb_v120__TEB-Default_alias",
    "fixed_teb_v125",
    "fixed_teb_v130",
    "fixed_teb_v135",
    "fixed_teb_v140",
    "TEB-Tuned-V1",
}
RAW_FILENAMES = {
    "activation": "activation.yaml",
    "evaluation": "evaluation.yaml",
    "legacy_trace": "trace.csv",
    "clearance": "clearance.yaml",
    "overlay": "overlay_trace.yaml",
    "raw_trace": "raw_trace.yaml",
    "teardown_receipt": "teardown_receipt.yaml",
    "process_log": "process.log",
}


class C1ExecutionError(RuntimeError):
    """Terminal C1 execution-integrity failure."""


def _load_module(name: str, relative: Path):
    path = WORKSPACE / relative
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise C1ExecutionError("cannot load {}".format(relative))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


P1 = _load_module("_p2_c1_p1_execution_primitives", P1_RUNNER_RELATIVE)
ASSESSOR = _load_module("_p2_c1_assessor", ASSESSOR_RELATIVE)

METHOD_BINDINGS: Dict[str, Mapping[str, Any]] = {}
THRESHOLDS: Mapping[str, Any] = {}


def _utc_now():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _profile_id(row: Mapping[str, Any]) -> str:
    if row["row_type"] == "calibration_navigation":
        return row["method_id"]
    return "qualification_{}_v{:03d}".format(
        row["probe_type"].replace("high_speed_to_conflict_overlay_transition", "overlay"),
        int(round(float(row["candidate_speed_mps"]) * 100.0)),
    )


def _method_for_row(row: Mapping[str, Any]) -> Mapping[str, Any]:
    method_id = (
        row["method_id"]
        if row["row_type"] == "calibration_navigation"
        else row["candidate_method_id"]
    )
    try:
        return METHOD_BINDINGS[method_id]
    except KeyError as exc:
        raise C1ExecutionError("method binding absent: {}".format(method_id)) from exc


def _semantic(row: Mapping[str, Any]) -> bool:
    return (
        row["row_type"] == "calibration_navigation"
        and row["method_id"] in CIRCLE_METHODS
    ) or (
        row["row_type"] == "qualification_probe"
        and row["probe_type"] == "high_speed_to_conflict_overlay_transition"
    )


def _identity(row):
    return ASSESSOR.identity(row)


def _attempt_name(row):
    discriminator = (
        row["method_id"]
        if row["row_type"] == "calibration_navigation"
        else "{}_v{:03d}".format(
            row["probe_type"], int(round(row["candidate_speed_mps"] * 100.0))
        )
    )
    return "{:04d}__{}__{}".format(
        row["sequence"], discriminator, row["scene_id"]
    )


def _runtime(row, closure):
    if not _semantic(row):
        return None
    method = _method_for_row(row)
    runtime = {}
    for label, binding_key in (
        ("supervisor", "supervisor"),
        ("anchor_bank", "materialized_anchor_bank"),
        ("mechanism", "mechanism"),
    ):
        binding = method[binding_key]
        relative = Path(binding["path"])
        payload = P1._rehash_relative(relative, closure)
        if _sha(payload) != binding["sha256"]:
            raise C1ExecutionError("{} method binding differs".format(label))
        runtime[label] = str(WORKSPACE / relative)
        runtime[label + "_sha256"] = binding["sha256"]
    return runtime


def _fixed_config(row, closure):
    method = _method_for_row(row)
    binding = (
        method["fixed_startup_config"]
        if _semantic(row)
        else method["materialized_config"]
    )
    payload = P1._rehash_relative(Path(binding["path"]), closure)
    if _sha(payload) != binding["sha256"]:
        raise C1ExecutionError("fixed startup config differs")
    return str(WORKSPACE / binding["path"])


def _base_command(row, scene, world_path, runtime, executables):
    semantic = _semantic(row)
    start_move_base = (
        row["row_type"] == "calibration_navigation"
        or row.get("probe_type")
        == "high_speed_to_conflict_overlay_transition"
    )
    command = [
        executables["roslaunch"],
        "m2_gazebo",
        LAUNCH_RELATIVE.name,
        "world:={}".format(world_path),
        "seed:={}".format(row["seed"]),
        "xacro_executable:={}".format(executables["xacro"]),
        "x:={}".format(scene["start"]["x_m"]),
        "y:={}".format(scene["start"]["y_m"]),
        "yaw:={}".format(scene["start"]["yaw_rad"]),
        "gui:=false",
        "paused:=true",
        "attempt_stage:={}".format(STAGE),
        "attempt_profile_id:={}".format(_profile_id(row)),
        "attempt_scene_id:={}".format(row["scene_id"]),
        "attempt_number:=1",
        "fixed_teb_config:={}".format(_fixed_config(row, _CURRENT_CLOSURE)),
        "start_move_base:={}".format("true" if start_move_base else "false"),
        "start_world_model:={}".format("true" if start_move_base else "false"),
        "start_rule_supervisor:={}".format("true" if semantic else "false"),
        "start_typed_transaction:=false",
        "allow_simulation_teb_parameter_write:=false",
        "allow_unfrozen_simulation_candidate:=true",
        "load_balanced_anchor:=false",
        "publish_teb_obstacles:={}".format("true" if semantic else "false"),
    ]
    if semantic:
        command.extend(
            [
                "rule_supervisor_config:={}".format(runtime["supervisor"]),
                "rule_supervisor_config_sha256:={}".format(
                    runtime["supervisor_sha256"]
                ),
                "anchor_bank:={}".format(runtime["anchor_bank"]),
                "mechanism_config:={}".format(runtime["mechanism"]),
            ]
        )
    return command


def _transaction_command(row, runtime, executables):
    return [
        executables["roslaunch"],
        "teb_mode_manager",
        "v2_04g_r6_simulation_typed_anchor.launch",
        "allow_simulation_teb_parameter_write:=true",
        "allow_unfrozen_simulation_candidate:=true",
        "anchor_bank:={}".format(runtime["anchor_bank"]),
        "mechanism_config:={}".format(runtime["mechanism"]),
        "attempt_stage:={}".format(STAGE),
        "attempt_profile_id:={}".format(_profile_id(row)),
        "attempt_scene_id:={}".format(row["scene_id"]),
        "attempt_seed:={}".format(row["seed"]),
        "attempt_number:=1",
        "supervisor_config_sha256:={}".format(runtime["supervisor_sha256"]),
    ]


def _control_command(mode, row, output, runtime, executables):
    command = [
        executables["python"],
        str(WORKSPACE / CONTROL_RELATIVE),
        "--mode",
        mode,
        "--output",
        str(output),
        "--stage",
        STAGE,
        "--profile-id",
        _profile_id(row),
        "--scene-id",
        row["scene_id"],
        "--seed",
        str(row["seed"]),
        "--attempt",
        "1",
    ]
    if mode == "initial-readback":
        command.extend(["--anchor-bank", runtime["anchor_bank"]])
    return command


def _listener_command(row, output, executables):
    gate = P1.READINESS_GATE
    return [
        executables["python"],
        str(WORKSPACE / LISTENER_RELATIVE),
        "--output",
        str(output),
        "--profile-id",
        _profile_id(row),
        "--scene-id",
        row["scene_id"],
        "--attempt",
        "1",
        "--repeat",
        str(row["sequence"]),
        "--seed",
        str(row["seed"]),
        "--warmup-timeout-s",
        str(gate["warmup_timeout_s"]),
        "--measurement-duration-s",
        str(gate["measurement_duration_s"]),
        "--minimum-message-count",
        str(gate["minimum_message_count_per_stream"]),
        "--minimum-valid-fraction",
        str(gate["minimum_valid_fraction"]),
        "--required-consecutive-stable-count",
        str(gate["required_consecutive_stable_count"]),
        "--maximum-expected-context-hold-count",
        str(gate["maximum_expected_context_hold_count_per_probe"]),
    ]


def _episode_command(row, instance_path, instance_sha256, output, executables):
    return [
        executables["python"],
        str(WORKSPACE / NAVIGATION_RELATIVE),
        "--instance",
        str(instance_path),
        "--instance-sha256",
        instance_sha256,
        "--method-id",
        row["method_id"],
        "--output-dir",
        str(output),
        "--stage",
        STAGE,
        "--sequence",
        str(row["sequence"]),
        "--attempt",
        "1",
    ]


def _probe_command(row, output, executables):
    return [
        executables["python"],
        str(WORKSPACE / QUALIFICATION_RELATIVE),
        "--probe",
        row["probe_type"],
        "--candidate-speed-mps",
        str(row["candidate_speed_mps"]),
        "--sequence",
        str(row["sequence"]),
        "--scene-id",
        row["scene_id"],
        "--seed",
        str(row["seed"]),
        "--output",
        str(output),
    ]


def _fixed_snapshot_command(mode, row, output, baseline, executables):
    command = [
        executables["python"],
        str(WORKSPACE / FIXED_RELATIVE),
        "--mode",
        mode,
        "--output",
        str(output),
        "--stage",
        STAGE,
        "--profile-id",
        row["method_id"],
        "--scene-id",
        row["scene_id"],
        "--seed",
        str(row["seed"]),
        "--attempt",
        "1",
    ]
    if baseline is not None:
        command.extend(["--baseline", str(baseline)])
    return command


def _snapshot_scene(row, work, closure):
    instance_payload = P1._rehash_relative(Path(row["instance"]["path"]), closure)
    world_payload = P1._rehash_relative(Path(row["world"]["path"]), closure)
    if (
        _sha(instance_payload) != row["instance"]["sha256"]
        or _sha(world_payload) != row["world"]["sha256"]
    ):
        raise C1ExecutionError("scheduled scene binding differs")
    snapshot = work / "scene_snapshot"
    snapshot.mkdir(parents=True, exist_ok=False)
    instance_path = snapshot / Path(row["instance"]["path"]).name
    world_path = snapshot / Path(row["world"]["path"]).name
    P1._exclusive_bytes(instance_path, instance_payload)
    P1._exclusive_bytes(world_path, world_payload)
    instance = yaml.safe_load(instance_payload)
    scene = instance.get("scene", {})
    if not (
        scene.get("scene_id") == row["scene_id"]
        and scene.get("seed") == row["seed"]
        and scene.get("split") == "calibration"
        and float(scene.get("timeout_s")) == float(row["timeout_s"])
    ):
        raise C1ExecutionError("scheduled scene identity differs")
    return instance, instance_path, world_path


def _populate_navigation_raw(work, raw, teardown_path):
    raw.mkdir(parents=True, exist_ok=False)
    sources = {
        "activation": work / "activation.yaml",
        "evaluation": work / "episode/evaluation.yaml",
        "legacy_trace": work / "episode/trace.csv",
        "clearance": work / "episode/clearance_audit.yaml",
        "overlay": work / "episode/overlay_trace.yaml",
        "raw_trace": work / "episode/raw_trace.yaml",
        "teardown_receipt": teardown_path,
    }
    for label, source in sources.items():
        if not source.is_file():
            raise C1ExecutionError("navigation raw resource absent: {}".format(label))
        P1._exclusive_bytes(raw / RAW_FILENAMES[label], source.read_bytes())
    P1._combine_logs(work, raw / RAW_FILENAMES["process_log"])
    receipts = {}
    for label, filename in RAW_FILENAMES.items():
        path = raw / filename
        payload = path.read_bytes()
        receipts[label] = {
            "path": str(path.relative_to(WORKSPACE)),
            "sha256": _sha(payload),
            "size_bytes": len(payload),
        }
    return receipts


def _validate_semantic_runtime(row, work, initial, runtime):
    activation = P1._load_yaml(work / "activation.yaml")
    evaluation = P1._load_yaml(work / "episode/evaluation.yaml")
    teardown = P1._load_yaml(work / "teardown_receipt.yaml")
    if activation.get("all_hard_gates_pass") is not True:
        raise C1ExecutionError("semantic activation gate failed")
    if not (
        evaluation.get("stage") == STAGE
        and evaluation.get("scene_id") == row["scene_id"]
        and evaluation.get("seed") == row["seed"]
        and evaluation.get("training_used") is False
        and evaluation.get("runtime_policy_manifest_access") is False
        and evaluation.get("runtime_scene_labels_available") is False
        and evaluation.get("typed_transaction_valid") is True
    ):
        raise C1ExecutionError("semantic episode interface gate failed")
    if not (
        teardown.get("service_response_success") is True
        and teardown.get("startup_profile_sha256")
        == initial["startup_profile_sha256"]
        and teardown.get("supervisor_config_sha256")
        == runtime["supervisor_sha256"]
    ):
        raise C1ExecutionError("semantic teardown provenance differs")


def _replay_navigation(workspace, row, journal_relative, release_sha256):
    return ASSESSOR.replay_one(
        workspace,
        row,
        workspace / journal_relative,
        release_sha256,
        THRESHOLDS,
    )


def _configure_p1_primitives():
    P1.STAGE = STAGE
    P1.OUTPUT_ROOT_RELATIVE = OUTPUT_ROOT_RELATIVE
    P1.OUTPUT_ROOT = OUTPUT_ROOT
    P1.ATTEMPTS_ROOT = ATTEMPTS_ROOT
    P1.JOURNAL_ROOT = JOURNAL_ROOT
    P1.CONSUMPTION_ROOT = CONSUMPTION_ROOT
    P1.STAGE_REPORT = STAGE_REPORT
    P1.LAUNCH_RELATIVE = LAUNCH_RELATIVE
    P1.EPISODE_RELATIVE = NAVIGATION_RELATIVE
    P1.CONTROL_RELATIVE = CONTROL_RELATIVE
    P1.LISTENER_RELATIVE = LISTENER_RELATIVE
    P1.FIXED_SNAPSHOT_RELATIVE = FIXED_RELATIVE
    P1.SEMANTIC_METHODS = set(CIRCLE_METHODS)
    P1.RAW_FILENAMES = dict(RAW_FILENAMES)
    P1._identity = _identity
    P1._attempt_name = _attempt_name
    P1._runtime = _runtime
    P1._base_command = _base_command
    P1._transaction_command = _transaction_command
    P1._control_command = _control_command
    P1._listener_command = _listener_command
    P1._episode_command = _episode_command
    P1._fixed_snapshot_command = _fixed_snapshot_command
    P1._snapshot_scene = _snapshot_scene
    P1._populate_raw = _populate_navigation_raw
    P1._validate_semantic_runtime = _validate_semantic_runtime
    P1.replay_one = _replay_navigation


def _bootstrap_direct(base, environment, work, executables, source_environment):
    barrier = P1.R6I2PositiveClockBarrier(timeout_s=45.0)
    spawned = time.monotonic()
    barrier.mark_base_spawned(spawned)
    P1._wait_service(
        base,
        environment,
        "/gazebo/unpause_physics",
        30.0,
        executables,
        source_environment,
    )
    requested = time.monotonic()
    barrier.observe_base_exit(base["process"].poll(), requested)
    barrier.mark_unpause_requested(requested)
    returncode, _ = P1._capture(
        [executables["rosservice"], "call", "/gazebo/unpause_physics"],
        environment,
        10.0,
        source_environment,
        work / "unpause.log",
    )
    acknowledged = time.monotonic()
    barrier.observe_base_exit(base["process"].poll(), acknowledged)
    barrier.mark_unpause_acknowledged(acknowledged, returncode == 0)
    while not barrier.receipt()["clock_progression_sample"]:
        now = time.monotonic()
        barrier.observe_base_exit(base["process"].poll(), now)
        barrier.check_deadline(now)
        try:
            result, payload = P1._capture(
                [executables["rostopic"], "echo", "-n", "1", "/clock"],
                environment,
                2.0,
                source_environment,
            )
        except P1.P1ExecutionError:
            continue
        if result != 0:
            continue
        sec, nsec = P1._clock_sample(payload)
        barrier.observe_clock(sec, nsec, time.monotonic())
    barrier.release_service_wait(time.monotonic(), base["process"].poll())
    receipt = {
        "schema_version": "1.0",
        "stage": STAGE,
        "required_order": [
            "base_spawn",
            "unpause_request",
            "successful_unpause_ack",
            "two_strictly_increasing_positive_post_ack_clock_samples",
            "release_direct_probe",
        ],
        "barrier": barrier.receipt(),
        "move_base_service_intentionally_absent": True,
    }
    P1._exclusive_bytes(work / "bootstrap_receipt.yaml", _yaml_bytes(receipt))
    return receipt


def _write_structural_record(path, row, record_type):
    document = {
        "schema_version": "1.0",
        "stage": STAGE,
        "record_type": record_type,
        **_identity(row),
        "supervisor_started": False,
        "typed_transaction_started": False,
        "teb_parameter_write_performed": False,
    }
    P1._exclusive_bytes(path, _yaml_bytes(document))


def _populate_qualification_raw(work, raw, teardown_path):
    raw.mkdir(parents=True, exist_ok=False)
    sources = {
        "activation": work / "activation.yaml",
        "raw_trace": work / "raw_trace.yaml",
        "teardown_receipt": teardown_path,
    }
    filenames = {
        "activation": "activation.yaml",
        "raw_trace": "raw_trace.yaml",
        "teardown_receipt": "teardown_receipt.yaml",
        "process_log": "process.log",
    }
    for label, source in sources.items():
        if not source.is_file():
            raise C1ExecutionError("qualification raw resource absent: {}".format(label))
        P1._exclusive_bytes(raw / filenames[label], source.read_bytes())
    P1._combine_logs(work, raw / filenames["process_log"])
    receipts = {}
    for label, filename in filenames.items():
        path = raw / filename
        payload = path.read_bytes()
        receipts[label] = {
            "path": str(path.relative_to(WORKSPACE)),
            "sha256": _sha(payload),
            "size_bytes": len(payload),
        }
    return receipts


def _commit_consumption(row, validation, report):
    document = {
        "schema_version": "1.0",
        "stage": STAGE,
        "release_sha256": validation["release_sha256"],
        "identity": _identity(row),
        "consumed": True,
        "consumption_boundary": "base_roslaunch_spawn_requested",
        "committed_at_utc": _utc_now(),
    }
    P1._exclusive_bytes(
        CONSUMPTION_ROOT / "sequence_{:04d}.yaml".format(row["sequence"]),
        _yaml_bytes(document),
    )
    report["evidence_units_consumed"] += 1
    report["attempt_ledger"].append(
        {
            "identity": _identity(row),
            "status": "base_roslaunch_spawn_request_committed",
        }
    )
    P1._atomic_yaml(STAGE_REPORT, report)


def _run_qualification(
    row,
    validation,
    report,
    source_environment,
    executables,
):
    closure = {
        receipt["path"]: receipt["sha256"]
        for receipt in validation["closure"]["local_resources"]
    }
    for relative in (
        LAUNCH_RELATIVE,
        QUALIFICATION_RELATIVE,
        CONTROL_RELATIVE,
        LISTENER_RELATIVE,
    ):
        P1._rehash_relative(relative, closure)
    runtime = _runtime(row, closure)
    attempt_root = ATTEMPTS_ROOT / _attempt_name(row)
    work = attempt_root / "work"
    raw = attempt_root / "raw"
    attempt_root.mkdir(parents=True, exist_ok=False)
    work.mkdir()
    instance, _, world_path = _snapshot_scene(row, work, closure)
    environment, environment_audit = P1._command_environment(
        work, source_environment
    )
    handles = {"base": None, "transaction": None, "listener": None, "episode": None}
    teardown_path = work / "teardown_receipt.yaml"
    initial = None
    restored = False
    failure = None
    exit_codes = {}
    _commit_consumption(row, validation, report)
    report["attempt_ledger"][-1]["environment_policy"] = environment_audit
    try:
        handles["base"] = P1._spawn(
            _base_command(row, instance["scene"], world_path, runtime, executables),
            environment,
            work / "base_launch.log",
            source_environment,
        )
        if _semantic(row):
            bootstrap = P1._bootstrap(
                handles["base"],
                environment,
                work,
                executables,
                source_environment,
            )
            initial_path = work / "initial_readback.yaml"
            P1._run(
                _control_command(
                    "initial-readback", row, initial_path, runtime, executables
                ),
                environment,
                work / "initial_readback.log",
                25.0,
                source_environment,
            )
            initial = P1._load_yaml(initial_path)
            handles["transaction"] = P1._spawn(
                _transaction_command(row, runtime, executables),
                environment,
                work / "transaction_launch.log",
                source_environment,
            )
            startup_path = work / "transaction_startup.yaml"
            P1._run(
                _control_command(
                    "transaction-startup", row, startup_path, runtime, executables
                ),
                environment,
                work / "transaction_startup.log",
                30.0,
                source_environment,
            )
            startup = P1._load_yaml(startup_path)
            if not (
                startup.get("startup_profile_sha256")
                == initial["startup_profile_sha256"]
                and startup.get("supervisor_config_sha256")
                == runtime["supervisor_sha256"]
            ):
                raise C1ExecutionError("qualification startup provenance differs")
            arm_path = work / "arm_receipt.yaml"
            P1._run(
                _control_command("arm", row, arm_path, runtime, executables),
                environment,
                work / "arm.log",
                20.0,
                source_environment,
            )
            arm = P1._load_yaml(arm_path)
            if arm.get("execution_armed") is not True:
                raise C1ExecutionError("qualification transaction arm failed")
            P1._wait_action_trace(
                handles["base"],
                handles["transaction"],
                environment,
                work,
                executables,
                source_environment,
            )
            handles["listener"] = P1._spawn(
                _listener_command(row, work / "activation.yaml", executables),
                environment,
                work / "listener.log",
                source_environment,
            )
        else:
            bootstrap = _bootstrap_direct(
                handles["base"],
                environment,
                work,
                executables,
                source_environment,
            )
            _write_structural_record(
                work / "activation.yaml",
                row,
                "direct_probe_structural_activation",
            )
        report["attempt_ledger"][-1]["bootstrap"] = bootstrap
        handles["episode"] = P1._spawn(
            _probe_command(row, work / "raw_trace.yaml", executables),
            environment,
            work / "probe.log",
            source_environment,
        )
        P1._wait_episode(handles, 240.0)
        exit_codes["probe"] = handles["episode"]["process"].returncode
        if handles["listener"] is not None:
            exit_codes["listener"] = handles["listener"]["process"].returncode
        if _semantic(row):
            P1._run(
                _control_command(
                    "restore", row, teardown_path, runtime, executables
                ),
                environment,
                work / "restore.log",
                25.0,
                source_environment,
            )
            restored = True
            activation = P1._load_yaml(work / "activation.yaml")
            teardown = P1._load_yaml(teardown_path)
            if not (
                activation.get("all_hard_gates_pass") is True
                and teardown.get("service_response_success") is True
                and teardown.get("startup_profile_sha256")
                == initial["startup_profile_sha256"]
                and teardown.get("supervisor_config_sha256")
                == runtime["supervisor_sha256"]
            ):
                raise C1ExecutionError("qualification semantic integrity gate failed")
        else:
            _write_structural_record(
                teardown_path, row, "direct_probe_structural_teardown"
            )
            restored = True
    except BaseException as exc:
        failure = exc
    finally:
        if (
            failure is not None
            and _semantic(row)
            and handles["transaction"] is not None
            and handles["transaction"]["process"].poll() is None
            and initial is not None
            and not restored
        ):
            try:
                P1._run(
                    _control_command(
                        "restore", row, teardown_path, runtime, executables
                    ),
                    environment,
                    work / "restore_after_failure.log",
                    25.0,
                    source_environment,
                )
                restored = True
            except BaseException as restore_exc:
                failure = C1ExecutionError(
                    "{}; teardown also failed: {}".format(failure, restore_exc)
                )
        for label in ("episode", "listener", "transaction", "base"):
            handle = handles[label]
            if handle is not None:
                try:
                    exit_codes.setdefault(label, P1._stop(handle))
                except BaseException as stop_exc:
                    failure = C1ExecutionError(
                        "{}; {} containment failed: {}".format(
                            failure or "attempt otherwise completed",
                            label,
                            stop_exc,
                        )
                    )
    if failure is not None:
        reason = P1._safe_reason(failure, source_environment)
        journal = P1._write_failure_journal(
            row, validation["release_sha256"], reason
        )
        report["attempt_ledger"][-1].update(
            {
                "status": "terminal_integrity_failure",
                "failure": reason,
                "journal": str(journal.relative_to(WORKSPACE)),
            }
        )
        raise C1ExecutionError(reason)
    raw_receipts = _populate_qualification_raw(work, raw, teardown_path)
    journal_document = {
        "schema_version": "1.0",
        "stage": STAGE,
        "release_sha256": validation["release_sha256"],
        "identity": _identity(row),
        "status": "completed",
        "consumed": True,
        "consumption_boundary": "base_roslaunch_spawn_requested",
        "raw_resources": raw_receipts,
        "integrity_failures": [],
        "process_exit_codes": exit_codes,
        "terminal_recorded_at_utc": _utc_now(),
    }
    journal_path = JOURNAL_ROOT / "sequence_{:04d}.yaml".format(row["sequence"])
    P1._exclusive_bytes(journal_path, _yaml_bytes(journal_document))
    normalized, audit = ASSESSOR.replay_one(
        WORKSPACE,
        row,
        journal_path,
        validation["release_sha256"],
        THRESHOLDS,
    )
    report["evidence_units_completed"] += 1
    report["attempt_ledger"][-1].update(
        {
            "status": "completed",
            "journal": str(journal_path.relative_to(WORKSPACE)),
            "journal_sha256": _sha(journal_path.read_bytes()),
            "normalized_terminal_sha256": audit["normalized_sha256"],
            "probe_pass": normalized["pass"],
        }
    )
    P1._atomic_yaml(STAGE_REPORT, report)
    P1._wait_host_clean()
    return normalized


def _speed_for_method(method):
    if method == "TEB-Tuned-V1":
        return 0.9
    return int(method.split("_v", 1)[1].split("_", 1)[0]) / 100.0


def _should_skip(row, disqualified_from):
    if disqualified_from is None:
        return False
    if row["row_type"] == "qualification_probe":
        return row["candidate_speed_mps"] > disqualified_from
    if row["method_id"] in CIRCLE_METHODS | (FIXED_METHODS - {"TEB-Tuned-V1"}):
        return _speed_for_method(row["method_id"]) >= disqualified_from
    return False


def _write_skipped(skipped):
    P1._atomic_yaml(
        SKIPPED_LEDGER,
        {
            "schema_version": "1.0",
            "stage": STAGE,
            "status": "qualification_gated_identity_ledger",
            "skipped_identities": skipped,
            "skipped_identity_consumes_evidence_budget": False,
        },
    )


def _write_failure_index(failures):
    P1._atomic_yaml(
        FAILURE_INDEX,
        {
            "schema_version": "1.0",
            "stage": STAGE,
            "status": "live_failure_index",
            "failure_count": len(failures),
            "failures": failures,
            "all_failures_retained": True,
        },
    )


def _record_skip(row, disqualified_from, skipped, report):
    record = {
        "sequence": row["sequence"],
        "identity": _identity(row),
        "reason": "qualification_speed_disqualified",
        "first_failed_speed_mps": disqualified_from,
        "consumed": False,
        "gazebo_spawned": False,
    }
    skipped.append(record)
    report["evidence_units_skipped"] += 1
    report["attempt_ledger"].append(
        {"identity": _identity(row), "status": "qualification_gated_not_executed"}
    )
    _write_skipped(skipped)
    P1._atomic_yaml(STAGE_REPORT, report)


def _build_terminal_assessment(validation):
    normalized, replay, assessment = ASSESSOR.build_assessment(
        WORKSPACE,
        validation["schedule"],
        JOURNAL_ROOT,
        SKIPPED_LEDGER,
        validation["release_sha256"],
        THRESHOLDS,
    )
    P1._exclusive_bytes(
        NORMALIZED_MATRIX,
        (
            json.dumps(
                normalized,
                sort_keys=True,
                separators=(",", ":"),
                ensure_ascii=False,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8"),
    )
    P1._exclusive_bytes(REPLAY_REPORT, _yaml_bytes(replay))
    P1._exclusive_bytes(ASSESSMENT_REPORT, _yaml_bytes(assessment))
    return replay, assessment


_CURRENT_CLOSURE: Mapping[str, str] = {}


def execute(expected_release_sha256):
    global METHOD_BINDINGS, THRESHOLDS, _CURRENT_CLOSURE
    source_environment = dict(os.environ)
    validation = validate_release(
        expected_release_sha256, WORKSPACE, require_fresh_state=True
    )
    P1._assert_host_clean()
    executables = P1._executables()
    _CURRENT_CLOSURE = {
        receipt["path"]: receipt["sha256"]
        for receipt in validation["closure"]["local_resources"]
    }
    for relative in (
        LAUNCH_RELATIVE,
        NAVIGATION_RELATIVE,
        QUALIFICATION_RELATIVE,
        CONTROL_RELATIVE,
        LISTENER_RELATIVE,
        FIXED_RELATIVE,
        ASSESSOR_RELATIVE,
        METHODS_RELATIVE,
        Q1_CONTRACT_RELATIVE,
    ):
        P1._rehash_relative(relative, _CURRENT_CLOSURE)
    methods = yaml.safe_load(
        P1._rehash_relative(METHODS_RELATIVE, _CURRENT_CLOSURE)
    )
    METHOD_BINDINGS = {
        row["method_id"]: row for row in methods["methods"]
    }
    q1_contract = yaml.safe_load(
        P1._rehash_relative(Q1_CONTRACT_RELATIVE, _CURRENT_CLOSURE)
    )
    THRESHOLDS = q1_contract["qualification_thresholds"]
    _configure_p1_primitives()

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    ATTEMPTS_ROOT.mkdir()
    JOURNAL_ROOT.mkdir()
    CONSUMPTION_ROOT.mkdir()
    skipped = []
    failures = []
    _write_skipped(skipped)
    _write_failure_index(failures)
    report = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "running_qualification_first_exact_342_max_schedule",
        "release_sha256": validation["release_sha256"],
        "schedule_sha256": validation["schedule_sha256"],
        "schedule_row_digest": validation["schedule"]["canonical_row_digest"],
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_authorized": True,
        "authorized_evidence_units_max": 342,
        "evidence_units_consumed": 0,
        "evidence_units_completed": 0,
        "evidence_units_skipped": 0,
        "evidence_units_forfeited": 0,
        "retry_resume_replacement_or_seed_substitution_allowed": False,
        "training_budget_steps": 0,
        "real_vehicle_used": False,
        "held_out_test_accessed": False,
        "started_at_utc": _utc_now(),
        "completed_at_utc": None,
        "first_failed_qualification_speed_mps": None,
        "attempt_ledger": [],
        "terminal_integrity_failures": [],
        "performance_improvement_demonstrated": False,
    }
    P1._atomic_yaml(STAGE_REPORT, report)
    stage_started = time.monotonic()
    disqualified_from = None
    current_speed_results = []
    for row in validation["schedule"]["rows"]:
        if time.monotonic() - stage_started > 20.0 * 3600.0:
            reason = "stage hard wall exceeded before sequence {}".format(
                row["sequence"]
            )
            report["status"] = "terminal_integrity_failure"
            report["terminal_integrity_failures"].append(reason)
            report["evidence_units_forfeited"] = (
                342
                - report["evidence_units_consumed"]
                - report["evidence_units_skipped"]
            )
            report["completed_at_utc"] = _utc_now()
            P1._atomic_yaml(STAGE_REPORT, report)
            raise C1ExecutionError(reason)
        if _should_skip(row, disqualified_from):
            _record_skip(row, disqualified_from, skipped, report)
            continue
        try:
            if row["row_type"] == "qualification_probe":
                normalized = _run_qualification(
                    row,
                    validation,
                    report,
                    source_environment,
                    executables,
                )
                current_speed_results.append(normalized)
                if len(current_speed_results) == 3:
                    speed = row["candidate_speed_mps"]
                    if not all(result["pass"] for result in current_speed_results):
                        disqualified_from = speed
                        report["first_failed_qualification_speed_mps"] = speed
                    current_speed_results = []
            else:
                before = report["evidence_units_completed"]
                P1._run_attempt(
                    row,
                    validation,
                    report,
                    source_environment,
                    executables,
                )
                if report["evidence_units_completed"] != before + 1:
                    raise C1ExecutionError("navigation completion accounting differs")
                normalized, _ = ASSESSOR.replay_one(
                    WORKSPACE,
                    row,
                    JOURNAL_ROOT / "sequence_{:04d}.yaml".format(row["sequence"]),
                    validation["release_sha256"],
                    THRESHOLDS,
                )
                if (
                    not normalized["success"]
                    or normalized["collision"]
                    or normalized["emergency"]
                ):
                    failures.append(
                        {
                            "sequence": row["sequence"],
                            "identity": _identity(row),
                            "kind": "ordinary_navigation_outcome",
                            "success": normalized["success"],
                            "collision": normalized["collision"],
                            "emergency": normalized["emergency"],
                        }
                    )
                    _write_failure_index(failures)
            if row["row_type"] == "qualification_probe" and not normalized["pass"]:
                failures.append(
                    {
                        "sequence": row["sequence"],
                        "identity": _identity(row),
                        "kind": "ordinary_qualification_failure",
                        "probe_pass": False,
                    }
                )
                _write_failure_index(failures)
        except BaseException as exc:
            reason = P1._safe_reason(exc, source_environment)
            report["status"] = "terminal_integrity_failure"
            report["terminal_integrity_failures"].append(reason)
            report["evidence_units_forfeited"] = max(
                0,
                342
                - report["evidence_units_consumed"]
                - report["evidence_units_skipped"],
            )
            report["completed_at_utc"] = _utc_now()
            report["formal_result"] = False
            P1._atomic_yaml(STAGE_REPORT, report)
            failures.append(
                {
                    "sequence": row["sequence"],
                    "identity": _identity(row),
                    "kind": "terminal_integrity_failure",
                    "reason": reason,
                }
            )
            _write_failure_index(failures)
            P1._wait_host_clean()
            raise C1ExecutionError(reason) from None
    replay, assessment = _build_terminal_assessment(validation)
    report.update(
        {
            "status": "calibration_execution_raw_replay_and_selection_complete",
            "formal_result": False,
            "evidence_units_forfeited": 0,
            "completed_at_utc": _utc_now(),
            "raw_replay_status": replay["status"],
            "raw_replay_report_sha256": _sha(REPLAY_REPORT.read_bytes()),
            "calibration_selection_sha256": _sha(ASSESSMENT_REPORT.read_bytes()),
            "selected_circle_candidate": assessment[
                "selected_circle_candidate"
            ],
            "selected_primary_fair_baseline": assessment[
                "selected_primary_fair_baseline"
            ],
            "held_out_test_design_eligible": assessment[
                "held_out_test_design_eligible"
            ],
            "performance_improvement_demonstrated": False,
        }
    )
    P1._atomic_yaml(STAGE_REPORT, report)
    P1._assert_host_clean()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required; use the offline reviewer for validation")
    try:
        report = execute(args.release_sha256)
    except (C1ReleaseError, C1ExecutionError, P1.P1ExecutionError) as exc:
        print("P2-C1 terminal failure: {}".format(exc), file=sys.stderr)
        return 2
    print(
        yaml.safe_dump(
            {
                "status": report["status"],
                "consumed": report["evidence_units_consumed"],
                "completed": report["evidence_units_completed"],
                "skipped": report["evidence_units_skipped"],
                "selected_circle_candidate": report["selected_circle_candidate"],
                "selected_primary_fair_baseline": report[
                    "selected_primary_fair_baseline"
                ],
                "held_out_test_design_eligible": report[
                    "held_out_test_design_eligible"
                ],
            },
            sort_keys=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
