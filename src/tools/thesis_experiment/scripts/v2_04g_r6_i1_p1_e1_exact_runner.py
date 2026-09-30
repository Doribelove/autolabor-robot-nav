#!/usr/bin/env python3
"""Fail-closed, non-resumable runner for the exact 270-row P1-E1 schedule."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SOURCE_ROOT = WORKSPACE / "src/tools/thesis_experiment/src"
DEVEL_PYTHON = WORKSPACE / "devel/lib/python3/dist-packages"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))
if str(DEVEL_PYTHON) not in sys.path:
    sys.path.insert(1, str(DEVEL_PYTHON))

from thesis_experiment.v2_04g_r6_i1_p1_e1_release import (  # noqa: E402
    P1ReleaseError,
    read_workspace_file_once,
    validate_release,
)
from thesis_experiment.v2_04g_r6_i1_p1_e1_replay import (  # noqa: E402
    build_terminal_assessment,
    replay_one,
)
from thesis_experiment.v2_04g_r6_i1_r6_i2_bootstrap import (  # noqa: E402
    R6I2PositiveClockBarrier,
    assert_credential_safe_environment,
    build_credential_safe_environment,
    redact_credential_material,
    validate_credential_safe_command,
)


STAGE = "V2-04G-P1-E1"
OUTPUT_ROOT_RELATIVE = Path("artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution")
OUTPUT_ROOT = WORKSPACE / OUTPUT_ROOT_RELATIVE
ATTEMPTS_ROOT = OUTPUT_ROOT / "attempts"
JOURNAL_ROOT = OUTPUT_ROOT / "journals"
CONSUMPTION_ROOT = OUTPUT_ROOT / "consumption"
STAGE_REPORT = OUTPUT_ROOT / "p1_e1_stage_report.yaml"
SCHEDULE_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness/"
    "v2_04g_p1_r1_exact_schedule.yaml"
)
RUNTIME_ROOT = WORKSPACE / "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs"
LAUNCH_RELATIVE = Path("src/simulation/m2_gazebo/launch/m2_v2_04g_p1_e1_execution.launch")
EPISODE_RELATIVE = Path("src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_episode.py")
CONTROL_RELATIVE = Path("src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_runtime_control.py")
LISTENER_RELATIVE = Path("src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_activation_probe_listener.py")
FIXED_SNAPSHOT_RELATIVE = Path("src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_fixed_snapshot.py")
ASSESS_RELATIVE = Path("src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_assess.py")
RAW_FILENAMES = {
    "activation": "activation.yaml",
    "evaluation": "evaluation.yaml",
    "trace": "trace.csv",
    "clearance": "clearance.yaml",
    "overlay": "overlay_trace.yaml",
    "teardown_receipt": "teardown_receipt.yaml",
    "process_log": "process.log",
}
SEMANTIC_METHODS = {
    "r6_semantics_legacy_control",
    "r6_semantics_circle_contact",
}
READINESS_GATE = {
    "minimum_message_count_per_stream": 20,
    "warmup_timeout_s": 12.0,
    "measurement_duration_s": 6.0,
    "minimum_valid_fraction": 0.95,
    "required_consecutive_stable_count": 10,
    "maximum_expected_context_hold_count_per_probe": 3,
}
_CLOCK_SEC = re.compile(rb"(?:^|\n)\s*secs:\s*([0-9]+)\s*(?:\n|$)")
_CLOCK_NSEC = re.compile(rb"(?:^|\n)\s*nsecs:\s*([0-9]+)\s*(?:\n|$)")


class P1ExecutionError(RuntimeError):
    """Terminal execution-integrity failure; no retry or resume is allowed."""


def _utc_now():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha(payload):
    return hashlib.sha256(payload).hexdigest()


def _yaml_bytes(value):
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _exclusive_bytes(path, payload):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("xb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())


def _atomic_bytes(path, payload):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=target.name + ".tmp.", dir=str(target.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(target))
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def _atomic_yaml(path, value):
    _atomic_bytes(path, _yaml_bytes(value))


def _load_yaml(path):
    value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if type(value) is not dict:
        raise P1ExecutionError("YAML root must be a mapping: {}".format(path))
    return value


def _safe_reason(exc, source_environment):
    try:
        reason = redact_credential_material(str(exc), source_environment)
    except BaseException:
        reason = "diagnostic redaction failed closed"
    safe = "".join(
        character if character in {"\n", "\t"} or ord(character) >= 32
        else "\\x{:02x}".format(ord(character))
        for character in reason
    )
    return "{}: {}".format(type(exc).__name__, safe[:4096])


def _identity(row):
    return {
        key: row[key]
        for key in (
            "sequence", "method_id", "scene_id", "seed", "attempt",
            "block_id", "scene_role",
        )
    }


def _attempt_name(row):
    return "{:04d}__{}__{}".format(
        row["sequence"], row["method_id"], row["scene_id"]
    )


def _active_runtime_processes():
    names = {
        "roscore", "rosmaster", "roslaunch", "gzserver", "gzclient",
        "move_base", "spawn_model",
    }
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            comm = (entry / "comm").read_text(encoding="utf-8").strip()
            argv = (entry / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        executable_name = Path(argv[0].decode("utf-8", errors="replace")).name if argv and argv[0] else ""
        if comm in names or executable_name in names:
            found.append({"pid": int(entry.name), "name": comm or executable_name})
    return found


def _listening(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.15)
    try:
        return sock.connect_ex(("127.0.0.1", port)) == 0
    finally:
        sock.close()


def _assert_host_clean():
    active = _active_runtime_processes()
    if active:
        raise P1ExecutionError("ROS/Gazebo processes already active: {}".format(active))
    if _listening(11311) or _listening(11345):
        raise P1ExecutionError("ROS/Gazebo master port already listening")


def _wait_host_clean(timeout_s=30.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not _active_runtime_processes() and not _listening(11311) and not _listening(11345):
            return
        time.sleep(0.25)
    raise P1ExecutionError("ROS/Gazebo isolation did not become clean")


def _closure_map(validation):
    return {
        row["path"]: row["sha256"]
        for row in validation["closure"]["local_resources"]
    }


def _rehash_relative(relative, closure):
    declared = Path(relative).as_posix()
    expected = closure.get(declared)
    if expected is None:
        raise P1ExecutionError("runtime dependency absent from closure: {}".format(declared))
    payload = read_workspace_file_once(WORKSPACE, Path(declared))
    if _sha(payload) != expected:
        raise P1ExecutionError("runtime dependency drifted: {}".format(declared))
    return payload


def _command_environment(work, source_environment):
    environment, audit = build_credential_safe_environment(source_environment, work)
    assert_credential_safe_environment(environment, work)
    Path(environment["ROS_HOME"]).mkdir(parents=True, exist_ok=False)
    Path(environment["ROS_LOG_DIR"]).mkdir(parents=True, exist_ok=False)
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    return environment, audit


def _validated_command(command, source_environment):
    return list(validate_credential_safe_command(command, source_environment))


def _spawn(command, environment, log_path, source_environment):
    safe = _validated_command(command, source_environment)
    stream = Path(log_path).open("xb")
    try:
        process = subprocess.Popen(
            safe,
            env=environment,
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except BaseException:
        stream.close()
        raise
    return {"process": process, "stream": stream, "command": safe}


def _stop(handle, timeout_s=12.0):
    if handle is None:
        return None
    process = handle["process"]
    if process.poll() is None:
        for signal_value, timeout in (
            (signal.SIGINT, timeout_s),
            (signal.SIGTERM, 5.0),
            (signal.SIGKILL, 5.0),
        ):
            try:
                os.killpg(process.pid, signal_value)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                continue
    if process.poll() is None:
        raise P1ExecutionError("process group survived containment")
    if not handle["stream"].closed:
        handle["stream"].flush()
        os.fsync(handle["stream"].fileno())
        handle["stream"].close()
    return process.returncode


def _capture(command, environment, timeout_s, source_environment, log_path=None):
    safe = _validated_command(command, source_environment)
    process = subprocess.Popen(
        safe,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    try:
        payload, _ = process.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired as exc:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.communicate(timeout=5.0)
        raise P1ExecutionError("command timed out: {}".format(Path(safe[0]).name)) from exc
    if log_path is not None:
        redacted = redact_credential_material(
            payload.decode("utf-8", errors="replace"), source_environment
        ).encode("utf-8")
        _exclusive_bytes(log_path, redacted)
    return process.returncode, payload


def _run(command, environment, log_path, timeout_s, source_environment):
    returncode, _ = _capture(
        command, environment, timeout_s, source_environment, log_path
    )
    if returncode != 0:
        raise P1ExecutionError(
            "command exited {}: {}".format(returncode, Path(command[0]).name)
        )


def _wait_service(base, environment, name, timeout_s, executables, source_environment):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if base["process"].poll() is not None:
            raise P1ExecutionError("base launch exited before service readiness")
        try:
            result, payload = _capture(
                [executables["rosservice"], "list"],
                environment,
                min(3.0, max(0.2, deadline - time.monotonic())),
                source_environment,
            )
        except P1ExecutionError:
            result, payload = 1, b""
        if result == 0 and name in payload.decode("utf-8", errors="replace").splitlines():
            return
        time.sleep(0.2)
    raise P1ExecutionError("service readiness timed out: {}".format(name))


def _clock_sample(payload):
    sec = _CLOCK_SEC.search(payload)
    nsec = _CLOCK_NSEC.search(payload)
    if sec is None or nsec is None:
        raise P1ExecutionError("/clock probe output malformed")
    return int(sec.group(1)), int(nsec.group(1))


def _bootstrap(base, environment, work, executables, source_environment):
    barrier = R6I2PositiveClockBarrier(timeout_s=45.0)
    spawned = time.monotonic()
    barrier.mark_base_spawned(spawned)
    _wait_service(
        base, environment, "/gazebo/unpause_physics", 30.0,
        executables, source_environment,
    )
    requested = time.monotonic()
    barrier.observe_base_exit(base["process"].poll(), requested)
    barrier.mark_unpause_requested(requested)
    returncode, _ = _capture(
        [executables["rosservice"], "call", "/gazebo/unpause_physics"],
        environment, 10.0, source_environment, work / "unpause.log",
    )
    acknowledged = time.monotonic()
    barrier.observe_base_exit(base["process"].poll(), acknowledged)
    barrier.mark_unpause_acknowledged(acknowledged, returncode == 0)
    while not barrier.receipt()["clock_progression_sample"]:
        now = time.monotonic()
        barrier.observe_base_exit(base["process"].poll(), now)
        barrier.check_deadline(now)
        try:
            result, payload = _capture(
                [executables["rostopic"], "echo", "-n", "1", "/clock"],
                environment, 2.0, source_environment,
            )
        except P1ExecutionError:
            continue
        if result != 0:
            continue
        sec, nsec = _clock_sample(payload)
        barrier.observe_clock(sec, nsec, time.monotonic())
    barrier.release_service_wait(time.monotonic(), base["process"].poll())
    _wait_service(
        base, environment, "/move_base/TebLocalPlannerROS/set_parameters",
        45.0, executables, source_environment,
    )
    receipt = {
        "schema_version": "1.0",
        "stage": STAGE,
        "required_order": [
            "base_spawn", "unpause_request", "successful_unpause_ack",
            "two_strictly_increasing_positive_post_ack_clock_samples",
            "release_move_base_and_teb_service_wait",
        ],
        "barrier": barrier.receipt(),
        "move_base_teb_service_wait_after_release": True,
    }
    _exclusive_bytes(work / "bootstrap_receipt.yaml", _yaml_bytes(receipt))
    return receipt


def _wait_action_trace(base, transaction, environment, work, executables, source_environment):
    probe = _spawn(
        [executables["rostopic"], "echo", "-n", "1", "/teb_rl_v2/action_trace"],
        environment, work / "action_ready.log", source_environment,
    )
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if base["process"].poll() is not None or transaction["process"].poll() is not None:
                raise P1ExecutionError("launch exited before action-trace readiness")
            result = probe["process"].poll()
            if result is not None:
                if result != 0:
                    raise P1ExecutionError("action-trace readiness probe failed")
                return
            time.sleep(0.1)
        raise P1ExecutionError("action-trace readiness timed out")
    finally:
        _stop(probe)


def _executables():
    values = {
        "python": shutil.which("python3"),
        "roslaunch": shutil.which("roslaunch"),
        "rosservice": shutil.which("rosservice"),
        "rostopic": shutil.which("rostopic"),
        "xacro": shutil.which("xacro"),
    }
    if any(value is None for value in values.values()):
        raise P1ExecutionError("required executable is unavailable")
    return values


def _runtime(row, closure):
    if row["method_id"] not in SEMANTIC_METHODS:
        return None
    directory = RUNTIME_ROOT / row["method_id"]
    runtime = {}
    for label, filename in (
        ("supervisor", "supervisor.yaml"),
        ("anchor_bank", "anchor_bank.yaml"),
        ("mechanism", "mechanism.yaml"),
    ):
        path = directory / filename
        relative = path.relative_to(WORKSPACE)
        payload = _rehash_relative(relative, closure)
        runtime[label] = str(path)
        runtime[label + "_sha256"] = _sha(payload)
    return runtime


def _base_command(row, scene, world_path, runtime, executables):
    semantic = row["method_id"] in SEMANTIC_METHODS
    command = [
        executables["roslaunch"], "m2_gazebo", LAUNCH_RELATIVE.name,
        "world:={}".format(world_path),
        "seed:={}".format(row["seed"]),
        "xacro_executable:={}".format(executables["xacro"]),
        "x:={}".format(scene["start"]["x_m"]),
        "y:={}".format(scene["start"]["y_m"]),
        "yaw:={}".format(scene["start"]["yaw_rad"]),
        "gui:=false", "paused:=true",
        "attempt_stage:={}".format(STAGE),
        "attempt_profile_id:={}".format(row["method_id"]),
        "attempt_scene_id:={}".format(row["scene_id"]),
        "attempt_number:=1",
        "start_rule_supervisor:={}".format("true" if semantic else "false"),
        "start_typed_transaction:=false",
        "load_balanced_anchor:={}".format("true" if semantic else "false"),
        "publish_teb_obstacles:={}".format("true" if semantic else "false"),
        "allow_simulation_teb_parameter_write:=false",
        "allow_unfrozen_simulation_candidate:=true",
    ]
    if semantic:
        command.extend([
            "rule_supervisor_config:={}".format(runtime["supervisor"]),
            "rule_supervisor_config_sha256:={}".format(runtime["supervisor_sha256"]),
            "anchor_bank:={}".format(runtime["anchor_bank"]),
            "mechanism_config:={}".format(runtime["mechanism"]),
        ])
    return command


def _transaction_command(row, runtime, executables):
    return [
        executables["roslaunch"], "teb_mode_manager",
        "v2_04g_r6_simulation_typed_anchor.launch",
        "allow_simulation_teb_parameter_write:=true",
        "allow_unfrozen_simulation_candidate:=true",
        "anchor_bank:={}".format(runtime["anchor_bank"]),
        "mechanism_config:={}".format(runtime["mechanism"]),
        "attempt_stage:={}".format(STAGE),
        "attempt_profile_id:={}".format(row["method_id"]),
        "attempt_scene_id:={}".format(row["scene_id"]),
        "attempt_seed:={}".format(row["seed"]),
        "attempt_number:=1",
        "supervisor_config_sha256:={}".format(runtime["supervisor_sha256"]),
    ]


def _control_command(mode, row, output, runtime, executables):
    command = [
        executables["python"], str(WORKSPACE / CONTROL_RELATIVE),
        "--mode", mode, "--output", str(output), "--stage", STAGE,
        "--profile-id", row["method_id"], "--scene-id", row["scene_id"],
        "--seed", str(row["seed"]), "--attempt", "1",
    ]
    if mode == "initial-readback":
        command.extend(["--anchor-bank", runtime["anchor_bank"]])
    return command


def _listener_command(row, output, executables):
    gate = READINESS_GATE
    return [
        executables["python"], str(WORKSPACE / LISTENER_RELATIVE),
        "--output", str(output), "--profile-id", row["method_id"],
        "--scene-id", row["scene_id"], "--attempt", "1",
        "--repeat", str(row["sequence"]), "--seed", str(row["seed"]),
        "--warmup-timeout-s", str(gate["warmup_timeout_s"]),
        "--measurement-duration-s", str(gate["measurement_duration_s"]),
        "--minimum-message-count", str(gate["minimum_message_count_per_stream"]),
        "--minimum-valid-fraction", str(gate["minimum_valid_fraction"]),
        "--required-consecutive-stable-count", str(gate["required_consecutive_stable_count"]),
        "--maximum-expected-context-hold-count", str(gate["maximum_expected_context_hold_count_per_probe"]),
    ]


def _episode_command(row, instance_path, instance_sha256, output, executables):
    return [
        executables["python"], str(WORKSPACE / EPISODE_RELATIVE),
        "--instance", str(instance_path), "--instance-sha256", instance_sha256,
        "--method-id", row["method_id"], "--output-dir", str(output),
        "--stage", STAGE, "--attempt", "1",
    ]


def _fixed_snapshot_command(mode, row, output, baseline, executables):
    command = [
        executables["python"], str(WORKSPACE / FIXED_SNAPSHOT_RELATIVE),
        "--mode", mode, "--output", str(output), "--stage", STAGE,
        "--profile-id", "fixed_teb_reference", "--scene-id", row["scene_id"],
        "--seed", str(row["seed"]), "--attempt", "1",
    ]
    if baseline is not None:
        command.extend(["--baseline", str(baseline)])
    return command


def _snapshot_scene(row, work, closure):
    instance_binding = row["instance"]
    world_binding = row["world"]
    instance_payload = _rehash_relative(Path(instance_binding["path"]), closure)
    world_payload = _rehash_relative(Path(world_binding["path"]), closure)
    if _sha(instance_payload) != instance_binding["sha256"] or _sha(world_payload) != world_binding["sha256"]:
        raise P1ExecutionError("scheduled scene binding differs")
    snapshot = work / "scene_snapshot"
    snapshot.mkdir(parents=True, exist_ok=False)
    instance_path = snapshot / Path(instance_binding["path"]).name
    world_path = snapshot / Path(world_binding["path"]).name
    _exclusive_bytes(instance_path, instance_payload)
    _exclusive_bytes(world_path, world_payload)
    instance = yaml.safe_load(instance_payload)
    scene = instance.get("scene", {})
    if not (
        scene.get("scene_id") == row["scene_id"]
        and scene.get("seed") == row["seed"]
        and scene.get("family") == "DYNAMIC"
        and scene.get("split") == "test_id"
        and float(scene.get("timeout_s")) == 80.0
    ):
        raise P1ExecutionError("scheduled scene identity differs")
    return instance, instance_path, world_path


def _wait_episode(handles, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for label in ("base", "transaction"):
            handle = handles.get(label)
            if handle is not None and handle["process"].poll() is not None:
                raise P1ExecutionError("{} launch exited during episode".format(label))
        episode = handles["episode"]["process"].poll()
        listener = handles.get("listener")
        listener_result = listener["process"].poll() if listener is not None else 0
        if episode not in (None, 0):
            raise P1ExecutionError("episode exited {}".format(episode))
        if listener_result not in (None, 0):
            raise P1ExecutionError("activation listener exited {}".format(listener_result))
        if episode == 0 and listener_result == 0:
            return
        time.sleep(0.1)
    raise P1ExecutionError("per-identity hard wall exceeded")


def _combine_logs(work, target):
    chunks = []
    for path in sorted(work.glob("*.log")):
        try:
            payload = path.read_bytes()
        except OSError:
            payload = b""
        chunks.append(b"\n===== " + path.name.encode("utf-8") + b" =====\n" + payload)
    _exclusive_bytes(target, b"".join(chunks) or b"no process log output\n")


def _populate_raw(work, raw, teardown_path):
    raw.mkdir(parents=True, exist_ok=False)
    sources = {
        "activation": work / "activation.yaml",
        "evaluation": work / "episode/evaluation.yaml",
        "trace": work / "episode/trace.csv",
        "clearance": work / "episode/clearance_audit.yaml",
        "overlay": work / "episode/overlay_trace.yaml",
        "teardown_receipt": teardown_path,
    }
    for label, source in sources.items():
        if not source.is_file():
            raise P1ExecutionError("raw resource missing: {}".format(label))
        _exclusive_bytes(raw / RAW_FILENAMES[label], source.read_bytes())
    _combine_logs(work, raw / RAW_FILENAMES["process_log"])
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
    activation = _load_yaml(work / "activation.yaml")
    evaluation = _load_yaml(work / "episode/evaluation.yaml")
    teardown = _load_yaml(work / "teardown_receipt.yaml")
    if activation.get("all_hard_gates_pass") is not True:
        raise P1ExecutionError("semantic activation gate failed")
    if not (
        evaluation.get("stage") == STAGE
        and evaluation.get("scene_id") == row["scene_id"]
        and evaluation.get("seed") == row["seed"]
        and evaluation.get("training_used") is False
        and evaluation.get("runtime_policy_manifest_access") is False
        and evaluation.get("runtime_scene_labels_available") is False
        and evaluation.get("typed_transaction_valid") is True
    ):
        raise P1ExecutionError("semantic episode interface gate failed")
    if not (
        teardown.get("service_response_success") is True
        and teardown.get("startup_profile_sha256") == initial["startup_profile_sha256"]
        and teardown.get("supervisor_config_sha256") == runtime["supervisor_sha256"]
    ):
        raise P1ExecutionError("semantic teardown provenance differs")


def _write_failure_journal(row, release_sha256, reason, raw_receipts=None):
    path = JOURNAL_ROOT / "sequence_{:04d}_terminal_integrity_failure.yaml".format(row["sequence"])
    document = {
        "schema_version": "1.0",
        "stage": STAGE,
        "release_sha256": release_sha256,
        "identity": _identity(row),
        "status": "terminal_integrity_failure",
        "consumed": True,
        "consumption_boundary": "base_roslaunch_spawn_requested",
        "raw_resources_produced": raw_receipts or {},
        "integrity_failures": [reason],
        "terminal_recorded_at_utc": _utc_now(),
        "retry_resume_replacement_allowed": False,
    }
    _exclusive_bytes(path, _yaml_bytes(document))
    return path


def _run_attempt(row, validation, report, source_environment, executables):
    closure = _closure_map(validation)
    for relative in (
        LAUNCH_RELATIVE, EPISODE_RELATIVE, FIXED_SNAPSHOT_RELATIVE,
        CONTROL_RELATIVE if row["method_id"] in SEMANTIC_METHODS else FIXED_SNAPSHOT_RELATIVE,
        LISTENER_RELATIVE if row["method_id"] in SEMANTIC_METHODS else EPISODE_RELATIVE,
    ):
        _rehash_relative(relative, closure)
    runtime = _runtime(row, closure)
    attempt_root = ATTEMPTS_ROOT / _attempt_name(row)
    work = attempt_root / "work"
    raw = attempt_root / "raw"
    attempt_root.mkdir(parents=True, exist_ok=False)
    work.mkdir()
    (work / "episode").mkdir()
    instance, instance_path, world_path = _snapshot_scene(row, work, closure)
    environment, environment_audit = _command_environment(work, source_environment)
    handles = {"base": None, "transaction": None, "listener": None, "episode": None}
    exit_codes = {}
    teardown_path = work / "teardown_receipt.yaml"
    restored = False
    initial = None
    failure = None
    raw_receipts = None

    consumption = {
        "schema_version": "1.0", "stage": STAGE,
        "release_sha256": validation["release_sha256"],
        "identity": _identity(row), "consumed": True,
        "consumption_boundary": "base_roslaunch_spawn_requested",
        "committed_at_utc": _utc_now(),
    }
    _exclusive_bytes(
        CONSUMPTION_ROOT / "sequence_{:04d}.yaml".format(row["sequence"]),
        _yaml_bytes(consumption),
    )
    report["evidence_units_consumed"] += 1
    report["attempt_ledger"].append({
        "identity": _identity(row),
        "status": "base_roslaunch_spawn_request_committed",
        "environment_policy": environment_audit,
    })
    _atomic_yaml(STAGE_REPORT, report)
    try:
        handles["base"] = _spawn(
            _base_command(row, instance["scene"], world_path, runtime, executables),
            environment, work / "base_launch.log", source_environment,
        )
        bootstrap = _bootstrap(
            handles["base"], environment, work, executables, source_environment
        )
        report["attempt_ledger"][-1]["bootstrap"] = bootstrap
        if row["method_id"] in SEMANTIC_METHODS:
            initial_path = work / "initial_readback.yaml"
            _run(
                _control_command("initial-readback", row, initial_path, runtime, executables),
                environment, work / "initial_readback.log", 25.0, source_environment,
            )
            initial = _load_yaml(initial_path)
            startup_payload = initial.get("startup_profile_canonical_json", "").encode("utf-8")
            if not startup_payload or _sha(startup_payload) != initial.get("startup_profile_sha256"):
                raise P1ExecutionError("initial startup profile hash differs")
            handles["transaction"] = _spawn(
                _transaction_command(row, runtime, executables),
                environment, work / "transaction_launch.log", source_environment,
            )
            startup_path = work / "transaction_startup.yaml"
            _run(
                _control_command("transaction-startup", row, startup_path, runtime, executables),
                environment, work / "transaction_startup.log", 30.0, source_environment,
            )
            startup = _load_yaml(startup_path)
            if not (
                startup.get("startup_profile_sha256") == initial["startup_profile_sha256"]
                and startup.get("supervisor_config_sha256") == runtime["supervisor_sha256"]
            ):
                raise P1ExecutionError("transaction startup provenance differs")
            arm_path = work / "arm_receipt.yaml"
            _run(
                _control_command("arm", row, arm_path, runtime, executables),
                environment, work / "arm.log", 20.0, source_environment,
            )
            arm = _load_yaml(arm_path)
            if not (
                arm.get("execution_armed") is True
                and arm.get("startup_profile_sha256") == initial["startup_profile_sha256"]
                and arm.get("supervisor_config_sha256") == runtime["supervisor_sha256"]
            ):
                raise P1ExecutionError("transaction arm provenance differs")
            _wait_action_trace(
                handles["base"], handles["transaction"], environment,
                work, executables, source_environment,
            )
            handles["listener"] = _spawn(
                _listener_command(row, work / "activation.yaml", executables),
                environment, work / "listener.log", source_environment,
            )
        else:
            baseline = work / "fixed_initial_readback.yaml"
            _run(
                _fixed_snapshot_command("capture", row, baseline, None, executables),
                environment, work / "fixed_initial_readback.log", 25.0, source_environment,
            )
            activation = {
                "schema_version": "1.0", "stage": STAGE,
                "record_type": "fixed_teb_structural_activation",
                **_identity(row),
                "supervisor_started": False,
                "typed_transaction_started": False,
                "dynamic_obstacle_bridge_started": False,
                "fixed_configuration_sha256": _load_yaml(baseline)["configuration_sha256"],
            }
            _exclusive_bytes(work / "activation.yaml", _yaml_bytes(activation))

        handles["episode"] = _spawn(
            _episode_command(
                row, instance_path, row["instance"]["sha256"],
                work / "episode", executables,
            ),
            environment, work / "episode.log", source_environment,
        )
        _wait_episode(handles, 240.0)
        exit_codes["episode"] = handles["episode"]["process"].returncode
        if handles["listener"] is not None:
            exit_codes["listener"] = handles["listener"]["process"].returncode

        if row["method_id"] in SEMANTIC_METHODS:
            _run(
                _control_command("restore", row, teardown_path, runtime, executables),
                environment, work / "restore.log", 25.0, source_environment,
            )
            restored = True
            _validate_semantic_runtime(row, work, initial, runtime)
        else:
            _run(
                _fixed_snapshot_command(
                    "verify", row, teardown_path,
                    work / "fixed_initial_readback.yaml", executables,
                ),
                environment, work / "fixed_verify.log", 25.0, source_environment,
            )
            restored = True
    except BaseException as exc:
        failure = exc
    finally:
        if (
            failure is not None
            and row["method_id"] in SEMANTIC_METHODS
            and handles["transaction"] is not None
            and handles["transaction"]["process"].poll() is None
            and initial is not None
            and not restored
        ):
            try:
                _run(
                    _control_command("restore", row, teardown_path, runtime, executables),
                    environment, work / "restore_after_failure.log", 25.0, source_environment,
                )
                restored = True
            except BaseException as restore_exc:
                failure = P1ExecutionError(
                    "{}; teardown also failed: {}".format(failure, restore_exc)
                )
        for label in ("episode", "listener", "transaction", "base"):
            handle = handles[label]
            if handle is not None:
                try:
                    exit_codes.setdefault(label, _stop(handle))
                except BaseException as stop_exc:
                    failure = P1ExecutionError(
                        "{}; {} containment failed: {}".format(
                            failure or "attempt otherwise completed", label, stop_exc
                        )
                    )
                handles[label] = None
    if failure is not None:
        reason = _safe_reason(failure, source_environment)
        journal = _write_failure_journal(
            row, validation["release_sha256"], reason, raw_receipts
        )
        report["attempt_ledger"][-1].update({
            "status": "terminal_integrity_failure",
            "failure": reason,
            "journal": str(journal.relative_to(WORKSPACE)),
        })
        raise P1ExecutionError(reason)

    raw_receipts = _populate_raw(work, raw, teardown_path)
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
    _exclusive_bytes(journal_path, _yaml_bytes(journal_document))
    normalized, audit = replay_one(
        WORKSPACE,
        row,
        journal_path.relative_to(WORKSPACE),
        validation["release_sha256"],
    )
    report["evidence_units_completed"] += 1
    report["attempt_ledger"][-1].update({
        "status": "completed",
        "journal": str(journal_path.relative_to(WORKSPACE)),
        "journal_sha256": _sha(journal_path.read_bytes()),
        "normalized_terminal_sha256": audit["normalized_sha256"],
        "terminal_status": normalized["terminal_status"],
    })
    _atomic_yaml(STAGE_REPORT, report)
    _wait_host_clean()


def _write_assessment(validation):
    normalized, replay, assessment = build_terminal_assessment(
        validation["schedule"],
        JOURNAL_ROOT.relative_to(WORKSPACE),
        validation["release_sha256"],
        WORKSPACE,
    )
    _exclusive_bytes(
        OUTPUT_ROOT / "normalized_terminal_matrix.json",
        (json.dumps(normalized, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8"),
    )
    _exclusive_bytes(OUTPUT_ROOT / "raw_replay_report.yaml", _yaml_bytes(replay))
    _exclusive_bytes(OUTPUT_ROOT / "p1_e1_four_gate_assessment.yaml", _yaml_bytes(assessment))
    return replay, assessment


def execute(expected_release_sha256):
    source_environment = dict(os.environ)
    validation = validate_release(expected_release_sha256, WORKSPACE, require_fresh_state=True)
    _assert_host_clean()
    executables = _executables()
    closure = _closure_map(validation)
    for relative in (
        LAUNCH_RELATIVE, EPISODE_RELATIVE, CONTROL_RELATIVE, LISTENER_RELATIVE,
        FIXED_SNAPSHOT_RELATIVE, ASSESS_RELATIVE,
    ):
        _rehash_relative(relative, closure)
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    ATTEMPTS_ROOT.mkdir(exist_ok=False)
    JOURNAL_ROOT.mkdir(exist_ok=False)
    CONSUMPTION_ROOT.mkdir(exist_ok=False)
    report = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "running_exact_270_row_schedule",
        "release_sha256": validation["release_sha256"],
        "schedule_sha256": validation["schedule_sha256"],
        "schedule_episode_digest": validation["schedule"]["canonical_episode_digest"],
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_authorized": True,
        "authorized_evidence_units": 270,
        "evidence_units_consumed": 0,
        "evidence_units_completed": 0,
        "evidence_units_forfeited": 0,
        "retry_resume_replacement_allowed": False,
        "training_budget_steps": 0,
        "real_vehicle_used": False,
        "i5_rerun_or_resume_performed": False,
        "started_at_utc": _utc_now(),
        "completed_at_utc": None,
        "attempt_ledger": [],
        "terminal_integrity_failures": [],
        "four_gate_decision": None,
        "performance_improvement_demonstrated": False,
        "claim_authorized": False,
    }
    _atomic_yaml(STAGE_REPORT, report)
    stage_started = time.monotonic()
    for row in validation["schedule"]["episodes"]:
        if time.monotonic() - stage_started > 20.0 * 3600.0:
            reason = "stage hard wall exceeded before sequence {}".format(row["sequence"])
            report["status"] = "terminal_integrity_failure"
            report["terminal_integrity_failures"].append(reason)
            report["evidence_units_forfeited"] = 270 - report["evidence_units_consumed"]
            report["completed_at_utc"] = _utc_now()
            _atomic_yaml(STAGE_REPORT, report)
            raise P1ExecutionError(reason)
        try:
            _run_attempt(row, validation, report, source_environment, executables)
        except BaseException as exc:
            reason = _safe_reason(exc, source_environment)
            report["status"] = "terminal_integrity_failure"
            report["terminal_integrity_failures"].append(reason)
            report["evidence_units_forfeited"] = 270 - report["evidence_units_consumed"]
            report["formal_result"] = False
            report["performance_improvement_demonstrated"] = False
            report["claim_authorized"] = False
            report["completed_at_utc"] = _utc_now()
            _atomic_yaml(STAGE_REPORT, report)
            _wait_host_clean()
            raise P1ExecutionError(reason) from None
    replay, assessment = _write_assessment(validation)
    report.update({
        "status": "exact_270_row_execution_and_raw_replay_complete",
        "formal_result": assessment["formal_result"],
        "evidence_units_forfeited": 0,
        "completed_at_utc": _utc_now(),
        "raw_replay_status": replay["status"],
        "raw_replay_report_sha256": _sha((OUTPUT_ROOT / "raw_replay_report.yaml").read_bytes()),
        "four_gate_assessment_sha256": _sha((OUTPUT_ROOT / "p1_e1_four_gate_assessment.yaml").read_bytes()),
        "four_gate_decision": assessment["four_gate_decision"],
        "performance_improvement_demonstrated": assessment["performance_improvement_demonstrated"],
        "claim_authorized": assessment["claim_authorized"],
    })
    _atomic_yaml(STAGE_REPORT, report)
    _assert_host_clean()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        parser.error("--execute is required; validation alone must use the offline reviewer")
    try:
        report = execute(args.release_sha256)
    except (P1ReleaseError, P1ExecutionError) as exc:
        print("P1-E1 terminal failure: {}".format(exc), file=sys.stderr)
        return 2
    print(yaml.safe_dump({
        "status": report["status"],
        "consumed": report["evidence_units_consumed"],
        "completed": report["evidence_units_completed"],
        "four_gate_decision": report["four_gate_decision"],
        "claim_authorized": report["claim_authorized"],
    }, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
