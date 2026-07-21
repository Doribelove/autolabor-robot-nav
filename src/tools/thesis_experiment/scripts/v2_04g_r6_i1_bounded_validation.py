#!/usr/bin/env python3
"""Run the bounded, no-resume R6-I1 fresh-seed simulation schedule."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import yaml

from thesis_experiment.v2_04g_r6_i1_dependency import (
    build_dependency_closure,
)
from thesis_experiment import v2_04g_r6_integrity as integrity


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R6-I1"
PREREGISTRATION = WORKSPACE / (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_execution_preregistration.yaml"
)
CONTRACT = WORKSPACE / (
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_execution_integration_contract.yaml"
)
CLOSURE = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "execution_dependency_closure.yaml"
)
INTEGRATION_REVIEW = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "v2_04g_r6_i1_integration_review.yaml"
)
ROOT = WORKSPACE / "artifacts/v2/integration/v2_04g_r6_i1"
EXECUTION_ROOT = ROOT / "execution"
JOURNAL_ROOT = EXECUTION_ROOT / "journals"
STAGE_REPORT = ROOT / "v2_04g_r6_i1_stage_report.yaml"
COMPILED_ROOT = ROOT / "compiled_scenes"
COMPILED_INDEX = COMPILED_ROOT / "compiled_scene_index.yaml"
RUNTIME_ROOT = ROOT / "runtime_candidate_configs"
LISTENER = Path(__file__).with_name(
    "v2_04g_r6_i1_activation_probe_listener.py"
)
EPISODE = Path(__file__).with_name(
    "v2_04g_r6_i1_mechanism_episode.py"
)
CONTROL = Path(__file__).with_name(
    "v2_04g_r6_i1_runtime_control.py"
)
RAW_FILENAMES = {
    "activation": "activation.yaml",
    "evaluation": "evaluation.yaml",
    "trace": "trace.csv",
    "clearance": "clearance.yaml",
    "process_log": "process.log",
    "teardown_receipt": "teardown_receipt.yaml",
}
PROCESS_MARKERS = (
    "roscore",
    "rosmaster",
    "roslaunch",
    "gzserver",
    "gzclient",
    "gazebo",
    "move_base",
    "sac_train",
    "residual_train",
)


class R6I1ExecutionError(RuntimeError):
    """Terminal R6-I1 execution failure."""


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _atomic_yaml(path, value):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
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
        directory = os.open(str(target.parent), os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def _exclusive_bytes(path, payload):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(
        str(target),
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0),
        0o444,
    )
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        try:
            target.unlink()
        except OSError:
            pass
        raise


def _load(path):
    return integrity.strict_yaml(path)


def _exact_document(actual, expected, label):
    if type(actual) is not type(expected):
        raise R6I1ExecutionError("{} type drifted".format(label))
    if isinstance(expected, dict):
        if set(actual) != set(expected):
            raise R6I1ExecutionError("{} keys drifted".format(label))
        for key in expected:
            _exact_document(
                actual[key], expected[key], "{}.{}".format(label, key)
            )
    elif isinstance(expected, list):
        if len(actual) != len(expected):
            raise R6I1ExecutionError("{} length drifted".format(label))
        for index, (left, right) in enumerate(zip(actual, expected)):
            _exact_document(left, right, "{}[{}]".format(label, index))
    elif actual != expected:
        raise R6I1ExecutionError("{} value drifted".format(label))


def _process_matches():
    current = os.getpid()
    ancestors = {current}
    cursor = current
    while True:
        try:
            status = Path("/proc/{}/status".format(cursor)).read_text()
            parent = int(
                next(
                    line.split()[1]
                    for line in status.splitlines()
                    if line.startswith("PPid:")
                )
            )
        except (OSError, StopIteration, ValueError):
            break
        if parent <= 1 or parent in ancestors:
            break
        ancestors.add(parent)
        cursor = parent
    matches = []
    for item in Path("/proc").iterdir():
        if not item.name.isdigit() or int(item.name) in ancestors:
            continue
        try:
            command = (item / "cmdline").read_bytes().replace(
                b"\0", b" "
            ).decode("utf-8", errors="replace").strip()
        except OSError:
            continue
        executable = Path(command.split(" ", 1)[0]).name.lower()
        tokens = command.lower().split()
        if (
            executable in PROCESS_MARKERS
            or any(
                Path(token).name in PROCESS_MARKERS
                for token in tokens[:3]
            )
            or any(
                marker in command.lower()
                for marker in ("sac_train.py", "residual_train.py")
            )
        ):
            matches.append({"pid": int(item.name), "command": command})
    return sorted(matches, key=lambda row: row["pid"])


def _verify_closure():
    frozen = _load(CLOSURE)
    generated = build_dependency_closure(WORKSPACE)
    _exact_document(frozen, generated, "execution dependency closure")
    result = integrity.verify_dependency_closure(
        WORKSPACE, frozen, generated["required_paths"]
    )
    if result["closure_sha256"] != frozen.get("closure_sha256"):
        raise R6I1ExecutionError("dependency closure digest drifted")
    return result


def _verify_preflight(authorization_path=None, authorization_sha256=None):
    prereg = _load(PREREGISTRATION)
    contract = _load(CONTRACT)
    if not (
        prereg.get("stage") == STAGE
        and prereg.get("execution_authorized") is False
        and prereg["budget"]["evidence_units_authorizable"] == 6
        and len(prereg["schedule"]) == 6
        and contract.get("stage") == STAGE
        and contract.get("execution_authorized") is False
    ):
        raise R6I1ExecutionError("R6-I1 preregistration boundary drifted")
    closure = _verify_closure()
    live = _process_matches()
    if live:
        raise R6I1ExecutionError(
            "live ROS/Gazebo/move_base/training process detected: {}".format(
                live
            )
        )
    if authorization_path is None:
        return prereg, contract, closure, None
    auth_path = Path(authorization_path).resolve()
    expected_auth_path = WORKSPACE / (
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i1_bounded_simulation_authorization.yaml"
    )
    if auth_path != expected_auth_path.resolve():
        raise R6I1ExecutionError("authorization path drifted")
    if (
        not isinstance(authorization_sha256, str)
        or len(authorization_sha256) != 64
        or sha256(auth_path) != authorization_sha256
    ):
        raise R6I1ExecutionError("authorization trust-anchor hash mismatch")
    authorization = _load(auth_path)
    if not (
        authorization.get("stage") == STAGE
        and authorization.get("execution_authorized") is True
        and authorization.get("evidence_budget_authorized") == 6
        and authorization.get("fresh_execution_seeds") == [5141, 5142, 5143]
        and authorization.get("attempt_limit_per_identity") == 1
        and authorization.get("retry_or_resume_allowed") is False
        and authorization.get("held_out_5001_5010_accessed") is False
        and authorization.get("r5_remaining_units_consumed") == 0
    ):
        raise R6I1ExecutionError("authorization safety boundary drifted")
    for label, path in (
        ("contract", CONTRACT),
        ("preregistration", PREREGISTRATION),
        ("dependency_closure", CLOSURE),
        ("integration_review", INTEGRATION_REVIEW),
        ("compiled_scene_index", COMPILED_INDEX),
    ):
        row = authorization["bound_resources"].get(label)
        if not (
            isinstance(row, dict)
            and row.get("path") == str(path.relative_to(WORKSPACE))
            and row.get("sha256") == sha256(path)
        ):
            raise R6I1ExecutionError(
                "authorization {} binding drifted".format(label)
            )
    review = _load(INTEGRATION_REVIEW)
    if not (
        review.get("review_result") == "pass"
        and review.get("execution_authorized") is False
        and review.get("seed_or_evidence_units_consumed") == 0
    ):
        raise R6I1ExecutionError("integration review did not pass")
    return prereg, contract, closure, authorization


def _identity(row):
    return {
        "stage": STAGE,
        "profile_id": row["profile_id"],
        "scene_id": row["scene_id"],
        "seed": row["seed"],
        "attempt": row["attempt"],
    }


def _attempt_name(row):
    return "{:02d}__{}__{}".format(
        row["sequence"], row["profile_id"], row["scene_id"]
    )


def _spawn(command, environment, log_path):
    stream = Path(log_path).open("x", encoding="utf-8")
    try:
        process = subprocess.Popen(
            command,
            env=environment,
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except BaseException:
        stream.close()
        raise
    return process, stream


def _stop_process(process, stream, timeout_s=12.0):
    if process is not None and process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=5.0)
    if stream is not None and not stream.closed:
        stream.flush()
        stream.close()


def _run_command(command, environment, log_path, timeout_s):
    process, stream = _spawn(command, environment, log_path)
    try:
        result = process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired as exc:
        _stop_process(process, stream)
        raise R6I1ExecutionError(
            "command timed out: {}".format(command[0])
        ) from exc
    stream.flush()
    stream.close()
    if result != 0:
        raise R6I1ExecutionError(
            "command exited {}: {}".format(result, command[0])
        )


def _wait_for_service(base_process, environment, service, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if base_process.poll() is not None:
            raise R6I1ExecutionError(
                "base roslaunch exited before service readiness"
            )
        result = subprocess.run(
            ["rosservice", "list"],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=5.0,
            check=False,
        )
        if result.returncode == 0 and service in result.stdout.splitlines():
            return
        time.sleep(0.25)
    raise R6I1ExecutionError(
        "service readiness timed out: {}".format(service)
    )


def _wait_action_trace(base, transaction, environment, log_path):
    process, stream = _spawn(
        ["rostopic", "echo", "-n", "1", "/teb_rl_v2/action_trace"],
        environment,
        log_path,
    )
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if base.poll() is not None or transaction.poll() is not None:
                raise R6I1ExecutionError(
                    "launch exited before transaction readiness"
                )
            result = process.poll()
            if result is not None:
                if result != 0:
                    raise R6I1ExecutionError(
                        "action trace readiness probe failed"
                    )
                stream.close()
                return
            time.sleep(0.1)
        raise R6I1ExecutionError("action trace readiness timed out")
    finally:
        if process.poll() is None:
            _stop_process(process, stream)
        elif not stream.closed:
            stream.close()


def _base_launch_command(row, scene, snapshot, runtime):
    document = snapshot.as_document()
    return [
        "roslaunch",
        "m2_gazebo",
        "m2_v2_04g_r6_execution_integration.launch",
        "world:={}".format(document["snapshot_world"]["path"]),
        "seed:={}".format(row["seed"]),
        "x:={}".format(scene["start"]["x_m"]),
        "y:={}".format(scene["start"]["y_m"]),
        "yaw:={}".format(scene["start"]["yaw_rad"]),
        "gui:=false",
        "paused:=true",
        "start_typed_transaction:=false",
        "rule_supervisor_config:={}".format(runtime["supervisor"]),
        "rule_supervisor_config_sha256:={}".format(
            runtime["supervisor_sha256"]
        ),
        "anchor_bank:={}".format(runtime["anchor_bank"]),
        "mechanism_config:={}".format(runtime["mechanism"]),
        "attempt_stage:={}".format(STAGE),
        "attempt_profile_id:={}".format(row["profile_id"]),
        "attempt_scene_id:={}".format(row["scene_id"]),
        "attempt_number:=1",
        "allow_simulation_teb_parameter_write:=false",
        "allow_unfrozen_simulation_candidate:=true",
    ]


def _transaction_launch_command(row, runtime):
    return [
        "roslaunch",
        "teb_mode_manager",
        "v2_04g_r6_simulation_typed_anchor.launch",
        "allow_simulation_teb_parameter_write:=true",
        "allow_unfrozen_simulation_candidate:=true",
        "anchor_bank:={}".format(runtime["anchor_bank"]),
        "mechanism_config:={}".format(runtime["mechanism"]),
        "attempt_stage:={}".format(STAGE),
        "attempt_profile_id:={}".format(row["profile_id"]),
        "attempt_scene_id:={}".format(row["scene_id"]),
        "attempt_seed:={}".format(row["seed"]),
        "attempt_number:=1",
        "supervisor_config_sha256:={}".format(
            runtime["supervisor_sha256"]
        ),
    ]


def _control_command(mode, row, output, runtime):
    command = [
        sys.executable,
        str(CONTROL),
        "--mode",
        mode,
        "--output",
        str(output),
        "--stage",
        STAGE,
        "--profile-id",
        row["profile_id"],
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


def _listener_command(row, prereg, output):
    gate = prereg["readiness_gate"]
    return [
        sys.executable,
        str(LISTENER),
        "--output",
        str(output),
        "--profile-id",
        row["profile_id"],
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


def _episode_command(row, instance_path, output):
    return [
        sys.executable,
        str(EPISODE),
        "--instance",
        str(instance_path),
        "--method",
        "rule_multi_anchor",
        "--output-dir",
        str(output),
        "--stage",
        STAGE,
        "--split",
        "calibration",
        "--profile-id",
        row["profile_id"],
        "--attempt",
        "1",
    ]


def _runtime(row):
    directory = RUNTIME_ROOT / row["profile_id"]
    result = {
        "supervisor": str(directory / "supervisor.yaml"),
        "anchor_bank": str(directory / "anchor_bank.yaml"),
        "mechanism": str(directory / "mechanism.yaml"),
    }
    if not all(Path(value).is_file() for value in result.values()):
        raise R6I1ExecutionError("runtime profile is incomplete")
    result["supervisor_sha256"] = sha256(result["supervisor"])
    return result


def _compiled_instance(row):
    instance = COMPILED_ROOT / (row["scene_id"] + ".instance.yaml")
    document = _load(instance)
    if not (
        document["scene"]["scene_id"] == row["scene_id"]
        and document["scene"]["seed"] == row["seed"]
        and document["scene"]["family"] == "DYNAMIC"
    ):
        raise R6I1ExecutionError("compiled scene identity drifted")
    return document, instance


def _validate_semantics(row, activation, evaluation, minimum):
    identity = _identity(row)
    integrity.validate_readiness_raw_evidence(
        identity, activation, evaluation, minimum
    )
    if not (
        activation.get("all_hard_gates_pass") is True
        and evaluation.get("ttc_status") == row["expected_ttc_status"]
        and evaluation.get("formal_result") is False
        and evaluation.get("runtime_ready") is False
        and evaluation.get("training_used") is False
        and evaluation.get("runtime_policy_manifest_access") is False
        and evaluation.get("runtime_scene_labels_available") is False
    ):
        raise R6I1ExecutionError("R6-I1 readiness/evaluator gate failed")
    overlay = evaluation.get("context_overlay_sample_counts", {})
    non_none = sum(
        int(value) for key, value in overlay.items() if key != "NONE"
    )
    finite = int(evaluation.get("finite_ttc_sample_count", 0))
    role = row["expected_overlay_semantics"]
    if role in {"non_none", "non_none_iff_finite_ttc"}:
        if finite <= 0 or non_none <= 0:
            raise R6I1ExecutionError(
                "finite conflict scene lacked TTC/overlay evidence"
            )
    elif role == "legacy_non_none_identifiability":
        if finite != 0 or non_none <= 0:
            raise R6I1ExecutionError(
                "legacy semantic-clear identifiability gate failed"
            )
    elif role == "none_iff_no_finite_ttc":
        if finite != 0 or non_none != 0:
            raise R6I1ExecutionError(
                "aligned no-finite-TTC eligibility parity failed"
            )
    else:
        raise R6I1ExecutionError("unknown overlay semantic role")
    return {"finite_ttc_sample_count": finite, "non_none_overlay_count": non_none}


def _combine_process_logs(work, target):
    chunks = []
    for path in sorted(work.glob("*.log")):
        try:
            payload = path.read_bytes()
        except OSError:
            payload = b""
        chunks.append(
            b"\n===== " + path.name.encode("utf-8") + b" =====\n" + payload
        )
    _exclusive_bytes(target, b"".join(chunks) or b"no process log output\n")


def _populate_raw(work, raw, teardown_path):
    raw.mkdir(parents=True, exist_ok=False)
    sources = {
        "activation": work / "activation.yaml",
        "evaluation": work / "episode/evaluation.yaml",
        "trace": work / "episode/trace.csv",
        "clearance": work / "episode/clearance_audit.yaml",
        "teardown_receipt": teardown_path,
    }
    for label, source in sources.items():
        _exclusive_bytes(raw / RAW_FILENAMES[label], source.read_bytes())
    _combine_process_logs(work, raw / RAW_FILENAMES["process_log"])
    return {
        label: {
            "path": str((raw / filename).relative_to(WORKSPACE)),
            "sha256": sha256(raw / filename),
        }
        for label, filename in RAW_FILENAMES.items()
    }


def _terminal_raw(work, raw, identity, phase, reason):
    raw.mkdir(parents=True, exist_ok=True)
    candidates = {
        "activation": work / "activation.yaml",
        "evaluation": work / "episode/evaluation.yaml",
        "trace": work / "episode/trace.csv",
        "clearance": work / "episode/clearance_audit.yaml",
        "teardown_receipt": work / "teardown_receipt.yaml",
    }
    if not (raw / RAW_FILENAMES["process_log"]).exists():
        _combine_process_logs(work, raw / RAW_FILENAMES["process_log"])
    resources = {}
    for label in sorted(integrity.RAW_EVIDENCE_LABELS):
        target = raw / RAW_FILENAMES[label]
        source = candidates.get(label)
        if label == "process_log":
            source = target
        if source is not None and source.is_file():
            if source != target and not target.exists():
                _exclusive_bytes(target, source.read_bytes())
            resources[label] = {
                "status": "produced",
                "path": str(target.relative_to(WORKSPACE)),
                "sha256": sha256(target),
            }
        else:
            resources[label] = {
                "status": "not_produced",
                "phase": phase,
                "reason": str(reason),
            }
    return resources


def _aux_row(path):
    return {
        "path": str(Path(path).relative_to(WORKSPACE)),
        "sha256": sha256(path),
    }


def _run_attempt(row, prereg, environment, ledger_entry, report):
    identity = _identity(row)
    attempt_root = EXECUTION_ROOT / "attempts" / _attempt_name(row)
    work = attempt_root / "work"
    raw = attempt_root / "raw"
    runtime = _runtime(row)
    scene, _ = _compiled_instance(row)
    base = transaction = listener = episode = None
    base_log = transaction_log = listener_log = episode_log = None
    snapshot = None
    teardown_path = work / "teardown_receipt.yaml"
    minimum = prereg["readiness_gate"]["minimum_message_count_per_stream"]
    with integrity.AtomicAttemptJournal(JOURNAL_ROOT, identity) as journal:
        ledger_entry.update({
            "journal_root": str(JOURNAL_ROOT.relative_to(WORKSPACE)),
            "journal": str(journal.path.relative_to(WORKSPACE)),
            "raw_evidence_root": str(raw.relative_to(WORKSPACE)),
            "status": "attempt_started",
        })
        _atomic_yaml(STAGE_REPORT, report)
        try:
            work.mkdir(parents=True, exist_ok=False)
            (work / "episode").mkdir()
            lease = integrity.acquire_compiled_scene_lease(
                WORKSPACE,
                COMPILED_INDEX,
                sha256(COMPILED_INDEX),
                row["scene_id"],
            )
            snapshot = integrity.materialize_scene_snapshot(
                lease, work / "scene_snapshot"
            )
            integrity.revalidate_scene_snapshot(snapshot, "pre_spawn")
            environment = dict(environment)
            environment["ROS_HOME"] = str(work / "ros_home")
            environment["ROS_LOG_DIR"] = str(work / "ros_logs")
            Path(environment["ROS_HOME"]).mkdir()
            Path(environment["ROS_LOG_DIR"]).mkdir()
            base, base_log = _spawn(
                _base_launch_command(row, scene["scene"], snapshot, runtime),
                environment,
                work / "base_launch.log",
            )
            ledger_entry.update({
                "seed_consumed": True,
                "evidence_units_consumed": 1,
                "consumption_boundary": "base_roslaunch_spawn_requested",
            })
            report["evidence_units_consumed"] = sum(
                entry.get("evidence_units_consumed", 0)
                for entry in report["attempt_ledger"]
            )
            _atomic_yaml(STAGE_REPORT, report)
            _wait_for_service(
                base,
                environment,
                "/move_base/TebLocalPlannerROS/set_parameters",
                45.0,
            )
            initial_path = work / "initial_readback.yaml"
            _run_command(
                _control_command(
                    "initial-readback", row, initial_path, runtime
                ),
                environment,
                work / "initial_readback.log",
                25.0,
            )
            initial = _load(initial_path)
            startup_payload = initial[
                "startup_profile_canonical_json"
            ].encode("utf-8")
            if sha256_bytes(startup_payload) != initial[
                "startup_profile_sha256"
            ]:
                raise R6I1ExecutionError("initial profile hash drifted")
            journal.capture_startup_profile(startup_payload)
            journal.bind_scene_snapshot(snapshot)
            _run_command(
                ["rosservice", "call", "/gazebo/unpause_physics"],
                environment,
                work / "unpause.log",
                10.0,
            )
            transaction, transaction_log = _spawn(
                _transaction_launch_command(row, runtime),
                environment,
                work / "transaction_launch.log",
            )
            startup_path = work / "transaction_startup.yaml"
            _run_command(
                _control_command(
                    "transaction-startup", row, startup_path, runtime
                ),
                environment,
                work / "transaction_startup.log",
                30.0,
            )
            transaction_startup = _load(startup_path)
            if not (
                transaction_startup["startup_profile_sha256"]
                == initial["startup_profile_sha256"]
                and transaction_startup.get("supervisor_config_sha256")
                == runtime["supervisor_sha256"]
            ):
                raise R6I1ExecutionError(
                    "transaction startup provenance mismatched"
                )
            arm_path = work / "arm_receipt.yaml"
            _run_command(
                _control_command("arm", row, arm_path, runtime),
                environment,
                work / "arm.log",
                20.0,
            )
            arm = _load(arm_path)
            if not (
                arm.get("startup_profile_sha256")
                == initial["startup_profile_sha256"]
                and arm.get("supervisor_config_sha256")
                == runtime["supervisor_sha256"]
                and arm.get("execution_armed") is True
            ):
                raise R6I1ExecutionError("arm provenance mismatched")
            journal.mark_execution_started()
            _wait_action_trace(
                base, transaction, environment, work / "action_ready.log"
            )
            listener, listener_log = _spawn(
                _listener_command(row, prereg, work / "activation.yaml"),
                environment,
                work / "listener.log",
            )
            episode, episode_log = _spawn(
                _episode_command(
                    row,
                    Path(snapshot.as_document()["snapshot_instance"]["path"]),
                    work / "episode",
                ),
                environment,
                work / "episode.log",
            )
            deadline = time.monotonic() + float(
                scene["scene"]["timeout_s"]
            ) + 90.0
            while True:
                listener_result = listener.poll()
                episode_result = episode.poll()
                if listener_result not in (None, 0):
                    raise R6I1ExecutionError(
                        "activation listener exited {}".format(
                            listener_result
                        )
                    )
                if episode_result not in (None, 0):
                    raise R6I1ExecutionError(
                        "episode runner exited {}".format(episode_result)
                    )
                if listener_result == 0 and episode_result == 0:
                    break
                if base.poll() is not None or transaction.poll() is not None:
                    raise R6I1ExecutionError(
                        "base/transaction launch exited during episode"
                    )
                if time.monotonic() > deadline:
                    raise R6I1ExecutionError("episode deadline exceeded")
                time.sleep(0.1)
            listener_log.flush()
            listener_log.close()
            listener_log = None
            episode_log.flush()
            episode_log.close()
            episode_log = None
            activation = _load(work / "activation.yaml")
            evaluation = _load(work / "episode/evaluation.yaml")
            semantic = _validate_semantics(
                row, activation, evaluation, minimum
            )
            post_scene = journal.verify_post_episode_scene()
            try:
                _run_command(
                    _control_command(
                        "restore", row, teardown_path, runtime
                    ),
                    environment,
                    work / "restore.log",
                    25.0,
                )
            except R6I1ExecutionError as exc:
                raise integrity.R6TeardownFailure(str(exc)) from exc
            receipt = _load(teardown_path)
            if receipt.get("supervisor_config_sha256") != runtime[
                "supervisor_sha256"
            ]:
                raise integrity.R6TeardownFailure(
                    "teardown supervisor config provenance mismatched"
                )
            verified_teardown = integrity.verify_teardown_restore(
                receipt,
                journal.startup_profile_lease,
                post_scene,
                identity,
            )
            resources = _populate_raw(work, raw, teardown_path)
            binding = integrity.bind_attempt_raw_evidence(
                WORKSPACE,
                raw,
                identity,
                resources,
                minimum,
                journal.startup_profile_lease,
                post_scene,
            )
            journal.authorize_launch_stop(verified_teardown)
            _stop_process(transaction, transaction_log)
            transaction = transaction_log = None
            _stop_process(base, base_log)
            base = base_log = None
            journal.complete(binding)
            ledger_entry.update({
                "status": "evidence_complete",
                "seed_consumed": True,
                "initial_readback": _aux_row(initial_path),
                "transaction_startup": _aux_row(startup_path),
                "arm_receipt": _aux_row(arm_path),
                "raw_resources": resources,
                "expected_ttc_status": row["expected_ttc_status"],
                "observed_ttc_status": evaluation["ttc_status"],
                "semantic_observation": semantic,
                "supervisor_config_sha256": runtime[
                    "supervisor_sha256"
                ],
            })
            return ledger_entry
        except BaseException as exc:
            # Best-effort restore is attempted while move_base is still alive.
            if transaction is not None and transaction.poll() is None:
                try:
                    if not teardown_path.exists():
                        _run_command(
                            _control_command(
                                "restore", row, teardown_path, runtime
                            ),
                            environment,
                            work / "emergency_restore.log",
                            25.0,
                        )
                except BaseException:
                    pass
            phase = journal.lifecycle_phase
            if phase == "post_episode_scene_verified":
                # A post-episode terminal record must still have all six raw
                # files.  Persist a failed receipt if the service vanished.
                if not teardown_path.exists():
                    _atomic_yaml(teardown_path, {
                        **identity,
                        "identity": identity,
                        "schema_version": "2.0",
                        "record_type": "r6_two_phase_teardown_receipt",
                        "status": "fail",
                        "failure_reason": str(exc),
                        "restore_requested_while_backend_alive": False,
                        "transaction_acknowledged": False,
                        "transaction_readback_match": False,
                        "independent_readback_match": False,
                    })
            try:
                terminal_resources = _terminal_raw(
                    work, raw, identity, phase, exc
                )
                terminal = integrity.bind_terminal_attempt_evidence(
                    WORKSPACE, raw, identity, terminal_resources
                )
                journal.attach_terminal_evidence(terminal)
            except BaseException as evidence_exc:
                ledger_entry["terminal_evidence_error"] = (
                    "{}: {}".format(type(evidence_exc).__name__, evidence_exc)
                )
            ledger_entry.update({
                "status": "terminal_failure_pending_journal_exit",
                "failure_type": type(exc).__name__,
                "failure_reason": str(exc),
                "emergency_process_containment": True,
            })
            _stop_process(listener, listener_log)
            _stop_process(episode, episode_log)
            _stop_process(transaction, transaction_log)
            _stop_process(base, base_log)
            raise


def _base_report(prereg, authorization):
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "in_progress",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "execution_authorization": {
            "path": str(
                Path(authorization["_path"]).relative_to(WORKSPACE)
            ),
            "sha256": authorization["_sha256"],
        },
        "evidence_budget_authorized": 6,
        "evidence_units_consumed": 0,
        "r5_remaining_units_consumed": 0,
        "held_out_5001_5010_accessed": False,
        "retry_count": 0,
        "resume_used": False,
        "attempt_limit_per_identity": 1,
        "planned_identity_count": len(prereg["schedule"]),
        "attempt_ledger": [],
        "terminal_failure": None,
        "assessment_complete": False,
        "winner_ranked_or_frozen": False,
    }


def execute(authorization_path, authorization_sha256):
    prereg, _, _, authorization = _verify_preflight(
        authorization_path, authorization_sha256
    )
    if EXECUTION_ROOT.exists() or STAGE_REPORT.exists():
        # Entering an existing identity is forbidden.  The journal itself seals
        # a non-terminal orphan before rejecting any attempted resume.
        raise R6I1ExecutionError(
            "R6-I1 execution state already exists; retry/resume forbidden"
        )
    authorization["_path"] = str(Path(authorization_path).resolve())
    authorization["_sha256"] = authorization_sha256
    EXECUTION_ROOT.mkdir(parents=True, exist_ok=False)
    report = _base_report(prereg, authorization)
    _atomic_yaml(STAGE_REPORT, report)
    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    for row in prereg["schedule"]:
        ledger = {
            "sequence": row["sequence"],
            "identity": _identity(row),
            "status": "scheduled",
            "seed_consumed": False,
            "evidence_units_consumed": 0,
            "attempt_limit": 1,
            "resume_forbidden": True,
        }
        report["attempt_ledger"].append(ledger)
        _atomic_yaml(STAGE_REPORT, report)
        try:
            _run_attempt(row, prereg, environment, ledger, report)
            report["evidence_units_consumed"] = sum(
                item.get("evidence_units_consumed", 0)
                for item in report["attempt_ledger"]
            )
            _atomic_yaml(STAGE_REPORT, report)
        except BaseException as exc:
            try:
                journal = _load(
                    integrity.canonical_attempt_state_path(
                        JOURNAL_ROOT, _identity(row)
                    )
                )
                ledger["status"] = journal.get(
                    "status", "terminal_failure"
                )
            except BaseException:
                ledger["status"] = "terminal_failure"
            ledger.update({
                "failure_type": type(exc).__name__,
                "failure_reason": str(exc),
                "resume_forbidden": True,
            })
            report.update({
                "status": "terminal_failure",
                "evidence_units_consumed": sum(
                    item.get("evidence_units_consumed", 0)
                    for item in report["attempt_ledger"]
                ),
                "terminal_failure": {
                    "identity": _identity(row),
                    "failure_type": type(exc).__name__,
                    "reason": str(exc),
                },
                "unattempted_budget_forfeited": (
                    6 - sum(
                        item.get("evidence_units_consumed", 0)
                        for item in report["attempt_ledger"]
                    )
                ),
                "resume_forbidden": True,
            })
            _atomic_yaml(STAGE_REPORT, report)
            if _process_matches():
                raise R6I1ExecutionError(
                    "terminal containment left a forbidden process"
                ) from exc
            raise
    report.update({
        "status": "execution_complete_pending_assessment",
        "evidence_units_consumed": 6,
        "unattempted_budget_forfeited": 0,
        "resume_forbidden": True,
    })
    _atomic_yaml(STAGE_REPORT, report)
    if _process_matches():
        raise R6I1ExecutionError(
            "completed execution left a forbidden process"
        )
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


def dry_run():
    prereg, _, closure, _ = _verify_preflight()
    print(
        "R6-I1 integration dry-run: {} identities, {} files; "
        "no authorization, journal, process, or seed consumption".format(
            len(prereg["schedule"]), closure["file_count"]
        )
    )
    for row in prereg["schedule"]:
        print(
            "{:02d} {} {} seed={} expected={}".format(
                row["sequence"],
                row["profile_id"],
                row["scene_id"],
                row["seed"],
                row["expected_ttc_status"],
            )
        )
    return 0


def sha256_bytes(payload):
    return hashlib.sha256(payload).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--authorization")
    parser.add_argument("--authorization-sha256")
    args = parser.parse_args()
    if args.dry_run:
        if args.authorization or args.authorization_sha256:
            parser.error("dry-run cannot accept execution authorization")
        return dry_run()
    if not args.authorization or not args.authorization_sha256:
        parser.error("execute requires authorization path and SHA256")
    return execute(args.authorization, args.authorization_sha256)


if __name__ == "__main__":
    raise SystemExit(main())
