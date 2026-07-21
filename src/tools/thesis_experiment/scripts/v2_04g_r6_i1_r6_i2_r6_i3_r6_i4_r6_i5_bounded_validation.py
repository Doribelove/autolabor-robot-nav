#!/usr/bin/env python3
"""Fail-closed runner for the bounded R6-I5 simulation schedule.

The canonical release does not exist during the execution-readiness review.
The current user instruction authorizes its later creation and this bounded
simulation execution, but this program cannot create a directory, journal, or
subprocess until both caller-supplied hashes and every release/closure binding
have passed the dedicated validator.

Only Python standard-library modules are imported at module import time.
Workspace, YAML, ROS, and process-launching code is loaded after the trust
bootstrap and, for execution, after the release gate.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import stat
import sys
import time
import types


sys.dont_write_bytecode = True

WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R6-I5"
EXPECTED_AUTHORIZATION_SHA256 = (
    "bc59820b0140b50503657966d735511a8007d9ec8e14f3f2cf237791ff170592"
)
EXPECTED_SCHEDULE_SHA256 = (
    "b52d00a2dc0c1f2edf149d30120451ea836fc1d0589109a1016dc48e9a9d5402"
)

PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_execution_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_bounded_simulation_authorization.yaml"
)
RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_execution_release.yaml"
)
READINESS_ROOT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution"
)
COMPILED_INDEX_RELATIVE = (
    READINESS_ROOT_RELATIVE / "compiled_scenes/compiled_scene_index.yaml"
)
CLOSURE_RELATIVE = READINESS_ROOT_RELATIVE / "execution_dependency_closure.yaml"
MACHINE_REVIEW_RELATIVE = (
    READINESS_ROOT_RELATIVE / "v2_04g_r6_i5_execution_readiness_review.yaml"
)
ATTEMPTS_ROOT = WORKSPACE / READINESS_ROOT_RELATIVE / "attempts"
JOURNAL_ROOT = WORKSPACE / READINESS_ROOT_RELATIVE / "journals"
STAGE_REPORT = (
    WORKSPACE / READINESS_ROOT_RELATIVE / "v2_04g_r6_i5_stage_report.yaml"
)
RUNTIME_ROOT = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/runtime_candidate_configs"
)

ENTRYPOINT_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_bounded_validation.py"
)
LISTENER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_activation_probe_listener.py"
)
EPISODE_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_mechanism_episode.py"
)
CONTROL_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_runtime_control.py"
)
RELEASE_VALIDATOR_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_release.py"
)
RELEASE_VALIDATOR_TEST_RELATIVE = Path(
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i5_release_validator.py"
)
ASSESSOR_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_assessment.py"
)
ASSESSOR_TEST_RELATIVE = Path(
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i5_assessment.py"
)
I2_BOOTSTRAP_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_bootstrap.py"
)
INTEGRITY_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_integrity.py"
)

EXPECTED_RELEASE_VALIDATOR_SHA256 = (
    "aab501ca0f8a227a75ca3dc913d4cb0591c3497aa79178af0026d1fc766d775f"
)
EXPECTED_RELEASE_VALIDATOR_TEST_SHA256 = (
    "bac5f85d399a90cd17c47d04d718d8554c689354add9f88df287ae8f84e6c10c"
)
EXPECTED_ASSESSOR_SHA256 = (
    "de584f6e9a94fc4c8b02d1b733688c620bb56c31d89289295f6b8a3472c2fe5a"
)
EXPECTED_ASSESSOR_TEST_SHA256 = (
    "c96b51748803e8d1fd6833aae7cf207f59295b6def7ed4d540e35583980ab50c"
)
TRUSTED_MODULE_SHA256 = {
    I2_BOOTSTRAP_RELATIVE: (
        "a5e7a6905d88eeb01a13e742f93fed9512e38e3ddef439655c241b1999d7ddda"
    ),
    INTEGRITY_RELATIVE: (
        "65887068fcc1d98296a04eb1b8d87f2d6e29139365555bdfa27323efee9b89f8"
    ),
    RELEASE_VALIDATOR_RELATIVE: EXPECTED_RELEASE_VALIDATOR_SHA256,
    RELEASE_VALIDATOR_TEST_RELATIVE: EXPECTED_RELEASE_VALIDATOR_TEST_SHA256,
    ASSESSOR_RELATIVE: EXPECTED_ASSESSOR_SHA256,
    ASSESSOR_TEST_RELATIVE: EXPECTED_ASSESSOR_TEST_SHA256,
}

EXPECTED_MACHINE_REVIEW_STATUS = (
    "r6_i5_execution_readiness_closure_pass_release_absent"
)
ROSLAUNCH_BINDING = "command-executable:roslaunch"
ROSSERVICE_BINDING = "command-executable:rosservice"
ROSTOPIC_BINDING = "command-executable:rostopic"
XACRO_BINDING = "package-executable:xacro:xacro"

# Machine-readable trust-policy marker used by the readiness reviewer:
# caller_supplied_exact_release_sha256

EXPECTED_SCHEDULE = [
    {
        "sequence": 1,
        "profile_id": "r6_semantics_legacy_control",
        "scene_id": "v2-04g-r6-i5-dynamic-conflict-single-s5161",
        "seed": 5161,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none",
    },
    {
        "sequence": 2,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r6-i5-dynamic-conflict-single-s5161",
        "seed": 5161,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none_iff_finite_ttc",
    },
    {
        "sequence": 3,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r6-i5-dynamic-conflict-multi-s5162",
        "seed": 5162,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none_iff_finite_ttc",
    },
    {
        "sequence": 4,
        "profile_id": "r6_semantics_legacy_control",
        "scene_id": "v2-04g-r6-i5-dynamic-conflict-multi-s5162",
        "seed": 5162,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none",
    },
    {
        "sequence": 5,
        "profile_id": "r6_semantics_legacy_control",
        "scene_id": "v2-04g-r6-i5-dynamic-semantic-clear-s5163",
        "seed": 5163,
        "attempt": 1,
        "expected_ttc_status": "NO_CONFLICT_IN_HORIZON",
        "expected_overlay_semantics": "legacy_non_none_identifiability",
    },
    {
        "sequence": 6,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r6-i5-dynamic-semantic-clear-s5163",
        "seed": 5163,
        "attempt": 1,
        "expected_ttc_status": "NO_CONFLICT_IN_HORIZON",
        "expected_overlay_semantics": "none_iff_no_finite_ttc",
    },
]

_SCENE_NAMES = (
    "v2-04g-r6-i5-dynamic-conflict-single-s5161.instance.yaml",
    "v2-04g-r6-i5-dynamic-conflict-single-s5161.world",
    "v2-04g-r6-i5-dynamic-conflict-multi-s5162.instance.yaml",
    "v2-04g-r6-i5-dynamic-conflict-multi-s5162.world",
    "v2-04g-r6-i5-dynamic-semantic-clear-s5163.instance.yaml",
    "v2-04g-r6-i5-dynamic-semantic-clear-s5163.world",
    "v2-04g-r6-i5-compile-support-cruise-s5164.instance.yaml",
    "v2-04g-r6-i5-compile-support-cruise-s5164.world",
    "v2-04g-r6-i5-compile-support-static-s5165.instance.yaml",
    "v2-04g-r6-i5-compile-support-static-s5165.world",
    "v2-04g-r6-i5-compile-support-corridor-s5166.instance.yaml",
    "v2-04g-r6-i5-compile-support-corridor-s5166.world",
    "v2-04g-r6-i5-compile-support-maneuver-s5167.instance.yaml",
    "v2-04g-r6-i5-compile-support-maneuver-s5167.world",
)
EXPECTED_RELEASE_RESOURCE_PATHS = {
    "execution_contract": (
        "config/thesis_experiments/v2/"
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_"
        "bounded_simulation_execution_contract.yaml"
    ),
    "preregistration": PREREGISTRATION_RELATIVE.as_posix(),
    "authorization_envelope": AUTHORIZATION_RELATIVE.as_posix(),
    "stage_transition": (
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i5_stage_transition.yaml"
    ),
    "scene_derivation": (
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i5_scene_derivation.yaml"
    ),
    "fresh_scene_index": COMPILED_INDEX_RELATIVE.as_posix(),
    "execution_entrypoint": ENTRYPOINT_RELATIVE.as_posix(),
    "release_validator": RELEASE_VALIDATOR_RELATIVE.as_posix(),
    "release_validator_tests": RELEASE_VALIDATOR_TEST_RELATIVE.as_posix(),
    "execution_dependency_closure": CLOSURE_RELATIVE.as_posix(),
    "execution_machine_review": MACHINE_REVIEW_RELATIVE.as_posix(),
    "i4_repaired_validator": (
        "src/tools/thesis_experiment/src/thesis_experiment/"
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_release.py"
    ),
    "i4_dependency_closure": (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "r6_i4_preflight_repair_review/execution_dependency_closure.yaml"
    ),
    "i4_machine_review": (
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "r6_i4_preflight_repair_review/"
        "v2_04g_r6_i4_preflight_integrity_readiness_review.yaml"
    ),
    "failed_i3_release": (
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i3_execution_release.yaml"
    ),
}
EXPECTED_RELEASE_RESOURCE_PATHS.update({
    "fresh_scene_child_{:02d}".format(index): (
        READINESS_ROOT_RELATIVE / "compiled_scenes" / name
    ).as_posix()
    for index, name in enumerate(_SCENE_NAMES, start=1)
})

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
FORBIDDEN_LISTEN_PORTS = (11311, 11345, 11346, 11347)
MAX_TRUSTED_SOURCE_BYTES = 32 * 1024 * 1024


class R6I5ExecutionError(RuntimeError):
    """Fail-closed R6-I5 execution error."""


def _is_sha256(value):
    return (
        isinstance(value, str)
        and len(value) == 64
        and value == value.lower()
        and all(character in "0123456789abcdef" for character in value)
    )


def _read_workspace_bytes_once(relative):
    """Read one workspace file through component-wise no-follow descriptors."""

    if not WORKSPACE.is_absolute() or WORKSPACE != WORKSPACE.resolve():
        raise R6I5ExecutionError("workspace root is not canonical")
    if WORKSPACE.is_symlink() or not WORKSPACE.is_dir():
        raise R6I5ExecutionError("workspace root is unsafe")
    declared = Path(relative)
    if (
        declared.is_absolute()
        or not declared.parts
        or any(part in {"", ".", ".."} for part in declared.parts)
    ):
        raise R6I5ExecutionError("trusted path is unsafe: {}".format(relative))
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags = (
        flags
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptors = []
    try:
        current = os.open(str(WORKSPACE), directory_flags)
        descriptors.append(current)
        for component in declared.parts[:-1]:
            current = os.open(component, directory_flags, dir_fd=current)
            descriptors.append(current)
        file_descriptor = os.open(
            declared.parts[-1],
            flags | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=current,
        )
        descriptors.append(file_descriptor)
        before = os.fstat(file_descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise R6I5ExecutionError(
                "trusted dependency is not regular: {}".format(relative)
            )
        if before.st_size > MAX_TRUSTED_SOURCE_BYTES:
            raise R6I5ExecutionError(
                "trusted dependency is too large: {}".format(relative)
            )
        chunks = []
        remaining = MAX_TRUSTED_SOURCE_BYTES + 1
        while remaining:
            chunk = os.read(file_descriptor, min(1024 * 1024, remaining))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        payload = b"".join(chunks)
        after = os.fstat(file_descriptor)
        stable_fields = (
            "st_dev", "st_ino", "st_mode", "st_size", "st_mtime_ns",
            "st_ctime_ns",
        )
        if not (
            len(payload) == before.st_size
            and all(
                getattr(before, field) == getattr(after, field)
                for field in stable_fields
            )
        ):
            raise R6I5ExecutionError(
                "trusted dependency changed during read: {}".format(relative)
            )
        return {
            "relative": declared.as_posix(),
            "path": str(WORKSPACE / declared),
            "payload": payload,
            "sha256": hashlib.sha256(payload).hexdigest(),
        }
    except OSError as exc:
        raise R6I5ExecutionError(
            "cannot safely open {}: {}".format(relative, exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass


def _trusted_module_snapshots():
    snapshots = {}
    for relative, expected_sha256 in TRUSTED_MODULE_SHA256.items():
        if not _is_sha256(expected_sha256):
            raise R6I5ExecutionError(
                "trusted module SHA256 is not finalized: {}".format(relative)
            )
        snapshot = _read_workspace_bytes_once(relative)
        if snapshot["sha256"] != expected_sha256:
            raise R6I5ExecutionError(
                "trusted module SHA256 drifted: {}".format(relative)
            )
        snapshots[relative] = snapshot
    return snapshots


def _namespace_package():
    name = "thesis_experiment"
    package = sys.modules.get(name)
    if package is None:
        package = types.ModuleType(name)
        package.__package__ = name
        package.__path__ = [
            str(
                WORKSPACE
                / "src/tools/thesis_experiment/src/thesis_experiment"
            )
        ]
        sys.modules[name] = package
    return package


def _load_verified_module(snapshot, full_name):
    """Execute the already verified bytes without reopening the source."""

    package = _namespace_package()
    module = types.ModuleType(full_name)
    module.__file__ = snapshot["path"]
    module.__package__ = full_name.rpartition(".")[0]
    sys.modules[full_name] = module
    try:
        exec(
            compile(snapshot["payload"], snapshot["path"], "exec"),
            module.__dict__,
        )
    except BaseException:
        sys.modules.pop(full_name, None)
        raise
    setattr(package, full_name.rpartition(".")[2], module)
    return module


def _runtime_state_present():
    """Return a symlink-sensitive snapshot of every execution-state path."""

    i3_root = Path(
        "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution"
    )
    i4_root = Path(
        "artifacts/v2/integration/v2_04g_r6_i1/"
        "r6_i4_preflight_repair_review"
    )
    paths = {
        "release": RELEASE_RELATIVE,
        "failed_i3_release": Path(
            "experiments/manifests/v2/integration/"
            "v2_04g_r6_i3_execution_release.yaml"
        ),
        "future_i4_release": Path(
            "experiments/manifests/v2/integration/"
            "v2_04g_r6_i4_execution_release.yaml"
        ),
        "attempts_root": READINESS_ROOT_RELATIVE / "attempts",
        "journals_root": READINESS_ROOT_RELATIVE / "journals",
        "receipts_root": READINESS_ROOT_RELATIVE / "receipts",
        "raw_evidence_root": READINESS_ROOT_RELATIVE / "raw_evidence",
        "semantic_evidence_root": READINESS_ROOT_RELATIVE / "semantic_evidence",
        "ros_home": READINESS_ROOT_RELATIVE / "ros_home",
        "ros_logs": READINESS_ROOT_RELATIVE / "ros_logs",
        "stage_report": READINESS_ROOT_RELATIVE / "v2_04g_r6_i5_stage_report.yaml",
        "execution_report": (
            READINESS_ROOT_RELATIVE / "v2_04g_r6_i5_execution_report.yaml"
        ),
        "i3_attempts": i3_root / "attempts",
        "i3_journals": i3_root / "journals",
        "i3_receipts": i3_root / "receipts",
        "i3_raw_evidence": i3_root / "raw_evidence",
        "i3_semantic_evidence": i3_root / "semantic_evidence",
        "i3_stage_report": i3_root / "v2_04g_r6_i3_stage_report.yaml",
        "i3_execution_report": i3_root / "v2_04g_r6_i3_execution_report.yaml",
        "i4_attempts": i4_root / "attempts",
        "i4_journals": i4_root / "journals",
        "i4_receipts": i4_root / "receipts",
        "i4_raw_evidence": i4_root / "raw_evidence",
        "i4_semantic_evidence": i4_root / "semantic_evidence",
        "i4_ros_home": i4_root / "ros_home",
        "i4_ros_logs": i4_root / "ros_logs",
        "i4_stage_report": i4_root / "v2_04g_r6_i4_stage_report.yaml",
        "i4_execution_report": i4_root / "v2_04g_r6_i4_execution_report.yaml",
    }
    return {
        label: os.path.lexists(str(WORKSPACE / relative))
        for label, relative in paths.items()
    }


def _expected_runtime_state(release_present):
    expected = {key: False for key in _runtime_state_present()}
    expected["release"] = bool(release_present)
    expected["failed_i3_release"] = True
    return expected


def _process_matches():
    current = os.getpid()
    matches = []
    try:
        process_entries = list(Path("/proc").iterdir())
    except OSError as exc:
        raise R6I5ExecutionError(
            "cannot enumerate /proc for host isolation: {}".format(exc)
        ) from exc
    for item in process_entries:
        if not item.name.isdigit() or int(item.name) == current:
            continue
        try:
            command = (item / "cmdline").read_bytes().replace(
                b"\0", b" "
            ).decode("utf-8", errors="replace").strip()
        except (FileNotFoundError, ProcessLookupError):
            continue
        except OSError as exc:
            raise R6I5ExecutionError(
                "cannot inspect process {}: {}".format(item.name, exc)
            ) from exc
        executable = Path(command.split(" ", 1)[0]).name.lower()
        tokens = command.lower().split()
        matched = (
            executable in PROCESS_MARKERS
            or any(Path(token).name in PROCESS_MARKERS for token in tokens[:3])
            or any(
                marker in command.lower()
                for marker in ("sac_train.py", "residual_train.py")
            )
        )
        if matched:
            # Never propagate another process's argv into diagnostics: it may
            # contain credentials even though this runner's own argv policy is
            # closed.  The basename and policy label are sufficient to stop.
            matches.append({
                "pid": int(item.name),
                "executable_basename": executable or "unknown",
                "policy_label": "forbidden_ros_gazebo_or_training_process",
            })
    return sorted(matches, key=lambda row: row["pid"])


def _forbidden_listening_ports():
    """Read Linux socket tables directly; any inspection failure fails closed."""

    found = set()
    for table in (Path("/proc/net/tcp"), Path("/proc/net/tcp6")):
        try:
            lines = table.read_text(encoding="ascii").splitlines()
        except OSError as exc:
            raise R6I5ExecutionError(
                "cannot inspect host TCP listeners: {}".format(exc)
            ) from exc
        for line in lines[1:]:
            fields = line.split()
            if len(fields) < 4 or fields[3] != "0A":
                continue
            try:
                port = int(fields[1].rsplit(":", 1)[1], 16)
            except (IndexError, ValueError) as exc:
                raise R6I5ExecutionError(
                    "host TCP listener table is malformed"
                ) from exc
            if port in FORBIDDEN_LISTEN_PORTS:
                found.add(port)
    return sorted(found)


def _assert_host_isolated(label):
    processes = _process_matches()
    ports = _forbidden_listening_ports()
    if processes or ports:
        raise R6I5ExecutionError(
            "{} host isolation failed: processes={} ports={}".format(
                label, processes, ports
            )
        )


def offline_review():
    """Verify the non-executing runner boundary without imports or processes."""

    _assert_host_isolated("offline review before")
    snapshots = _trusted_module_snapshots()
    state = _runtime_state_present()
    if state != _expected_runtime_state(False):
        raise R6I5ExecutionError(
            "offline review found execution material: {}".format(state)
        )
    result = {
        "schema_version": "1.0",
        "stage": STAGE,
        "status": "runner_offline_review_pass_execution_release_absent",
        "execution_authorized_by_explicit_user_instruction": True,
        "execution_release_present": False,
        "execution_ready": False,
        "execution_start_gate_passed": False,
        "journal_or_attempt_root_created": False,
        "ros_or_subprocess_started": False,
        "seed_or_evidence_units_consumed": 0,
        "exact_schedule_sha256": EXPECTED_SCHEDULE_SHA256,
        "exact_identity_count": len(EXPECTED_SCHEDULE),
        "trusted_module_sha256": {
            snapshot["relative"]: snapshot["sha256"]
            for snapshot in snapshots.values()
        },
        "bootstrap_order": [
            "base_spawn",
            "unpause_request",
            "successful_unpause_ack",
            "first_strictly_positive_post_ack_clock",
            "second_strictly_greater_positive_post_ack_clock",
            "release_move_base_and_teb_service_wait",
        ],
    }
    _assert_host_isolated("offline review after")
    if _runtime_state_present() != state:
        raise R6I5ExecutionError("offline review changed execution state")
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0


def _canonical_cli_path(raw, expected_relative, label):
    if not isinstance(raw, str) or not raw:
        raise R6I5ExecutionError("{} path is required".format(label))
    candidate = Path(raw)
    if not candidate.is_absolute():
        candidate = WORKSPACE / candidate
    if candidate.absolute() != (WORKSPACE / expected_relative).absolute():
        raise R6I5ExecutionError("{} path drifted".format(label))
    return expected_relative.as_posix()


def _exact(actual, expected):
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            _exact(actual[key], expected[key]) for key in expected
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            _exact(left, right) for left, right in zip(actual, expected)
        )
    return actual == expected


def _runtime_executable(validation, binding):
    executables = validation.runtime_executables
    if binding not in executables:
        raise R6I5ExecutionError(
            "release closure lacks runtime executable {}".format(binding)
        )
    raw = executables[binding]
    path = Path(raw)
    if not path.is_absolute() or path != path.resolve():
        raise R6I5ExecutionError(
            "runtime executable is not canonical: {}".format(binding)
        )
    if not path.is_file() or path.is_symlink() or not os.access(str(path), os.X_OK):
        raise R6I5ExecutionError(
            "runtime executable is unsafe: {}".format(binding)
        )
    return str(path)


def _execution_preflight(
    release_path,
    release_sha256,
    authorization_path,
    authorization_sha256,
):
    """Complete every read-only trust gate before any execution mutation."""

    if not _is_sha256(release_sha256):
        raise R6I5ExecutionError("caller release SHA256 is invalid")
    if authorization_sha256 != EXPECTED_AUTHORIZATION_SHA256:
        raise R6I5ExecutionError("caller authorization SHA256 drifted")
    release_declared = _canonical_cli_path(
        release_path, RELEASE_RELATIVE, "release"
    )
    authorization_declared = _canonical_cli_path(
        authorization_path, AUTHORIZATION_RELATIVE, "authorization"
    )

    snapshots = _trusted_module_snapshots()
    release_module = _load_verified_module(
        snapshots[RELEASE_VALIDATOR_RELATIVE],
        "thesis_experiment.v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_release",
    )
    preloaded_snapshots = {
        snapshot["relative"]: release_module.FileSnapshot(
            declared_path=snapshot["relative"],
            path=Path(snapshot["path"]),
            sha256=snapshot["sha256"],
            size_bytes=len(snapshot["payload"]),
            payload=snapshot["payload"],
            document=None,
        )
        for snapshot in snapshots.values()
    }
    validation = release_module.load_and_validate_execution_release(
        WORKSPACE,
        release_declared,
        release_sha256,
        authorization_declared,
        authorization_sha256,
        expected_resource_paths=EXPECTED_RELEASE_RESOURCE_PATHS,
        expected_machine_review_status=EXPECTED_MACHINE_REVIEW_STATUS,
        preloaded_snapshots=preloaded_snapshots,
    )
    preregistration = validation.preregistration.document
    if not _exact(preregistration.get("schedule"), EXPECTED_SCHEDULE):
        raise R6I5ExecutionError("validated exact schedule drifted")
    if validation.schedule_sha256 != EXPECTED_SCHEDULE_SHA256:
        raise R6I5ExecutionError("validated schedule digest drifted")
    if (
        validation.identity_count != 6
        or tuple(validation.execution_seeds) != (5161, 5162, 5163)
    ):
        raise R6I5ExecutionError("validated seed/budget boundary drifted")

    # These modules are imported from the exact bytes already verified above,
    # never via importlib/path reopening.
    bootstrap_module = _load_verified_module(
        snapshots[I2_BOOTSTRAP_RELATIVE],
        "thesis_experiment.v2_04g_r6_i1_r6_i2_bootstrap",
    )
    integrity_module = _load_verified_module(
        snapshots[INTEGRITY_RELATIVE],
        "thesis_experiment.v2_04g_r6_integrity",
    )
    assessment_module = _load_verified_module(
        snapshots[ASSESSOR_RELATIVE],
        "thesis_experiment."
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_r6_i5_assessment",
    )
    executables = {
        "python": _runtime_executable(validation, "python_interpreter"),
        "roslaunch": _runtime_executable(validation, ROSLAUNCH_BINDING),
        "rosservice": _runtime_executable(validation, ROSSERVICE_BINDING),
        "rostopic": _runtime_executable(validation, ROSTOPIC_BINDING),
        "xacro": _runtime_executable(validation, XACRO_BINDING),
    }
    _assert_host_isolated("execution prejournal before unit validation")
    runtime_state = _runtime_state_present()
    expected_state = _expected_runtime_state(True)
    if runtime_state != expected_state:
        raise R6I5ExecutionError(
            "execution state is not fresh: {}".format(runtime_state)
        )
    context = {
        "validation": validation,
        "authorization": release_module,
        "bootstrap": bootstrap_module,
        "integrity": integrity_module,
        "assessment": assessment_module,
        "executables": executables,
        "preregistration": preregistration,
        "release_path": release_declared,
        "release_sha256": release_sha256,
        "authorization_path": authorization_declared,
        "authorization_sha256": authorization_sha256,
    }
    _prejournal_unit_gate(context)
    # Recheck host/state isolation after all unit-specific, scene and profile
    # validation.  Nothing above is allowed to create execution-owned state.
    _assert_host_isolated("execution prejournal after unit validation")
    if _runtime_state_present() != expected_state:
        raise R6I5ExecutionError(
            "execution state changed during full prejournal validation"
        )
    return context


def _atomic_yaml(path, value, yaml_module):
    import tempfile

    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = yaml_module.safe_dump(
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


def _closure_local_sha256(context, relative):
    closure = context["validation"].dependency_closure.document
    local = closure.get("local")
    files = local.get("files") if isinstance(local, dict) else None
    matches = [
        row for row in (files or [])
        if isinstance(row, dict) and row.get("path") == relative
    ]
    if len(matches) != 1 or not _is_sha256(matches[0].get("sha256")):
        raise R6I5ExecutionError(
            "local closure lacks exact resource {}".format(relative)
        )
    return matches[0]["sha256"]


def _closure_external_sha256(context, canonical_path):
    closure = context["validation"].dependency_closure.document
    external = closure.get("external")
    files = external.get("files") if isinstance(external, dict) else None
    matches = [
        row for row in (files or [])
        if isinstance(row, dict)
        and row.get("canonical_path") == canonical_path
    ]
    if len(matches) != 1 or not _is_sha256(matches[0].get("sha256")):
        raise R6I5ExecutionError(
            "external closure lacks exact resource {}".format(canonical_path)
        )
    return matches[0]["sha256"]


def _revalidate_workspace_dependency(context, relative):
    declared = Path(relative).as_posix()
    expected = _closure_local_sha256(context, declared)
    snapshot = context["authorization"].read_workspace_file_once(
        WORKSPACE, declared, parse_yaml=False
    )
    if snapshot.sha256 != expected:
        raise R6I5ExecutionError(
            "workspace dependency drifted immediately before use: {}".format(
                declared
            )
        )


def _revalidate_external_dependency(context, canonical_path):
    expected = _closure_external_sha256(context, canonical_path)
    payload = context["authorization"]._read_external_absolute_bytes_once(
        canonical_path
    )
    if hashlib.sha256(payload).hexdigest() != expected:
        raise R6I5ExecutionError(
            "external dependency drifted immediately before use: {}".format(
                canonical_path
            )
        )


def _revalidate_command_dependencies(command, context):
    """Close the preflight-to-Popen window for every path-bearing command."""

    if not command:
        raise R6I5ExecutionError("empty subprocess command")
    candidates = [str(command[0])]
    for token in command[1:]:
        value = str(token).split(":=", 1)[-1]
        if value.startswith("/"):
            candidates.append(value)
    seen = set()
    for raw in candidates:
        path = Path(raw)
        if raw in seen or not path.is_absolute():
            continue
        seen.add(raw)
        # Attempt-owned outputs and immutable scene snapshots are protected by
        # the atomic journal/snapshot layer, not the dependency closure.
        try:
            path.relative_to(ATTEMPTS_ROOT)
            continue
        except ValueError:
            pass
        if not os.path.lexists(str(path)):
            continue
        try:
            relative = path.relative_to(WORKSPACE)
        except ValueError:
            _revalidate_external_dependency(context, path.as_posix())
        else:
            _revalidate_workspace_dependency(context, relative)

    # roslaunch resolves package/name pairs internally.  Bind the exact local
    # launch source again immediately before that resolver is invoked.
    if Path(str(command[0])).name == "roslaunch" and len(command) >= 3:
        launch_name = str(command[2])
        closure = context["validation"].dependency_closure.document
        rows = closure.get("local", {}).get("files", [])
        matches = [
            row["path"] for row in rows
            if isinstance(row, dict)
            and isinstance(row.get("path"), str)
            and row["path"].endswith("/launch/" + launch_name)
        ]
        if len(matches) != 1:
            raise R6I5ExecutionError(
                "roslaunch source closure is not unique: {}".format(
                    launch_name
                )
            )
        _revalidate_workspace_dependency(context, matches[0])


def _prejournal_unit_gate(context):
    """Validate every scheduled scene/profile before execution state exists."""

    integrity = context["integrity"]
    index = context["validation"].bound_resources["fresh_scene_index"]
    checked_profiles = set()
    checked_scenes = set()
    for row in context["preregistration"]["schedule"]:
        if row["profile_id"] not in checked_profiles:
            _runtime_profile(row, context)
            checked_profiles.add(row["profile_id"])
        if row["scene_id"] in checked_scenes:
            continue
        lease = integrity.acquire_compiled_scene_lease(
            WORKSPACE,
            COMPILED_INDEX_RELATIVE,
            index.sha256,
            row["scene_id"],
        )
        scene = context["authorization"]._parse_yaml_mapping(
            lease.instance_bytes, "prejournal compiled scene instance"
        )["scene"]
        if not (
            scene.get("scene_id") == row["scene_id"]
            and type(scene.get("seed")) is int
            and scene["seed"] == row["seed"]
            and scene.get("family") == "DYNAMIC"
        ):
            raise R6I5ExecutionError(
                "prejournal compiled execution scene identity drifted"
            )
        checked_scenes.add(row["scene_id"])
    if checked_profiles != {
        "r6_semantics_legacy_control",
        "r6_semantics_circle_contact",
    } or checked_scenes != {
        "v2-04g-r6-i5-dynamic-conflict-single-s5161",
        "v2-04g-r6-i5-dynamic-conflict-multi-s5162",
        "v2-04g-r6-i5-dynamic-semantic-clear-s5163",
    }:
        raise R6I5ExecutionError("prejournal unit roster drifted")


def _runtime_profile(row, context):
    directory = RUNTIME_ROOT / row["profile_id"]
    relative_paths = {
        "supervisor": (directory / "supervisor.yaml").relative_to(WORKSPACE),
        "anchor_bank": (directory / "anchor_bank.yaml").relative_to(WORKSPACE),
        "mechanism": (directory / "mechanism.yaml").relative_to(WORKSPACE),
    }
    result = {}
    for label, relative in relative_paths.items():
        declared = relative.as_posix()
        snapshot = context["authorization"].read_workspace_file_once(
            WORKSPACE, declared
        )
        expected = _closure_local_sha256(context, declared)
        if snapshot.sha256 != expected:
            raise R6I5ExecutionError(
                "runtime profile closure drifted: {}".format(label)
            )
        result[label] = str(snapshot.path)
        result[label + "_sha256"] = snapshot.sha256
    return result


def _base_launch_command(row, scene, snapshot, runtime, context):
    document = snapshot.as_document()
    return [
        context["executables"]["roslaunch"],
        "m2_gazebo",
        "m2_v2_04g_r6_i2_execution_integration.launch",
        "world:={}".format(document["snapshot_world"]["path"]),
        "seed:={}".format(row["seed"]),
        "xacro_executable:={}".format(context["executables"]["xacro"]),
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


def _transaction_launch_command(row, runtime, context):
    return [
        context["executables"]["roslaunch"],
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


def _control_command(mode, row, output, runtime, context):
    command = [
        context["executables"]["python"],
        str(WORKSPACE / CONTROL_RELATIVE),
        "--mode", mode,
        "--output", str(output),
        "--stage", STAGE,
        "--profile-id", row["profile_id"],
        "--scene-id", row["scene_id"],
        "--seed", str(row["seed"]),
        "--attempt", "1",
    ]
    if mode == "initial-readback":
        command.extend(["--anchor-bank", runtime["anchor_bank"]])
    return command


def _listener_command(row, preregistration, output, context):
    gate = preregistration["readiness_gate"]
    return [
        context["executables"]["python"],
        str(WORKSPACE / LISTENER_RELATIVE),
        "--output", str(output),
        "--profile-id", row["profile_id"],
        "--scene-id", row["scene_id"],
        "--attempt", "1",
        "--repeat", str(row["sequence"]),
        "--seed", str(row["seed"]),
        "--warmup-timeout-s", str(gate["warmup_timeout_s"]),
        "--measurement-duration-s", str(gate["measurement_duration_s"]),
        "--minimum-message-count",
        str(gate["minimum_message_count_per_stream"]),
        "--minimum-valid-fraction", str(gate["minimum_valid_fraction"]),
        "--required-consecutive-stable-count",
        str(gate["required_consecutive_stable_count"]),
        "--maximum-expected-context-hold-count",
        str(gate["maximum_expected_context_hold_count_per_probe"]),
    ]


def _episode_command(row, instance_path, output, context):
    return [
        context["executables"]["python"],
        str(WORKSPACE / EPISODE_RELATIVE),
        "--instance", str(instance_path),
        "--method", "rule_multi_anchor",
        "--output-dir", str(output),
        "--stage", STAGE,
        "--split", "calibration",
        "--profile-id", row["profile_id"],
        "--attempt", "1",
    ]


def _validate_command(command, context, source_environment):
    return list(context["bootstrap"].validate_credential_safe_command(
        command, source_environment
    ))


def _subprocess_module():
    # Deliberately deferred: offline review never imports process-launch code.
    import subprocess

    return subprocess


def _spawn_sanitized(
    command,
    environment,
    log_path,
    context,
    source_environment,
):
    import threading

    subprocess_module = _subprocess_module()
    safe_command = _validate_command(command, context, source_environment)
    _revalidate_command_dependencies(safe_command, context)
    stream = Path(log_path).open("x", encoding="utf-8")
    process = None
    drain_errors = []
    try:
        process = subprocess_module.Popen(
            safe_command,
            env=environment,
            stdout=subprocess_module.PIPE,
            stderr=subprocess_module.STDOUT,
            start_new_session=True,
        )

        def drain():
            try:
                for payload in iter(process.stdout.readline, b""):
                    text_value = payload.decode("utf-8", errors="replace")
                    stream.write(context["bootstrap"].redact_credential_material(
                        text_value, source_environment
                    ))
                    stream.flush()
            except BaseException as exc:  # pragma: no cover - runtime only
                drain_errors.append(exc)

        thread = threading.Thread(
            target=drain,
            name="r6_i5_sanitized_process_log",
            daemon=True,
        )
        thread.start()
        return {
            "process": process,
            "thread": thread,
            "stream": stream,
            "drain_errors": drain_errors,
        }
    except BaseException:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        stream.close()
        raise


def _stop_process(handle, timeout_s=12.0):
    if handle is None:
        return
    subprocess_module = _subprocess_module()
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
            except subprocess_module.TimeoutExpired:
                continue
    handle["thread"].join(timeout=5.0)
    if process.stdout is not None:
        process.stdout.close()
    stream = handle["stream"]
    if not stream.closed:
        stream.flush()
        os.fsync(stream.fileno())
        stream.close()
    if handle["drain_errors"]:
        raise R6I5ExecutionError("sanitized process-log drain failed")
    if process.poll() is None:
        raise R6I5ExecutionError(
            "process group survived SIGINT/SIGTERM/SIGKILL containment"
        )


def _capture_command(
    command,
    environment,
    timeout_s,
    context,
    source_environment,
    log_path=None,
):
    subprocess_module = _subprocess_module()
    safe_command = _validate_command(command, context, source_environment)
    _revalidate_command_dependencies(safe_command, context)
    process = subprocess_module.Popen(
        safe_command,
        env=environment,
        stdout=subprocess_module.PIPE,
        stderr=subprocess_module.STDOUT,
        start_new_session=True,
    )
    try:
        payload, _ = process.communicate(timeout=timeout_s)
    except subprocess_module.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        payload, _ = process.communicate(timeout=5.0)
        raise R6I5ExecutionError(
            "command timed out: {}".format(Path(safe_command[0]).name)
        )
    text_value = payload.decode("utf-8", errors="replace")
    redacted = context["bootstrap"].redact_credential_material(
        text_value, source_environment
    )
    if log_path is not None:
        _exclusive_bytes(Path(log_path), redacted.encode("utf-8"))
    return process.returncode, payload


def _run_command(
    command,
    environment,
    log_path,
    timeout_s,
    context,
    source_environment,
):
    returncode, _ = _capture_command(
        command,
        environment,
        timeout_s,
        context,
        source_environment,
        log_path=log_path,
    )
    if returncode != 0:
        raise R6I5ExecutionError(
            "command exited {}: {}".format(returncode, Path(command[0]).name)
        )


def _base_process(handle):
    return handle["process"]


def _wait_for_service(
    base_handle,
    environment,
    service,
    timeout_s,
    context,
    source_environment,
):
    deadline = time.monotonic() + timeout_s
    command = [context["executables"]["rosservice"], "list"]
    while time.monotonic() < deadline:
        if _base_process(base_handle).poll() is not None:
            raise R6I5ExecutionError(
                "base roslaunch exited before service readiness"
            )
        try:
            result, payload = _capture_command(
                command,
                environment,
                min(3.0, max(0.1, deadline - time.monotonic())),
                context,
                source_environment,
            )
        except R6I5ExecutionError:
            result, payload = 1, b""
        if result == 0 and service in payload.decode(
            "utf-8", errors="replace"
        ).splitlines():
            return
        time.sleep(0.20)
    raise R6I5ExecutionError(
        "service readiness timed out: {}".format(service)
    )


_CLOCK_SEC = re.compile(rb"(?:^|\n)\s*secs:\s*([0-9]+)\s*(?:\n|$)")
_CLOCK_NSEC = re.compile(rb"(?:^|\n)\s*nsecs:\s*([0-9]+)\s*(?:\n|$)")


def _clock_sample(payload):
    sec = _CLOCK_SEC.search(payload)
    nsec = _CLOCK_NSEC.search(payload)
    if sec is None or nsec is None:
        raise R6I5ExecutionError("/clock probe output is malformed")
    return int(sec.group(1)), int(nsec.group(1))


def _bootstrap_clock_and_services(
    base_handle,
    environment,
    work,
    context,
    source_environment,
):
    """Enforce the repaired paused-clock ordering on a live base launch."""

    barrier = context["bootstrap"].R6I2PositiveClockBarrier(timeout_s=45.0)
    spawned = time.monotonic()
    barrier.mark_base_spawned(spawned)

    # Waiting for the Gazebo unpause endpoint is infrastructure discovery, not
    # the forbidden move_base/TEB readiness wait.  The latter appears only
    # after ``release_service_wait`` below.
    _wait_for_service(
        base_handle,
        environment,
        "/gazebo/unpause_physics",
        30.0,
        context,
        source_environment,
    )
    request_time = time.monotonic()
    barrier.observe_base_exit(_base_process(base_handle).poll(), request_time)
    barrier.mark_unpause_requested(request_time)
    returncode, _ = _capture_command(
        [
            context["executables"]["rosservice"],
            "call",
            "/gazebo/unpause_physics",
        ],
        environment,
        10.0,
        context,
        source_environment,
        log_path=work / "unpause.log",
    )
    acknowledgement_time = time.monotonic()
    barrier.observe_base_exit(
        _base_process(base_handle).poll(), acknowledgement_time
    )
    barrier.mark_unpause_acknowledged(
        acknowledgement_time, service_success=(returncode == 0)
    )

    clock_command = [
        context["executables"]["rostopic"],
        "echo", "-n", "1", "/clock",
    ]
    while not barrier.receipt()["clock_progression_sample"]:
        now = time.monotonic()
        barrier.observe_base_exit(_base_process(base_handle).poll(), now)
        barrier.check_deadline(now)
        try:
            result, payload = _capture_command(
                clock_command,
                environment,
                2.0,
                context,
                source_environment,
            )
        except R6I5ExecutionError:
            continue
        if result != 0:
            continue
        sec, nsec = _clock_sample(payload)
        observed = time.monotonic()
        barrier.observe_base_exit(_base_process(base_handle).poll(), observed)
        barrier.observe_clock(sec, nsec, observed)

    released = time.monotonic()
    barrier.release_service_wait(
        released, base_returncode=_base_process(base_handle).poll()
    )
    if not barrier.service_wait_allowed:
        raise R6I5ExecutionError("positive /clock barrier did not release")
    _wait_for_service(
        base_handle,
        environment,
        "/move_base/TebLocalPlannerROS/set_parameters",
        45.0,
        context,
        source_environment,
    )
    receipt = barrier.receipt()
    _atomic_yaml(
        work / "bootstrap_receipt.yaml",
        {
            "schema_version": "1.0",
            "stage": STAGE,
            "required_order": [
                "base_spawn",
                "unpause_request",
                "successful_unpause_ack",
                "first_strictly_positive_post_ack_clock",
                "second_strictly_greater_positive_post_ack_clock",
                "release_move_base_and_teb_service_wait",
            ],
            "barrier": receipt,
            "move_base_teb_service_wait_after_release": True,
        },
        context["authorization"].yaml,
    )
    return receipt


def _wait_action_trace(
    base_handle,
    transaction_handle,
    environment,
    log_path,
    context,
    source_environment,
):
    handle = _spawn_sanitized(
        [
            context["executables"]["rostopic"],
            "echo", "-n", "1", "/teb_rl_v2/action_trace",
        ],
        environment,
        log_path,
        context,
        source_environment,
    )
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if (
                _base_process(base_handle).poll() is not None
                or _base_process(transaction_handle).poll() is not None
            ):
                raise R6I5ExecutionError(
                    "launch exited before transaction readiness"
                )
            result = _base_process(handle).poll()
            if result is not None:
                if result != 0:
                    raise R6I5ExecutionError(
                        "action trace readiness probe failed"
                    )
                return
            time.sleep(0.1)
        raise R6I5ExecutionError("action trace readiness timed out")
    finally:
        _stop_process(handle)


def _validate_semantics(row, activation, evaluation, minimum, integrity):
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
        raise R6I5ExecutionError("R6-I5 readiness/evaluator gate failed")
    overlay = evaluation.get("context_overlay_sample_counts", {})
    non_none = sum(int(value) for key, value in overlay.items() if key != "NONE")
    finite = int(evaluation.get("finite_ttc_sample_count", 0))
    role = row["expected_overlay_semantics"]
    if role in {"non_none", "non_none_iff_finite_ttc"}:
        if finite <= 0 or non_none <= 0:
            raise R6I5ExecutionError(
                "finite conflict scene lacked TTC/overlay evidence"
            )
    elif role == "legacy_non_none_identifiability":
        if finite != 0 or non_none <= 0:
            raise R6I5ExecutionError(
                "legacy semantic-clear identifiability gate failed"
            )
    elif role == "none_iff_no_finite_ttc":
        if finite != 0 or non_none != 0:
            raise R6I5ExecutionError(
                "aligned no-finite-TTC eligibility parity failed"
            )
    else:
        raise R6I5ExecutionError("unknown overlay semantic role")
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
            "sha256": hashlib.sha256((raw / filename).read_bytes()).hexdigest(),
        }
        for label, filename in RAW_FILENAMES.items()
    }


def _terminal_raw(work, raw, identity, phase, reason, integrity):
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
        source = target if label == "process_log" else candidates.get(label)
        if source is not None and source.is_file():
            if source != target and not target.exists():
                _exclusive_bytes(target, source.read_bytes())
            resources[label] = {
                "status": "produced",
                "path": str(target.relative_to(WORKSPACE)),
                "sha256": hashlib.sha256(target.read_bytes()).hexdigest(),
            }
        else:
            resources[label] = {
                "status": "not_produced",
                "phase": phase,
                "reason": str(reason),
            }
    return resources


def _load_yaml(path, context):
    return context["integrity"].strict_yaml(path)


def _safe_failure_reason(context, reason, source_environment):
    """Return a bounded credential-redacted diagnostic for persistence."""

    failure_type = type(reason).__name__
    try:
        raw_reason = str(reason)
    except BaseException:
        return "{}: diagnostic unavailable".format(failure_type)
    try:
        redacted = context["bootstrap"].redact_credential_material(
            raw_reason, source_environment
        )
    except BaseException:
        return "{}: diagnostic redaction failed closed".format(failure_type)
    # Keep persisted diagnostics bounded and remove control characters that
    # could alter terminal rendering if a later reviewer prints the YAML.
    safe_characters = []
    for character in redacted:
        if character in {"\n", "\t"} or ord(character) >= 32:
            safe_characters.append(character)
        else:
            safe_characters.append("\\x{:02x}".format(ord(character)))
    bounded = "".join(safe_characters)
    if len(bounded) > 4096:
        bounded = bounded[:4096] + "...[diagnostic truncated]"
    return "{}: {}".format(failure_type, bounded or "diagnostic empty")


def _run_attempt(row, context, ledger_entry, report, source_environment):
    integrity = context["integrity"]
    yaml_module = context["authorization"].yaml
    identity = _identity(row)
    attempt_root = ATTEMPTS_ROOT / _attempt_name(row)
    work = attempt_root / "work"
    raw = attempt_root / "raw"
    runtime = _runtime_profile(row, context)
    index_snapshot = context["validation"].bound_resources["fresh_scene_index"]
    lease = integrity.acquire_compiled_scene_lease(
        WORKSPACE,
        COMPILED_INDEX_RELATIVE,
        index_snapshot.sha256,
        row["scene_id"],
    )
    scene_document = context["authorization"]._parse_yaml_mapping(
        lease.instance_bytes, "compiled scene instance"
    )
    if not (
        scene_document["scene"]["scene_id"] == row["scene_id"]
        and scene_document["scene"]["seed"] == row["seed"]
        and scene_document["scene"]["family"] == "DYNAMIC"
    ):
        raise R6I5ExecutionError("compiled execution scene identity drifted")

    child_environment, environment_audit = (
        context["bootstrap"].build_credential_safe_environment(
            source_environment, work
        )
    )
    context["bootstrap"].assert_credential_safe_environment(
        child_environment, work
    )
    base = transaction = listener = episode = None
    snapshot = None
    teardown_path = work / "teardown_receipt.yaml"
    minimum = context["preregistration"]["readiness_gate"][
        "minimum_message_count_per_stream"
    ]
    with integrity.AtomicAttemptJournal(JOURNAL_ROOT, identity) as journal:
        ledger_entry.update({
            "journal_root": str(JOURNAL_ROOT.relative_to(WORKSPACE)),
            "journal": str(journal.path.relative_to(WORKSPACE)),
            "raw_evidence_root": str(raw.relative_to(WORKSPACE)),
            "status": "attempt_started",
            "credential_safe_environment_audit": environment_audit,
        })
        _atomic_yaml(STAGE_REPORT, report, yaml_module)
        try:
            work.mkdir(parents=True, exist_ok=False)
            (work / "episode").mkdir()
            snapshot = integrity.materialize_scene_snapshot(
                lease, work / "scene_snapshot"
            )
            integrity.revalidate_scene_snapshot(snapshot, "pre_spawn")
            Path(child_environment["ROS_HOME"]).mkdir()
            Path(child_environment["ROS_LOG_DIR"]).mkdir()

            # Commit the single allowed attempt and its evidence unit before
            # the irreversible Popen request.  A power loss or spawn error can
            # never leave an unrecorded launch that could later be retried.
            ledger_entry.update({
                "status": "base_roslaunch_spawn_request_committed",
                "seed_consumed": True,
                "evidence_units_consumed": 1,
                "consumption_boundary": "base_roslaunch_spawn_requested",
            })
            report["evidence_units_consumed"] = sum(
                entry.get("evidence_units_consumed", 0)
                for entry in report["attempt_ledger"]
            )
            _atomic_yaml(STAGE_REPORT, report, yaml_module)
            base = _spawn_sanitized(
                _base_launch_command(
                    row, scene_document["scene"], snapshot, runtime, context
                ),
                child_environment,
                work / "base_launch.log",
                context,
                source_environment,
            )
            bootstrap_receipt = _bootstrap_clock_and_services(
                base,
                child_environment,
                work,
                context,
                source_environment,
            )
            ledger_entry["bootstrap_receipt"] = bootstrap_receipt
            initial_path = work / "initial_readback.yaml"
            _run_command(
                _control_command(
                    "initial-readback", row, initial_path, runtime, context
                ),
                child_environment,
                work / "initial_readback.log",
                25.0,
                context,
                source_environment,
            )
            initial = _load_yaml(initial_path, context)
            startup_payload = initial[
                "startup_profile_canonical_json"
            ].encode("utf-8")
            if hashlib.sha256(startup_payload).hexdigest() != initial[
                "startup_profile_sha256"
            ]:
                raise R6I5ExecutionError("initial profile hash drifted")
            journal.capture_startup_profile(startup_payload)
            journal.bind_scene_snapshot(snapshot)

            transaction = _spawn_sanitized(
                _transaction_launch_command(row, runtime, context),
                child_environment,
                work / "transaction_launch.log",
                context,
                source_environment,
            )
            startup_path = work / "transaction_startup.yaml"
            _run_command(
                _control_command(
                    "transaction-startup", row, startup_path, runtime, context
                ),
                child_environment,
                work / "transaction_startup.log",
                30.0,
                context,
                source_environment,
            )
            transaction_startup = _load_yaml(startup_path, context)
            if not (
                transaction_startup["startup_profile_sha256"]
                == initial["startup_profile_sha256"]
                and transaction_startup.get("supervisor_config_sha256")
                == runtime["supervisor_sha256"]
            ):
                raise R6I5ExecutionError(
                    "transaction startup provenance mismatched"
                )
            arm_path = work / "arm_receipt.yaml"
            _run_command(
                _control_command("arm", row, arm_path, runtime, context),
                child_environment,
                work / "arm.log",
                20.0,
                context,
                source_environment,
            )
            arm = _load_yaml(arm_path, context)
            if not (
                arm.get("startup_profile_sha256")
                == initial["startup_profile_sha256"]
                and arm.get("supervisor_config_sha256")
                == runtime["supervisor_sha256"]
                and arm.get("execution_armed") is True
            ):
                raise R6I5ExecutionError("arm provenance mismatched")
            journal.mark_execution_started()
            _wait_action_trace(
                base,
                transaction,
                child_environment,
                work / "action_ready.log",
                context,
                source_environment,
            )

            listener = _spawn_sanitized(
                _listener_command(
                    row,
                    context["preregistration"],
                    work / "activation.yaml",
                    context,
                ),
                child_environment,
                work / "listener.log",
                context,
                source_environment,
            )
            episode = _spawn_sanitized(
                _episode_command(
                    row,
                    Path(snapshot.as_document()["snapshot_instance"]["path"]),
                    work / "episode",
                    context,
                ),
                child_environment,
                work / "episode.log",
                context,
                source_environment,
            )
            deadline = time.monotonic() + float(
                scene_document["scene"]["timeout_s"]
            ) + 90.0
            while True:
                listener_result = _base_process(listener).poll()
                episode_result = _base_process(episode).poll()
                if listener_result not in (None, 0):
                    raise R6I5ExecutionError(
                        "activation listener exited {}".format(listener_result)
                    )
                if episode_result not in (None, 0):
                    raise R6I5ExecutionError(
                        "episode runner exited {}".format(episode_result)
                    )
                if listener_result == 0 and episode_result == 0:
                    break
                if (
                    _base_process(base).poll() is not None
                    or _base_process(transaction).poll() is not None
                ):
                    raise R6I5ExecutionError(
                        "base/transaction launch exited during episode"
                    )
                if time.monotonic() > deadline:
                    raise R6I5ExecutionError("episode deadline exceeded")
                time.sleep(0.1)
            _stop_process(listener)
            listener = None
            _stop_process(episode)
            episode = None

            activation = _load_yaml(work / "activation.yaml", context)
            evaluation = _load_yaml(work / "episode/evaluation.yaml", context)
            semantic = _validate_semantics(
                row, activation, evaluation, minimum, integrity
            )
            post_scene = journal.verify_post_episode_scene()
            try:
                _run_command(
                    _control_command(
                        "restore", row, teardown_path, runtime, context
                    ),
                    child_environment,
                    work / "restore.log",
                    25.0,
                    context,
                    source_environment,
                )
            except R6I5ExecutionError as exc:
                safe_restore_reason = _safe_failure_reason(
                    context, exc, source_environment
                )
                raise integrity.R6TeardownFailure(
                    safe_restore_reason
                ) from None
            receipt = _load_yaml(teardown_path, context)
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
            journal.authorize_launch_stop(verified_teardown)
            _stop_process(transaction)
            transaction = None
            _stop_process(base)
            base = None
            # Capture process logs only after the authorized shutdown has
            # completed so teardown and exit diagnostics are evidence-bound.
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
            journal.complete(binding)
            ledger_entry.update({
                "status": "evidence_complete",
                "seed_consumed": True,
                "raw_resources": resources,
                "expected_ttc_status": row["expected_ttc_status"],
                "observed_ttc_status": evaluation["ttc_status"],
                "semantic_observation": semantic,
                "supervisor_config_sha256": runtime["supervisor_sha256"],
            })
            return ledger_entry
        except BaseException as exc:
            safe_reason = _safe_failure_reason(
                context, exc, source_environment
            )
            if transaction is not None and _base_process(transaction).poll() is None:
                try:
                    if not teardown_path.exists():
                        _run_command(
                            _control_command(
                                "restore", row, teardown_path, runtime, context
                            ),
                            child_environment,
                            work / "emergency_restore.log",
                            25.0,
                            context,
                            source_environment,
                        )
                except BaseException:
                    pass
            for handle in (listener, episode, transaction, base):
                try:
                    _stop_process(handle)
                except BaseException:
                    pass
            phase = journal.lifecycle_phase
            try:
                terminal_resources = _terminal_raw(
                    work, raw, identity, phase, safe_reason, integrity
                )
                terminal = integrity.bind_terminal_attempt_evidence(
                    WORKSPACE, raw, identity, terminal_resources
                )
                journal.attach_terminal_evidence(terminal)
            except BaseException as evidence_exc:
                ledger_entry["terminal_evidence_error"] = _safe_failure_reason(
                    context, evidence_exc, source_environment
                )
            ledger_entry.update({
                "status": "terminal_failure_pending_journal_exit",
                "failure_type": type(exc).__name__,
                "failure_reason": safe_reason,
                "emergency_process_containment": True,
            })
            raise R6I5ExecutionError(safe_reason) from None


def _base_report(context):
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
        "execution_release": {
            "path": context["release_path"],
            "sha256": context["release_sha256"],
        },
        "authorization_envelope": {
            "path": context["authorization_path"],
            "sha256": context["authorization_sha256"],
        },
        "evidence_budget_authorized": len(EXPECTED_SCHEDULE),
        "evidence_units_consumed": 0,
        "r5_remaining_units_consumed": 0,
        "r6_i1_forfeited_units_consumed": 0,
        "held_out_5001_5010_accessed": False,
        "retry_count": 0,
        "resume_used": False,
        "attempt_limit_per_identity": 1,
        "planned_identity_count": len(EXPECTED_SCHEDULE),
        "attempt_ledger": [],
        "terminal_failure": None,
        "assessment_complete": False,
        "winner_ranked_or_frozen": False,
    }


def _persist_terminal_assessment(context):
    """Replay terminal evidence and exclusively persist the frozen assessment."""

    stage_relative = STAGE_REPORT.relative_to(WORKSPACE)
    stage_snapshot = _read_workspace_bytes_once(stage_relative)
    preregistration = context["validation"].preregistration
    assessment = context["assessment"].build_assessment(
        WORKSPACE,
        PREREGISTRATION_RELATIVE.as_posix(),
        preregistration.sha256,
        stage_relative.as_posix(),
        stage_snapshot["sha256"],
    )
    receipt = context["assessment"].write_assessment_once(
        WORKSPACE, assessment
    )
    return assessment, receipt


def _execute_validated(
    release_path,
    release_sha256,
    authorization_path,
    authorization_sha256,
):
    """Execute only after the complete read-only release preflight succeeds."""

    context = _execution_preflight(
        release_path,
        release_sha256,
        authorization_path,
        authorization_sha256,
    )
    # This is the first point at which execution-owned state may be created.
    ATTEMPTS_ROOT.mkdir(parents=False, exist_ok=False)
    report = _base_report(context)
    yaml_module = context["authorization"].yaml
    _atomic_yaml(STAGE_REPORT, report, yaml_module)
    source_environment = dict(os.environ)
    schedule = context["preregistration"]["schedule"]
    report["attempt_ledger"] = [
        {
            "sequence": row["sequence"],
            "identity": _identity(row),
            "status": "scheduled",
            "seed_consumed": False,
            "evidence_units_consumed": 0,
            "attempt_limit": 1,
            "resume_forbidden": True,
        }
        for row in schedule
    ]
    _atomic_yaml(STAGE_REPORT, report, yaml_module)
    for row, ledger in zip(schedule, report["attempt_ledger"]):
        try:
            _run_attempt(row, context, ledger, report, source_environment)
            _assert_host_isolated("completed unit containment")
            report["evidence_units_consumed"] = sum(
                item.get("evidence_units_consumed", 0)
                for item in report["attempt_ledger"]
            )
            _atomic_yaml(STAGE_REPORT, report, yaml_module)
        except BaseException as exc:
            safe_reason = _safe_failure_reason(
                context, exc, source_environment
            )
            ledger.update({
                "status": "terminal_failure",
                "failure_type": type(exc).__name__,
                "failure_reason": safe_reason,
                "resume_forbidden": True,
            })
            consumed = sum(
                item.get("evidence_units_consumed", 0)
                for item in report["attempt_ledger"]
            )
            for unattempted in report["attempt_ledger"][row["sequence"]:]:
                unattempted.update({
                    "status": "forfeited_unattempted_after_terminal_failure",
                    "seed_consumed": False,
                    "evidence_units_consumed": 0,
                    "forfeiture_trigger_sequence": row["sequence"],
                    "retry_forbidden": True,
                    "resume_forbidden": True,
                })
            ledger["evidence_unit_disposition"] = (
                "consumed_terminal_failure"
                if ledger.get("evidence_units_consumed") == 1
                else "forfeited_terminal_before_consumption"
            )
            report.update({
                "status": "terminal_failure",
                "evidence_units_consumed": consumed,
                "terminal_failure": {
                    "identity": _identity(row),
                    "failure_type": type(exc).__name__,
                    "reason": safe_reason,
                },
                "unattempted_budget_forfeited": len(EXPECTED_SCHEDULE) - consumed,
                "explicit_unattempted_identity_forfeitures": (
                    len(EXPECTED_SCHEDULE) - row["sequence"]
                ),
                "resume_forbidden": True,
            })
            _atomic_yaml(STAGE_REPORT, report, yaml_module)
            _assert_host_isolated("terminal containment")
            _persist_terminal_assessment(context)
            raise R6I5ExecutionError(safe_reason) from None
    report.update({
        "status": "execution_complete_pending_assessment",
        "evidence_units_consumed": len(EXPECTED_SCHEDULE),
        "unattempted_budget_forfeited": 0,
        "resume_forbidden": True,
    })
    _atomic_yaml(STAGE_REPORT, report, yaml_module)
    _assert_host_isolated("completed execution containment")
    assessment, assessment_receipt = _persist_terminal_assessment(context)
    if not (
        assessment.get("assessment_result") == "pass"
        and assessment.get("integration_validation_pass") is True
    ):
        raise R6I5ExecutionError(
            "deterministic terminal assessment did not pass"
        )
    print(yaml_module.safe_dump({
        "stage_report": report,
        "execution_assessment": assessment,
        "assessment_receipt": assessment_receipt,
    }, sort_keys=False))
    return 0


def execute(
    release_path,
    release_sha256,
    authorization_path,
    authorization_sha256,
):
    """Expose no unredacted exception chain at the canonical API boundary."""

    try:
        return _execute_validated(
            release_path,
            release_sha256,
            authorization_path,
            authorization_sha256,
        )
    except BaseException:
        # Preflight failures occur before the verified bootstrap redactor is
        # returned, so they are intentionally reported without source text.
        # Post-preflight detail, when available, is already redacted in the
        # terminal stage report and attempt journal.
        safe_reason = (
            "R6-I5 execution failed closed; inspect only sanitized evidence "
            "created after a successful release preflight"
        )
        raise R6I5ExecutionError(safe_reason) from None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--offline-review", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--release")
    parser.add_argument("--release-sha256")
    parser.add_argument("--authorization")
    parser.add_argument("--authorization-sha256")
    args = parser.parse_args()
    supplied = (
        args.release,
        args.release_sha256,
        args.authorization,
        args.authorization_sha256,
    )
    if args.offline_review:
        if any(value is not None for value in supplied):
            parser.error("offline review rejects release/authorization inputs")
        return offline_review()
    if any(value is None for value in supplied):
        parser.error(
            "execute requires release, release SHA256, authorization, and "
            "authorization SHA256"
        )
    return execute(*supplied)


if __name__ == "__main__":
    raise SystemExit(main())
