#!/usr/bin/env python3
"""Offline-only R6-I4 preflight-integrity runner.

This program replays the complete read-only prejournal validation of the
frozen failed R6-I3 release with the versioned R6-I4 validator.  It has no
execution mode and cannot create attempts, journals, evidence, ROS processes,
or Gazebo processes.  A future simulation run requires a different release
path and a new explicit user instruction.

Only standard-library modules are imported before the repaired validator bytes
have been opened once, hashed against the trusted constants below, and loaded
from that same snapshot.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
import types


sys.dont_write_bytecode = True

WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R6-I4"
SOURCE_STAGE = "V2-04G-R6-I3"

FAILED_RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_execution_release.yaml"
)
FUTURE_RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i4_execution_release.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i3_bounded_simulation_authorization.yaml"
)
REPAIRED_VALIDATOR_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_release.py"
)
REPAIRED_VALIDATOR_TEST_RELATIVE = Path(
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i4_release_validator.py"
)
I3_ROOT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i3_execution"
)
I4_ROOT_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/"
    "r6_i4_preflight_repair_review"
)

EXPECTED_FAILED_RELEASE_SHA256 = (
    "5c47557f539f5d2dcf91349d1d7fda87d81de4d08f75be174644930879ac7fb6"
)
EXPECTED_AUTHORIZATION_SHA256 = (
    "ef0a5886bacfd9e439d56e5586a851a33e2ab4076ac11f6b84033425b9b305d2"
)
EXPECTED_SCHEDULE_SHA256 = (
    "ee89717421f2dd82cdaddb2c8e8722c5d1d4b52db97311c3b7231ba9d161571c"
)
EXPECTED_REPAIRED_VALIDATOR_SHA256 = (
    "9b9dd3fc580d0f880705bf87e120cdb9d30fdce812f585ee99e3f0fdf1fa3994"
)
EXPECTED_REPAIRED_VALIDATOR_TEST_SHA256 = (
    "663fd2da6a8781e3cc4041aad46141e3ffdfee38883d4bcf2fe0e1eb59cc3a89"
)
EXPECTED_MACHINE_REVIEW_STATUS = (
    "execution_readiness_closure_pass_release_absent"
)

TRUSTED_SOURCE_SHA256 = {
    REPAIRED_VALIDATOR_RELATIVE: EXPECTED_REPAIRED_VALIDATOR_SHA256,
    REPAIRED_VALIDATOR_TEST_RELATIVE: EXPECTED_REPAIRED_VALIDATOR_TEST_SHA256,
}

_SCENE_NAMES = (
    "v2-04g-r6-i3-dynamic-conflict-single-s5151.instance.yaml",
    "v2-04g-r6-i3-dynamic-conflict-single-s5151.world",
    "v2-04g-r6-i3-dynamic-conflict-multi-s5152.instance.yaml",
    "v2-04g-r6-i3-dynamic-conflict-multi-s5152.world",
    "v2-04g-r6-i3-dynamic-semantic-clear-s5153.instance.yaml",
    "v2-04g-r6-i3-dynamic-semantic-clear-s5153.world",
    "v2-04g-r6-i3-compile-support-cruise-s5154.instance.yaml",
    "v2-04g-r6-i3-compile-support-cruise-s5154.world",
    "v2-04g-r6-i3-compile-support-static-s5155.instance.yaml",
    "v2-04g-r6-i3-compile-support-static-s5155.world",
    "v2-04g-r6-i3-compile-support-corridor-s5156.instance.yaml",
    "v2-04g-r6-i3-compile-support-corridor-s5156.world",
    "v2-04g-r6-i3-compile-support-maneuver-s5157.instance.yaml",
    "v2-04g-r6-i3-compile-support-maneuver-s5157.world",
)
EXPECTED_RELEASE_RESOURCE_PATHS = {
    "preregistration": (
        "experiments/manifests/v2/integration/"
        "v2_04g_r6_i3_execution_preregistration.yaml"
    ),
    "authorization_envelope": AUTHORIZATION_RELATIVE.as_posix(),
    "fresh_scene_index": (
        I3_ROOT_RELATIVE / "compiled_scenes/compiled_scene_index.yaml"
    ).as_posix(),
    "execution_entrypoint": (
        "src/tools/thesis_experiment/scripts/"
        "v2_04g_r6_i1_r6_i2_r6_i3_bounded_validation.py"
    ),
    "release_validator": (
        "src/tools/thesis_experiment/src/thesis_experiment/"
        "v2_04g_r6_i1_r6_i2_r6_i3_release.py"
    ),
    "release_validator_tests": (
        "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
        "test_v2_04g_r6_i3_release_validator.py"
    ),
    "execution_dependency_closure": (
        I3_ROOT_RELATIVE / "execution_dependency_closure.yaml"
    ).as_posix(),
    "execution_machine_review": (
        I3_ROOT_RELATIVE / "v2_04g_r6_i3_execution_readiness_review.yaml"
    ).as_posix(),
}
EXPECTED_RELEASE_RESOURCE_PATHS.update(
    {
        "fresh_scene_child_{:02d}".format(index): (
            I3_ROOT_RELATIVE / "compiled_scenes" / name
        ).as_posix()
        for index, name in enumerate(_SCENE_NAMES, start=1)
    }
)

MAX_TRUSTED_SOURCE_BYTES = 32 * 1024 * 1024
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


class R6I4OfflinePreflightError(RuntimeError):
    """Raised when the I4 offline integrity boundary fails closed."""


def _read_workspace_bytes_once(relative):
    """Read one regular workspace file through no-follow descriptors."""

    if not WORKSPACE.is_absolute() or WORKSPACE != WORKSPACE.resolve():
        raise R6I4OfflinePreflightError("workspace root is not canonical")
    if WORKSPACE.is_symlink() or not WORKSPACE.is_dir():
        raise R6I4OfflinePreflightError("workspace root is unsafe")
    declared = Path(relative)
    if (
        declared.is_absolute()
        or not declared.parts
        or any(part in {"", ".", ".."} for part in declared.parts)
    ):
        raise R6I4OfflinePreflightError(
            "trusted path is unsafe: {}".format(relative)
        )
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
        descriptor = os.open(
            declared.parts[-1],
            flags | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=current,
        )
        descriptors.append(descriptor)
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode):
            raise R6I4OfflinePreflightError(
                "trusted dependency is not regular: {}".format(relative)
            )
        if before.st_size > MAX_TRUSTED_SOURCE_BYTES:
            raise R6I4OfflinePreflightError(
                "trusted dependency is too large: {}".format(relative)
            )
        chunks = []
        remaining = MAX_TRUSTED_SOURCE_BYTES + 1
        while remaining:
            chunk = os.read(descriptor, min(1024 * 1024, remaining))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        payload = b"".join(chunks)
        after = os.fstat(descriptor)
        stable_fields = (
            "st_dev",
            "st_ino",
            "st_mode",
            "st_size",
            "st_mtime_ns",
            "st_ctime_ns",
        )
        if not (
            len(payload) == before.st_size
            and all(
                getattr(before, field) == getattr(after, field)
                for field in stable_fields
            )
        ):
            raise R6I4OfflinePreflightError(
                "trusted dependency changed during read: {}".format(relative)
            )
        return {
            "relative": declared.as_posix(),
            "path": str(WORKSPACE / declared),
            "payload": payload,
            "sha256": hashlib.sha256(payload).hexdigest(),
            "size_bytes": len(payload),
        }
    except OSError as exc:
        raise R6I4OfflinePreflightError(
            "cannot safely open {}: {}".format(relative, exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass


def _trusted_source_snapshots():
    snapshots = {}
    for relative, expected_sha256 in TRUSTED_SOURCE_SHA256.items():
        snapshot = _read_workspace_bytes_once(relative)
        if snapshot["sha256"] != expected_sha256:
            raise R6I4OfflinePreflightError(
                "trusted source SHA256 drifted: {}".format(relative)
            )
        snapshots[relative] = snapshot
    return snapshots


def _load_repaired_validator(snapshot):
    package_name = "thesis_experiment"
    package = sys.modules.get(package_name)
    if package is None:
        package = types.ModuleType(package_name)
        package.__package__ = package_name
        package.__path__ = [
            str(
                WORKSPACE
                / "src/tools/thesis_experiment/src/thesis_experiment"
            )
        ]
        sys.modules[package_name] = package
    module_name = (
        "thesis_experiment."
        "v2_04g_r6_i1_r6_i2_r6_i3_r6_i4_release"
    )
    module = types.ModuleType(module_name)
    module.__file__ = snapshot["path"]
    module.__package__ = package_name
    sys.modules[module_name] = module
    try:
        exec(
            compile(snapshot["payload"], snapshot["path"], "exec"),
            module.__dict__,
        )
    except BaseException:
        sys.modules.pop(module_name, None)
        raise
    setattr(package, module_name.rpartition(".")[2], module)
    return module


def _state_snapshot():
    paths = {
        "failed_i3_release": FAILED_RELEASE_RELATIVE,
        "future_i4_release": FUTURE_RELEASE_RELATIVE,
        "i3_attempts": I3_ROOT_RELATIVE / "attempts",
        "i3_journals": I3_ROOT_RELATIVE / "journals",
        "i3_receipts": I3_ROOT_RELATIVE / "receipts",
        "i3_raw_evidence": I3_ROOT_RELATIVE / "raw_evidence",
        "i3_semantic_evidence": I3_ROOT_RELATIVE / "semantic_evidence",
        "i3_stage_report": I3_ROOT_RELATIVE / "v2_04g_r6_i3_stage_report.yaml",
        "i3_execution_report": (
            I3_ROOT_RELATIVE / "v2_04g_r6_i3_execution_report.yaml"
        ),
        "i4_attempts": I4_ROOT_RELATIVE / "attempts",
        "i4_journals": I4_ROOT_RELATIVE / "journals",
        "i4_receipts": I4_ROOT_RELATIVE / "receipts",
        "i4_raw_evidence": I4_ROOT_RELATIVE / "raw_evidence",
        "i4_semantic_evidence": I4_ROOT_RELATIVE / "semantic_evidence",
        "i4_ros_home": I4_ROOT_RELATIVE / "ros_home",
        "i4_ros_logs": I4_ROOT_RELATIVE / "ros_logs",
        "i4_stage_report": I4_ROOT_RELATIVE / "v2_04g_r6_i4_stage_report.yaml",
        "i4_execution_report": (
            I4_ROOT_RELATIVE / "v2_04g_r6_i4_execution_report.yaml"
        ),
    }
    return {
        label: os.path.lexists(str(WORKSPACE / relative))
        for label, relative in paths.items()
    }


def _require_expected_state(state):
    expected = {
        "failed_i3_release": True,
        "future_i4_release": False,
        "i3_attempts": False,
        "i3_journals": False,
        "i3_receipts": False,
        "i3_raw_evidence": False,
        "i3_semantic_evidence": False,
        "i3_stage_report": False,
        "i3_execution_report": False,
        "i4_attempts": False,
        "i4_journals": False,
        "i4_receipts": False,
        "i4_raw_evidence": False,
        "i4_semantic_evidence": False,
        "i4_ros_home": False,
        "i4_ros_logs": False,
        "i4_stage_report": False,
        "i4_execution_report": False,
    }
    if state != expected:
        raise R6I4OfflinePreflightError(
            "offline execution-state gate failed: {}".format(state)
        )


def _process_matches():
    current = os.getpid()
    matches = []
    try:
        entries = list(Path("/proc").iterdir())
    except OSError as exc:
        raise R6I4OfflinePreflightError(
            "cannot enumerate /proc for host isolation: {}".format(exc)
        ) from exc
    for item in entries:
        if not item.name.isdigit() or int(item.name) == current:
            continue
        try:
            command = (item / "cmdline").read_bytes().replace(
                b"\0", b" "
            ).decode("utf-8", errors="replace").strip()
        except (FileNotFoundError, ProcessLookupError):
            continue
        except OSError as exc:
            raise R6I4OfflinePreflightError(
                "cannot inspect process {}: {}".format(item.name, exc)
            ) from exc
        executable = Path(command.split(" ", 1)[0]).name.lower()
        tokens = command.lower().split()
        matched = (
            executable in PROCESS_MARKERS
            or any(Path(token).name in PROCESS_MARKERS for token in tokens[:3])
            or "sac_train.py" in command.lower()
            or "residual_train.py" in command.lower()
        )
        if matched:
            matches.append(
                {
                    "pid": int(item.name),
                    "executable_basename": executable or "unknown",
                    "policy_label": (
                        "forbidden_ros_gazebo_move_base_or_training_process"
                    ),
                }
            )
    return sorted(matches, key=lambda row: row["pid"])


def offline_preflight():
    """Return a deterministic receipt after all read-only I4 gates pass."""

    before_state = _state_snapshot()
    _require_expected_state(before_state)
    before_processes = _process_matches()
    if before_processes:
        raise R6I4OfflinePreflightError(
            "host process isolation failed: {}".format(before_processes)
        )
    snapshots = _trusted_source_snapshots()
    validator = _load_repaired_validator(
        snapshots[REPAIRED_VALIDATOR_RELATIVE]
    )
    validation = validator.load_and_validate_execution_release(
        WORKSPACE,
        FAILED_RELEASE_RELATIVE.as_posix(),
        EXPECTED_FAILED_RELEASE_SHA256,
        AUTHORIZATION_RELATIVE.as_posix(),
        EXPECTED_AUTHORIZATION_SHA256,
        expected_resource_paths=EXPECTED_RELEASE_RESOURCE_PATHS,
        expected_machine_review_status=EXPECTED_MACHINE_REVIEW_STATUS,
    )
    if validation.schedule_sha256 != EXPECTED_SCHEDULE_SHA256:
        raise R6I4OfflinePreflightError("historical schedule digest drifted")
    if validation.identity_count != 6:
        raise R6I4OfflinePreflightError("historical identity count drifted")
    if len(validation.bound_resources) != 22:
        raise R6I4OfflinePreflightError("historical release roster drifted")
    if len(validation.authorization_bound_resources) != 12:
        raise R6I4OfflinePreflightError(
            "historical authorization roster drifted"
        )
    if len(validation.authorization_parsed_labels) != 2:
        raise R6I4OfflinePreflightError("authorization parse scope drifted")
    if len(validation.authorization_hash_only_labels) != 10:
        raise R6I4OfflinePreflightError("authorization hash-only scope drifted")
    after_state = _state_snapshot()
    _require_expected_state(after_state)
    after_processes = _process_matches()
    if after_processes:
        raise R6I4OfflinePreflightError(
            "host process isolation changed: {}".format(after_processes)
        )
    if before_state != after_state:
        raise R6I4OfflinePreflightError("offline validation changed state")
    return {
        "schema_version": "1.0",
        "stage": STAGE,
        "source_stage": SOURCE_STAGE,
        "status": (
            "offline_preflight_integrity_repair_pass_"
            "failed_release_preserved"
        ),
        "execution_authorized": False,
        "execution_ready": False,
        "formal_result": False,
        "runtime_ready": False,
        "offline_only": True,
        "simulation_only": True,
        "failed_i3_release": {
            "path": FAILED_RELEASE_RELATIVE.as_posix(),
            "sha256": validation.release.sha256,
            "present": True,
            "release_resource_count": len(validation.bound_resources),
            "historical_identity_count": validation.identity_count,
            "historical_schedule_sha256": validation.schedule_sha256,
            "authorized_units": 6,
            "consumed_units": 0,
            "forfeited_units": 0,
            "executable_under_i4": False,
        },
        "future_i4_release": {
            "path": FUTURE_RELEASE_RELATIVE.as_posix(),
            "present": False,
            "new_explicit_simulation_instruction_required": True,
        },
        "authorization_resource_audit": {
            "resource_count": len(validation.authorization_bound_resources),
            "all_exact_bytes_sha256_rehashed": True,
            "parsed_labels": list(validation.authorization_parsed_labels),
            "parsed_count": len(validation.authorization_parsed_labels),
            "hash_only_labels": list(
                validation.authorization_hash_only_labels
            ),
            "hash_only_count": len(
                validation.authorization_hash_only_labels
            ),
            "legacy_integer_key_resource_parsed": False,
            "pass": True,
        },
        "trusted_sources": [
            {
                "path": relative.as_posix(),
                "sha256": snapshots[relative]["sha256"],
                "size_bytes": snapshots[relative]["size_bytes"],
            }
            for relative in sorted(snapshots, key=lambda item: item.as_posix())
        ],
        "state_gate": after_state,
        "host_process_isolation": {
            "forbidden_matches": [],
            "pass": True,
        },
        "i4_allocation": {
            "execution_seeds": [],
            "schedule": [],
            "evidence_units_authorized": 0,
            "evidence_units_consumed": 0,
            "evidence_units_forfeited": 0,
        },
        "side_effects": {
            "execution_state_created": False,
            "attempt_or_journal_created": False,
            "subprocess_started": False,
            "ros_or_gazebo_started": False,
            "seed_or_budget_consumed": False,
        },
    }


def main():
    parser = argparse.ArgumentParser(
        description="Run the offline-only R6-I4 repaired preflight"
    )
    parser.add_argument("--check-only", action="store_true", required=True)
    args = parser.parse_args()
    if not args.check_only:
        parser.error("only --check-only is available")
    print(
        json.dumps(
            offline_preflight(),
            sort_keys=True,
            separators=(",", ":"),
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
