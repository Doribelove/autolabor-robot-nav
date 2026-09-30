"""Fail-closed validator for the unique V2-04G-P1-E1 270-unit release."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import stat
from typing import Any, Dict, Mapping

import yaml

from thesis_experiment import v2_04g_r6_i1_p1_statistics as statistics


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P1-E1"
RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/v2_04g_r6_i1_p1_e1_execution_release.yaml"
)
SCHEDULE_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness/"
    "v2_04g_p1_r1_exact_schedule.yaml"
)
OUTPUT_ROOT = Path("artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution")
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class P1ReleaseError(ValueError):
    """Raised before execution state exists when a release binding differs."""


class _UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise P1ReleaseError("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P1ReleaseError(message)


def _safe_relative(value: Any) -> Path:
    _require(type(value) is str, "path must be text")
    path = Path(value)
    _require(
        not path.is_absolute() and bool(path.parts)
        and all(part not in {"", ".", ".."} for part in path.parts),
        "unsafe workspace-relative path: {}".format(value),
    )
    return path


def read_workspace_file_once(workspace: Path, relative: Path) -> bytes:
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
            _require(stat.S_ISREG(before.st_mode), "dependency is not regular")
            _require(before.st_nlink == 1, "hard-linked dependency forbidden")
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
                "dependency changed during read",
            )
            payload = b"".join(chunks)
            _require(len(payload) == before.st_size, "short dependency read")
            return payload
        finally:
            os.close(file_descriptor)
    except OSError as exc:
        raise P1ReleaseError("cannot read {}: {}".format(relative, exc)) from exc
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def _yaml(payload: bytes, label: str) -> Mapping[str, Any]:
    try:
        value = yaml.load(payload.decode("utf-8"), Loader=_UniqueLoader)
    except (UnicodeError, yaml.YAMLError) as exc:
        raise P1ReleaseError("cannot parse {}: {}".format(label, exc)) from exc
    _require(type(value) is dict, "{} must be a mapping".format(label))
    return value


def _canonical_sha(value: Any) -> str:
    return hashlib.sha256(json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")).hexdigest()


def _bound_yaml(workspace: Path, binding: Mapping[str, Any], label: str):
    _require(type(binding) is dict and set(binding) == {"path", "sha256"}, "{} binding schema differs".format(label))
    _require(HEX_SHA256.fullmatch(str(binding["sha256"])) is not None, "{} SHA differs".format(label))
    relative = _safe_relative(binding["path"])
    payload = read_workspace_file_once(workspace, relative)
    _require(hashlib.sha256(payload).hexdigest() == binding["sha256"], "{} hash differs".format(label))
    return relative, payload, _yaml(payload, label)


def validate_release(
    expected_release_sha256: str,
    workspace: Path = WORKSPACE,
    *,
    require_fresh_state: bool = True,
) -> Dict[str, Any]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    _require(HEX_SHA256.fullmatch(expected_release_sha256) is not None, "caller release SHA differs")
    release_payload = read_workspace_file_once(workspace, RELEASE_RELATIVE)
    _require(hashlib.sha256(release_payload).hexdigest() == expected_release_sha256, "release bytes differ from caller SHA")
    release = _yaml(release_payload, "P1-E1 release")
    required = {
        "schema_version", "architecture_generation", "stage", "release_id",
        "status", "simulation_only", "formal_result", "runtime_ready",
        "execution_authorized", "authorized_evidence_units", "attempt_per_identity",
        "retry_resume_replacement_allowed", "training_allowed", "training_budget_steps",
        "real_vehicle_use_forbidden", "i5_rerun_forbidden", "schedule",
        "contract", "authorization", "dependency_closure", "offline_review",
        "runner", "raw_replay_adapter", "episode_adapter", "launch_surface",
        "resource_caps", "claim_boundary",
    }
    _require(set(release) == required, "release schema differs")
    _require(release["stage"] == STAGE, "release stage differs")
    _require(release["status"] == "unique_270_unit_simulation_release", "release status differs")
    _require(release["simulation_only"] is True, "release is not simulation-only")
    _require(release["formal_result"] is False and release["runtime_ready"] is False, "release preclaims a result")
    _require(release["execution_authorized"] is True, "release lacks explicit authorization")
    _require(release["authorized_evidence_units"] == 270, "release budget is not exact 270")
    _require(release["attempt_per_identity"] == 1, "attempt policy differs")
    _require(release["retry_resume_replacement_allowed"] is False, "retry/resume/replacement enabled")
    _require(release["training_allowed"] is False and release["training_budget_steps"] == 0, "training budget differs")
    _require(release["real_vehicle_use_forbidden"] is True, "real vehicle is not forbidden")
    _require(release["i5_rerun_forbidden"] is True, "I5 rerun is not forbidden")

    _, schedule_payload, schedule = _bound_yaml(workspace, release["schedule"], "schedule")
    _require(release["schedule"]["path"] == SCHEDULE_RELATIVE.as_posix(), "schedule path differs")
    planned = statistics.validate_schedule(schedule)
    _require(len(planned) == 270, "schedule is not exact 270")
    _require(schedule["canonical_episode_digest"] == release["claim_boundary"]["schedule_episode_digest"], "schedule episode digest differs")

    _, _, contract = _bound_yaml(workspace, release["contract"], "contract")
    _, _, authorization = _bound_yaml(workspace, release["authorization"], "authorization")
    _, closure_payload, closure = _bound_yaml(workspace, release["dependency_closure"], "dependency closure")
    _, _, review = _bound_yaml(workspace, release["offline_review"], "offline review")
    for label in ("runner", "raw_replay_adapter", "episode_adapter", "launch_surface"):
        binding = release[label]
        _bound_yaml(workspace, binding, label) if binding["path"].endswith((".yaml", ".yml")) else (
            _require(
                hashlib.sha256(read_workspace_file_once(workspace, _safe_relative(binding["path"]))).hexdigest() == binding["sha256"],
                "{} hash differs".format(label),
            )
        )
    _require(contract.get("stage") == STAGE and contract.get("execution_authorized") is True, "contract authority differs")
    _require(contract.get("authorized_evidence_units") == 270, "contract budget differs")
    _require(authorization.get("stage") == STAGE and authorization.get("execution_authorized") is True, "authorization differs")
    _require(authorization.get("authorized_evidence_units") == 270, "authorization budget differs")
    _require(authorization.get("explicit_user_authorization_recorded") is True, "explicit user authorization not recorded")
    _require(review.get("status") == "p1_e1_offline_execution_readiness_pass_release_absent", "offline review did not pass")
    _require(review.get("release_present_during_review") is False, "offline review observed a release")
    _require(closure.get("stage") == STAGE and closure.get("unresolved") == [], "dependency closure differs")
    logical = dict(closure)
    declared_logical = logical.pop("logical_sha256", None)
    _require(_canonical_sha(logical) == declared_logical, "closure logical SHA differs")
    verified = 0
    for receipt in closure.get("local_resources", []):
        _require(type(receipt) is dict and set(receipt) == {"path", "sha256", "size_bytes"}, "closure receipt schema differs")
        payload = read_workspace_file_once(workspace, _safe_relative(receipt["path"]))
        _require(hashlib.sha256(payload).hexdigest() == receipt["sha256"], "closure local hash differs: {}".format(receipt["path"]))
        _require(len(payload) == receipt["size_bytes"], "closure local size differs")
        verified += 1
    for receipt in closure.get("external_resources", []):
        _require(type(receipt) is dict and set(receipt) == {"path", "sha256", "size_bytes"}, "external receipt schema differs")
        path = Path(receipt["path"])
        _require(path.is_absolute() and path.resolve(strict=True) == path, "external path is not canonical")
        payload = path.read_bytes()
        _require(hashlib.sha256(payload).hexdigest() == receipt["sha256"], "external resource hash differs")
        _require(len(payload) == receipt["size_bytes"], "external resource size differs")

    _require(release["resource_caps"] == {
        "serial_concurrency": 1,
        "per_identity_hard_wall_cap_s": 240,
        "stage_hard_wall_cap_h": 20,
        "canonical_artifact_cap_bytes": 536870912,
        "minimum_free_disk_before_execution_bytes": 4294967296,
    }, "release resource caps differ")
    free = os.statvfs(str(workspace))
    free_bytes = free.f_bavail * free.f_frsize
    _require(free_bytes >= 4294967296, "free disk is below frozen execution minimum")
    if require_fresh_state:
        for relative in (
            OUTPUT_ROOT / "attempts",
            OUTPUT_ROOT / "journals",
            OUTPUT_ROOT / "consumption",
            OUTPUT_ROOT / "p1_e1_stage_report.yaml",
        ):
            _require(not (workspace / relative).exists(), "execution state already exists; resume forbidden")
    return {
        "release": release,
        "release_sha256": expected_release_sha256,
        "schedule": schedule,
        "schedule_sha256": hashlib.sha256(schedule_payload).hexdigest(),
        "closure": closure,
        "closure_sha256": hashlib.sha256(closure_payload).hexdigest(),
        "local_resources_verified": verified,
        "free_disk_bytes": free_bytes,
    }
