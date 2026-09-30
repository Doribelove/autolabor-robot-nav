"""Fail-closed validator for the unique V2-04G-P2-C1 release."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import stat
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P2-C1"
RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p2_c1_execution_release.yaml"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_execution"
)
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class C1ReleaseError(ValueError):
    """Raised before any C1 execution state is created."""


class _UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise C1ReleaseError("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise C1ReleaseError(message)


def _safe_relative(value: Any) -> Path:
    _require(type(value) is str, "path must be text")
    path = Path(value)
    _require(
        not path.is_absolute()
        and path.parts
        and all(part not in {"", ".", ".."} for part in path.parts),
        "unsafe workspace-relative path: {}".format(value),
    )
    return path


def read_workspace_file_once(workspace: Path, relative: Path) -> bytes:
    root = workspace.resolve(strict=True)
    parts = _safe_relative(Path(relative).as_posix()).parts
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags = flags | getattr(os, "O_DIRECTORY", 0)
    directory_flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptors = []
    try:
        descriptor = os.open(str(root), directory_flags)
        descriptors.append(descriptor)
        for component in parts[:-1]:
            descriptor = os.open(component, directory_flags, dir_fd=descriptor)
            descriptors.append(descriptor)
        file_descriptor = os.open(
            parts[-1],
            flags | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=descriptors[-1],
        )
        try:
            before = os.fstat(file_descriptor)
            _require(stat.S_ISREG(before.st_mode), "dependency is not regular")
            chunks = []
            while True:
                chunk = os.read(file_descriptor, 1024 * 1024)
                if not chunk:
                    break
                chunks.append(chunk)
            after = os.fstat(file_descriptor)
            fields = (
                "st_dev",
                "st_ino",
                "st_mode",
                "st_size",
                "st_mtime_ns",
                "st_ctime_ns",
            )
            _require(
                all(getattr(before, field) == getattr(after, field) for field in fields),
                "dependency changed during read",
            )
            payload = b"".join(chunks)
            _require(len(payload) == before.st_size, "short dependency read")
            return payload
        finally:
            os.close(file_descriptor)
    except OSError as exc:
        raise C1ReleaseError("cannot read {}: {}".format(relative, exc)) from exc
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def _yaml(payload: bytes, label: str) -> Mapping[str, Any]:
    try:
        value = yaml.load(payload.decode("utf-8"), Loader=_UniqueLoader)
    except (UnicodeError, yaml.YAMLError) as exc:
        raise C1ReleaseError("cannot parse {}: {}".format(label, exc)) from exc
    _require(type(value) is dict, "{} must be a mapping".format(label))
    return value


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_sha(value: Any) -> str:
    return _sha(
        json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    )


def _bound(workspace: Path, binding: Mapping[str, Any], label: str):
    _require(
        type(binding) is dict and set(binding) == {"path", "sha256"},
        "{} binding schema differs".format(label),
    )
    _require(
        HEX_SHA256.fullmatch(str(binding["sha256"])) is not None,
        "{} SHA differs".format(label),
    )
    relative = _safe_relative(binding["path"])
    payload = read_workspace_file_once(workspace, relative)
    _require(_sha(payload) == binding["sha256"], "{} hash differs".format(label))
    return relative, payload


def validate_release(
    expected_release_sha256: str,
    workspace: Path = WORKSPACE,
    *,
    require_fresh_state: bool = True,
) -> Dict[str, Any]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    _require(
        HEX_SHA256.fullmatch(expected_release_sha256) is not None,
        "caller release SHA differs",
    )
    release_payload = read_workspace_file_once(workspace, RELEASE_RELATIVE)
    _require(
        _sha(release_payload) == expected_release_sha256,
        "release bytes differ from caller SHA",
    )
    release = _yaml(release_payload, "P2-C1 release")
    required = {
        "schema_version",
        "architecture_generation",
        "stage",
        "release_id",
        "status",
        "simulation_only",
        "formal_result",
        "runtime_ready",
        "execution_authorized",
        "authorized_evidence_units_max",
        "attempt_per_identity",
        "retry_resume_replacement_or_seed_substitution_allowed",
        "training_allowed",
        "training_budget_steps",
        "real_vehicle_use_forbidden",
        "schedule",
        "command_plan",
        "contract",
        "authorization",
        "dependency_closure",
        "offline_review",
        "runner",
        "raw_replay_adapter",
        "assessor",
        "launch_surface",
        "resource_caps",
        "claim_boundary",
    }
    _require(set(release) == required, "release schema differs")
    _require(release["stage"] == STAGE, "release stage differs")
    _require(
        release["status"] == "unique_342_unit_max_calibration_simulation_release",
        "release status differs",
    )
    _require(release["simulation_only"] is True, "release is not simulation-only")
    _require(
        release["formal_result"] is False and release["runtime_ready"] is False,
        "release preclaims a result",
    )
    _require(release["execution_authorized"] is True, "release lacks authority")
    _require(
        release["authorized_evidence_units_max"] == 342,
        "release budget differs",
    )
    _require(release["attempt_per_identity"] == 1, "attempt policy differs")
    _require(
        release["retry_resume_replacement_or_seed_substitution_allowed"] is False,
        "release permits identity changes",
    )
    _require(
        release["training_allowed"] is False and release["training_budget_steps"] == 0,
        "release training budget differs",
    )
    _require(
        release["real_vehicle_use_forbidden"] is True,
        "release does not forbid real vehicle",
    )

    _, schedule_payload = _bound(workspace, release["schedule"], "schedule")
    schedule = _yaml(schedule_payload, "schedule")
    _, plan_payload = _bound(workspace, release["command_plan"], "command plan")
    plan = _yaml(plan_payload, "command plan")
    _, contract_payload = _bound(workspace, release["contract"], "contract")
    contract = _yaml(contract_payload, "contract")
    _, authorization_payload = _bound(
        workspace, release["authorization"], "authorization"
    )
    authorization = _yaml(authorization_payload, "authorization")
    _, closure_payload = _bound(
        workspace, release["dependency_closure"], "dependency closure"
    )
    closure = _yaml(closure_payload, "dependency closure")
    _, review_payload = _bound(workspace, release["offline_review"], "offline review")
    review = _yaml(review_payload, "offline review")
    for label in (
        "runner",
        "raw_replay_adapter",
        "assessor",
        "launch_surface",
    ):
        _bound(workspace, release[label], label)

    _require(contract.get("stage") == STAGE, "contract stage differs")
    _require(contract.get("execution_authorized") is True, "contract authority differs")
    _require(
        contract.get("authorized_evidence_units_max") == 342,
        "contract budget differs",
    )
    _require(authorization.get("stage") == STAGE, "authorization stage differs")
    _require(
        authorization.get("explicit_user_authorization_recorded") is True
        and authorization.get("execution_authorized") is True,
        "explicit authorization missing",
    )
    _require(
        authorization.get("authorized_evidence_units_max") == 342,
        "authorization budget differs",
    )
    _require(
        review.get("status")
        == "p2_c1_independent_execution_readiness_pass_release_absent",
        "offline review did not pass",
    )
    _require(review.get("release_present_during_review") is False, "review observed release")

    rows = schedule.get("rows")
    _require(type(rows) is list and len(rows) == 342, "schedule is not exact 342")
    _require(
        [row["sequence"] for row in rows] == list(range(1, 343)),
        "schedule sequence differs",
    )
    _require(
        all(row["row_type"] == "qualification_probe" for row in rows[:12])
        and all(row["row_type"] == "calibration_navigation" for row in rows[12:]),
        "qualification-first schedule differs",
    )
    _require(
        not set(range(5401, 5491)) & {row["seed"] for row in rows},
        "held-out seed leaked",
    )
    _require(schedule["held_out_test_row_count"] == 0, "held-out row exists")
    _require(
        schedule["canonical_row_digest"]
        == release["claim_boundary"]["schedule_row_digest"],
        "schedule digest differs",
    )
    _require(
        type(plan.get("rows")) is list
        and len(plan["rows"]) == 342
        and plan["canonical_plan_digest"]
        == release["claim_boundary"]["command_plan_digest"],
        "command plan differs",
    )

    _require(
        closure.get("stage") == STAGE and closure.get("unresolved") == [],
        "dependency closure differs",
    )
    logical = dict(closure)
    declared_logical = logical.pop("logical_sha256", None)
    _require(_canonical_sha(logical) == declared_logical, "closure logical SHA differs")
    local_verified = 0
    for receipt in closure["local_resources"]:
        _require(
            type(receipt) is dict
            and set(receipt) == {"path", "sha256", "size_bytes"},
            "local receipt schema differs",
        )
        payload = read_workspace_file_once(
            workspace, _safe_relative(receipt["path"])
        )
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "local dependency differs: {}".format(receipt["path"]),
        )
        local_verified += 1
    external_verified = 0
    for receipt in closure["external_resources"]:
        path = Path(receipt["path"])
        _require(path.is_absolute() and path.is_file(), "external dependency absent")
        payload = path.read_bytes()
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "external dependency differs",
        )
        external_verified += 1

    expected_caps = {
        "serial_concurrency": 1,
        "per_identity_hard_wall_cap_s": 240,
        "stage_hard_wall_cap_h": 20,
        "canonical_artifact_cap_bytes": 1073741824,
        "minimum_free_disk_before_execution_bytes": 4294967296,
    }
    _require(release["resource_caps"] == expected_caps, "resource caps differ")
    filesystem = os.statvfs(str(workspace))
    free_bytes = filesystem.f_bavail * filesystem.f_frsize
    _require(
        free_bytes >= expected_caps["minimum_free_disk_before_execution_bytes"],
        "free disk is below frozen minimum",
    )
    if require_fresh_state:
        _require(not (workspace / OUTPUT_ROOT).exists(), "C1 execution state exists")
    return {
        "release": release,
        "release_sha256": expected_release_sha256,
        "schedule": schedule,
        "schedule_sha256": _sha(schedule_payload),
        "command_plan": plan,
        "command_plan_sha256": _sha(plan_payload),
        "closure": closure,
        "closure_sha256": _sha(closure_payload),
        "local_resources_verified": local_verified,
        "external_resources_verified": external_verified,
        "free_disk_bytes": free_bytes,
    }
