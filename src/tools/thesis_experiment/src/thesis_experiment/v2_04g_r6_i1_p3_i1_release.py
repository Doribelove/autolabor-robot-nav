"""Fail-closed validator for the unique one-unit P3-I1 release."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P3-I1"
RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p3_i1_execution_release.yaml"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p3_i1_execution"
)
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class P3I1ReleaseError(ValueError):
    """Raised before any P3-I1 execution state is created."""


class _UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise P3I1ReleaseError("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P3I1ReleaseError(message)


def _safe_relative(value: Any) -> Path:
    _require(type(value) is str, "path must be text")
    path = Path(value)
    _require(
        not path.is_absolute()
        and path.parts
        and all(part not in {"", ".", ".."} for part in path.parts),
        "unsafe relative path",
    )
    return path


def _read_once(workspace: Path, relative: Path) -> bytes:
    from thesis_experiment.v2_04g_r6_i1_p2_c1_release import (
        read_workspace_file_once,
    )

    try:
        return read_workspace_file_once(workspace, relative)
    except BaseException as exc:
        raise P3I1ReleaseError("cannot read {}: {}".format(relative, exc)) from exc


def _yaml(payload: bytes, label: str) -> Mapping[str, Any]:
    try:
        value = yaml.load(payload.decode("utf-8"), Loader=_UniqueLoader)
    except (UnicodeError, yaml.YAMLError) as exc:
        raise P3I1ReleaseError("cannot parse {}: {}".format(label, exc)) from exc
    _require(type(value) is dict, "{} root differs".format(label))
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
        "{} binding differs".format(label),
    )
    _require(HEX_SHA256.fullmatch(str(binding["sha256"])) is not None, "bad SHA")
    relative = _safe_relative(binding["path"])
    payload = _read_once(workspace, relative)
    _require(_sha(payload) == binding["sha256"], "{} SHA differs".format(label))
    return relative, payload


def validate_release(
    expected_release_sha256: str,
    workspace: Path = WORKSPACE,
    *,
    require_fresh_state: bool = True,
) -> Dict[str, Any]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    _require(HEX_SHA256.fullmatch(expected_release_sha256) is not None, "caller SHA differs")
    release_payload = _read_once(workspace, RELEASE_RELATIVE)
    _require(_sha(release_payload) == expected_release_sha256, "release SHA differs")
    release = _yaml(release_payload, "release")
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
        "global_budget",
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
    _require(
        release["stage"] == STAGE
        and release["status"] == "unique_one_unit_semantic_preflight_release"
        and release["simulation_only"] is True
        and release["formal_result"] is False
        and release["runtime_ready"] is False
        and release["execution_authorized"] is True
        and release["authorized_evidence_units_max"] == 1
        and release["attempt_per_identity"] == 1
        and release["retry_resume_replacement_or_seed_substitution_allowed"]
        is False
        and release["training_allowed"] is False
        and release["training_budget_steps"] == 0
        and release["real_vehicle_use_forbidden"] is True,
        "release boundary differs",
    )
    _require(
        release["global_budget"]
        == {
            "round": 2,
            "maximum_units": 704,
            "this_release_units": 1,
            "future_units_not_authorized": 703,
            "additive_expansion": False,
        },
        "global budget differs",
    )
    _, schedule_payload = _bound(workspace, release["schedule"], "schedule")
    schedule = _yaml(schedule_payload, "schedule")
    _, plan_payload = _bound(workspace, release["command_plan"], "plan")
    plan = _yaml(plan_payload, "plan")
    _, contract_payload = _bound(workspace, release["contract"], "contract")
    contract = _yaml(contract_payload, "contract")
    _, auth_payload = _bound(workspace, release["authorization"], "authorization")
    authorization = _yaml(auth_payload, "authorization")
    _, closure_payload = _bound(workspace, release["dependency_closure"], "closure")
    closure = _yaml(closure_payload, "closure")
    _, review_payload = _bound(workspace, release["offline_review"], "review")
    review = _yaml(review_payload, "review")
    for label in ("runner", "raw_replay_adapter", "assessor", "launch_surface"):
        _bound(workspace, release[label], label)
    _require(
        contract.get("stage") == STAGE
        and contract.get("authorized_evidence_units_max") == 1
        and contract.get("single_factor_binding", {}).get("runtime_profile_id")
        == "r6_semantics_circle_contact",
        "contract differs",
    )
    _require(
        authorization.get("stage") == STAGE
        and authorization.get("explicit_user_authorization_recorded") is True
        and authorization.get("authorized_evidence_units_max") == 1,
        "authorization differs",
    )
    _require(
        review.get("status")
        == "p3_i1_independent_one_unit_execution_readiness_pass_release_absent"
        and review.get("release_present_during_review") is False,
        "review differs",
    )
    rows = schedule.get("rows")
    plan_rows = plan.get("rows")
    _require(type(rows) is list and len(rows) == 1, "schedule count differs")
    _require(type(plan_rows) is list and len(plan_rows) == 1, "plan count differs")
    row = rows[0]
    planned = plan_rows[0]
    _require(
        row["sequence"] == 1
        and row["seed"] == 5624
        and row["attempt"] == 1
        and row["probe_type"] == "high_speed_to_conflict_overlay_transition"
        and row["candidate_speed_mps"] == 1.25
        and row["candidate_method_id"] == "circle_contact_v125",
        "identity differs",
    )
    _require(
        not set(range(5691, 5781)) & {row["seed"]},
        "held-out seed leaked",
    )
    _require(
        schedule["canonical_row_digest"]
        == _canonical_sha(rows)
        == release["claim_boundary"]["schedule_row_digest"],
        "schedule digest differs",
    )
    _require(
        plan["canonical_plan_digest"]
        == _canonical_sha(plan_rows)
        == release["claim_boundary"]["command_plan_digest"],
        "plan digest differs",
    )
    _require(
        planned["runtime_profile_id"] == "r6_semantics_circle_contact"
        and planned["evidence_candidate_method_id"] == "circle_contact_v125"
        and planned["evidence_qualification_profile_id"]
        == "qualification_overlay_v125",
        "identity separation differs",
    )
    for argv in (
        planned["base_launch_argv"],
        planned["semantic_transaction_launch_argv"],
    ):
        profile = [
            value for value in argv if value.startswith("attempt_profile_id:=")
        ]
        _require(
            profile == ["attempt_profile_id:=r6_semantics_circle_contact"],
            "runtime profile argv differs",
        )
    for label in ("instance", "world"):
        receipt = row[label]
        relative = _safe_relative(receipt["path"])
        payload = _read_once(workspace, relative)
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "scene receipt differs",
        )
        if label == "instance":
            instance = _yaml(payload, "instance")
    scene = instance["scene"]
    _require(
        scene["scene_id"] == row["scene_id"]
        and scene["seed"] == 5624
        and scene["split"] == "calibration"
        and float(scene["timeout_s"]) == float(row["timeout_s"]) == 80.0
        and planned["scene_execution_binding"]["start"] == scene["start"],
        "compiled scene differs",
    )
    _require(
        closure.get("stage") == STAGE
        and closure.get("unresolved") == []
        and closure.get("schedule_row_count") == 1,
        "closure differs",
    )
    logical = dict(closure)
    declared = logical.pop("logical_sha256", None)
    _require(_canonical_sha(logical) == declared, "closure logical SHA differs")
    local_verified = 0
    for receipt in closure["local_resources"]:
        payload = _read_once(workspace, _safe_relative(receipt["path"]))
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "local dependency differs",
        )
        local_verified += 1
    external_verified = 0
    for receipt in closure["external_resources"]:
        path = Path(receipt["path"])
        payload = path.read_bytes()
        _require(
            path.is_absolute()
            and _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "external dependency differs",
        )
        external_verified += 1
    caps = {
        "serial_concurrency": 1,
        "per_identity_hard_wall_cap_s": 240,
        "stage_hard_wall_cap_s": 300,
        "minimum_free_disk_before_execution_bytes": 4294967296,
    }
    _require(release["resource_caps"] == caps, "resource caps differ")
    filesystem = os.statvfs(str(workspace))
    free_bytes = filesystem.f_bavail * filesystem.f_frsize
    _require(
        free_bytes >= caps["minimum_free_disk_before_execution_bytes"],
        "free disk below minimum",
    )
    if require_fresh_state:
        _require(not (workspace / OUTPUT_ROOT).exists(), "P3-I1 execution state exists")
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
