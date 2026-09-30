"""Fail-closed validator for the unique V2-04G-P2-C1-R1 release."""

from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
import re
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P2-C1-R1"
RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p2_c1_r1_execution_release.yaml"
)
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_r1_execution"
)
OLD_OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_execution"
)
HEX_SHA256 = re.compile(r"^[0-9a-f]{64}$")
EXPECTED_TIMEOUT_REPAIRS = {
    1: (80.0, 55.0),
    3: (80.0, 55.0),
    4: (80.0, 55.0),
    6: (80.0, 55.0),
    7: (80.0, 55.0),
    9: (80.0, 55.0),
    10: (80.0, 55.0),
    12: (80.0, 55.0),
}


class C1R1ReleaseError(ValueError):
    """Raised before any C1-R1 execution state is created."""


class _UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise C1R1ReleaseError("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise C1R1ReleaseError(message)


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


def _read_once(workspace: Path, relative: Path) -> bytes:
    """Use the frozen C1 single-open/no-follow implementation."""
    from thesis_experiment.v2_04g_r6_i1_p2_c1_release import (
        read_workspace_file_once,
    )

    try:
        return read_workspace_file_once(workspace, relative)
    except BaseException as exc:
        raise C1R1ReleaseError(
            "cannot read {}: {}".format(relative, exc)
        ) from exc


def _yaml(payload: bytes, label: str) -> Mapping[str, Any]:
    try:
        value = yaml.load(payload.decode("utf-8"), Loader=_UniqueLoader)
    except (UnicodeError, yaml.YAMLError) as exc:
        raise C1R1ReleaseError("cannot parse {}: {}".format(label, exc)) from exc
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


def _bound(
    workspace: Path, binding: Mapping[str, Any], label: str
) -> tuple[Path, bytes]:
    _require(
        type(binding) is dict and set(binding) == {"path", "sha256"},
        "{} binding schema differs".format(label),
    )
    _require(
        HEX_SHA256.fullmatch(str(binding["sha256"])) is not None,
        "{} SHA differs".format(label),
    )
    relative = _safe_relative(binding["path"])
    payload = _read_once(workspace, relative)
    _require(_sha(payload) == binding["sha256"], "{} hash differs".format(label))
    return relative, payload


def _identity(row: Mapping[str, Any]) -> Dict[str, Any]:
    value = {
        "sequence": row["sequence"],
        "row_type": row["row_type"],
        "scene_id": row["scene_id"],
        "seed": row["seed"],
        "attempt": row["attempt"],
    }
    if row["row_type"] == "calibration_navigation":
        value.update(
            {
                "method_id": row["method_id"],
                "block_id": row["block_id"],
                "scene_role": row["scene_role"],
            }
        )
    else:
        value.update(
            {
                "probe_type": row["probe_type"],
                "candidate_speed_mps": row["candidate_speed_mps"],
            }
        )
    return value


def _verify_scene_row(
    workspace: Path, row: Mapping[str, Any], plan_row: Mapping[str, Any]
) -> None:
    for label in ("instance", "world"):
        receipt = row[label]
        _require(
            type(receipt) is dict
            and set(receipt) == {"path", "sha256", "size_bytes"},
            "{} receipt differs".format(label),
        )
        payload = _read_once(workspace, _safe_relative(receipt["path"]))
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "{} bytes differ at sequence {}".format(label, row["sequence"]),
        )
        if label == "instance":
            instance_payload = payload
    instance = _yaml(instance_payload, "compiled scene instance")
    scene = instance.get("scene", {})
    _require(
        scene.get("scene_id") == row["scene_id"]
        and scene.get("seed") == row["seed"]
        and scene.get("split") == "calibration"
        and float(scene.get("timeout_s")) == float(row["timeout_s"]),
        "compiled scene identity differs at sequence {}".format(row["sequence"]),
    )
    binding = plan_row.get("scene_execution_binding")
    _require(
        type(binding) is dict
        and binding.get("source_instance") == row["instance"]
        and binding.get("source_world") == row["world"]
        and binding.get("start") == scene["start"]
        and float(binding.get("timeout_s")) == float(scene["timeout_s"]),
        "command-plan scene binding differs at sequence {}".format(row["sequence"]),
    )


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
    release_payload = _read_once(workspace, RELEASE_RELATIVE)
    _require(
        _sha(release_payload) == expected_release_sha256,
        "release bytes differ from caller SHA",
    )
    release = _yaml(release_payload, "P2-C1-R1 release")
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
        "predecessor_terminal_freeze",
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
        release["training_allowed"] is False
        and release["training_budget_steps"] == 0,
        "release training boundary differs",
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
    for label in ("runner", "raw_replay_adapter", "assessor", "launch_surface"):
        _bound(workspace, release[label], label)

    _require(contract.get("stage") == STAGE, "contract stage differs")
    _require(contract.get("execution_authorized") is True, "contract authority differs")
    _require(
        contract.get("authorized_evidence_units_max") == 342,
        "contract budget differs",
    )
    _require(
        contract["single_integrity_amendment"]["affected_sequences"]
        == list(EXPECTED_TIMEOUT_REPAIRS)
        and contract["single_integrity_amendment"]["behavioral_factor_count_changed"]
        == 0,
        "single integrity amendment differs",
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
        == "p2_c1_r1_independent_execution_readiness_pass_release_absent"
        and review.get("release_present_during_review") is False,
        "offline review did not pass release-absent",
    )

    predecessor = release["predecessor_terminal_freeze"]
    _require(
        type(predecessor) is dict
        and predecessor.get("execute_called") is False
        and predecessor.get("consumed_evidence_units") == 0
        and predecessor.get("forfeited_authority_units") == 342
        and predecessor.get("release_reuse_allowed") is False,
        "predecessor terminal state differs",
    )
    for label in ("old_release", "old_preexecution_audit"):
        _bound(workspace, predecessor[label], label)
    _require(not (workspace / OLD_OUTPUT_ROOT).exists(), "old C1 execution state exists")

    rows = schedule.get("rows")
    plan_rows = plan.get("rows")
    _require(type(rows) is list and len(rows) == 342, "schedule is not exact 342")
    _require(type(plan_rows) is list and len(plan_rows) == 342, "plan is not exact 342")
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
        == _canonical_sha(rows)
        == release["claim_boundary"]["schedule_row_digest"],
        "schedule digest differs",
    )
    _require(
        plan["canonical_plan_digest"]
        == _canonical_sha(plan_rows)
        == release["claim_boundary"]["command_plan_digest"],
        "command plan digest differs",
    )

    old_schedule_binding = contract["predecessor_terminal_freeze"]["old_schedule"]
    _, old_schedule_payload = _bound(
        workspace, old_schedule_binding, "old schedule"
    )
    old_schedule = _yaml(old_schedule_payload, "old schedule")
    _require(len(old_schedule["rows"]) == 342, "old schedule count differs")
    observed_repairs = {}
    for old_row, row, plan_row in zip(old_schedule["rows"], rows, plan_rows):
        _require(row["sequence"] == plan_row["sequence"], "plan sequence differs")
        _require(plan_row["identity"] == _identity(row), "plan identity differs")
        reconstructed = copy.deepcopy(row)
        reconstructed["stage"] = "V2-04G-P2-C1"
        if row["sequence"] in EXPECTED_TIMEOUT_REPAIRS:
            old_timeout, new_timeout = EXPECTED_TIMEOUT_REPAIRS[row["sequence"]]
            _require(
                float(row["timeout_s"]) == new_timeout,
                "repaired timeout differs",
            )
            reconstructed["timeout_s"] = old_timeout
            observed_repairs[row["sequence"]] = (old_timeout, new_timeout)
        _require(reconstructed == old_row, "non-amendment row field differs")
        _verify_scene_row(workspace, row, plan_row)
    _require(observed_repairs == EXPECTED_TIMEOUT_REPAIRS, "repair set differs")

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
        payload = _read_once(workspace, _safe_relative(receipt["path"]))
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
        _require(not (workspace / OUTPUT_ROOT).exists(), "C1-R1 execution state exists")
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
        "timeout_repairs_verified": list(EXPECTED_TIMEOUT_REPAIRS),
    }
