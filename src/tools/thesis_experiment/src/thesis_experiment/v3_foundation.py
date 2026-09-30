"""Pure-offline V3-00 foundation contracts and deterministic fixtures.

This module intentionally has no ROS imports and no execution entrypoint.  It
validates the V3 namespace, builds static candidate/actor fixtures, discovers
the required source packages, and produces a deterministic evidence bundle.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import stat
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Sequence, Tuple
import xml.etree.ElementTree as ET

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_00_foundation_contract.yaml"
)
FIXTURE_RELATIVE = Path(
    "config/thesis_experiments/v3/fixtures/v3_00_static_fixture_spec.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_00_foundation_preregistration.yaml"
)
SYSTEM_GUIDE_RELATIVE = Path("docs/thesis_experiment/V3_SYSTEM_GUIDE.md")
OUTPUT_ROOT = Path("artifacts/v3/foundation/v3_00")

SUMMARY_RELATIVE = OUTPUT_ROOT / "v3_00_foundation_summary.yaml"
INTERFACE_RELATIVE = OUTPUT_ROOT / "v3_00_interface_registry.yaml"
PACKAGE_CLOSURE_RELATIVE = OUTPUT_ROOT / "v3_00_hermetic_package_closure.yaml"
CANDIDATE_FIXTURE_RELATIVE = OUTPUT_ROOT / "v3_00_candidate_fixture.yaml"
ACTOR_CONTRACT_RELATIVE = (
    OUTPUT_ROOT / "v3_00_reactive_actor_cloned_rollout_contract.yaml"
)
FIXTURE_REPORT_RELATIVE = OUTPUT_ROOT / "v3_00_static_fixture_report.yaml"
ZERO_AUTHORITY_RELATIVE = OUTPUT_ROOT / "v3_00_zero_authority_receipt.yaml"
DEPENDENCY_CLOSURE_RELATIVE = OUTPUT_ROOT / "v3_00_dependency_closure.yaml"
EVIDENCE_FREEZE_RELATIVE = OUTPUT_ROOT / "v3_00_foundation_evidence_freeze.sha256"
REVIEW_RELATIVE = OUTPUT_ROOT / "v3_00_offline_review.yaml"
TERMINAL_FREEZE_RELATIVE = OUTPUT_ROOT / "v3_00_terminal_freeze.sha256"

MESSAGE_ROOT = Path("src/application/v3_scope_planner/msg")
MESSAGE_NAMES = (
    "TrajectoryPoint.msg",
    "CandidateTrajectory.msg",
    "CandidateSet.msg",
    "CandidateEvaluation.msg",
    "CandidateDecision.msg",
)
REQUIRED_PACKAGES = (
    "v3_scope_planner",
    "thesis_experiment",
    "m2_gazebo",
    "nav_world_model",
    "teb_mode_manager",
    "gps_module",
)
FORBIDDEN_RUNTIME_TOKENS = (
    b"roscore",
    b"rosmaster",
    b"roslaunch",
    b"gzserver",
    b"gzclient",
    b"move_base",
    b"t11_formal_run",
    b"run_t12_residual",
)


class V3FoundationError(ValueError):
    """Raised when a V3-00 fail-closed invariant differs."""


class _UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise V3FoundationError("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise V3FoundationError(message)


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def canonical_sha(value: Any) -> str:
    return sha256_bytes(
        json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    )


def yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(
        value,
        sort_keys=False,
        allow_unicode=True,
        default_flow_style=False,
    ).encode("utf-8")


def _safe_parts(relative: Path) -> Tuple[str, ...]:
    value = Path(relative)
    require(
        not value.is_absolute()
        and bool(value.parts)
        and all(part not in {"", ".", ".."} for part in value.parts),
        "unsafe workspace-relative path: {}".format(relative),
    )
    return value.parts


def read_workspace_file_once(
    workspace: Path, relative: Path, *, allow_hard_links: bool = False
) -> bytes:
    """Read a workspace file with component-wise no-follow and stable metadata."""

    root = Path(workspace).resolve(strict=True)
    parts = _safe_parts(relative)
    directory_flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags |= getattr(os, "O_DIRECTORY", 0)
    directory_flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptors: List[int] = []
    try:
        descriptor = os.open(str(root), directory_flags)
        descriptors.append(descriptor)
        for component in parts[:-1]:
            descriptor = os.open(component, directory_flags, dir_fd=descriptor)
            descriptors.append(descriptor)
        flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_NOFOLLOW", 0)
        file_descriptor = os.open(parts[-1], flags, dir_fd=descriptors[-1])
        try:
            before = os.fstat(file_descriptor)
            require(
                stat.S_ISREG(before.st_mode),
                "not a regular file: {}".format(relative),
            )
            if not allow_hard_links:
                require(
                    before.st_nlink == 1,
                    "hard-linked frozen file forbidden: {}".format(relative),
                )
            chunks = []
            while True:
                chunk = os.read(file_descriptor, 1024 * 1024)
                if not chunk:
                    break
                chunks.append(chunk)
            after = os.fstat(file_descriptor)
            stable_fields = (
                "st_dev",
                "st_ino",
                "st_mode",
                "st_nlink",
                "st_size",
                "st_mtime_ns",
                "st_ctime_ns",
            )
            require(
                tuple(getattr(before, field) for field in stable_fields)
                == tuple(getattr(after, field) for field in stable_fields),
                "file changed during read: {}".format(relative),
            )
            payload = b"".join(chunks)
            require(
                len(payload) == before.st_size,
                "short file read: {}".format(relative),
            )
            return payload
        finally:
            os.close(file_descriptor)
    except OSError as exc:
        raise V3FoundationError(
            "cannot no-follow read {}: {}".format(relative, exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def load_yaml_bytes(payload: bytes, label: str) -> Mapping[str, Any]:
    try:
        value = yaml.load(payload.decode("utf-8"), Loader=_UniqueLoader)
    except (UnicodeError, yaml.YAMLError) as exc:
        raise V3FoundationError("cannot parse {}: {}".format(label, exc)) from exc
    require(type(value) is dict, "YAML mapping required: {}".format(label))
    return value


def load_yaml(workspace: Path, relative: Path) -> Mapping[str, Any]:
    return load_yaml_bytes(
        read_workspace_file_once(workspace, relative), relative.as_posix()
    )


def receipt(workspace: Path, relative: Path) -> Dict[str, Any]:
    payload = read_workspace_file_once(workspace, relative)
    return {
        "path": Path(relative).as_posix(),
        "sha256": sha256_bytes(payload),
        "size_bytes": len(payload),
    }


def _bool(value: Any, label: str, expected: bool) -> None:
    require(type(value) is bool and value is expected, "{} differs".format(label))


def _exact_keys(value: Mapping[str, Any], expected: Iterable[str], label: str) -> None:
    require(
        set(value) == set(expected),
        "{} keys differ: {}".format(label, sorted(set(value) ^ set(expected))),
    )


def active_forbidden_processes() -> List[Dict[str, Any]]:
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ")
        except OSError:
            continue
        lowered = command.lower()
        if any(token in lowered for token in FORBIDDEN_RUNTIME_TOKENS):
            found.append(
                {
                    "pid": int(entry.name),
                    "command_sha256": sha256_bytes(command),
                }
            )
    return sorted(found, key=lambda row: row["pid"])


def _verify_source_freeze(
    workspace: Path, contract: Mapping[str, Any]
) -> List[Dict[str, Any]]:
    rows = []
    for label, binding in contract["source_freeze"].items():
        _exact_keys(binding, ("path", "sha256"), "source_freeze.{}".format(label))
        relative = Path(binding["path"])
        payload = read_workspace_file_once(workspace, relative)
        actual = sha256_bytes(payload)
        require(actual == binding["sha256"], "source freeze drift: {}".format(label))
        rows.append(
            {
                "label": label,
                "path": relative.as_posix(),
                "sha256": actual,
                "size_bytes": len(payload),
            }
        )

    p4 = load_yaml(
        workspace,
        Path(contract["source_freeze"]["p4_terminal_conclusion"]["path"]),
    )
    require(
        p4["status"] == "global_budget_exhausted_obvious_improvement_not_demonstrated"
        and p4["terminal_decision"]
        == "obvious_performance_improvement_not_demonstrated"
        and p4["global_budget"]["simulation_units_remaining"] == 0
        and p4["candidate_winner_frozen"] is False,
        "P4 terminal firewall differs",
    )
    review = load_yaml(
        workspace,
        Path(contract["source_freeze"]["p4_independent_review"]["path"]),
    )
    require(
        review["status"] == "p4_f1_independent_terminal_program_closure_pass"
        and review["simulation_units_remaining"] == 0
        and review["candidate_winner_frozen"] is False,
        "P4 independent review firewall differs",
    )

    concept_freeze = read_workspace_file_once(
        workspace,
        Path(contract["source_freeze"]["v3_concept_freeze"]["path"]),
    )
    concept_rows = []
    for line in concept_freeze.decode("utf-8").splitlines():
        if not line.strip():
            continue
        digest, path_text = line.split(None, 1)
        relative = Path(path_text.strip())
        payload = read_workspace_file_once(workspace, relative)
        require(
            sha256_bytes(payload) == digest,
            "V3 concept freeze resource drift: {}".format(relative),
        )
        concept_rows.append(relative.as_posix())
    require(len(concept_rows) == 14, "V3 concept freeze roster differs")
    return sorted(rows, key=lambda row: row["label"])


def validate_contract(
    workspace: Path, contract: Mapping[str, Any]
) -> Dict[str, Any]:
    require(contract["architecture_generation"] == "v3", "architecture differs")
    require(contract["stage"] == "V3-00", "stage differs")
    require(
        contract["status"] == "pure_offline_foundation_preregistered",
        "contract status differs",
    )
    _bool(contract["independent_stage"], "independent_stage", True)
    _bool(
        contract["supersedes_v2_stage_identity"],
        "supersedes_v2_stage_identity",
        False,
    )
    state = contract["stage_state"]
    for field, expected in (
        ("concept_only", True),
        ("offline_only", True),
        ("simulation_only", True),
        ("formal_result", False),
        ("runtime_ready", False),
        ("execution_ready", False),
        ("execution_authorized", False),
        ("training_authorized", False),
        ("real_vehicle_authorized", False),
        ("stable_workspace_modification_allowed", False),
    ):
        _bool(state[field], "stage_state.{}".format(field), expected)
    for field in (
        "evidence_budget_authorized",
        "training_budget_steps",
        "seed_allocation_count",
        "schedule_row_count",
    ):
        require(
            type(state[field]) is int and state[field] == 0,
            "stage_state.{} must be exact integer zero".format(field),
        )

    namespace = contract["namespace"]
    require(
        namespace
        == {
            "ros_package": "v3_scope_planner",
            "python_module": "thesis_experiment.v3_foundation",
            "config_root": "config/thesis_experiments/v3",
            "manifest_root": "experiments/manifests/v3",
            "artifact_root": "artifacts/v3",
            "runtime_topic_root": "/v3_scope",
            "v2_artifact_write_allowed": False,
        },
        "V3 namespace differs",
    )
    boundary = contract["hardware_and_command_boundary"]
    require(
        boundary["only_allowed_future_command_topic"] == "/cmd_vel_navigation"
        and boundary["direct_cmd_vel_publish_forbidden"] is True
        and boundary["direct_m2_driver_access_forbidden"] is True
        and boundary["stable_chain"]
        == [
            "/cmd_vel_navigation",
            "/cmd_vel_gps",
            "/fod_navigation_mode",
            "/cmd_vel",
            "m2_driver",
        ],
        "command safety boundary differs",
    )
    interface = contract["interface_contract"]
    require(
        interface["package_contract"]
        == "src/application/v3_scope_planner/config/interface_contract.yaml"
        and tuple(interface["messages"]) == MESSAGE_NAMES
        and interface["runtime_nodes_present_in_v3_00"] is False
        and interface["launch_files_present_in_v3_00"] is False
        and interface["model_files_present_in_v3_00"] is False,
        "interface contract differs",
    )
    actor = contract["reactive_actor_and_clone_contract"]
    require(
        actor["actor_policy_ids"] == ["cautious", "assertive", "hesitant"]
        and actor["actor_hidden_type_evaluator_only"] is True
        and actor["runtime_may_read_actor_hidden_type"] is False
        and actor["runtime_may_read_scene_role"] is False
        and actor["runtime_may_read_future_truth"] is False
        and actor["behavior_propensity_strictly_positive"] is True
        and actor["fixture_is_schema_test_not_simulation_evidence"] is True,
        "actor/clone firewall differs",
    )
    environment = contract["hermetic_environment_contract"]
    require(
        environment["future_process_start"] == "env_i_non_login_shell"
        and environment["package_resolution_probe_required_before_consumption"]
        is True
        and environment[
            "package_resolution_failure_may_cross_consumption_boundary"
        ]
        is False
        and environment["credential_keys_in_child_environment_allowed"] is False
        and tuple(environment["required_workspace_packages"]) == REQUIRED_PACKAGES
        and environment["current_runtime_environment_snapshot_created"] is False,
        "hermetic environment policy differs",
    )
    require(
        contract["forbidden_outputs"]
        == [
            "execution_release.yaml",
            "simulation_authorization.yaml",
            "seed_allocation.yaml",
            "exact_schedule.yaml",
            "attempts",
            "journals",
            "models",
            "checkpoints",
        ],
        "forbidden output roster differs",
    )
    source_rows = _verify_source_freeze(workspace, contract)
    return {
        "contract_valid": True,
        "source_freeze_count": len(source_rows),
        "source_freeze": source_rows,
    }


def validate_preregistration(preregistration: Mapping[str, Any]) -> None:
    require(preregistration["stage"] == "V3-00", "preregistration stage differs")
    require(
        preregistration["status"] == "frozen_before_foundation_materialization",
        "preregistration status differs",
    )
    authority = preregistration["authority"]
    _bool(authority["offline_design_authorized"], "offline authority", True)
    for field in (
        "ros_authorized",
        "gazebo_authorized",
        "execution_authorized",
        "training_authorized",
        "real_vehicle_authorized",
    ):
        _bool(authority[field], "authority.{}".format(field), False)
    for field in (
        "evidence_budget_authorized",
        "seed_allocation_count",
        "schedule_row_count",
    ):
        require(
            type(authority[field]) is int and authority[field] == 0,
            "authority.{} must be exact integer zero".format(field),
        )
    require(
        preregistration["planned_outputs"]["root"] == OUTPUT_ROOT.as_posix(),
        "planned output root differs",
    )
    require(
        "no_runtime_process_and_no_execution_state"
        in preregistration["acceptance_gates"],
        "zero-runtime gate absent",
    )


def _package_name(package_xml: bytes, label: str) -> str:
    try:
        root = ET.fromstring(package_xml)
    except ET.ParseError as exc:
        raise V3FoundationError(
            "cannot parse package XML {}: {}".format(label, exc)
        ) from exc
    node = root.find("name")
    require(node is not None and node.text, "package name absent: {}".format(label))
    return node.text.strip()


def discover_required_packages(
    workspace: Path, required: Sequence[str] = REQUIRED_PACKAGES
) -> List[Dict[str, Any]]:
    matches: MutableMapping[str, List[Dict[str, Any]]] = {
        name: [] for name in required
    }
    source_root = Path(workspace) / "src"
    for directory, names, files in os.walk(str(source_root), followlinks=False):
        names[:] = sorted(
            name
            for name in names
            if name
            not in {
                ".git",
                "build",
                "devel",
                "install",
                "log",
                "__pycache__",
            }
        )
        if "package.xml" not in files:
            continue
        absolute = Path(directory) / "package.xml"
        relative = absolute.relative_to(Path(workspace))
        payload = read_workspace_file_once(workspace, relative)
        name = _package_name(payload, relative.as_posix())
        if name in matches:
            matches[name].append(
                {
                    "package_name": name,
                    "source_root": relative.parent.as_posix(),
                    "package_xml_path": relative.as_posix(),
                    "package_xml_sha256": sha256_bytes(payload),
                }
            )
    rows = []
    for name in required:
        require(
            len(matches[name]) == 1,
            "required package resolution is not unique: {} -> {}".format(
                name, len(matches[name])
            ),
        )
        rows.append(matches[name][0])
    return rows


def parse_message_registry(
    workspace: Path, names: Sequence[str] = MESSAGE_NAMES
) -> List[Dict[str, Any]]:
    registry = []
    for name in names:
        relative = MESSAGE_ROOT / name
        payload = read_workspace_file_once(workspace, relative)
        fields = []
        constants = []
        for line_number, raw in enumerate(
            payload.decode("utf-8").splitlines(), start=1
        ):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.split()
            require(
                len(parts) == 2,
                "invalid message declaration {}:{}".format(relative, line_number),
            )
            message_type, declaration = parts
            row = {
                "type": message_type,
                "name": declaration.split("=", 1)[0],
                "line": line_number,
            }
            if "=" in declaration:
                row["value"] = declaration.split("=", 1)[1]
                constants.append(row)
            else:
                fields.append(row)
        require(fields, "message has no fields: {}".format(relative))
        registry.append(
            {
                "message": name[:-4],
                "path": relative.as_posix(),
                "sha256": sha256_bytes(payload),
                "field_count": len(fields),
                "constant_count": len(constants),
                "fields": fields,
                "constants": constants,
            }
        )
    require(
        [row["message"] + ".msg" for row in registry] == list(names),
        "message registry order differs",
    )
    return registry


def _finite_number(value: Any, label: str) -> float:
    require(
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value)),
        "{} must be finite numeric".format(label),
    )
    return float(value)


def compile_candidate_fixture(
    contract: Mapping[str, Any], fixture: Mapping[str, Any]
) -> Dict[str, Any]:
    require(fixture["simulation_evidence"] is False, "fixture claims evidence")
    candidate_contract = contract["candidate_contract"]
    vehicle = fixture["vehicle"]
    for field in (
        "wheelbase_m",
        "min_turning_radius_m",
        "max_center_steering_angle_rad",
        "max_steering_rate_radps",
        "max_speed_mps",
    ):
        require(
            _finite_number(vehicle[field], "vehicle.{}".format(field))
            == _finite_number(
                candidate_contract["simulation_vehicle"][field],
                "contract vehicle.{}".format(field),
            ),
            "fixture vehicle differs: {}".format(field),
        )
    require(
        vehicle["calibration_status"] == "simulation_candidate",
        "fixture vehicle status differs",
    )
    dt = _finite_number(fixture["integration"]["dt_s"], "dt")
    point_count = fixture["integration"]["point_count"]
    require(dt > 0.0, "dt must be positive")
    require(
        type(point_count) is int and point_count >= 3,
        "point_count must be integer >= 3",
    )
    source_candidates = fixture["candidates"]
    require(
        len(source_candidates) == candidate_contract["fixture_candidate_count"],
        "fixture candidate count differs",
    )
    candidate_ids = [row["candidate_id"] for row in source_candidates]
    require(
        len(candidate_ids) == len(set(candidate_ids)),
        "fixture candidate IDs are not unique",
    )
    wheelbase = float(vehicle["wheelbase_m"])
    max_steer = float(vehicle["max_center_steering_angle_rad"])
    max_rate = float(vehicle["max_steering_rate_radps"])
    max_speed = float(vehicle["max_speed_mps"])
    initial = fixture["initial_state"]
    initial_digest = canonical_sha(
        {
            "fixture_id": fixture["fixture_id"],
            "initial_state": initial,
            "vehicle": vehicle,
        }
    )

    compiled = []
    maximum_residual = 0.0
    for source in source_candidates:
        candidate_id = source["candidate_id"]
        require(
            isinstance(candidate_id, str) and candidate_id,
            "candidate ID must be non-empty",
        )
        velocity = _finite_number(
            source["velocity_mps"], "{}.velocity".format(candidate_id)
        )
        target_steer = _finite_number(
            source["steering_angle_rad"], "{}.steering".format(candidate_id)
        )
        assertiveness = _finite_number(
            source["assertiveness"], "{}.assertiveness".format(candidate_id)
        )
        require(abs(velocity) <= max_speed, "candidate speed exceeds limit")
        require(abs(target_steer) <= max_steer, "candidate steering exceeds limit")
        require(
            -1.0 <= assertiveness <= 1.0,
            "candidate assertiveness outside fixture range",
        )
        x = _finite_number(initial["x_m"], "initial x")
        y = _finite_number(initial["y_m"], "initial y")
        yaw = _finite_number(initial["yaw_rad"], "initial yaw")
        points = []
        previous_steer = _finite_number(
            initial["steering_angle_rad"], "initial steering"
        )
        for index in range(point_count):
            time_s = index * dt
            if index == 0:
                steer = previous_steer
                steer_rate = 0.0
            else:
                maximum_step = max_rate * dt
                difference = target_steer - previous_steer
                steer = previous_steer + max(
                    -maximum_step, min(maximum_step, difference)
                )
                steer_rate = (steer - previous_steer) / dt
            curvature = math.tan(steer) / wheelbase
            point = {
                "time_from_start_s": time_s,
                "x_m": x,
                "y_m": y,
                "yaw_rad": yaw,
                "longitudinal_velocity_mps": velocity,
                "longitudinal_acceleration_mps2": 0.0,
                "steering_angle_rad": steer,
                "steering_rate_radps": steer_rate,
                "curvature_inv_m": curvature,
            }
            points.append(point)
            if index < point_count - 1:
                x = x + velocity * math.cos(yaw) * dt
                y = y + velocity * math.sin(yaw) * dt
                yaw = yaw + velocity * curvature * dt
            previous_steer = steer

        residuals = []
        for left, right in zip(points[:-1], points[1:]):
            expected_x = (
                left["x_m"]
                + left["longitudinal_velocity_mps"]
                * math.cos(left["yaw_rad"])
                * dt
            )
            expected_y = (
                left["y_m"]
                + left["longitudinal_velocity_mps"]
                * math.sin(left["yaw_rad"])
                * dt
            )
            expected_yaw = (
                left["yaw_rad"]
                + left["longitudinal_velocity_mps"]
                * left["curvature_inv_m"]
                * dt
            )
            residual = max(
                abs(right["x_m"] - expected_x),
                abs(right["y_m"] - expected_y),
                abs(right["yaw_rad"] - expected_yaw),
            )
            residuals.append(residual)
            maximum_residual = max(maximum_residual, residual)
        candidate = {
            "candidate_id": candidate_id,
            "assertiveness_fixture_only": assertiveness,
            "certified_stop": bool(source["certified_stop"]),
            "teb_fallback": bool(source["teb_fallback"]),
            "points": points,
            "maximum_discrete_residual": max(residuals),
        }
        candidate["candidate_digest"] = canonical_sha(candidate)
        compiled.append(candidate)

    require(
        sum(bool(row["certified_stop"]) for row in compiled)
        >= candidate_contract["certified_stop_min_count"],
        "certified stop candidate absent",
    )
    require(
        sum(bool(row["teb_fallback"]) for row in compiled)
        >= candidate_contract["teb_fallback_min_count"],
        "TEB fallback candidate absent",
    )
    require(
        maximum_residual <= candidate_contract["maximum_discrete_residual"],
        "candidate kinematic residual exceeds contract",
    )
    output = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-00",
        "status": "static_candidate_schema_fixture_pass",
        "simulation_evidence": False,
        "formal_result": False,
        "runtime_ready": False,
        "scene_snapshot_id": initial_digest,
        "vehicle": vehicle,
        "integration": fixture["integration"],
        "candidate_count": len(compiled),
        "certified_stop_count": sum(
            bool(row["certified_stop"]) for row in compiled
        ),
        "teb_fallback_count": sum(bool(row["teb_fallback"]) for row in compiled),
        "maximum_discrete_residual": maximum_residual,
        "candidates": compiled,
    }
    output["candidate_set_digest"] = canonical_sha(
        {
            "scene_snapshot_id": initial_digest,
            "vehicle": vehicle,
            "candidates": compiled,
        }
    )
    return output


def compile_actor_clone_contract(
    contract: Mapping[str, Any],
    fixture: Mapping[str, Any],
    candidate_fixture: Mapping[str, Any],
) -> Dict[str, Any]:
    actor_contract = contract["reactive_actor_and_clone_contract"]
    actor_fixture = fixture["actor_fixture"]
    policy_ids = [row["policy_id"] for row in actor_fixture["policies"]]
    require(
        policy_ids == actor_contract["actor_policy_ids"],
        "actor fixture policy roster differs",
    )
    expected_propensity = float(
        actor_contract["fixture_propensity_per_candidate"]
    )
    require(
        float(actor_fixture["behavior_propensity_per_candidate"])
        == expected_propensity,
        "fixture propensity differs",
    )
    candidates = candidate_fixture["candidates"]
    require(
        math.isclose(
            expected_propensity * len(candidates),
            1.0,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ),
        "fixture propensities do not sum to one",
    )
    noise_floor = float(actor_contract["fixture_response_noise_floor_s"])
    require(
        float(actor_fixture["response_noise_floor_s"]) == noise_floor,
        "actor fixture noise floor differs",
    )
    block_digest = canonical_sha(
        {
            "initial_state_label": actor_fixture["initial_state_label"],
            "scene_snapshot_id": candidate_fixture["scene_snapshot_id"],
            "candidate_set_digest": candidate_fixture["candidate_set_digest"],
        }
    )
    policy_audits = []
    rows = []
    for policy in actor_fixture["policies"]:
        require(policy["hidden_from_runtime"] is True, "actor label is not hidden")
        gain = _finite_number(
            policy["response_gain_s"], "{}.gain".format(policy["policy_id"])
        )
        bias = _finite_number(
            policy["response_bias_s"], "{}.bias".format(policy["policy_id"])
        )
        responses = []
        for candidate in candidates:
            response = bias + gain * candidate["assertiveness_fixture_only"]
            responses.append(response)
            rows.append(
                {
                    "block_id": block_digest,
                    "clone_id": "{}__{}".format(
                        policy["policy_id"], candidate["candidate_id"]
                    ),
                    "scene_snapshot_id": candidate_fixture["scene_snapshot_id"],
                    "candidate_set_digest": candidate_fixture[
                        "candidate_set_digest"
                    ],
                    "candidate_id": candidate["candidate_id"],
                    "candidate_digest": candidate["candidate_digest"],
                    "behavior_propensity": expected_propensity,
                    "actor_policy_id_evaluator_only": policy["policy_id"],
                    "actor_policy_exposed_to_runtime": False,
                    "synthetic_response_delta_s": response,
                    "simulation_evidence": False,
                }
            )
        effect_range = max(responses) - min(responses)
        require(
            effect_range > noise_floor,
            "actor response fixture does not clear noise floor: {}".format(
                policy["policy_id"]
            ),
        )
        policy_audits.append(
            {
                "policy_id": policy["policy_id"],
                "clone_count": len(candidates),
                "response_range_s": effect_range,
                "noise_floor_s": noise_floor,
                "response_effect_fixture_pass": True,
            }
        )
    output = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-00",
        "status": "reactive_actor_cloned_rollout_data_contract_fixture_pass",
        "simulation_evidence": False,
        "counterfactual_identifiability_demonstrated": False,
        "runtime_ready": False,
        "actor_policy_ids": policy_ids,
        "runtime_label_firewall": {
            "actor_hidden_type_evaluator_only": True,
            "runtime_may_read_actor_hidden_type": False,
            "runtime_may_read_scene_role": False,
            "runtime_may_read_future_truth": False,
        },
        "required_future_scene_families": actor_contract[
            "future_scene_families"
        ],
        "block_id": block_digest,
        "candidate_count_per_policy": len(candidates),
        "clone_count": len(rows),
        "behavior_propensity_per_candidate": expected_propensity,
        "all_propensities_strictly_positive": all(
            row["behavior_propensity"] > 0.0 for row in rows
        ),
        "same_initial_state_digest_within_block": len(
            {row["scene_snapshot_id"] for row in rows}
        )
        == 1,
        "candidate_coverage_each_policy": True,
        "policy_audits": policy_audits,
        "static_rows": rows,
        "future_raw_record_required_fields": [
            "release_id",
            "environment_fingerprint",
            "block_id",
            "scene_family",
            "geometry_family",
            "actor_policy_family_evaluator_only",
            "clone_id",
            "scene_snapshot_id",
            "candidate_set_digest",
            "candidate_id",
            "candidate_digest",
            "behavior_propensity",
            "prediction_digest",
            "occupancy_tube_digest",
            "safety_verdict",
            "rejection_codes",
            "executed_candidate_id",
            "tracker_deviation_trace",
            "actor_response_trace",
            "outcome",
            "terminal_reason",
            "raw_resource_bindings",
        ],
    }
    output["contract_digest"] = canonical_sha(output)
    return output


def build_interface_registry(
    workspace: Path, contract: Mapping[str, Any]
) -> Dict[str, Any]:
    package_contract_relative = Path(
        contract["interface_contract"]["package_contract"]
    )
    package_contract = load_yaml(workspace, package_contract_relative)
    require(
        package_contract["package"] == "v3_scope_planner"
        and package_contract["runtime_ready"] is False
        and package_contract["execution_authorized"] is False
        and package_contract["training_authorized"] is False
        and package_contract["publish_boundary"]["only_allowed_command_topic"]
        == "/cmd_vel_navigation"
        and package_contract["publish_boundary"][
            "direct_cmd_vel_publish_forbidden"
        ]
        is True,
        "package interface contract differs",
    )
    registry = parse_message_registry(workspace)
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-00",
        "status": "interface_registry_pass",
        "offline_only": True,
        "runtime_ready": False,
        "execution_authorized": False,
        "ros_package": "v3_scope_planner",
        "message_count": len(registry),
        "messages": registry,
        "publish_boundary": package_contract["publish_boundary"],
        "future_runtime_requirements": package_contract[
            "future_runtime_requirements"
        ],
    }


def build_package_closure(
    workspace: Path, contract: Mapping[str, Any]
) -> Dict[str, Any]:
    packages = discover_required_packages(
        workspace,
        tuple(
            contract["hermetic_environment_contract"][
                "required_workspace_packages"
            ]
        ),
    )
    environment = contract["hermetic_environment_contract"]
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-00",
        "status": "source_package_resolution_and_future_environment_design_pass",
        "offline_only": True,
        "execution_ready": False,
        "execution_authorized": False,
        "current_runtime_environment_snapshot_created": False,
        "required_package_count": len(packages),
        "required_packages": packages,
        "future_environment_policy": {
            "process_start": environment["future_process_start"],
            "required_source_order": environment["required_source_order"],
            "required_bound_environment_keys": environment[
                "required_bound_environment_keys"
            ],
            "credential_keys_in_child_environment_allowed": environment[
                "credential_keys_in_child_environment_allowed"
            ],
            "source_script_path_and_sha_required": environment[
                "source_script_path_and_sha_required"
            ],
            "executable_path_and_sha_required": environment[
                "executable_path_and_sha_required"
            ],
            "package_resolution_probe_required_before_consumption": environment[
                "package_resolution_probe_required_before_consumption"
            ],
            "package_resolution_failure_may_cross_consumption_boundary": environment[
                "package_resolution_failure_may_cross_consumption_boundary"
            ],
        },
        "future_release_missing_bindings": [
            "exact_source_script_receipts",
            "exact_environment_key_value_receipts",
            "exact_executable_receipts",
            "launch_file_receipts",
            "package_lookup_receipts",
            "runtime_ports_and_process_ownership",
        ],
    }


def _dependency_paths(
    contract: Mapping[str, Any],
) -> List[Path]:
    paths = {
        CONTRACT_RELATIVE,
        FIXTURE_RELATIVE,
        PREREGISTRATION_RELATIVE,
        SYSTEM_GUIDE_RELATIVE,
        Path("src/application/v3_scope_planner/package.xml"),
        Path("src/application/v3_scope_planner/CMakeLists.txt"),
        Path("src/application/v3_scope_planner/config/interface_contract.yaml"),
        Path("src/application/v3_scope_planner/docs/INTERFACES.md"),
        Path(
            "src/tools/thesis_experiment/src/thesis_experiment/v3_foundation.py"
        ),
        Path("src/tools/thesis_experiment/scripts/build_v3_00_foundation.py"),
        Path("src/tools/thesis_experiment/scripts/review_v3_00_foundation.py"),
        Path("src/tools/thesis_experiment/tests/test_v3_00_foundation.py"),
        Path("src/tools/thesis_experiment/CMakeLists.txt"),
    }
    paths.update(MESSAGE_ROOT / name for name in MESSAGE_NAMES)
    for binding in contract["source_freeze"].values():
        paths.add(Path(binding["path"]))
    return sorted(paths, key=lambda path: path.as_posix())


def build_dependency_closure(
    workspace: Path, contract: Mapping[str, Any]
) -> Dict[str, Any]:
    resources = []
    unresolved = []
    for relative in _dependency_paths(contract):
        try:
            resources.append(receipt(workspace, relative))
        except V3FoundationError as exc:
            unresolved.append(
                {"path": relative.as_posix(), "reason": str(exc)}
            )
    output = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-00",
        "status": "dependency_closure_complete"
        if not unresolved
        else "dependency_closure_incomplete",
        "single_open_component_no_follow": True,
        "hard_links_allowed": False,
        "resource_count": len(resources),
        "local_resources": resources,
        "unresolved": unresolved,
    }
    output["logical_sha256"] = canonical_sha(output)
    return output


def _output_receipt(relative: Path, payload: bytes) -> Dict[str, Any]:
    return {
        "path": relative.as_posix(),
        "sha256": sha256_bytes(payload),
        "size_bytes": len(payload),
    }


def build_bundle(
    workspace: Path = WORKSPACE,
) -> Dict[Path, bytes]:
    workspace = Path(workspace).resolve()
    require(
        workspace == WORKSPACE.resolve(),
        "V3-00 workspace identity differs",
    )
    contract = load_yaml(workspace, CONTRACT_RELATIVE)
    contract_audit = validate_contract(workspace, contract)
    preregistration = load_yaml(workspace, PREREGISTRATION_RELATIVE)
    validate_preregistration(preregistration)
    fixture = load_yaml(workspace, FIXTURE_RELATIVE)

    interface_registry = build_interface_registry(workspace, contract)
    package_closure = build_package_closure(workspace, contract)
    candidate_fixture = compile_candidate_fixture(contract, fixture)
    actor_contract = compile_actor_clone_contract(
        contract, fixture, candidate_fixture
    )
    dependency_closure = build_dependency_closure(workspace, contract)
    require(
        dependency_closure["unresolved"] == [],
        "V3-00 dependency closure contains unresolved resources",
    )
    require(
        not active_forbidden_processes(),
        "forbidden ROS/Gazebo/navigation process is active",
    )

    fixture_report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-00",
        "status": "all_static_foundation_fixtures_pass",
        "simulation_evidence": False,
        "candidate_fixture": {
            "pass": True,
            "candidate_count": candidate_fixture["candidate_count"],
            "certified_stop_count": candidate_fixture["certified_stop_count"],
            "teb_fallback_count": candidate_fixture["teb_fallback_count"],
            "maximum_discrete_residual": candidate_fixture[
                "maximum_discrete_residual"
            ],
            "candidate_set_digest": candidate_fixture[
                "candidate_set_digest"
            ],
        },
        "actor_clone_fixture": {
            "pass": True,
            "policy_count": len(actor_contract["actor_policy_ids"]),
            "clone_count": actor_contract["clone_count"],
            "all_propensities_strictly_positive": actor_contract[
                "all_propensities_strictly_positive"
            ],
            "same_initial_state_digest_within_block": actor_contract[
                "same_initial_state_digest_within_block"
            ],
            "counterfactual_identifiability_demonstrated": False,
        },
        "package_resolution_fixture": {
            "pass": True,
            "required_package_count": package_closure[
                "required_package_count"
            ],
            "current_runtime_environment_snapshot_created": False,
        },
    }
    zero_authority = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-00",
        "status": "zero_authority_boundary_pass",
        "concept_only": True,
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "training_authorized": False,
        "evidence_budget_authorized": 0,
        "training_budget_steps": 0,
        "seed_allocation_count": 0,
        "schedule_row_count": 0,
        "real_vehicle_authorized": False,
        "active_forbidden_process_count": 0,
        "next_stage_automatically_authorized": False,
        "forbidden_outputs_absent": True,
    }
    summary = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-00",
        "status": "pure_offline_foundation_materialization_ready",
        "offline_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "training_authorized": False,
        "evidence_budget_authorized": 0,
        "source_freeze_count": contract_audit["source_freeze_count"],
        "dependency_resource_count": dependency_closure["resource_count"],
        "required_package_count": package_closure["required_package_count"],
        "message_count": interface_registry["message_count"],
        "candidate_count": candidate_fixture["candidate_count"],
        "actor_policy_count": len(actor_contract["actor_policy_ids"]),
        "static_clone_count": actor_contract["clone_count"],
        "simulation_evidence_created": False,
        "next_safe_entry": "independent_v3_00_review_then_v3_01_offline_design",
    }

    documents = {
        SUMMARY_RELATIVE: summary,
        INTERFACE_RELATIVE: interface_registry,
        PACKAGE_CLOSURE_RELATIVE: package_closure,
        CANDIDATE_FIXTURE_RELATIVE: candidate_fixture,
        ACTOR_CONTRACT_RELATIVE: actor_contract,
        FIXTURE_REPORT_RELATIVE: fixture_report,
        ZERO_AUTHORITY_RELATIVE: zero_authority,
        DEPENDENCY_CLOSURE_RELATIVE: dependency_closure,
    }
    outputs: Dict[Path, bytes] = {
        relative: yaml_bytes(value) for relative, value in documents.items()
    }
    freeze_rows = list(dependency_closure["local_resources"])
    freeze_rows.extend(
        _output_receipt(relative, payload)
        for relative, payload in sorted(
            outputs.items(), key=lambda item: item[0].as_posix()
        )
    )
    unique = {}
    for row in freeze_rows:
        unique[row["path"]] = row
    freeze_payload = "".join(
        "{}  {}\n".format(unique[path]["sha256"], path)
        for path in sorted(unique)
    ).encode("utf-8")
    outputs[EVIDENCE_FREEZE_RELATIVE] = freeze_payload
    return outputs


def validate_persisted_bundle(
    workspace: Path, bundle: Mapping[Path, bytes]
) -> None:
    for relative, expected in bundle.items():
        actual = read_workspace_file_once(workspace, relative)
        require(actual == expected, "persisted V3-00 bundle drift: {}".format(relative))
    for forbidden in (
        OUTPUT_ROOT / "execution_release.yaml",
        OUTPUT_ROOT / "simulation_authorization.yaml",
        OUTPUT_ROOT / "seed_allocation.yaml",
        OUTPUT_ROOT / "exact_schedule.yaml",
        OUTPUT_ROOT / "attempts",
        OUTPUT_ROOT / "journals",
        OUTPUT_ROOT / "models",
        OUTPUT_ROOT / "checkpoints",
    ):
        require(
            not (Path(workspace) / forbidden).exists(),
            "forbidden V3-00 execution state exists: {}".format(forbidden),
        )
