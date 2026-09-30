#!/usr/bin/env python3
"""Independent release-absent review for the one-unit P3-I1 preflight."""

from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.util
import os
from pathlib import Path
import socket
from typing import Any, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P3-I1"
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p3_i1_release_readiness"
)
REVIEW_RELATIVE = OUTPUT_ROOT / "v2_04g_p3_i1_offline_review.yaml"
RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p3_i1_execution_release.yaml"
)
EXECUTION_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p3_i1_execution"
)
BUILDER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_i1_release_builder.py"
)
RUNNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_i1_exact_runner.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p3_i1_offline_reviewer.py"
)
SUPERVISOR_RELATIVE = Path(
    "src/application/teb_mode_manager/scripts/r6_rule_context_supervisor_node.py"
)


class P3I1ReviewError(ValueError):
    """Raised when one-unit preflight readiness differs."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P3I1ReviewError(message)


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    _require(type(value) is dict, "YAML root differs")
    return value


def _load(name: str, relative: Path):
    path = WORKSPACE / relative
    spec = importlib.util.spec_from_file_location(name, path)
    _require(spec is not None and spec.loader is not None, "cannot load module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BUILDER = _load("_p3_i1_builder_review", BUILDER_RELATIVE)
RUNNER = _load("_p3_i1_runner_review", RUNNER_RELATIVE)


def _active_runtime_processes():
    tokens = (
        b"roscore",
        b"rosmaster",
        b"roslaunch",
        b"gzserver",
        b"gzclient",
        b"move_base",
    )
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ")
        except OSError:
            continue
        if any(token in command.lower() for token in tokens):
            found.append({"pid": int(entry.name), "command_sha256": _sha(command)})
    return found


def _listening(port: int) -> bool:
    connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connection.settimeout(0.1)
    try:
        return connection.connect_ex(("127.0.0.1", port)) == 0
    finally:
        connection.close()


def _normalize(argv):
    prefix = str(WORKSPACE) + "/"
    return [str(item).replace(prefix, "") for item in argv]


def _supervisor_profile_keys():
    tree = ast.parse((WORKSPACE / SUPERVISOR_RELATIVE).read_text(encoding="utf-8"))
    for node in tree.body:
        if (
            isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and node.targets[0].id == "EXPECTED_ESTIMATOR_BY_PROFILE"
            and isinstance(node.value, ast.Dict)
        ):
            return [key.value for key in node.value.keys]
    raise P3I1ReviewError("supervisor profile allowlist absent")


def build_review(workspace: Path = WORKSPACE):
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    _require(not (workspace / RELEASE_RELATIVE).exists(), "release exists during review")
    _require(not (workspace / EXECUTION_ROOT).exists(), "execution state exists")
    expected = BUILDER.build_bundle(workspace)
    for relative, payload in expected.items():
        path = workspace / relative
        _require(path.is_file() and path.read_bytes() == payload, "bundle bytes differ")
    schedule = _load_yaml(workspace / BUILDER.SCHEDULE_RELATIVE)
    plan = _load_yaml(workspace / BUILDER.PLAN_RELATIVE)
    closure = _load_yaml(workspace / BUILDER.CLOSURE_RELATIVE)
    methods = _load_yaml(workspace / BUILDER.METHODS_RELATIVE)
    _require(len(schedule["rows"]) == len(plan["rows"]) == 1, "row count differs")
    row = schedule["rows"][0]
    planned = plan["rows"][0]
    _require(
        row["seed"] == 5624
        and row["probe_type"] == "high_speed_to_conflict_overlay_transition"
        and row["candidate_method_id"] == "circle_contact_v125"
        and planned["runtime_profile_id"] == "r6_semantics_circle_contact"
        and planned["evidence_candidate_method_id"] == "circle_contact_v125",
        "identity separation differs",
    )
    _require(
        "r6_semantics_circle_contact" in _supervisor_profile_keys(),
        "runtime profile absent from supervisor allowlist",
    )
    closure_map = {
        receipt["path"]: receipt["sha256"]
        for receipt in closure["local_resources"]
    }
    RUNNER.configure_static_bindings(methods, closure_map)
    instance = _load_yaml(workspace / row["instance"]["path"])
    runtime = RUNNER.runtime(row, closure_map)
    executables = {
        "roslaunch": "{ROSLAUNCH_EXECUTABLE}",
        "xacro": "{XACRO_EXECUTABLE}",
        "python": "{PYTHON_EXECUTABLE}",
    }
    actual_base = RUNNER.base_command(
        row,
        instance["scene"],
        "{FUTURE_SCENE_SNAPSHOT_WORLD}",
        runtime,
        executables,
    )
    actual_transaction = RUNNER.transaction_command(row, runtime, executables)
    actual_probe = RUNNER.probe_command(
        row, Path("{FUTURE_ATTEMPT_ROOT}/raw_trace.yaml"), executables
    )
    _require(_normalize(actual_base) == planned["base_launch_argv"], "base argv differs")
    _require(
        _normalize(actual_transaction) == planned["semantic_transaction_launch_argv"],
        "transaction argv differs",
    )
    _require(_normalize(actual_probe) == planned["probe_argv"], "probe argv differs")
    profile_args = [
        value
        for argv in (actual_base, actual_transaction)
        for value in argv
        if value.startswith("attempt_profile_id:=")
    ]
    _require(
        profile_args
        == [
            "attempt_profile_id:=r6_semantics_circle_contact",
            "attempt_profile_id:=r6_semantics_circle_contact",
        ],
        "runtime profile command binding differs",
    )
    _require(closure["unresolved"] == [], "closure unresolved")
    for receipt in closure["local_resources"]:
        payload = (workspace / receipt["path"]).read_bytes()
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "local closure receipt differs",
        )
    for receipt in closure["external_resources"]:
        path = Path(receipt["path"])
        payload = path.read_bytes()
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "external closure receipt differs",
        )
    processes = _active_runtime_processes()
    _require(processes == [], "runtime process active")
    _require(not _listening(11311) and not _listening(11345), "ROS port active")
    reviewer_payload = (workspace / REVIEWER_RELATIVE).read_bytes()
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "p3_i1_independent_one_unit_execution_readiness_pass_release_absent",
        "review_result": "pass",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": True,
        "execution_authorized": True,
        "authorized_evidence_units_max": 1,
        "consumed_evidence_units": 0,
        "release_present_during_review": False,
        "fresh_identity": {
            "seed": 5624,
            "probe": "high_speed_to_conflict_overlay_transition",
            "candidate_speed_mps": 1.25,
        },
        "single_factor_review": {
            "runtime_profile_id": "r6_semantics_circle_contact",
            "evidence_candidate_method_id": "circle_contact_v125",
            "runtime_profile_accepted_by_consumer": True,
            "base_and_transaction_commands_match_runner": True,
            "behavioral_TEB_parameter_change": False,
        },
        "dependency_closure": {
            "local_resources_verified": closure["local_resource_count"],
            "external_resources_verified": closure["external_resource_count"],
            "unresolved": [],
        },
        "budget_review": {
            "round_2_maximum_units": 704,
            "this_release_units": 1,
            "future_units_authorized": 0,
            "additive_expansion": False,
        },
        "side_effect_audit": {
            "execution_root_absent": True,
            "release_absent": True,
            "runtime_processes": processes,
            "ROS_or_Gazebo_started_by_review": False,
            "held_out_test_accessed": False,
        },
        "reviewer": {
            "path": REVIEWER_RELATIVE.as_posix(),
            "sha256": _sha(reviewer_payload),
        },
        "next_safe_entry": "create_one_exact_release_then_execute_once",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--materialize", action="store_true")
    action.add_argument("--check-only", action="store_true")
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    first = build_review(workspace)
    second = build_review(workspace)
    _require(first == second, "review not deterministic")
    payload = _yaml_bytes(first)
    target = workspace / REVIEW_RELATIVE
    if args.materialize:
        _require(not target.exists(), "review exists")
        with target.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    else:
        _require(target.is_file() and target.read_bytes() == payload, "review differs")
    print(
        "P3-I1 independent review {}: {}".format(
            "materialized" if args.materialize else "verified",
            first["status"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
