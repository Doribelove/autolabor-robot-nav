#!/usr/bin/env python3
"""Independent release-absent review for P2-C1-R1 execution readiness."""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import os
from pathlib import Path
import socket
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P2-C1-R1"
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_r1_release_readiness"
)
REVIEW_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_c1_r1_offline_review.yaml"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_c1_r1_exact_calibration_execution_contract.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p2_c1_r1_simulation_authorization.yaml"
)
RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p2_c1_r1_execution_release.yaml"
)
EXECUTION_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_r1_execution"
)
OLD_EXECUTION_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_execution"
)
BUILDER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_release_builder.py"
)
RUNNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_exact_runner.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_r1_offline_reviewer.py"
)
METHODS_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_offline_readiness/"
    "v2_04g_p2_q1_method_bindings.yaml"
)


class C1R1OfflineReviewError(ValueError):
    """Raised when an R1 release-readiness invariant fails."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise C1R1OfflineReviewError(message)


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    _require(type(value) is dict, "YAML root differs: {}".format(path))
    return value


def _load_module(name: str, relative: Path):
    path = WORKSPACE / relative
    spec = importlib.util.spec_from_file_location(name, path)
    _require(spec is not None and spec.loader is not None, "cannot load {}".format(relative))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BUILDER = _load_module("_p2_c1_r1_builder_for_review", BUILDER_RELATIVE)
RUNNER = _load_module("_p2_c1_r1_runner_for_review", RUNNER_RELATIVE)


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


def _normalize_command(argv):
    prefix = str(WORKSPACE) + "/"
    return [str(item).replace(prefix, "") for item in argv]


def _review_schedule_and_scenes(
    old_schedule: Mapping[str, Any],
    schedule: Mapping[str, Any],
    plan: Mapping[str, Any],
) -> Dict[str, Any]:
    expected = BUILDER.EXPECTED_TIMEOUT_REPAIRS
    _require(len(old_schedule["rows"]) == len(schedule["rows"]) == 342, "row count differs")
    _require(len(plan["rows"]) == 342, "plan row count differs")
    observed = {}
    for old_row, row, plan_row in zip(
        old_schedule["rows"], schedule["rows"], plan["rows"]
    ):
        reconstructed = copy.deepcopy(row)
        reconstructed["stage"] = "V2-04G-P2-C1"
        if row["sequence"] in expected:
            old_timeout, repaired_timeout = expected[row["sequence"]]
            _require(float(row["timeout_s"]) == repaired_timeout, "repair value differs")
            reconstructed["timeout_s"] = old_timeout
            observed[row["sequence"]] = [old_timeout, repaired_timeout]
        _require(reconstructed == old_row, "non-amendment schedule field differs")
        instance_payload = (WORKSPACE / row["instance"]["path"]).read_bytes()
        world_payload = (WORKSPACE / row["world"]["path"]).read_bytes()
        _require(
            _sha(instance_payload) == row["instance"]["sha256"]
            and len(instance_payload) == row["instance"]["size_bytes"],
            "instance receipt differs",
        )
        _require(
            _sha(world_payload) == row["world"]["sha256"]
            and len(world_payload) == row["world"]["size_bytes"],
            "world receipt differs",
        )
        scene = yaml.safe_load(instance_payload)["scene"]
        _require(
            scene["scene_id"] == row["scene_id"]
            and scene["seed"] == row["seed"]
            and scene["split"] == "calibration"
            and float(scene["timeout_s"]) == float(row["timeout_s"]),
            "compiled scene binding differs",
        )
        _require(
            plan_row["identity"] == RUNNER.identity(row)
            and plan_row["scene_execution_binding"]["start"] == scene["start"]
            and float(plan_row["scene_execution_binding"]["timeout_s"])
            == float(scene["timeout_s"]),
            "plan identity or scene binding differs",
        )
    _require(
        {key: tuple(value) for key, value in observed.items()} == expected,
        "observed timeout repair set differs",
    )
    return {
        "rows_verified": 342,
        "timeout_repaired_sequences": list(expected),
        "unaffected_rows_byte_equivalent_after_stage_projection": 334,
        "all_rows_match_compiled_scene_identity_and_timeout": True,
    }


def _review_runtime_commands(
    schedule: Mapping[str, Any],
    plan: Mapping[str, Any],
    methods: Mapping[str, Any],
    closure: Mapping[str, Any],
) -> Dict[str, Any]:
    closure_map = {
        receipt["path"]: receipt["sha256"]
        for receipt in closure["local_resources"]
    }
    RUNNER.configure_static_bindings(methods, closure_map)
    executables = {
        "roslaunch": "{ROSLAUNCH_EXECUTABLE}",
        "xacro": "{XACRO_EXECUTABLE}",
        "python": "{PYTHON_EXECUTABLE}",
    }
    semantic_count = 0
    direct_count = 0
    for row, planned in zip(schedule["rows"], plan["rows"]):
        instance = _load_yaml(WORKSPACE / row["instance"]["path"])
        runtime = RUNNER.runtime(row, closure_map)
        actual_base = RUNNER.base_command(
            row,
            instance["scene"],
            "{FUTURE_SCENE_SNAPSHOT_WORLD}",
            runtime,
            executables,
        )
        _require(
            _normalize_command(actual_base) == planned["base_launch_argv"],
            "resolved base command differs at sequence {}".format(row["sequence"]),
        )
        if RUNNER.semantic(row):
            semantic_count += 1
            actual_transaction = RUNNER.transaction_command(
                row, runtime, executables
            )
            _require(
                _normalize_command(actual_transaction)
                == planned["semantic_transaction_launch_argv"],
                "transaction command differs at sequence {}".format(row["sequence"]),
            )
        else:
            _require(
                planned["semantic_transaction_launch_argv"] is None,
                "unexpected transaction command",
            )
        if planned["bootstrap_mode"] == "direct_probe_positive_clock_only":
            direct_count += 1
        if row["row_type"] == "calibration_navigation":
            actual_episode = RUNNER.episode_command(
                row,
                Path("{FUTURE_SCENE_SNAPSHOT_INSTANCE}"),
                row["instance"]["sha256"],
                Path("{FUTURE_ATTEMPT_ROOT}/episode"),
                executables,
            )
            _require(
                _normalize_command(actual_episode) == planned["episode_argv"],
                "episode command differs at sequence {}".format(row["sequence"]),
            )
        else:
            actual_probe = RUNNER.probe_command(
                row,
                Path("{FUTURE_ATTEMPT_ROOT}/raw_trace.yaml"),
                executables,
            )
            _require(
                _normalize_command(actual_probe) == planned["probe_argv"],
                "probe command differs at sequence {}".format(row["sequence"]),
            )
    return {
        "resolved_rows_compared_to_runner": 342,
        "semantic_transaction_rows": semantic_count,
        "direct_positive_clock_rows": direct_count,
        "start_pose_world_fixed_config_and_runtime_resources_bound": True,
    }


def _review_closure(closure: Mapping[str, Any]) -> Dict[str, Any]:
    _require(closure["stage"] == STAGE and closure["unresolved"] == [], "closure differs")
    _require(
        closure["local_resource_count"] == len(closure["local_resources"])
        and closure["external_resource_count"] == len(closure["external_resources"]),
        "closure counts differ",
    )
    for receipt in closure["local_resources"]:
        path = WORKSPACE / receipt["path"]
        _require(path.is_file(), "local dependency absent")
        payload = path.read_bytes()
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "local dependency drifted: {}".format(receipt["path"]),
        )
    for receipt in closure["external_resources"]:
        path = Path(receipt["path"])
        _require(path.is_absolute() and path.is_file(), "external dependency absent")
        payload = path.read_bytes()
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "external dependency drifted",
        )
    return {
        "local_resources_verified": closure["local_resource_count"],
        "external_resources_verified": closure["external_resource_count"],
        "unresolved": [],
    }


def build_review(workspace: Path = WORKSPACE) -> Dict[str, Any]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    _require(not (workspace / RELEASE_RELATIVE).exists(), "R1 release exists during review")
    _require(not (workspace / EXECUTION_ROOT).exists(), "R1 execution state exists")
    _require(not (workspace / OLD_EXECUTION_ROOT).exists(), "old C1 execution state exists")
    contract = _load_yaml(workspace / CONTRACT_RELATIVE)
    authorization = _load_yaml(workspace / AUTHORIZATION_RELATIVE)
    _require(contract["stage"] == STAGE, "contract stage differs")
    _require(contract["execution_authorized"] is True, "contract authority differs")
    _require(contract["authorized_evidence_units_max"] == 342, "contract budget differs")
    _require(
        contract["global_budget_accounting"]["additive_budget_expansion"] is False,
        "repair adds budget",
    )
    _require(
        authorization["explicit_user_authorization_recorded"] is True
        and authorization["execution_authorized"] is True,
        "explicit authorization absent",
    )
    _require(
        authorization["authorized_evidence_units_max"] == 342,
        "authorization budget differs",
    )
    expected_files = BUILDER.build_bundle(workspace)
    for relative, expected in expected_files.items():
        path = workspace / relative
        _require(path.is_file(), "readiness resource absent: {}".format(relative))
        _require(path.read_bytes() == expected, "readiness bytes differ: {}".format(relative))
    schedule = _load_yaml(workspace / BUILDER.SCHEDULE_RELATIVE)
    plan = _load_yaml(workspace / BUILDER.PLAN_RELATIVE)
    closure = _load_yaml(workspace / BUILDER.CLOSURE_RELATIVE)
    old_schedule = _load_yaml(workspace / BUILDER.OLD_SCHEDULE_RELATIVE)
    methods = _load_yaml(workspace / METHODS_RELATIVE)
    schedule_review = _review_schedule_and_scenes(old_schedule, schedule, plan)
    command_review = _review_runtime_commands(schedule, plan, methods, closure)
    closure_review = _review_closure(closure)
    _require(
        all(row["row_type"] == "qualification_probe" for row in schedule["rows"][:12])
        and all(
            row["row_type"] == "calibration_navigation"
            for row in schedule["rows"][12:]
        ),
        "qualification-first order differs",
    )
    _require(
        not set(range(5401, 5491)) & {row["seed"] for row in schedule["rows"]},
        "held-out seed leaked",
    )
    _require(schedule["behavioral_factor_count_changed"] == 0, "behavioral factor changed")
    old_audit = _load_yaml(workspace / BUILDER.OLD_AUDIT_RELATIVE)
    _require(
        old_audit["execute_called"] is False
        and old_audit["consumed_evidence_units"] == 0
        and old_audit["forfeited_authority_units"] == 342,
        "old terminal audit differs",
    )
    processes = _active_runtime_processes()
    _require(processes == [], "ROS/Gazebo/move_base process active")
    _require(
        not _listening(11311) and not _listening(11345),
        "ROS master port active",
    )
    reviewer_payload = (workspace / REVIEWER_RELATIVE).read_bytes()
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "p2_c1_r1_independent_execution_readiness_pass_release_absent",
        "review_result": "pass",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": True,
        "execution_authorized": True,
        "authorized_evidence_units_max": 342,
        "consumed_evidence_units": 0,
        "release_present_during_review": False,
        "predecessor_terminal_freeze": {
            "old_release_preserved": True,
            "old_execute_called": False,
            "old_execution_root_absent": True,
            "old_consumed_units": 0,
            "old_authority_forfeited": 342,
        },
        "single_integrity_amendment_review": {
            "classification": (
                "qualification_compiled_scene_and_startup_binding_only"
            ),
            "behavioral_factor_count_changed": 0,
            "direct_probe_startup_config_rows": list(
                BUILDER.EXPECTED_TIMEOUT_REPAIRS
            ),
            **schedule_review,
        },
        "runtime_command_review": command_review,
        "dependency_closure": closure_review,
        "budget_review": {
            "round_1_maximum_units": 792,
            "R1_calibration_maximum_units": 342,
            "held_out_reserved_maximum_units": 450,
            "additive_budget_expansion": False,
        },
        "side_effect_audit": {
            "R1_execution_root_absent": True,
            "release_absent": True,
            "forbidden_processes": processes,
            "ros_gazebo_started_by_review": False,
            "held_out_test_accessed": False,
        },
        "reviewer": {
            "path": REVIEWER_RELATIVE.as_posix(),
            "sha256": _sha(reviewer_payload),
            "deterministic_double_review_required": True,
        },
        "next_safe_entry": "create_one_exact_R1_release_bind_its_SHA_then_execute_once",
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
    _require(first == second, "R1 review is not deterministic")
    payload = _yaml_bytes(first)
    target = workspace / REVIEW_RELATIVE
    if args.materialize:
        _require(not target.exists(), "R1 review already exists")
        with target.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    else:
        _require(target.is_file(), "R1 review absent")
        _require(target.read_bytes() == payload, "R1 review bytes differ")
    print(
        "P2-C1-R1 independent review {}: {}".format(
            "materialized" if args.materialize else "verified",
            first["status"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
