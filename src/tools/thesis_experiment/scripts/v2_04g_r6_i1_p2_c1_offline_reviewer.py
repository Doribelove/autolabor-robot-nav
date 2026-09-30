#!/usr/bin/env python3
"""Independent release-absent review for P2-C1 execution readiness."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import re
import socket
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P2-C1"
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_release_readiness"
)
REVIEW_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_c1_offline_review.yaml"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_c1_exact_calibration_execution_contract.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p2_c1_simulation_authorization.yaml"
)
RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i1_p2_c1_execution_release.yaml"
)
EXECUTION_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_c1_execution"
)
BUILDER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_release_builder.py"
)
RUNNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_exact_runner.py"
)
ASSESSOR_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_assess.py"
)
REPLAY_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p2_q1_replay.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_c1_offline_reviewer.py"
)


class C1OfflineReviewError(ValueError):
    """Raised when a C1 release-readiness invariant fails."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise C1OfflineReviewError(message)


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _load_module(name: str, relative: Path):
    path = WORKSPACE / relative
    spec = importlib.util.spec_from_file_location(name, path)
    _require(spec is not None and spec.loader is not None, "cannot load {}".format(relative))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BUILDER = _load_module("_p2_c1_builder_for_review", BUILDER_RELATIVE)
ASSESSOR = _load_module("_p2_c1_assessor_for_review", ASSESSOR_RELATIVE)
REPLAY = _load_module("_p2_c1_replay_for_review", REPLAY_RELATIVE)


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    _require(type(value) is dict, "YAML root differs: {}".format(path))
    return value


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


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


def _listening(port):
    connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connection.settimeout(0.1)
    try:
        return connection.connect_ex(("127.0.0.1", port)) == 0
    finally:
        connection.close()


def _qualification_sample(
    stamp,
    x,
    speed,
    yaw=0.0,
    overlay="NONE",
    trigger=False,
    transaction=True,
):
    return {
        "stamp_s": stamp,
        "x_m": x,
        "y_m": 0.0,
        "linear_velocity_mps": speed,
        "yaw_rate_radps": yaw,
        "clearance_m": 0.5,
        "collision": False,
        "dynamic_overlay": overlay,
        "conflict_triggered": trigger,
        "emergency": False,
        "projection_applied": False,
        "transaction_valid": transaction,
    }


def _synthetic_fixtures(thresholds):
    def plan(probe):
        return {
            "sequence": 1,
            "row_type": "qualification_probe",
            "probe_type": probe,
            "candidate_speed_mps": 1.3,
            "scene_id": "fixture",
            "seed": 5334,
            "attempt": 1,
        }

    braking = [
        _qualification_sample(0.0, 0.0, 1.3),
        _qualification_sample(0.5, 0.5, 0.7),
        _qualification_sample(1.0, 0.8, 0.0),
    ]
    overlay = [
        _qualification_sample(0.0, 0.0, 1.3),
        _qualification_sample(0.5, 0.6, 1.28, trigger=True),
        _qualification_sample(
            0.9, 1.0, 1.2, overlay="CROSSING", trigger=True
        ),
    ]
    circle = [
        _qualification_sample(
            index * 0.1, index * 0.1, 1.3, yaw=1.3 / 1.2
        )
        for index in range(8)
    ]
    _require(
        REPLAY.assess_qualification_trace(plan("braking"), braking, thresholds)["pass"],
        "braking fixture failed",
    )
    _require(
        REPLAY.assess_qualification_trace(
            plan("high_speed_to_conflict_overlay_transition"),
            overlay,
            thresholds,
        )["pass"],
        "overlay fixture failed",
    )
    _require(
        REPLAY.assess_qualification_trace(
            plan("ackermann_circle"), circle, thresholds
        )["pass"],
        "circle fixture failed",
    )
    invalid = [dict(row) for row in overlay]
    invalid[-1]["transaction_valid"] = False
    _require(
        not REPLAY.assess_qualification_trace(
            plan("high_speed_to_conflict_overlay_transition"),
            invalid,
            thresholds,
        )["pass"],
        "invalid transaction fixture did not fail",
    )
    return {
        "braking_positive": True,
        "overlay_positive": True,
        "circle_positive": True,
        "invalid_transaction_negative": True,
    }


def build_review(workspace: Path = WORKSPACE) -> Dict[str, Any]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    _require(not (workspace / RELEASE_RELATIVE).exists(), "C1 release exists during review")
    _require(not (workspace / EXECUTION_ROOT).exists(), "C1 execution state exists")
    contract = _load_yaml(workspace / CONTRACT_RELATIVE)
    authorization = _load_yaml(workspace / AUTHORIZATION_RELATIVE)
    _require(contract["stage"] == STAGE, "contract stage differs")
    _require(contract["execution_authorized"] is True, "contract authority differs")
    _require(contract["authorized_evidence_units_max"] == 342, "contract budget differs")
    _require(
        authorization["explicit_user_authorization_recorded"] is True
        and authorization["execution_authorized"] is True,
        "explicit authorization is absent",
    )
    _require(
        authorization["authorized_evidence_units_max"] == 342,
        "authorization budget differs",
    )

    rebuilt = BUILDER.build_bundle(workspace)
    for relative, expected in rebuilt.items():
        path = workspace / relative
        _require(path.is_file(), "release-readiness resource absent")
        _require(path.read_bytes() == expected, "release-readiness bytes differ")
    schedule = _load_yaml(workspace / BUILDER.SCHEDULE_RELATIVE)
    plan = _load_yaml(workspace / BUILDER.PLAN_RELATIVE)
    closure = _load_yaml(workspace / BUILDER.CLOSURE_RELATIVE)
    _require(len(schedule["rows"]) == 342, "schedule row count differs")
    _require(len(plan["rows"]) == 342, "command plan row count differs")
    _require(
        all(row["row_type"] == "qualification_probe" for row in schedule["rows"][:12])
        and all(
            row["row_type"] == "calibration_navigation"
            for row in schedule["rows"][12:]
        ),
        "qualification-first order differs",
    )
    _require(
        {row["q1_source_sequence"] for row in schedule["rows"]}
        == set(range(1, 343)),
        "Q1 source rows not preserved",
    )
    _require(
        not set(range(5401, 5491)) & {row["seed"] for row in schedule["rows"]},
        "held-out seed leaked",
    )
    _require(schedule["behavioral_factor_count_changed"] == 0, "behavioral factor changed")
    semantic_rows = [
        row for row in plan["rows"] if row["semantic_transaction_launch_argv"] is not None
    ]
    _require(semantic_rows, "semantic command rows absent")
    for row in semantic_rows:
        base = row["base_launch_argv"]
        _require(
            sum(item.startswith("rule_supervisor_config:=") for item in base) == 1
            and sum(
                item.startswith("rule_supervisor_config_sha256:=") for item in base
            )
            == 1
            and "publish_teb_obstacles:=true" in base,
            "semantic execution binding repair differs",
        )
    direct = [
        row
        for row in plan["rows"][:12]
        if "start_move_base:=false" in row["base_launch_argv"]
    ]
    _require(
        direct
        and all(row["bootstrap_mode"] == "direct_probe_positive_clock_only" for row in direct),
        "direct bootstrap repair differs",
    )
    _require(closure["unresolved"] == [], "dependency closure unresolved")
    _require(
        closure["local_resource_count"] == len(closure["local_resources"])
        and closure["external_resource_count"] == len(closure["external_resources"]),
        "dependency closure counts differ",
    )
    for receipt in closure["local_resources"]:
        path = workspace / receipt["path"]
        _require(path.is_file(), "local dependency absent")
        payload = path.read_bytes()
        _require(
            _sha(payload) == receipt["sha256"]
            and len(payload) == receipt["size_bytes"],
            "local dependency drifted",
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

    q1_contract = _load_yaml(
        workspace
        / "config/thesis_experiments/v2/"
        "v2_04g_r6_i1_p2_q1_offline_implementation_readiness_contract.yaml"
    )
    fixtures = _synthetic_fixtures(q1_contract["qualification_thresholds"])
    runner_text = (workspace / RUNNER_RELATIVE).read_text(encoding="utf-8")
    _require(
        "validate_release(" in runner_text
        and "qualification_speed_disqualified" in runner_text
        and "held_out_test_accessed" in runner_text,
        "runner gate source differs",
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
        "status": "p2_c1_independent_execution_readiness_pass_release_absent",
        "review_result": "pass",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": True,
        "execution_authorized": True,
        "authorized_evidence_units_max": 342,
        "consumed_evidence_units": 0,
        "release_present_during_review": False,
        "schedule_review": {
            "qualification_rows_first": 12,
            "calibration_rows_after_gate": 330,
            "source_q1_rows_preserved": 342,
            "held_out_test_rows": 0,
            "behavioral_factor_count_changed": 0,
        },
        "execution_binding_repairs": {
            "semantic_supervisor_path_and_sha": True,
            "semantic_dynamic_obstacle_bridge": True,
            "direct_positive_clock_bootstrap": True,
            "qualification_first_monotone_gate": True,
        },
        "dependency_closure": {
            "local_resources_verified": closure["local_resource_count"],
            "external_resources_verified": closure["external_resource_count"],
            "unresolved": [],
        },
        "raw_replay_fixtures": fixtures,
        "side_effect_audit": {
            "execution_root_absent": True,
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
        "next_safe_entry": (
            "create_the_one_exact_release_bind_its_SHA_then_execute_once"
        ),
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
    _require(first == second, "C1 review is not deterministic")
    payload = _yaml_bytes(first)
    target = workspace / REVIEW_RELATIVE
    if args.materialize:
        _require(not target.exists(), "C1 review already exists")
        with target.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    else:
        _require(target.is_file(), "C1 review absent")
        _require(target.read_bytes() == payload, "C1 review bytes differ")
    print(
        "P2-C1 independent review {}: {}".format(
            "materialized" if args.materialize else "verified",
            first["status"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
