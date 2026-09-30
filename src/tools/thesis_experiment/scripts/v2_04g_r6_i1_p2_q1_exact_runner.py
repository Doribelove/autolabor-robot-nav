#!/usr/bin/env python3
"""Fail-closed preflight for the P2-Q1 exact 342-row command plan.

Q1 is an offline readiness stage.  ``--preflight`` rehashes and rebuilds every
persisted input without importing ROS.  ``--execute`` is intentionally and
unconditionally disabled; a later versioned release must provide a separately
reviewed executable runner and corresponding simulation authorization.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import re
import socket
import sys
from typing import Any, Dict, Mapping

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_offline_readiness"
)
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_q1_offline_implementation_readiness_contract.yaml"
)
METHOD_BINDINGS_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_q1_method_bindings.yaml"
SCHEDULE_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_q1_exact_342_schedule.yaml"
COMMAND_PLAN_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_q1_exact_342_command_plan.yaml"
SOURCE_FREEZE_RELATIVE = OUTPUT_ROOT / "p2_q1_source_freeze.sha256"
BUILDER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_q1_offline_builder.py"
)
HEX_SHA = re.compile(r"^[0-9a-f]{64}$")


class Q1RunnerError(RuntimeError):
    """Raised when Q1 preflight or its zero-authority boundary fails."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise Q1RunnerError(message)


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    _require(type(value) is dict, "YAML root is not a mapping: {}".format(path))
    return value


def _load_builder():
    path = WORKSPACE / BUILDER_RELATIVE
    spec = importlib.util.spec_from_file_location("p2_q1_runner_rebuild", path)
    _require(spec is not None and spec.loader is not None, "cannot load Q1 builder")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _active_runtime_processes():
    names = {
        b"roscore",
        b"rosmaster",
        b"roslaunch",
        b"gzserver",
        b"gzclient",
        b"move_base",
        b"spawn_model",
    }
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ")
        except OSError:
            continue
        lowered = command.lower()
        if any(name in lowered for name in names):
            found.append({"pid": int(entry.name), "command_sha256": _sha(command)})
    return found


def _listening(port: int) -> bool:
    connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connection.settimeout(0.1)
    try:
        return connection.connect_ex(("127.0.0.1", port)) == 0
    finally:
        connection.close()


def _verify_source_freeze(path: Path) -> int:
    count = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        _require(match is not None, "source freeze line differs")
        digest, path_text = match.groups()
        _require(HEX_SHA.fullmatch(digest) is not None, "source freeze SHA differs")
        relative = Path(path_text)
        _require(
            not relative.is_absolute() and ".." not in relative.parts,
            "source freeze path is unsafe",
        )
        target = WORKSPACE / relative
        _require(target.is_file() and not target.is_symlink(), "source freeze target absent")
        _require(_sha(target.read_bytes()) == digest, "source freeze target drifted")
        count += 1
    _require(count >= 20, "source freeze is incomplete")
    return count


def _verify_receipt(receipt: Mapping[str, Any]) -> None:
    _require(
        type(receipt) is dict
        and set(receipt) == {"path", "sha256", "size_bytes"},
        "resource receipt schema differs",
    )
    relative = Path(receipt["path"])
    _require(
        not relative.is_absolute() and ".." not in relative.parts,
        "resource receipt path is unsafe",
    )
    path = WORKSPACE / relative
    _require(path.is_file() and not path.is_symlink(), "resource receipt target absent")
    payload = path.read_bytes()
    _require(_sha(payload) == receipt["sha256"], "resource receipt SHA differs")
    _require(len(payload) == receipt["size_bytes"], "resource receipt size differs")


def preflight() -> Dict[str, Any]:
    contract = _load_yaml(WORKSPACE / CONTRACT_RELATIVE)
    _require(contract.get("stage") == "V2-04G-P2-Q1", "contract stage differs")
    _require(contract.get("offline_only") is True, "contract is not offline-only")
    _require(contract.get("execution_authorized") is False, "contract authorizes execution")
    _require(contract.get("evidence_budget_authorized") == 0, "contract budget differs")

    builder = _load_builder()
    expected = builder.build_bundle(WORKSPACE)
    for relative, payload in expected.items():
        target = WORKSPACE / relative
        _require(target.is_file(), "offline bundle resource absent: {}".format(relative))
        _require(
            target.read_bytes() == payload,
            "offline bundle resource drifted: {}".format(relative),
        )

    methods = _load_yaml(WORKSPACE / METHOD_BINDINGS_RELATIVE)
    schedule = _load_yaml(WORKSPACE / SCHEDULE_RELATIVE)
    plan = _load_yaml(WORKSPACE / COMMAND_PLAN_RELATIVE)
    _require(len(methods["methods"]) == 11, "method count differs")
    _require(len(schedule["rows"]) == 342, "schedule count differs")
    _require(len(plan["rows"]) == 342, "command plan count differs")
    _require(
        [row["sequence"] for row in schedule["rows"]] == list(range(1, 343)),
        "schedule sequence differs",
    )
    _require(
        [row["sequence"] for row in plan["rows"]] == list(range(1, 343)),
        "command plan sequence differs",
    )
    _require(schedule["held_out_test_row_count"] == 0, "held-out test row exists")
    _require(
        not set(range(5401, 5491)) & {row["seed"] for row in schedule["rows"]},
        "held-out test seed leaked",
    )
    for method in methods["methods"]:
        if "materialized_config" in method:
            _verify_receipt(method["materialized_config"])
        if "materialized_anchor_bank" in method:
            _verify_receipt(method["materialized_anchor_bank"])
            _verify_receipt(method["fixed_startup_config"])
    for row in schedule["rows"]:
        _verify_receipt(row["instance"])
        _verify_receipt(row["world"])

    source_count = _verify_source_freeze(WORKSPACE / SOURCE_FREEZE_RELATIVE)
    forbidden = [
        Path(
            "experiments/manifests/v2/integration/"
            "v2_04g_r6_i1_p2_q1_execution_release.yaml"
        ),
        Path(
            "experiments/manifests/v2/integration/"
            "v2_04g_r6_i1_p2_q1_simulation_authorization.yaml"
        ),
        Path(
            "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_execution"
        ),
    ]
    _require(
        not any((WORKSPACE / path).exists() for path in forbidden),
        "forbidden Q1 execution state exists",
    )
    processes = _active_runtime_processes()
    _require(processes == [], "ROS/Gazebo/move_base process is active")
    _require(
        not _listening(11311) and not _listening(11345),
        "ROS master port is active",
    )
    return {
        "stage": "V2-04G-P2-Q1",
        "status": "offline_preflight_pass_execute_hard_disabled",
        "methods": 11,
        "scenes": 48,
        "schedule_rows": 342,
        "command_plan_rows": 342,
        "source_freeze_resources": source_count,
        "execution_authorized": False,
        "authorized_evidence_units": 0,
        "held_out_test_rows": 0,
        "forbidden_runtime_processes": processes,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--preflight", action="store_true")
    action.add_argument("--execute", action="store_true")
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    args = parser.parse_args()
    _require(args.workspace.resolve() == WORKSPACE.resolve(), "workspace identity differs")
    report = preflight()
    if args.execute:
        raise Q1RunnerError(
            "P2-Q1 execute is hard-disabled: exact execution release, closed "
            "runtime dependency roster, nonzero budget, and corresponding "
            "simulation authorization are absent"
        )
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Q1RunnerError as exc:
        print("P2-Q1 runner refused: {}".format(exc), file=sys.stderr)
        raise SystemExit(2)
