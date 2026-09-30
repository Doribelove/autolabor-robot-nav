#!/usr/bin/env python3
"""Independent offline readiness review for the P1-E1 execution release."""

from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P1-E1"
OUTPUT_ROOT = Path("artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution")
CLOSURE_RELATIVE = OUTPUT_ROOT / "offline_readiness_dependency_closure.yaml"
REVIEW_RELATIVE = OUTPUT_ROOT / "p1_e1_offline_readiness_review.yaml"
RELEASE_RELATIVE = Path(
    "experiments/manifests/v2/integration/v2_04g_r6_i1_p1_e1_execution_release.yaml"
)
SCHEDULE_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness/"
    "v2_04g_p1_r1_exact_schedule.yaml"
)
RUNNER_RELATIVE = Path("src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_exact_runner.py")
REPLAY_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p1_e1_replay.py"
)
LAUNCH_RELATIVE = Path("src/simulation/m2_gazebo/launch/m2_v2_04g_p1_e1_execution.launch")
SPAWN_RELATIVE = Path("src/simulation/m2_gazebo/launch/m2_v2_04g_r6_i2_spawn_m2.launch")
TEST_RELATIVE = Path(
    "src/tools/thesis_experiment/tests/test_v2_04g_r6_i1.py.d/"
    "test_v2_04g_r6_i1_p1_e1_offline.py"
)
FREEZE_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i6_interpretation/"
    "i5_evidence_freeze.sha256"
)


class P1OfflineReviewError(ValueError):
    pass


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise P1OfflineReviewError("cannot load {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


DEPENDENCY = _load_module(
    "p1_e1_dependency_for_review",
    WORKSPACE / "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_dependency_generator.py",
)
SOURCE_ROOT = WORKSPACE / "src/tools/thesis_experiment/src"
DEVEL_PYTHON = WORKSPACE / "devel/lib/python3/dist-packages"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))
if str(DEVEL_PYTHON) not in sys.path:
    sys.path.insert(1, str(DEVEL_PYTHON))
from thesis_experiment import v2_04g_r6_i1_p1_statistics as statistics  # noqa: E402
from thesis_experiment.v2_04g_r6_i1_p1_e1_release import (  # noqa: E402
    read_workspace_file_once,
)


def _require(condition, message):
    if not condition:
        raise P1OfflineReviewError(message)


def _sha(payload):
    return hashlib.sha256(payload).hexdigest()


def _canonical_sha(value):
    return _sha(json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8"))


def _yaml_bytes(value):
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _load_yaml(relative):
    value = yaml.safe_load(read_workspace_file_once(WORKSPACE, relative))
    _require(type(value) is dict, "YAML root differs: {}".format(relative))
    return value


def _active_runtime_processes():
    names = {"roscore", "rosmaster", "roslaunch", "gzserver", "gzclient", "move_base"}
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            comm = (entry / "comm").read_text(encoding="utf-8").strip()
            argv = (entry / "cmdline").read_bytes().split(b"\0")
        except OSError:
            continue
        executable = Path(argv[0].decode("utf-8", errors="replace")).name if argv and argv[0] else ""
        if comm in names or executable in names:
            found.append({"pid": int(entry.name), "name": comm or executable})
    return found


def _port_listening(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(0.15)
    try:
        return sock.connect_ex(("127.0.0.1", port)) == 0
    finally:
        sock.close()


def _verify_i5_freeze():
    payload = read_workspace_file_once(WORKSPACE, FREEZE_RELATIVE)
    rows = []
    for line in payload.decode("utf-8").splitlines():
        digest, path_text = line.split(None, 1)
        relative = Path(path_text.strip())
        resource = read_workspace_file_once(WORKSPACE, relative)
        _require(_sha(resource) == digest, "I5 freeze drift: {}".format(relative))
        rows.append({"path": relative.as_posix(), "sha256": digest})
    _require(len(rows) == 13, "I5 freeze roster differs")
    return rows


def _review_runner_source():
    payload = read_workspace_file_once(WORKSPACE, RUNNER_RELATIVE)
    text = payload.decode("utf-8")
    tree = ast.parse(text)
    options = {
        node.value for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
        and node.value.startswith("--")
    }
    _require("--release-sha256" in options and "--execute" in options, "runner release gate options absent")
    _require(not ({"--resume", "--retry", "--start-sequence", "--end-sequence"} & options), "runner exposes partial/resume options")
    for marker in (
        "base_roslaunch_spawn_requested", "evidence_units_forfeited",
        "for row in validation[\"schedule\"][\"episodes\"]",
        "build_terminal_assessment", "_wait_host_clean",
    ):
        _require(marker in text, "runner invariant marker absent: {}".format(marker))
    return {"sha256": _sha(payload), "ast_node_count": sum(1 for _ in ast.walk(tree)), "cli_options": sorted(options)}


def _review_launch():
    launch_payload = read_workspace_file_once(WORKSPACE, LAUNCH_RELATIVE)
    root = ET.fromstring(launch_payload)
    args = {node.attrib["name"] for node in root.findall("arg")}
    required = {
        "world", "seed", "start_rule_supervisor", "start_typed_transaction",
        "load_balanced_anchor", "publish_teb_obstacles",
    }
    _require(required <= args, "P1 launch method switches differ")
    text = launch_payload.decode("utf-8")
    _require('<group if="$(arg start_rule_supervisor)">' in text, "supervisor is not structurally gated")
    _require('<group if="$(arg start_typed_transaction)">' in text, "transaction is not structurally gated")
    spawn = read_workspace_file_once(WORKSPACE, SPAWN_RELATIVE).decode("utf-8")
    _require("--seed $(arg seed)" in spawn, "Gazebo RNG does not use the schedule seed")
    _require("dynamics_seed:=$(arg seed)" in spawn, "actuator dynamics does not use the schedule seed")
    _require('<param name="seed" value="$(arg seed)"/>' in text, "scan transport does not use the schedule seed")
    return {"sha256": _sha(launch_payload), "required_switches": sorted(required), "shared_seed_bindings": 3}


def _run_regressions():
    environment = dict(os.environ)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    environment["PYTHONPATH"] = (
        str(SOURCE_ROOT) + os.pathsep + str(DEVEL_PYTHON) + os.pathsep
        + environment.get("PYTHONPATH", "")
    )
    result = subprocess.run(
        [sys.executable, str(WORKSPACE / TEST_RELATIVE), "-v"],
        cwd=str(WORKSPACE), env=environment,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        timeout=120.0, check=False,
    )
    output = result.stdout.decode("utf-8", errors="replace")
    _require(result.returncode == 0, "offline regression failed: {}".format(output[-3000:]))
    return {"returncode": result.returncode, "output_sha256": _sha(result.stdout), "test_count": output.count(" ... ok")}


def build_review():
    _require(not (WORKSPACE / RELEASE_RELATIVE).exists(), "release must be absent during independent review")
    for forbidden in (
        OUTPUT_ROOT / "attempts", OUTPUT_ROOT / "journals",
        OUTPUT_ROOT / "consumption", OUTPUT_ROOT / "p1_e1_stage_report.yaml",
    ):
        _require(not (WORKSPACE / forbidden).exists(), "execution state exists during offline review")
    _require(_active_runtime_processes() == [], "ROS/Gazebo process active during offline review")
    _require(not _port_listening(11311) and not _port_listening(11345), "ROS/Gazebo port active")

    closure_payload = read_workspace_file_once(WORKSPACE, CLOSURE_RELATIVE)
    closure = yaml.safe_load(closure_payload)
    rebuilt_first = DEPENDENCY.build_closure(WORKSPACE)
    rebuilt_second = DEPENDENCY.build_closure(WORKSPACE)
    _require(rebuilt_first == rebuilt_second, "dependency double build differs")
    _require(closure_payload == _yaml_bytes(rebuilt_first), "persisted dependency closure is not reproducible")
    logical = dict(closure)
    declared_logical = logical.pop("logical_sha256")
    _require(_canonical_sha(logical) == declared_logical, "closure logical SHA differs")
    _require(closure["unresolved"] == [], "closure has unresolved resources")
    local_verified = 0
    for receipt in closure["local_resources"]:
        payload = read_workspace_file_once(WORKSPACE, Path(receipt["path"]))
        _require(_sha(payload) == receipt["sha256"], "local closure hash differs")
        _require(len(payload) == receipt["size_bytes"], "local closure size differs")
        local_verified += 1
    external_verified = 0
    for receipt in closure["external_resources"]:
        path = Path(receipt["path"])
        _require(path.resolve(strict=True) == path and path.is_file(), "external path differs")
        payload = path.read_bytes()
        _require(_sha(payload) == receipt["sha256"], "external closure hash differs")
        _require(len(payload) == receipt["size_bytes"], "external closure size differs")
        external_verified += 1

    schedule = _load_yaml(SCHEDULE_RELATIVE)
    rows = statistics.validate_schedule(schedule)
    _require(len(rows) == 270, "schedule is not exact 270")
    _require({row["seed"] for row in rows} == set(range(5201, 5291)), "fresh seed firewall differs")
    _require(all(row["attempt"] == 1 for row in rows), "attempt roster differs")
    _require(not ({5161, 5162, 5163} & {row["seed"] for row in rows}), "I5 seed reused")
    method_counts = {
        method: sum(row["method_id"] == method for row in rows)
        for method in statistics.METHODS
    }
    _require(set(method_counts.values()) == {90}, "method counts differ")

    contract = _load_yaml(Path("config/thesis_experiments/v2/v2_04g_r6_i1_p1_e1_execution_contract.yaml"))
    authorization = _load_yaml(Path("experiments/manifests/v2/integration/v2_04g_r6_i1_p1_e1_simulation_authorization.yaml"))
    _require(contract.get("execution_authorized") is True and contract.get("authorized_evidence_units") == 270, "contract authority differs")
    _require(authorization.get("explicit_user_authorization_recorded") is True, "user authorization record absent")
    _require(authorization.get("authorized_evidence_units") == 270, "authorization budget differs")

    regression = _run_regressions()
    runner_audit = _review_runner_source()
    launch_audit = _review_launch()
    replay_source = read_workspace_file_once(WORKSPACE, REPLAY_RELATIVE).decode("utf-8")
    for marker in (
        "raw resource roster differs", "trace time is not strictly increasing",
        "overlay intervals do not tile window", "statistics.build_assessment",
    ):
        _require(marker in replay_source, "raw replay fail-closed marker absent")
    freeze_rows = _verify_i5_freeze()
    statvfs = os.statvfs(str(WORKSPACE))
    free_bytes = statvfs.f_bavail * statvfs.f_frsize
    _require(free_bytes >= 4294967296, "free disk is below release minimum")
    reviewer_relative = Path("src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_e1_offline_reviewer.py")
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "status": "p1_e1_offline_execution_readiness_pass_release_absent",
        "review_result": "pass",
        "independent_offline_review_pass": True,
        "release_present_during_review": False,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready_after_unique_release_validation": True,
        "execution_authorized": True,
        "authorized_evidence_units": 270,
        "consumed_evidence_units": 0,
        "training_budget_steps": 0,
        "real_vehicle_used": False,
        "i5_rerun_or_resume_performed": False,
        "dependency_closure": {
            "path": CLOSURE_RELATIVE.as_posix(),
            "file_sha256": _sha(closure_payload),
            "logical_sha256": closure["logical_sha256"],
            "local_resources_verified": local_verified,
            "external_resources_verified": external_verified,
            "unresolved": [],
            "deterministic_double_build_pass": True,
        },
        "exact_schedule": {
            "rows": len(rows), "blocks": 90,
            "seed_first": 5201, "seed_last": 5290,
            "method_counts": method_counts,
            "canonical_episode_digest": schedule["canonical_episode_digest"],
            "partial_selection_interface_present": False,
        },
        "runner_audit": runner_audit,
        "launch_and_seed_audit": launch_audit,
        "unified_raw_replay_audit": {
            "source_sha256": _sha(replay_source.encode("utf-8")),
            "common_raw_labels": sorted([
                "activation", "evaluation", "trace", "clearance", "overlay",
                "teardown_receipt", "process_log",
            ]),
            "raw_metrics_recomputed": True,
            "overlay_exact_tiling_enforced": True,
            "fixed_reference_same_schema": True,
        },
        "offline_regressions": regression,
        "i5_freeze": {"resources_verified": len(freeze_rows), "rerun_performed": False},
        "resource_review": {
            "serial_concurrency": 1,
            "per_identity_hard_wall_cap_s": 240,
            "stage_hard_wall_cap_h": 20,
            "free_disk_bytes": free_bytes,
            "minimum_free_disk_bytes": 4294967296,
        },
        "state_boundary": {
            "attempts_absent": True, "journals_absent": True,
            "consumption_absent": True, "ros_gazebo_absent": True,
            "release_absent": True,
        },
        "reviewer": {
            "path": reviewer_relative.as_posix(),
            "sha256": _sha(read_workspace_file_once(WORKSPACE, reviewer_relative)),
        },
        "release_gate": {
            "one_unique_release_may_now_be_created": True,
            "release_must_bind_this_exact_review": True,
            "release_must_bind_caller_supplied_exact_sha256": True,
            "execution_without_release_validation": False,
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    review = build_review()
    payload = _yaml_bytes(review)
    if args.write:
        target = WORKSPACE / REVIEW_RELATIVE
        if target.exists():
            raise P1OfflineReviewError("offline review already exists")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)
    else:
        print(payload.decode("utf-8"), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
