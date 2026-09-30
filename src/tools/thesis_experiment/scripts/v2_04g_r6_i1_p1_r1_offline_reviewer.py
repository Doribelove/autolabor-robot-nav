#!/usr/bin/env python3
"""Independent fail-closed offline review for V2-04G-P1-R1.

The reviewer has no execute option and imports no ROS code.  It rehashes the
dependency graph with component-wise no-follow reads, rebuilds the full scene
and schedule bundle in memory, and runs deterministic statistical fixtures.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import stat
from typing import Any, Dict, Mapping, Sequence

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness"
)
CLOSURE_RELATIVE = OUTPUT_ROOT / "p1_r1_dependency_closure.yaml"
REVIEW_RELATIVE = OUTPUT_ROOT / "v2_04g_p1_r1_offline_review.yaml"
SCHEDULE_RELATIVE = OUTPUT_ROOT / "v2_04g_p1_r1_exact_schedule.yaml"
MANIFEST_RELATIVE = OUTPUT_ROOT / "v2_04g_p1_r1_scenes.yaml"
INDEX_RELATIVE = OUTPUT_ROOT / "compiled_scenes/compiled_scene_index.yaml"
EMPTY_ASSESSMENT_RELATIVE = OUTPUT_ROOT / "v2_04g_p1_r1_assessment_not_run.yaml"
FREEZE_RELATIVE = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i6_interpretation/"
    "i5_evidence_freeze.sha256"
)


class P1OfflineReviewError(ValueError):
    pass


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise P1OfflineReviewError("cannot load {}".format(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BUILDER = _load_module(
    "_p1_r1_builder_for_review",
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_r1_offline_builder.py",
)
DEPENDENCY = _load_module(
    "_p1_r1_dependency_for_review",
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_r1_dependency_generator.py",
)
STATS = _load_module(
    "_p1_r1_statistics_for_review",
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p1_statistics.py",
)


class _UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise P1OfflineReviewError("duplicate YAML key: {!r}".format(key))
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise P1OfflineReviewError(message)


def _safe_parts(relative: Path):
    path = Path(relative)
    _require(
        not path.is_absolute()
        and bool(path.parts)
        and all(part not in {"", ".", ".."} for part in path.parts),
        "unsafe relative path: {}".format(relative),
    )
    return path.parts


def _read_once_no_follow(workspace: Path, relative: Path) -> bytes:
    """Component-wise openat/O_NOFOLLOW and one-FD stable snapshot."""

    root = workspace.resolve(strict=True)
    parts = _safe_parts(relative)
    directory_flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags |= getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptors = []
    try:
        descriptor = os.open(str(root), directory_flags)
        descriptors.append(descriptor)
        for component in parts[:-1]:
            descriptor = os.open(component, directory_flags, dir_fd=descriptor)
            descriptors.append(descriptor)
        flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
        file_descriptor = os.open(parts[-1], flags, dir_fd=descriptors[-1])
        try:
            before = os.fstat(file_descriptor)
            _require(stat.S_ISREG(before.st_mode), "not a regular file: {}".format(relative))
            _require(before.st_nlink == 1, "hard links forbidden: {}".format(relative))
            chunks = []
            while True:
                chunk = os.read(file_descriptor, 1024 * 1024)
                if not chunk:
                    break
                chunks.append(chunk)
            after = os.fstat(file_descriptor)
            stable = ("st_dev", "st_ino", "st_mode", "st_nlink", "st_size", "st_mtime_ns", "st_ctime_ns")
            _require(
                tuple(getattr(before, key) for key in stable)
                == tuple(getattr(after, key) for key in stable),
                "file changed during read: {}".format(relative),
            )
            payload = b"".join(chunks)
            _require(len(payload) == before.st_size, "short read: {}".format(relative))
            return payload
        finally:
            os.close(file_descriptor)
    except OSError as exc:
        raise P1OfflineReviewError("cannot no-follow read {}: {}".format(relative, exc)) from exc
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def _load_yaml_bytes(payload: bytes, label: str) -> Mapping[str, Any]:
    try:
        value = yaml.load(payload.decode("utf-8"), Loader=_UniqueLoader)
    except (UnicodeError, yaml.YAMLError) as exc:
        raise P1OfflineReviewError("cannot parse {}: {}".format(label, exc)) from exc
    _require(type(value) is dict, "mapping required: {}".format(label))
    return value


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _trace(fraction: float):
    if fraction == 0.0:
        return [{"start_s": 0.0, "end_s": 10.0, "state": "NONE"}]
    return [
        {"start_s": 0.0, "end_s": 10.0 * fraction, "state": "LEGACY_DYNAMIC"},
        {"start_s": 10.0 * fraction, "end_s": 10.0, "state": "NONE"},
    ]


def _synthetic_episodes(schedule: Mapping[str, Any]):
    rows = []
    for planned in schedule["episodes"]:
        clear = planned["scene_role"] == STATS.CLEAR_ROLE
        method = planned["method_id"]
        time_s = 50.0
        overlay = 0.5
        if clear:
            time_s = {
                STATS.CONTROL: 50.0,
                STATS.TREATMENT: 45.0,
                STATS.REFERENCE: 48.0,
            }[method]
            overlay = {
                STATS.CONTROL: 0.6,
                STATS.TREATMENT: 0.3,
                STATS.REFERENCE: 0.4,
            }[method]
        rows.append({
            "sequence": planned["sequence"],
            "method_id": method,
            "scene_id": planned["scene_id"],
            "seed": planned["seed"],
            "attempt": 1,
            "block_id": planned["block_id"],
            "scene_role": planned["scene_role"],
            "consumed": True,
            "terminal_status": "completed",
            "success": True,
            "collision": False,
            "navigation_time_s": time_s,
            "timeout_s": 80.0,
            "minimum_footprint_clearance_m": 0.5,
            "observed_ttc_status": planned["expected_ttc_status"],
            "algorithmic_emergency": False,
            "interface_failure": False,
            "observation_duration_s": 10.0,
            "overlay_intervals": _trace(overlay),
            "raw_resources_complete": True,
            "integrity_failures": [],
        })
    return rows


def _active_forbidden_processes():
    tokens = (b"roscore", b"rosmaster", b"roslaunch", b"gzserver", b"gzclient", b"move_base")
    found = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ")
        except OSError:
            continue
        lowered = command.lower()
        if any(token in lowered for token in tokens):
            found.append({"pid": int(entry.name), "command_sha256": _sha(command)})
    return found


def _verify_freeze(workspace: Path):
    payload = _read_once_no_follow(workspace, FREEZE_RELATIVE)
    rows = []
    for line in payload.decode("utf-8").splitlines():
        digest, path_text = line.split(None, 1)
        relative = Path(path_text.strip())
        resource = _read_once_no_follow(workspace, relative)
        _require(_sha(resource) == digest, "I5 freeze resource drift: {}".format(relative))
        rows.append({"path": relative.as_posix(), "sha256": digest})
    _require(len(rows) == 13, "I5 freeze roster differs")
    return rows


def build_review(workspace: Path = WORKSPACE) -> Dict[str, Any]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    closure_payload = _read_once_no_follow(workspace, CLOSURE_RELATIVE)
    closure = _load_yaml_bytes(closure_payload, str(CLOSURE_RELATIVE))
    rebuilt_closure = DEPENDENCY.build_closure(workspace)
    _require(closure_payload == _yaml_bytes(rebuilt_closure), "dependency closure is not reproducible")
    logical_input = dict(closure)
    logical_digest = logical_input.pop("logical_sha256")
    _require(DEPENDENCY._logical_sha(logical_input) == logical_digest, "closure logical SHA differs")
    _require(closure["unresolved"] == [], "closure has unresolved resources")
    verified = 0
    for receipt in closure["local_resources"]:
        relative = Path(receipt["path"])
        payload = _read_once_no_follow(workspace, relative)
        _require(_sha(payload) == receipt["sha256"], "closure resource drift: {}".format(relative))
        _require(len(payload) == receipt["size_bytes"], "closure resource size drift")
        verified += 1
    for provider in closure["external_providers"]:
        if "path" in provider:
            path = Path(provider["path"])
            _require(path.resolve(strict=True).is_file(), "external provider absent")
            _require(_sha(path.resolve().read_bytes()) == provider["sha256"], "external provider drift")

    bundle = BUILDER.build_bundle(workspace)
    for relative, expected in bundle.items():
        _require(_read_once_no_follow(workspace, relative) == expected, "offline bundle drift: {}".format(relative))
    manifest = _load_yaml_bytes(_read_once_no_follow(workspace, MANIFEST_RELATIVE), "P1 manifest")
    index = _load_yaml_bytes(_read_once_no_follow(workspace, INDEX_RELATIVE), "P1 index")
    schedule = _load_yaml_bytes(_read_once_no_follow(workspace, SCHEDULE_RELATIVE), "P1 schedule")
    STATS.validate_schedule(schedule)
    _require(len(manifest["scenes"]) == 94, "scene count differs")
    _require(len(index["files"]) == 188, "compiled child count differs")
    _require({row["seed"] for row in schedule["episodes"]} == set(range(5201, 5291)), "schedule seed set differs")
    _require(not ({5291, 5292, 5293, 5294} & {row["seed"] for row in schedule["episodes"]}), "support seed scheduled")

    synthetic = _synthetic_episodes(schedule)
    positive = STATS.build_assessment(schedule, synthetic)
    _require(positive["assessment_status"] == "all_four_confirmatory_gates_pass", "positive fixture failed")
    _require(all(positive["four_gate_decision"].values()), "positive fixture gate failed")
    _require(
        STATS.clopper_pearson_upper(0, 90) < 0.05
        and STATS.clopper_pearson_upper(1, 90) > 0.05
        and abs(STATS.clopper_pearson_upper(0, 60) - 0.0487029) < 1e-6,
        "exact binary boundary regression",
    )
    missing = STATS.build_assessment(schedule, synthetic[:-1])
    _require(missing["four_gate_decision"]["integrity"] is False, "missing-pair fixture did not fail")
    interface = copy.deepcopy(synthetic)
    interface[0]["interface_failure"] = True
    _require(
        STATS.build_assessment(schedule, interface)["four_gate_decision"]["integrity"] is False,
        "interface fixture did not fail",
    )
    negative = copy.deepcopy(synthetic)
    for row in negative:
        if row["scene_role"] == STATS.CLEAR_ROLE and row["method_id"] == STATS.TREATMENT:
            row["navigation_time_s"] = 50.0
    negative_report = STATS.build_assessment(schedule, negative)
    _require(negative_report["formal_study_complete"] is True, "negative study is not formal")
    _require(negative_report["claim_authorized"] is False, "negative fixture authorized a claim")

    empty = _load_yaml_bytes(
        _read_once_no_follow(workspace, EMPTY_ASSESSMENT_RELATIVE), "empty assessment"
    )
    _require(empty["four_gate_assessment_run"] is False, "P1 assessment unexpectedly ran")
    _require(empty["evidence_units_consumed"] == 0, "P1 evidence was consumed")
    forbidden_paths = [
        OUTPUT_ROOT / "execution_release.yaml",
        OUTPUT_ROOT / "authorization.yaml",
        OUTPUT_ROOT / "attempts",
        OUTPUT_ROOT / "journals",
        Path("experiments/manifests/v2/integration/v2_04g_r6_i1_p1_execution_release.yaml"),
        Path("experiments/manifests/v2/integration/v2_04g_r6_i1_p1_execution_authorization.yaml"),
    ]
    _require(not any((workspace / path).exists() for path in forbidden_paths), "forbidden P1 execution state exists")
    processes = _active_forbidden_processes()
    _require(processes == [], "ROS/Gazebo/move_base process is active")
    freeze_rows = _verify_freeze(workspace)
    reviewer_path = Path(
        "src/tools/thesis_experiment/scripts/v2_04g_r6_i1_p1_r1_offline_reviewer.py"
    )
    reviewer_sha = _sha(_read_once_no_follow(workspace, reviewer_path))
    closure_file_sha = _sha(closure_payload)
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": "V2-04G-P1-R1",
        "status": "p1_r1_offline_implementation_readiness_closure_pass_release_absent",
        "review_result": "pass",
        "offline_review_pass": True,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "planned_evidence_units": 270,
        "authorized_evidence_units": 0,
        "consumed_evidence_units": 0,
        "training_budget_steps": 0,
        "real_vehicle_used": False,
        "i5_rerun_or_resume_performed": False,
        "dependency_closure": {
            "path": CLOSURE_RELATIVE.as_posix(),
            "file_sha256": closure_file_sha,
            "logical_sha256": closure["logical_sha256"],
            "local_resources_verified": verified,
            "historical_transitive_resources": closure["historical_transitive_resource_count"],
            "unresolved": [],
        },
        "i5_freeze": {
            "path": FREEZE_RELATIVE.as_posix(),
            "sha256": _sha(_read_once_no_follow(workspace, FREEZE_RELATIVE)),
            "resource_count": len(freeze_rows),
            "all_resources_verified": True,
        },
        "scene_schedule": {
            "scenes": 94,
            "evidence_blocks": 90,
            "compiled_children": 188,
            "episodes": 270,
            "schedule_digest": schedule["canonical_episode_digest"],
            "deterministic_rebuild_pass": True,
            "population": "three_frozen_dynamic_templates_under_fresh_simulator_stochastic_seeds",
            "geometry_or_timing_generalization_allowed": False,
        },
        "statistics": {
            "positive_fixture_all_four_gates_pass": True,
            "negative_complete_study_claim_false": True,
            "missing_pair_integrity_fail": True,
            "interface_failure_integrity_fail": True,
            "exact_binary_boundaries_pass": True,
            "timestamped_overlay_contract_present": True,
        },
        "fresh_seed_firewall": {
            "reserved": [5201, 5294],
            "materialized_evidence": [5201, 5290],
            "materialized_compile_support": [5291, 5294],
            "authorized_count": 0,
            "consumed_count": 0,
            "historical_seed_or_budget_reused": False,
            "globally_never_referenced_claim_made": False,
        },
        "resource_time_budget": {
            "estimate_only": True,
            "authorized_budget": 0,
            "serial_concurrency": 1,
            "canonical_artifact_cap_bytes": 536870912,
            "minimum_future_free_disk_bytes": 4294967296,
            "future_stage_wall_cap_h": 20,
        },
        "execution_gap": {
            "p1_exact_runner_adapter_present": False,
            "p1_raw_replay_adapter_present": False,
            "requires_separate_future_implementation_release_and_review": True,
            "current_offline_pass_is_execution_authorization": False,
        },
        "side_effect_audit": {
            "forbidden_execution_paths_absent": True,
            "forbidden_processes": processes,
            "ros_gazebo_started_by_review": False,
            "reviewer_has_execute_option": False,
        },
        "reviewer": {
            "path": reviewer_path.as_posix(),
            "sha256": reviewer_sha,
            "single_open_no_follow_reads": True,
            "deterministic_double_build_required": True,
        },
        "claim_state": {
            "assessment_run": False,
            "performance_improvement_demonstrated": False,
            "claim_authorized": False,
            "future_positive_claim_requires_all_four_gates": True,
        },
        "next_safe_entry": (
            "separately_explicitly_authorized_P1_execution_implementation_release_"
            "with_exact_runner_raw_replay_adapter_270_unit_budget_and_fresh_rehash"
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
    _require(first == second, "review is not deterministic")
    payload = _yaml_bytes(first)
    target = workspace / REVIEW_RELATIVE
    if args.materialize:
        _require(not target.exists(), "review already exists")
        with target.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    else:
        _require(target.is_file(), "persisted review absent")
        _require(_read_once_no_follow(workspace, REVIEW_RELATIVE) == payload, "persisted review differs")
    print("P1-R1 independent offline review {}: {}".format(
        "materialized" if args.materialize else "verified", first["status"]
    ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

