#!/usr/bin/env python3
"""Independent fail-closed offline review for V2-04G-P2-Q1.

The reviewer rebuilds every Q1 output, independently checks candidate and scene
diffs, replays synthetic raw traces, verifies historical P1/I5/T12 freezes, and
confirms the current runner cannot execute.  It imports no ROS package and has
no simulation mode.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import re
import stat
from typing import Any, Dict, Mapping, Sequence

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
OUTPUT_ROOT = Path(
    "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_offline_readiness"
)
REVIEW_RELATIVE = OUTPUT_ROOT / "v2_04g_p2_q1_offline_review.yaml"
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v2/"
    "v2_04g_r6_i1_p2_q1_offline_implementation_readiness_contract.yaml"
)
BUILDER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_q1_offline_builder.py"
)
RUNNER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_q1_exact_runner.py"
)
REPLAY_RELATIVE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p2_q1_replay.py"
)
REVIEWER_RELATIVE = Path(
    "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_q1_offline_reviewer.py"
)
HEX_SHA = re.compile(r"^[0-9a-f]{64}$")


class Q1OfflineReviewError(ValueError):
    """Raised when any independent Q1 invariant fails."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise Q1OfflineReviewError(message)


def _load_module(name: str, relative: Path):
    path = WORKSPACE / relative
    spec = importlib.util.spec_from_file_location(name, path)
    _require(spec is not None and spec.loader is not None, "cannot load {}".format(relative))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BUILDER = _load_module("_p2_q1_builder_for_review", BUILDER_RELATIVE)
RUNNER = _load_module("_p2_q1_runner_for_review", RUNNER_RELATIVE)
REPLAY = _load_module("_p2_q1_replay_for_review", REPLAY_RELATIVE)


class _UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise Q1OfflineReviewError("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def _safe_parts(relative: Path):
    relative = Path(relative)
    _require(
        not relative.is_absolute()
        and relative.parts
        and all(part not in {"", ".", ".."} for part in relative.parts),
        "unsafe relative path: {}".format(relative),
    )
    return relative.parts


def _read_once_no_follow(workspace: Path, relative: Path) -> bytes:
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
            chunks = []
            while True:
                chunk = os.read(file_descriptor, 1024 * 1024)
                if not chunk:
                    break
                chunks.append(chunk)
            after = os.fstat(file_descriptor)
            stable = (
                "st_dev",
                "st_ino",
                "st_mode",
                "st_size",
                "st_mtime_ns",
                "st_ctime_ns",
            )
            _require(
                tuple(getattr(before, field) for field in stable)
                == tuple(getattr(after, field) for field in stable),
                "file changed during review: {}".format(relative),
            )
            payload = b"".join(chunks)
            _require(len(payload) == before.st_size, "short read: {}".format(relative))
            return payload
        finally:
            os.close(file_descriptor)
    except OSError as exc:
        raise Q1OfflineReviewError(
            "cannot no-follow read {}: {}".format(relative, exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            os.close(descriptor)


def _load_yaml_bytes(payload: bytes, label: str) -> Mapping[str, Any]:
    try:
        value = yaml.load(payload.decode("utf-8"), Loader=_UniqueLoader)
    except (UnicodeError, yaml.YAMLError) as exc:
        raise Q1OfflineReviewError("cannot parse {}: {}".format(label, exc)) from exc
    _require(type(value) is dict, "{} is not a mapping".format(label))
    return value


def _load_yaml(workspace: Path, relative: Path) -> Mapping[str, Any]:
    return _load_yaml_bytes(_read_once_no_follow(workspace, relative), str(relative))


def _sha(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def _leaf_differences(source: Any, target: Any, prefix: str = ""):
    if type(source) is not type(target):
        return [prefix]
    if isinstance(source, dict):
        output = []
        for key in sorted(set(source) | set(target), key=str):
            child = "{}.{}".format(prefix, key) if prefix else str(key)
            if key not in source or key not in target:
                output.append(child)
            else:
                output.extend(_leaf_differences(source[key], target[key], child))
        return output
    if isinstance(source, list):
        if len(source) != len(target):
            return [prefix]
        output = []
        for index, (left, right) in enumerate(zip(source, target)):
            output.extend(
                _leaf_differences(left, right, "{}[{}]".format(prefix, index))
            )
        return output
    return [] if source == target else [prefix]


def _verify_receipt(workspace: Path, receipt: Mapping[str, Any]) -> bytes:
    _require(
        type(receipt) is dict
        and set(receipt) == {"path", "sha256", "size_bytes"},
        "receipt schema differs",
    )
    payload = _read_once_no_follow(workspace, Path(receipt["path"]))
    _require(_sha(payload) == receipt["sha256"], "receipt SHA differs")
    _require(len(payload) == receipt["size_bytes"], "receipt size differs")
    return payload


def _verify_hash_manifest(
    workspace: Path, relative: Path, paths_are_workspace_relative: bool
) -> int:
    payload = _read_once_no_follow(workspace, relative)
    count = 0
    for line in payload.decode("utf-8").splitlines():
        if not line.strip():
            continue
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        _require(match is not None, "hash manifest line differs: {}".format(relative))
        digest, path_text = match.groups()
        _require(HEX_SHA.fullmatch(digest) is not None, "manifest SHA differs")
        item = Path(path_text)
        target_relative = item if paths_are_workspace_relative else relative.parent / item
        _require(
            _sha(_read_once_no_follow(workspace, target_relative)) == digest,
            "frozen resource drifted: {}".format(target_relative),
        )
        count += 1
    _require(count > 0, "hash manifest is empty")
    return count


def _qualification_sample(
    stamp: float,
    x: float,
    speed: float,
    yaw: float = 0.0,
    overlay: str = "NONE",
    trigger: bool = False,
    transaction_valid: bool = True,
) -> Dict[str, Any]:
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
        "transaction_valid": transaction_valid,
    }


def _synthetic_replay_fixtures(thresholds: Mapping[str, Any]) -> Dict[str, bool]:
    navigation_plan = {
        "sequence": 1,
        "row_type": "calibration_navigation",
        "method_id": "circle_contact_v130",
        "scene_id": "fixture",
        "scene_role": "single_true_conflict",
        "seed": 5301,
        "attempt": 1,
        "block_id": "p2-q1-cal-block-s5301",
        "timeout_s": 80.0,
    }
    navigation = []
    for stamp, x, speed, goal, ttc in (
        (0.0, 0.0, 1.0, False, None),
        (1.0, 1.0, 1.0, False, 1.0),
        (2.0, 2.0, 0.0, True, 0.5),
    ):
        navigation.append(
            {
                "stamp_s": stamp,
                "x_m": x,
                "y_m": 0.0,
                "linear_velocity_mps": speed,
                "yaw_rate_radps": 0.0,
                "clearance_m": 0.5,
                "predicted_ttc_s": ttc,
                "collision": False,
                "goal_reached": goal,
                "emergency": False,
                "projection_applied": False,
                "transaction_valid": True,
                "tracker_valid": True,
            }
        )
    navigation_result = REPLAY.normalize_navigation_trace(
        navigation_plan, navigation
    )
    _require(navigation_result["success"] is True, "navigation fixture failed")
    _require(
        navigation_result["observed_ttc_status"] == "OBSERVED_CONFLICT",
        "navigation TTC fixture failed",
    )

    def plan(probe):
        return {
            "sequence": 331,
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
        REPLAY.assess_qualification_trace(plan("braking"), braking, thresholds)["pass"]
        is True,
        "braking fixture failed",
    )
    _require(
        REPLAY.assess_qualification_trace(
            plan("high_speed_to_conflict_overlay_transition"),
            overlay,
            thresholds,
        )["pass"]
        is True,
        "overlay fixture failed",
    )
    _require(
        REPLAY.assess_qualification_trace(
            plan("ackermann_circle"), circle, thresholds
        )["pass"]
        is True,
        "circle fixture failed",
    )
    broken = [dict(row) for row in overlay]
    broken[-1]["transaction_valid"] = False
    _require(
        REPLAY.assess_qualification_trace(
            plan("high_speed_to_conflict_overlay_transition"),
            broken,
            thresholds,
        )["pass"]
        is False,
        "invalid transaction fixture did not fail",
    )
    return {
        "navigation_positive": True,
        "braking_positive": True,
        "overlay_positive": True,
        "circle_positive": True,
        "invalid_transaction_negative": True,
    }


def _verify_candidate_diffs(
    workspace: Path, contract: Mapping[str, Any], methods: Mapping[str, Any]
) -> Dict[str, int]:
    source_bank = _load_yaml(
        workspace, Path(contract["source_bindings"]["circle_contact_anchor_bank"]["path"])
    )
    fixed = _load_yaml(
        workspace, Path(contract["source_bindings"]["fixed_teb_reference"]["path"])
    )
    counts = {"circle": 0, "fixed": 0, "tuned": 0}
    for row in methods["methods"]:
        if row["family"] == "fam_teb_circle_contact_single_factor_grid":
            candidate = _load_yaml_bytes(
                _verify_receipt(workspace, row["materialized_anchor_bank"]),
                row["method_id"],
            )
            diffs = _leaf_differences(source_bank, candidate)
            allowed = {
                "bank_id",
                "status",
                "source_provenance.p2_q1",
                "parameters[0].bounds[1]",
                "anchors.anchor_cruise.profile_id",
                "anchors.anchor_cruise.values.max_vel_x",
            }
            _require(set(diffs) <= allowed, "circle diff escaped independent allowlist")
            behavioral = [
                item
                for item in diffs
                if item == "anchors.anchor_cruise.values.max_vel_x"
            ]
            _require(
                len(behavioral)
                == (0 if row["candidate_speed_mps"] == 1.2 else 1),
                "circle behavioral factor count differs",
            )
            counts["circle"] += 1
        elif row["family"] == "independently_tuned_fixed_teb_grid":
            candidate = _load_yaml_bytes(
                _verify_receipt(workspace, row["materialized_config"]),
                row["method_id"],
            )
            diffs = _leaf_differences(fixed, candidate)
            _require(
                diffs
                == (
                    []
                    if row["candidate_speed_mps"] == 1.2
                    else ["TebLocalPlannerROS.max_vel_x"]
                ),
                "fixed grid diff differs",
            )
            counts["fixed"] += 1
        else:
            candidate = _load_yaml_bytes(
                _verify_receipt(workspace, row["materialized_config"]),
                row["method_id"],
            )
            diffs = _leaf_differences(fixed, candidate)
            _require(
                len(diffs) == 9
                and set(diffs)
                == {
                    "TebLocalPlannerROS.{}".format(field)
                    for field in row["exact_t08_theta"]
                },
                "TEB-Tuned-V1 exact theta differs",
            )
            counts["tuned"] += 1
    _require(counts == {"circle": 5, "fixed": 5, "tuned": 1}, "candidate counts differ")
    return counts


def build_review(workspace: Path = WORKSPACE) -> Dict[str, Any]:
    workspace = Path(workspace).resolve()
    _require(workspace == WORKSPACE.resolve(), "workspace identity differs")
    contract = _load_yaml(workspace, CONTRACT_RELATIVE)
    _require(contract["stage"] == "V2-04G-P2-Q1", "contract stage differs")
    _require(contract["offline_only"] is True, "contract is not offline-only")
    _require(contract["execution_authorized"] is False, "contract authorizes execution")
    _require(contract["evidence_budget_authorized"] == 0, "contract budget differs")
    _require(
        contract["training_budget_steps_authorized"] == 0,
        "contract training budget differs",
    )

    rebuilt = BUILDER.build_bundle(workspace)
    for relative, expected in rebuilt.items():
        _require(
            _read_once_no_follow(workspace, relative) == expected,
            "deterministic bundle rebuild differs: {}".format(relative),
        )
    methods = _load_yaml(workspace, BUILDER.METHOD_BINDINGS_RELATIVE)
    manifest = _load_yaml(workspace, BUILDER.SCENES_RELATIVE)
    index = _load_yaml(workspace, BUILDER.COMPILED_INDEX_RELATIVE)
    schedule = _load_yaml(workspace, BUILDER.SCHEDULE_RELATIVE)
    plan = _load_yaml(workspace, BUILDER.COMMAND_PLAN_RELATIVE)
    audit = _load_yaml(workspace, BUILDER.AUDIT_RELATIVE)
    report = _load_yaml(workspace, BUILDER.REPORT_RELATIVE)
    failure_index = _load_yaml(workspace, BUILDER.FAILURE_INDEX_RELATIVE)

    candidate_counts = _verify_candidate_diffs(workspace, contract, methods)
    _require(len(manifest["scenes"]) == 48, "scene count differs")
    _require(index["compiled_child_count"] == 96, "compiled child count differs")
    _require(len(index["files"]) == 96, "compiled receipt count differs")
    for receipt in index["files"]:
        _verify_receipt(workspace, receipt)
    source_scenes = _load_yaml(
        workspace, Path(contract["source_bindings"]["i5_source_scenes"]["path"])
    )
    source_by_id = {row["scene_id"]: row for row in source_scenes["scenes"]}
    target_by_id = {row["scene_id"]: row for row in manifest["scenes"]}
    for row in audit["scene_template_equivalence"]:
        diffs = _leaf_differences(
            source_by_id[row["source_scene_id"]],
            target_by_id[row["target_scene_id"]],
        )
        _require(diffs == row["changed_fields"], "scene diff audit differs")
        _require(
            set(diffs) <= {"scene_id", "seed", "split", "evaluator_only.reason"},
            "scene clone escaped independent allowlist",
        )

    _require(len(schedule["rows"]) == 342, "schedule row count differs")
    _require(len(plan["rows"]) == 342, "command plan row count differs")
    _require(
        [row["sequence"] for row in schedule["rows"]]
        == [row["sequence"] for row in plan["rows"]]
        == list(range(1, 343)),
        "schedule/plan sequence differs",
    )
    calibration = schedule["rows"][:330]
    blocks = {}
    for row in calibration:
        blocks.setdefault(row["block_id"], []).append(row)
    _require(len(blocks) == 30, "calibration block count differs")
    _require(
        all(
            len(rows) == 11
            and {row["method_id"] for row in rows} == set(BUILDER.METHODS)
            and len({row["seed"] for row in rows}) == 1
            for rows in blocks.values()
        ),
        "paired calibration block differs",
    )
    _require(
        not set(range(5401, 5491)) & {row["seed"] for row in schedule["rows"]},
        "held-out seed leaked",
    )
    _require(schedule["held_out_test_row_count"] == 0, "held-out row exists")
    _require(
        audit["schedule_audit"]["maximum_method_position_count_difference"] <= 1,
        "position balance differs",
    )
    _require(
        plan["execution_authorized"] is False
        and plan["evidence_budget_authorized"] == 0,
        "command plan has authority",
    )

    source_freeze_count = 0
    for line in _read_once_no_follow(
        workspace, BUILDER.SOURCE_FREEZE_RELATIVE
    ).decode("utf-8").splitlines():
        match = re.fullmatch(r"([0-9a-f]{64})  (.+)", line)
        _require(match is not None, "source freeze line differs")
        digest, path_text = match.groups()
        _require(
            _sha(_read_once_no_follow(workspace, Path(path_text))) == digest,
            "source freeze target drifted",
        )
        source_freeze_count += 1
    _require(source_freeze_count >= 23, "source freeze roster is incomplete")

    historical = contract["historical_freeze_bindings"]
    p1_count = _verify_hash_manifest(
        workspace, Path(historical["p1_terminal_evidence"]["path"]), True
    )
    i5_count = _verify_hash_manifest(
        workspace, Path(historical["i5_terminal_evidence"]["path"]), True
    )
    t12_count = _verify_hash_manifest(
        workspace, Path(historical["t12_terminal_two_seed_pilot"]["path"]), False
    )
    _require(p1_count == 10, "P1 terminal freeze roster differs")
    _require(i5_count == 13, "I5 terminal freeze roster differs")
    _require(t12_count == 5, "T12 terminal freeze roster differs")

    fixtures = _synthetic_replay_fixtures(contract["qualification_thresholds"])
    preflight = RUNNER.preflight()
    _require(
        preflight["status"] == "offline_preflight_pass_execute_hard_disabled",
        "runner preflight differs",
    )
    runner_text = _read_once_no_follow(workspace, RUNNER_RELATIVE).decode("utf-8")
    _require(
        "P2-Q1 execute is hard-disabled" in runner_text
        and "subprocess" not in runner_text,
        "runner hard-disable source boundary differs",
    )
    forbidden = [
        Path(
            "experiments/manifests/v2/integration/"
            "v2_04g_r6_i1_p2_q1_execution_release.yaml"
        ),
        Path(
            "experiments/manifests/v2/integration/"
            "v2_04g_r6_i1_p2_q1_simulation_authorization.yaml"
        ),
        Path("artifacts/v2/integration/v2_04g_r6_i1/p2_q1_execution"),
    ]
    _require(
        not any((workspace / path).exists() for path in forbidden),
        "forbidden Q1 execution state exists",
    )
    _require(failure_index["evidence_units_consumed"] == 0, "Q1 evidence consumed")
    _require(failure_index["failures"] == [], "Q1 failure index is not empty")
    _require(report["formal_result"] is False, "Q1 has a formal result")
    _require(
        report["claim_state"]["performance_improvement_demonstrated"] is False,
        "Q1 claims performance improvement",
    )
    reviewer_payload = _read_once_no_follow(workspace, REVIEWER_RELATIVE)
    return {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": "V2-04G-P2-Q1",
        "status": "p2_q1_independent_offline_readiness_closure_pass_release_absent",
        "review_result": "pass",
        "offline_review_pass": True,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "execution_ready": False,
        "execution_authorized": False,
        "planned_future_evidence_units": 342,
        "authorized_evidence_units": 0,
        "consumed_evidence_units": 0,
        "training_budget_steps": 0,
        "real_vehicle_used": False,
        "deterministic_rebuild": {
            "resource_count": len(rebuilt),
            "all_bytes_equal": True,
            "source_freeze_resources": source_freeze_count,
        },
        "candidate_review": {
            **candidate_counts,
            "sole_new_behavioral_field": "TebLocalPlannerROS.max_vel_x",
            "intentional_baseline_weakening_detected": False,
            "strongest_safe_fair_baseline_selection_deferred_to_calibration": True,
        },
        "scene_schedule_review": {
            "scenes": 48,
            "compiled_children": 96,
            "calibration_blocks": 30,
            "calibration_rows": 330,
            "qualification_rows": 12,
            "total_rows": 342,
            "held_out_test_rows": 0,
            "held_out_test_artifacts": 0,
            "maximum_method_position_count_difference": audit[
                "schedule_audit"
            ]["maximum_method_position_count_difference"],
        },
        "raw_replay_review": fixtures,
        "historical_freezes": {
            "p1_resources_verified": p1_count,
            "i5_resources_verified": i5_count,
            "t12_terminal_resources_verified": t12_count,
            "historical_rerun_or_overwrite_performed": False,
        },
        "runner_review": {
            "preflight_pass": True,
            "execute_hard_disabled": True,
            "subprocess_imported": False,
            "ros_gazebo_started": False,
            "release_present": False,
            "authorization_present": False,
            "authorized_budget": 0,
        },
        "reviewer": {
            "path": REVIEWER_RELATIVE.as_posix(),
            "sha256": _sha(reviewer_payload),
            "single_open_no_follow_reads": True,
            "deterministic_double_review_required": True,
        },
        "claim_state": {
            "candidate_safety_qualified": False,
            "calibration_result_exists": False,
            "primary_baseline_selected": False,
            "performance_improvement_demonstrated": False,
            "held_out_test_accessed": False,
        },
        "next_safe_entry": (
            "new_versioned_exact_P2_calibration_release_with_closed_transitive_"
            "runtime_roster_342_unit_budget_and_corresponding_simulation_authorization"
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
        _require(not target.exists(), "Q1 review already exists")
        with target.open("xb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    else:
        _require(target.is_file(), "persisted Q1 review absent")
        _require(
            _read_once_no_follow(workspace, REVIEW_RELATIVE) == payload,
            "persisted Q1 review differs",
        )
    print(
        "P2-Q1 independent offline review {}: {}".format(
            "materialized" if args.materialize else "verified",
            first["status"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
