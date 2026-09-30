#!/usr/bin/env python3
"""Build deterministic offline R19 terminal-atomicity diagnosis."""

import argparse
import hashlib
import inspect
import math
import os
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
sys.path.insert(0, str(W / "src/application/v3_scope_planner/src"))
from v3_scope_planner import goal_aligned_execution_adapter_r1 as guard  # noqa: E402
from v3_scope_planner import progress_recovery_candidate_core as candidate_core  # noqa: E402

CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_d1_terminal_atomicity_diagnosis_contract.yaml"
)
D0 = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
GEOMETRY = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r17_goal_aligned_recovery_contract.yaml"
)
ASSESSMENT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r19_assessment/"
    "support_assessment.yaml"
)
FREEZE = ASSESSMENT.parent / "evidence_freeze.yaml"
RESULT_REVIEW = ASSESSMENT.parent / "independent_result_review.yaml"
SOURCE_ROOT = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_goal_aligned_support/episodes"
)
LOOP_SOURCE = Path(
    "src/application/v3_scope_planner/scripts/"
    "v3_scope_multiactor_clone_episode.py"
)
ROOT = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_d1_terminal_atomicity_diagnosis"
)
DIAGNOSIS = ROOT / "terminal_atomicity_diagnosis.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r19_d1_terminal_atomicity_diagnosis.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r19_d1_terminal_atomicity_diagnosis.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/"
    "test_v3_13_q1_r19_d1_terminal_atomicity_diagnosis.py"
)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(path):
    target = W / path
    require(target.is_file(), "required file absent: {}".format(path))
    return target.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load(path):
    return yaml.safe_load(read(path))


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def write_or_check(path, payload, check):
    target = W / path
    if check:
        require(target.is_file(), "output absent")
        require(target.read_bytes() == payload, "output drift")
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    require(not target.exists(), "refusing overwrite")
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_bytes(payload)
    os.replace(str(temporary), str(target))


def r3_reports():
    roots = sorted(
        path
        for path in (W / SOURCE_ROOT).glob(
            "*recovery_evasive_right_v120_r3"
        )
        if path.is_dir()
    )
    require(len(roots) == 8, "r3 report count")
    return [
        (
            root.relative_to(W),
            yaml.safe_load((root / "episode_report.yaml").read_bytes()),
        )
        for root in roots
    ]


def build_outputs():
    contract = load(CONTRACT)
    upstream = contract["upstream"]
    for key, path in (
        ("R19_assessment_sha256", ASSESSMENT),
        ("R19_freeze_sha256", FREEZE),
        ("R19_review_sha256", RESULT_REVIEW),
    ):
        require(sha(read(path)) == upstream[key], "{} differs".format(path))
    planner = load(D0)
    geometry = load(GEOMETRY)
    rows = []
    raw_resources = []
    for root, report in r3_reports():
        raw_resources.extend(
            root / name
            for name in (
                "episode_report.yaml",
                "matrix_receipt.yaml",
                "scene_spec.yaml",
                "actor_specs.yaml",
                "world.sdf",
                "roslaunch.log",
            )
        )
        decision = report["common_stop_prefix"]
        pose = decision["decision_pose"]
        snapshot = {
            "scene_snapshot_id": "R19-D1-{}".format(
                report["block_index"]
            ),
            "world_model_seq": 1,
            "ego": {
                "x_m": pose["x_m"],
                "y_m": pose["y_m"],
                "yaw_rad": pose["yaw_rad"],
                "speed_mps": decision["decision_speed_mps"],
            },
        }
        candidate, _ = candidate_core.generate_recovery_candidate(
            snapshot, planner, geometry, True
        )
        final_pose = report["result"]["final_pose"]
        replay = guard.pure_pursuit_command(
            candidate["points"],
            final_pose["x_m"],
            final_pose["y_m"],
            final_pose["yaw_rad"],
            minimum_index=len(candidate["points"]) - 3,
        )
        rows.append(
            {
                "block_index": report["block_index"],
                "seed": report["seed"],
                "scene_id": report["scene_id"],
                "observed_guard_activation_count": report[
                    "terminal_guard_activation_count"
                ],
                "trace_duration_s": report["result"][
                    "trace_duration_s"
                ],
                "final_common_goal_distance_m": math.hypot(
                    final_pose["x_m"] - 5.2, final_pose["y_m"]
                ),
                "final_candidate_terminal_distance_m": replay[
                    "terminal_distance_m"
                ],
                "final_terminal_plane_progress_m": replay[
                    "terminal_plane_progress_m"
                ],
                "final_terminal_window": replay["terminal_window"],
                "final_pose_would_trigger_frozen_guard": replay[
                    "terminal_reached_fail_closed"
                ],
                "minimum_dynamic_clearance_m": report["result"][
                    "minimum_conservative_clearance_m"
                ],
                "minimum_static_clearance_m": report[
                    "minimum_trace_static_clearance_m"
                ],
                "contact_state_count": report["result"][
                    "contact_state_count"
                ],
            }
        )
    source = read(LOOP_SOURCE).decode("utf-8")
    horizon_index = source.index(
        "if elapsed_s >= self.horizon_s:", source.index("def execute(")
    )
    command_index = source.index(
        "command = execution_core.pure_pursuit_command(", horizon_index
    )
    observed_pass = sum(
        row["observed_guard_activation_count"] >= 1 for row in rows
    )
    failures = [
        row for row in rows
        if row["observed_guard_activation_count"] == 0
    ]
    checks = {
        "r3_episode_count": len(rows)
        == int(contract["diagnostic_gates"]["r3_episode_count"]),
        "observed_guard_counts": (
            observed_pass
            == int(
                contract["diagnostic_gates"][
                    "observed_guard_pass_count"
                ]
            )
            and len(failures)
            == int(
                contract["diagnostic_gates"][
                    "observed_guard_fail_count"
                ]
            )
        ),
        "every_final_pose_would_trigger_guard": all(
            row["final_pose_would_trigger_frozen_guard"]
            for row in rows
        ),
        "failures_exactly_at_horizon": all(
            abs(row["trace_duration_s"] - 7.55) <= 1.0e-9
            for row in failures
        ),
        "break_precedes_command_in_source": horizon_index < command_index,
        "all_r3_dynamic_safe": min(
            row["minimum_dynamic_clearance_m"] for row in rows
        )
        >= float(
            contract["diagnostic_gates"][
                "all_r3_dynamic_clearance_m_min"
            ]
        ),
        "all_r3_static_safe": min(
            row["minimum_static_clearance_m"] for row in rows
        )
        >= float(
            contract["diagnostic_gates"][
                "all_r3_static_clearance_m_min"
            ]
        ),
        "all_r3_contact_free": all(
            row["contact_state_count"] == 0 for row in rows
        ),
    }
    passed = all(checks.values())
    diagnosis = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-D1",
        "status": (
            "offline_terminal_boundary_atomicity_diagnosis_pass"
            if passed
            else "offline_terminal_boundary_atomicity_diagnosis_fail"
        ),
        "checks": checks,
        "rows": rows,
        "aggregate": {
            "observed_guard_activation_rate": observed_pass / len(rows),
            "offline_final_pose_guard_rate": sum(
                row["final_pose_would_trigger_frozen_guard"]
                for row in rows
            )
            / len(rows),
            "minimum_dynamic_clearance_m": min(
                row["minimum_dynamic_clearance_m"] for row in rows
            ),
            "minimum_static_clearance_m": min(
                row["minimum_static_clearance_m"] for row in rows
            ),
            "maximum_final_common_goal_distance_m": max(
                row["final_common_goal_distance_m"] for row in rows
            ),
        },
        "confirmed_cause": (
            "At an exact 7.55 s final sample the inherited loop exits "
            "before evaluating the frozen terminal guard. Both guard-zero "
            "final poses satisfy the guard; 7.60 s traces received the "
            "extra control evaluation."
        ),
        "repair_boundary": {
            "evaluate_terminal_guard_atomically_at_horizon_boundary": True,
            "publish_only_zero_command_at_or_after_horizon": True,
            "geometry_actor_model_threshold_and_horizon_changes": 0,
            "fresh_stage_and_seed_required": True,
        },
        "R19_remains_failed": True,
        "R19_predictor_fit_authorized": False,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = yaml_bytes(diagnosis)
    resources = [
        CONTRACT,
        D0,
        GEOMETRY,
        ASSESSMENT,
        FREEZE,
        RESULT_REVIEW,
        LOOP_SOURCE,
        Path(
            "src/application/v3_scope_planner/src/v3_scope_planner/"
            "goal_aligned_execution_adapter_r1.py"
        ),
        Path(
            "src/application/v3_scope_planner/src/v3_scope_planner/"
            "progress_recovery_candidate_core.py"
        ),
        BUILDER,
        REVIEWER,
        TEST,
    ] + raw_resources
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in resources
    ).encode("utf-8")
    return {DIAGNOSIS: payload, MANIFEST: manifest}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        write_or_check(path, payload, args.check)
    diagnosis = yaml.safe_load(outputs[DIAGNOSIS])
    print(
        "status={} diagnosis_sha256={} manifest_sha256={}".format(
            diagnosis["status"],
            sha(outputs[DIAGNOSIS]),
            sha(outputs[MANIFEST]),
        )
    )


if __name__ == "__main__":
    main()
