#!/usr/bin/env python3
"""Build the pure-offline R17 goal-aligned recovery closure."""

import argparse
import copy
import hashlib
import math
import os
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
sys.path.insert(
    0, str(W / "src/application/v3_scope_planner/src")
)
from v3_scope_planner import goal_aligned_execution_core as guard  # noqa: E402
from v3_scope_planner import multi_actor_core as base  # noqa: E402
from v3_scope_planner import multi_actor_core_v1 as predictor  # noqa: E402
from v3_scope_planner import progress_recovery_candidate_core as candidate_core  # noqa: E402


CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r17_goal_aligned_recovery_contract.yaml"
)
D0 = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
MODEL = Path(
    "artifacts/v3/multiactor/v3_13_q1_r13_predictor/"
    "expanded_progress_predictor.yaml"
)
R14_SOURCE = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r14_r1_progress_qualification/episodes"
)
R16_REPORT = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r16_actual_teb_preflight/episodes/"
    "002_01_7666_v3_13_analytic/episode_report.yaml"
)
DIAGNOSIS = Path(
    "artifacts/v3/multiactor/v3_13_q1_r16_d1_diagnosis/"
    "execution_failure_attribution.yaml"
)
DIAGNOSIS_MANIFEST = DIAGNOSIS.parent / "evidence_manifest.sha256"
DIAGNOSIS_REVIEW = DIAGNOSIS.parent / "independent_review.yaml"
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r17_goal_aligned_recovery"
)
AUDIT = ROOT / "geometry_and_terminal_guard_audit.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r17_goal_aligned_recovery.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r17_goal_aligned_recovery.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/"
    "test_v3_13_q1_r17_goal_aligned_recovery.py"
)
RESOURCES = [
    CONTRACT,
    D0,
    MODEL,
    DIAGNOSIS,
    DIAGNOSIS_MANIFEST,
    DIAGNOSIS_REVIEW,
    R16_REPORT,
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "progress_recovery_candidate_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "goal_aligned_execution_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "execution_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "multi_actor_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "multi_actor_core_v1.py"
    ),
    BUILDER,
    REVIEWER,
    TEST,
]


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


def r14_roots():
    roots = []
    for path in sorted((W / R14_SOURCE).glob(
        "*recovery_evasive_right_v120_r2"
    )):
        roots.append(path.relative_to(W))
    require(len(roots) == 8, "R14 design root count")
    return roots


def snapshot_from_report(report, scene, identifier, planner_contract):
    decision = report["common_stop_prefix"]
    pose = decision["decision_pose"]
    return {
        "scene_snapshot_id": identifier,
        "world_model_seq": 1,
        "ego": {
            "x_m": float(pose["x_m"]),
            "y_m": float(pose["y_m"]),
            "yaw_rad": float(pose["yaw_rad"]),
            "speed_mps": float(decision["decision_speed_mps"]),
        },
        "actors": decision["decision_actor_tracks"],
        "scene": scene,
        "health": {
            field: True
            for field in planner_contract["hard_verifier"][
                "required_health_fields"
            ]
        },
    }


def design_states(planner_contract):
    states = []
    for root in r14_roots():
        report = load(root / "episode_report.yaml")
        scene = load(root / "scene_spec.yaml")
        states.append(
            (
                "R14-R1-block-{}".format(report["block_index"]),
                snapshot_from_report(
                    report,
                    scene,
                    "R17_R14_{}".format(report["block_index"]),
                    planner_contract,
                ),
            )
        )
    report = load(R16_REPORT)
    snapshot = copy.deepcopy(
        report["result"]["decisions"][-1]["snapshot"]
    )
    snapshot["scene_snapshot_id"] = "R17_R16"
    states.append(("R16-failed-preflight", snapshot))
    return states


def build_outputs():
    contract = load(CONTRACT)
    planner_contract = load(D0)
    model = load(MODEL)
    upstream = contract["upstream"]
    for key, path in (
        ("R16_D1_diagnosis_sha256", DIAGNOSIS),
        ("R16_D1_manifest_sha256", DIAGNOSIS_MANIFEST),
        ("R16_D1_review_sha256", DIAGNOSIS_REVIEW),
        ("progress_candidate_core_sha256", RESOURCES[7]),
        ("terminal_guard_core_sha256", RESOURCES[8]),
    ):
        require(
            sha(read(path)) == upstream[key],
            "{} differs".format(path),
        )
    surrogate_extension = copy.deepcopy(contract)
    surrogate_extension["candidate_extension"]["candidate_id"] = (
        "recovery_evasive_right_v120_r2"
    )
    rows = []
    all_points = []
    for state_id, snapshot in design_states(planner_contract):
        legacy = base.generate_candidate_set(snapshot, planner_contract)
        expanded = candidate_core.generate_candidate_set(
            snapshot, planner_contract, contract, True
        )
        require(len(expanded["candidates"]) == 5, "candidate count")
        legacy_digests = [
            item["candidate_digest"] for item in legacy["candidates"]
        ]
        expanded_legacy_digests = [
            item["candidate_digest"]
            for item in expanded["candidates"][:4]
        ]
        candidate = expanded["candidates"][-1]
        failures = base.validate_candidate(
            candidate,
            candidate_core.evaluation_contract(
                planner_contract, contract
            ),
        )
        surrogate_candidate, surrogate_local = (
            candidate_core.generate_recovery_candidate(
                snapshot,
                planner_contract,
                surrogate_extension,
                True,
            )
        )
        evaluation = predictor.evaluate_candidate(
            snapshot,
            surrogate_candidate,
            model,
            surrogate_local,
        )
        terminal = candidate["points"][-1]
        goal_distance = math.hypot(
            float(terminal["x_m"]) - 5.2,
            float(terminal["y_m"]),
        )
        relative_lateral = max(
            abs(
                float(point["y_m"])
                - float(snapshot["ego"]["y_m"])
            )
            for point in candidate["points"]
        )
        rows.append(
            {
                "state_id": state_id,
                "scene_id": snapshot["scene"]["scene_id"],
                "legacy_candidate_digest_change_count": sum(
                    left != right
                    for left, right in zip(
                        legacy_digests, expanded_legacy_digests
                    )
                ),
                "candidate_validation_failures": list(failures),
                "terminal_world_x_m": terminal["x_m"],
                "terminal_world_y_m": terminal["y_m"],
                "terminal_yaw_rad": terminal["yaw_rad"],
                "terminal_common_goal_distance_m": goal_distance,
                "maximum_relative_lateral_offset_m": relative_lateral,
                "surrogate_dynamic_clearance_m": evaluation[
                    "minimum_dynamic_clearance_m"
                ],
                "static_clearance_m": evaluation[
                    "minimum_static_clearance_m"
                ],
            }
        )
        all_points.extend(candidate["points"])
    r16_candidate, _ = candidate_core.generate_recovery_candidate(
        design_states(planner_contract)[-1][1],
        planner_contract,
        contract,
        True,
    )
    points = r16_candidate["points"]
    terminal = points[-1]
    previous = points[-2]
    tangent_x = float(terminal["x_m"]) - float(previous["x_m"])
    tangent_y = float(terminal["y_m"]) - float(previous["y_m"])
    norm = math.hypot(tangent_x, tangent_y)
    tangent_x /= norm
    tangent_y /= norm

    def command(offset):
        return guard.pure_pursuit_command(
            points,
            float(terminal["x_m"]) + offset * tangent_x,
            float(terminal["y_m"]) + offset * tangent_y,
            float(terminal["yaw_rad"]),
            minimum_index=len(points) - 3,
            terminal_tolerance_m=float(
                contract["terminal_guard"]["terminal_tolerance_m"]
            ),
        )

    approach = command(-0.30)
    at_terminal = command(0.0)
    beyond_terminal = command(0.30)
    gates = contract["offline_geometry_gates"]
    checks = {
        "exactly_one_candidate_appended": all(
            row["legacy_candidate_digest_change_count"] == 0
            for row in rows
        ),
        "legacy_candidate_digest_change_count": all(
            row["legacy_candidate_digest_change_count"]
            <= int(gates["legacy_candidate_digest_change_count_max"])
            for row in rows
        ),
        "kinematic_validation": all(
            len(row["candidate_validation_failures"])
            <= int(gates["kinematic_validation_failure_count_max"])
            for row in rows
        ),
        "maximum_acceleration": max(
            abs(float(point["longitudinal_acceleration_mps2"]))
            for point in all_points
        )
        <= float(gates["maximum_acceleration_mps2_max"]),
        "maximum_steering_angle": max(
            abs(float(point["steering_angle_rad"]))
            for point in all_points
        )
        <= float(gates["maximum_steering_angle_rad_max"]),
        "maximum_steering_rate": max(
            abs(float(point["steering_rate_radps"]))
            for point in all_points
        )
        <= float(gates["maximum_steering_rate_radps_max"]),
        "maximum_curvature": max(
            abs(float(point["curvature_inv_m"]))
            for point in all_points
        )
        <= float(gates["maximum_curvature_inv_m_max"]),
        "maximum_relative_lateral_offset": max(
            row["maximum_relative_lateral_offset_m"] for row in rows
        )
        <= float(gates["maximum_relative_lateral_offset_m_max"]),
        "terminal_common_goal_distance": max(
            row["terminal_common_goal_distance_m"] for row in rows
        )
        <= float(gates["maximum_terminal_common_goal_distance_m_max"]),
        "terminal_yaw": max(
            abs(float(row["terminal_yaw_rad"])) for row in rows
        )
        <= float(gates["maximum_terminal_absolute_yaw_rad_max"]),
        "surrogate_dynamic_clearance": min(
            row["surrogate_dynamic_clearance_m"] for row in rows
        )
        >= float(
            gates[
                "every_design_state_surrogate_dynamic_clearance_m_min"
            ]
        ),
        "static_clearance": min(
            row["static_clearance_m"] for row in rows
        )
        >= float(
            gates["every_design_state_static_clearance_m_min"]
        ),
        "terminal_guard_zero_at_terminal": (
            at_terminal["terminal_reached_fail_closed"]
            and at_terminal["linear_x_mps"] == 0.0
            and at_terminal["angular_z_radps"] == 0.0
        ),
        "terminal_guard_zero_beyond_terminal": (
            beyond_terminal["terminal_reached_fail_closed"]
            and beyond_terminal["linear_x_mps"] == 0.0
            and beyond_terminal["angular_z_radps"] == 0.0
        ),
        "terminal_guard_preserves_approach": (
            not approach["terminal_reached_fail_closed"]
            and approach["linear_x_mps"] > 0.0
        ),
    }
    passed = all(checks.values())
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R17",
        "status": (
            "offline_goal_aligned_recovery_and_guard_pass"
            if passed
            else "offline_goal_aligned_recovery_and_guard_fail"
        ),
        "design_state_count": len(rows),
        "rows": rows,
        "gate_checks": checks,
        "maximum_terminal_common_goal_distance_m": max(
            row["terminal_common_goal_distance_m"] for row in rows
        ),
        "maximum_terminal_absolute_yaw_rad": max(
            abs(float(row["terminal_yaw_rad"])) for row in rows
        ),
        "maximum_relative_lateral_offset_m": max(
            row["maximum_relative_lateral_offset_m"] for row in rows
        ),
        "minimum_surrogate_dynamic_clearance_m": min(
            row["surrogate_dynamic_clearance_m"] for row in rows
        ),
        "minimum_static_clearance_m": min(
            row["static_clearance_m"] for row in rows
        ),
        "maximum_steering_angle_rad": max(
            abs(float(point["steering_angle_rad"]))
            for point in all_points
        ),
        "maximum_steering_rate_radps": max(
            abs(float(point["steering_rate_radps"]))
            for point in all_points
        ),
        "maximum_curvature_inv_m": max(
            abs(float(point["curvature_inv_m"]))
            for point in all_points
        ),
        "terminal_guard_fixtures": {
            "approach": approach,
            "at_terminal": at_terminal,
            "beyond_terminal": beyond_terminal,
        },
        "offline_design_pass": passed,
        "fresh_runtime_preflight_design_authorized": passed,
        "candidate_specific_support_authorized": False,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "new_simulation_units_used": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    return {AUDIT: yaml_bytes(value)}


def evidence_resources():
    resources = list(RESOURCES)
    for root in r14_roots():
        resources.extend(
            root / name
            for name in (
                "episode_report.yaml",
                "scene_spec.yaml",
                "matrix_receipt.yaml",
            )
        )
    return resources


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        write_or_check(path, payload, args.check)
    resources = evidence_resources() + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in resources
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, args.check)
    value = yaml.safe_load(outputs[AUDIT])
    print(
        "status={} audit_sha256={} manifest_sha256={}".format(
            value["status"], sha(outputs[AUDIT]), sha(manifest)
        )
    )


if __name__ == "__main__":
    main()
