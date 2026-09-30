#!/usr/bin/env python3
"""Build the pure-offline R10 progress-relative geometry closure."""

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
from v3_scope_planner import multi_actor_core as base  # noqa: E402
from v3_scope_planner import multi_actor_core_v1 as predictor  # noqa: E402
from v3_scope_planner import progress_recovery_candidate_core as progress  # noqa: E402


CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r10_progress_recovery_geometry_contract.yaml"
)
D0 = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
MODEL = Path(
    "artifacts/v3/multiactor/v3_13_q1_r7_predictor/"
    "expanded_recovery_predictor.yaml"
)
R8 = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r8_r2_recovery_qualification/episodes"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r10_progress_geometry"
)
AUDIT = ROOT / "candidate_geometry_audit.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r10_progress_geometry.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r10_progress_geometry.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/"
    "test_v3_13_q1_r10_progress_geometry.py"
)
RESOURCES = [
    CONTRACT,
    D0,
    MODEL,
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "progress_recovery_candidate_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "multi_actor_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "multi_actor_core_v1.py"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r9_planner_audit/"
        "expanded_planner_audit.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r9_planner_audit/"
        "evidence_manifest.sha256"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r9_planner_audit/"
        "independent_review.yaml"
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


def source_roots():
    roots = {}
    for path in sorted((W / R8).glob(
        "*recovery_evasive_right_v120_r1"
    )):
        report = yaml.safe_load(
            (path / "episode_report.yaml").read_bytes()
        )
        roots[int(report["block_index"])] = path.relative_to(W)
    require(set(roots) == set(range(1, 9)), "R8 block roster")
    return roots


def make_snapshot(report, scene, planner_contract, snapshot_id):
    decision = report["common_stop_prefix"]
    pose = decision["decision_pose"]
    return {
        "scene_snapshot_id": snapshot_id,
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


def surrogate_candidate(candidate):
    value = copy.deepcopy(candidate)
    value["candidate_id"] = "recovery_evasive_right_v120_r1"
    body = copy.deepcopy(value)
    body.pop("candidate_digest", None)
    value["candidate_digest"] = base.digest(body)
    return value


def normalized_points(candidate, snapshot):
    origin_x = float(snapshot["ego"]["x_m"])
    origin_y = float(snapshot["ego"]["y_m"])
    yaw = float(snapshot["ego"]["yaw_rad"])
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    values = []
    for point in candidate["points"]:
        dx = float(point["x_m"]) - origin_x
        dy = float(point["y_m"]) - origin_y
        values.append(
            (
                cosine * dx + sine * dy,
                -sine * dx + cosine * dy,
                float(point["yaw_rad"]) - yaw,
            )
        )
    return values


def build_outputs():
    contract = load(CONTRACT)
    planner_contract = load(D0)
    model = load(MODEL)
    upstream = contract["upstream"]
    for key, path in (
        ("R9_audit_sha256", RESOURCES[6]),
        ("R9_manifest_sha256", RESOURCES[7]),
        ("R9_review_sha256", RESOURCES[8]),
        ("progress_candidate_core_sha256", RESOURCES[3]),
    ):
        require(sha(read(path)) == upstream[key], "{} differs".format(path))
    rows = []
    legacy_change_count = 0
    validation_failures = []
    reference_normalized = None
    translation_residual = 0.0
    for block_index, root in source_roots().items():
        report = load(root / "episode_report.yaml")
        scene = load(root / "scene_spec.yaml")
        state = make_snapshot(
            report, scene, planner_contract, "R10_{}".format(block_index)
        )
        legacy = base.generate_candidate_set(state, planner_contract)
        expanded = progress.generate_candidate_set(
            state, planner_contract, contract, True
        )
        for before, after in zip(
            legacy["candidates"], expanded["candidates"][:4]
        ):
            legacy_change_count += (
                before["candidate_digest"]
                != after["candidate_digest"]
            )
        candidate = expanded["candidates"][-1]
        evaluation_contract = progress.evaluation_contract(
            planner_contract, contract
        )
        failures = list(
            base.validate_candidate(candidate, evaluation_contract)
        )
        validation_failures.extend(failures)
        evaluation = predictor.evaluate_candidate(
            state,
            surrogate_candidate(candidate),
            model,
            evaluation_contract,
        )
        points = candidate["points"]
        rows.append(
            {
                "block_index": block_index,
                "scene_id": report["scene_id"],
                "source_root": str(root),
                "surrogate_dynamic_clearance_m": evaluation[
                    "minimum_dynamic_clearance_m"
                ],
                "static_clearance_m": evaluation[
                    "minimum_static_clearance_m"
                ],
                "maximum_absolute_lateral_offset_m": max(
                    abs(
                        float(point["y_m"])
                        - float(state["ego"]["y_m"])
                    )
                    for point in points
                ),
                "terminal_progress_m": math.hypot(
                    float(points[-1]["x_m"])
                    - float(points[0]["x_m"]),
                    float(points[-1]["y_m"])
                    - float(points[0]["y_m"]),
                ),
                "terminal_absolute_lateral_offset_m": abs(
                    normalized_points(candidate, state)[-1][1]
                ),
                "terminal_absolute_yaw_rad": abs(
                    normalized_points(candidate, state)[-1][2]
                ),
            }
        )
        normalized = normalized_points(candidate, state)
        if reference_normalized is None:
            reference_normalized = normalized
        translation_residual = max(
            translation_residual,
            max(
                abs(left - right)
                for current, reference in zip(
                    normalized, reference_normalized
                )
                for left, right in zip(current, reference)
            ),
        )
    fixture_state = {
        "scene_snapshot_id": "R10_translation_fixture",
        "world_model_seq": 1,
        "ego": {
            "x_m": 1.0,
            "y_m": -1.0,
            "yaw_rad": 0.2,
            "speed_mps": 0.0,
        },
    }
    fixture_candidate, evaluation_contract = (
        progress.generate_recovery_candidate(
            fixture_state, planner_contract, contract, True
        )
    )
    fixture_normalized = normalized_points(
        fixture_candidate, fixture_state
    )
    translation_residual = max(
        translation_residual,
        max(
            abs(left - right)
            for current, reference in zip(
                fixture_normalized, reference_normalized
            )
            for left, right in zip(current, reference)
        ),
    )
    points = fixture_candidate["points"]
    metrics = {
        "maximum_acceleration_mps2": max(
            abs(float(point["longitudinal_acceleration_mps2"]))
            for point in points
        ),
        "maximum_steering_angle_rad": max(
            abs(float(point["steering_angle_rad"])) for point in points
        ),
        "maximum_steering_rate_radps": max(
            abs(float(point["steering_rate_radps"])) for point in points
        ),
        "maximum_curvature_inv_m": max(
            abs(float(point["curvature_inv_m"])) for point in points
        ),
    }
    gates = contract["offline_geometry_gates"]
    checks = {
        "exactly_one_candidate_appended": True,
        "legacy_candidate_digests_unchanged": legacy_change_count
        <= int(gates["legacy_candidate_digest_change_count_max"]),
        "kinematic_validation": len(validation_failures)
        <= int(gates["kinematic_validation_failure_count_max"]),
        "acceleration_bound": metrics["maximum_acceleration_mps2"]
        <= float(gates["maximum_acceleration_mps2_max"]),
        "steering_angle_bound": metrics["maximum_steering_angle_rad"]
        <= float(gates["maximum_steering_angle_rad_max"]),
        "steering_rate_bound": metrics["maximum_steering_rate_radps"]
        <= float(gates["maximum_steering_rate_radps_max"]),
        "curvature_bound": metrics["maximum_curvature_inv_m"]
        <= float(gates["maximum_curvature_inv_m_max"]),
        "lateral_offset_bound": max(
            row["maximum_absolute_lateral_offset_m"] for row in rows
        )
        <= float(gates["maximum_absolute_lateral_offset_m_max"]),
        "terminal_progress": min(
            row["terminal_progress_m"] for row in rows
        )
        >= float(gates["terminal_progress_m_min"]),
        "terminal_lateral_offset": max(
            row["terminal_absolute_lateral_offset_m"] for row in rows
        )
        <= float(gates["terminal_absolute_lateral_offset_m_max"]),
        "terminal_yaw": max(
            row["terminal_absolute_yaw_rad"] for row in rows
        )
        <= float(gates["terminal_absolute_yaw_rad_max"]),
        "every_state_surrogate_dynamic_clearance": min(
            row["surrogate_dynamic_clearance_m"] for row in rows
        )
        >= float(
            gates[
                "every_R8_state_surrogate_dynamic_clearance_m_min"
            ]
        ),
        "every_state_static_clearance": min(
            row["static_clearance_m"] for row in rows
        )
        >= float(
            gates["every_R8_state_static_clearance_m_min"]
        ),
        "translation_invariance": translation_residual
        <= float(
            gates[
                "translation_normalized_geometry_residual_max"
            ]
        ),
    }
    passed = all(checks.values())
    audit = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R10",
        "status": (
            "translation_invariant_progress_geometry_closure_pass"
            if passed
            else "translation_invariant_progress_geometry_closure_fail"
        ),
        "candidate_id": contract["candidate_extension"][
            "candidate_id"
        ],
        "rows": rows,
        "observed_geometry": metrics,
        "maximum_absolute_lateral_offset_m": max(
            row["maximum_absolute_lateral_offset_m"] for row in rows
        ),
        "minimum_terminal_progress_m": min(
            row["terminal_progress_m"] for row in rows
        ),
        "maximum_terminal_absolute_lateral_offset_m": max(
            row["terminal_absolute_lateral_offset_m"] for row in rows
        ),
        "maximum_terminal_absolute_yaw_rad": max(
            row["terminal_absolute_yaw_rad"] for row in rows
        ),
        "minimum_surrogate_dynamic_clearance_m": min(
            row["surrogate_dynamic_clearance_m"] for row in rows
        ),
        "minimum_static_clearance_m": min(
            row["static_clearance_m"] for row in rows
        ),
        "translation_normalized_geometry_residual": translation_residual,
        "legacy_candidate_change_count": legacy_change_count,
        "validation_failures": validation_failures,
        "gate_checks": checks,
        "all_offline_geometry_gates_pass": passed,
        "fresh_non_evidence_preflight_design_authorized": passed,
        "candidate_specific_support_authorized": False,
        "direct_actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "policy_or_ranker_training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    require(passed, "R10 geometry gates failed")
    return {AUDIT: yaml_bytes(audit)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        write_or_check(path, payload, args.check)
    roots = source_roots().values()
    resources = RESOURCES + [
        root / name
        for root in roots
        for name in ("episode_report.yaml", "scene_spec.yaml")
    ] + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in resources
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, args.check)
    print(
        "status={} audit_sha256={} manifest_sha256={}".format(
            yaml.safe_load(outputs[AUDIT])["status"],
            sha(outputs[AUDIT]),
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
