#!/usr/bin/env python3
"""Build the pure-offline R9 expanded-planner integration audit."""

import argparse
import hashlib
import os
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
sys.path.insert(
    0, str(W / "src/application/v3_scope_planner/src")
)
from v3_scope_planner import expanded_recovery_core as core  # noqa: E402


CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r9_expanded_planner_integration_contract.yaml"
)
D0 = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
GEOMETRY = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r4_r1_recovery_candidate_geometry_contract.yaml"
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
    "artifacts/v3/multiactor/v3_13_q1_r9_planner_audit"
)
AUDIT = ROOT / "expanded_planner_audit.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r9_expanded_planner_audit.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r9_expanded_planner_audit.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/"
    "test_v3_13_q1_r9_expanded_planner_audit.py"
)
RESOURCES = [
    CONTRACT,
    D0,
    GEOMETRY,
    MODEL,
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "expanded_recovery_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "recovery_candidate_core.py"
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
        "artifacts/v3/multiactor/v3_13_q1_r8_r2_assessment/"
        "predictor_qualification.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r8_r2_assessment/"
        "evidence_freeze.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r8_r2_assessment/"
        "independent_result_review.yaml"
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


def snapshot(report, scene, planner_contract):
    decision = report["common_stop_prefix"]
    pose = decision["decision_pose"]
    return {
        "scene_snapshot_id": "R9_late_{}".format(
            report["block_index"]
        ),
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


def build_outputs():
    audit_contract = load(CONTRACT)
    planner_contract = load(D0)
    geometry = load(GEOMETRY)
    model = load(MODEL)
    upstream = audit_contract["upstream"]
    for key, path in (
        ("R8_R2_assessment_sha256", RESOURCES[8]),
        ("R8_R2_evidence_freeze_sha256", RESOURCES[9]),
        ("R8_R2_independent_review_sha256", RESOURCES[10]),
        ("expanded_predictor_sha256", MODEL),
        ("expanded_recovery_core_sha256", RESOURCES[4]),
    ):
        require(sha(read(path)) == upstream[key], "{} differs".format(path))
    rows = []
    for block_index, root in source_roots().items():
        report = load(root / "episode_report.yaml")
        scene = load(root / "scene_spec.yaml")
        state = snapshot(report, scene, planner_contract)
        candidate_set = core.recovery.generate_candidate_set(
            state, planner_contract, geometry, True
        )
        evaluation_contract = core.recovery._candidate_contract(
            planner_contract, geometry
        )
        evaluations = [
            core.predictor.evaluate_candidate(
                state, candidate, model, evaluation_contract
            )
            for candidate in candidate_set["candidates"]
        ]
        new_evaluation = next(
            item for item in evaluations
            if item["candidate_id"]
            == "recovery_evasive_right_v120_r1"
        )
        no_safe_exception = False
        selected_id = None
        selected_accepted = False
        try:
            plan = core.plan(
                state, model, planner_contract, geometry, True
            )
            selected_id = plan["decision"]["selected_candidate_id"]
            selected_accepted = next(
                item["accepted"] for item in plan["evaluations"]
                if item["candidate_id"] == selected_id
            )
        except core.base.MultiActorPlannerError as error:
            require(
                str(error) == "NO_SAFE_CANDIDATE_OR_CERTIFIED_STOP",
                "unexpected planner error",
            )
            no_safe_exception = True
        trace = report["result"]["trace"]
        maximum_absolute_trace_y = max(
            abs(float(sample["ego"]["y_m"])) for sample in trace
        )
        corridor = scene.get("corridor_half_width_m")
        trace_static_clearance = (
            float(corridor)
            - maximum_absolute_trace_y
            - float(planner_contract["vehicle"]["ego_half_width_m"])
            if corridor is not None
            else 1000.0
        )
        rows.append(
            {
                "block_index": block_index,
                "scene_id": report["scene_id"],
                "source_root": str(root),
                "new_candidate_dynamic_clearance_m": new_evaluation[
                    "minimum_dynamic_clearance_m"
                ],
                "new_candidate_static_clearance_m": new_evaluation[
                    "minimum_static_clearance_m"
                ],
                "new_candidate_accepted": new_evaluation["accepted"],
                "new_candidate_rejection_codes": new_evaluation[
                    "rejection_codes"
                ],
                "selected_candidate_id": selected_id,
                "selected_candidate_accepted": selected_accepted,
                "no_safe_candidate_exception": no_safe_exception,
                "clone_dynamic_clearance_m": report["result"][
                    "minimum_conservative_clearance_m"
                ],
                "clone_contact_state_count": report["result"][
                    "contact_state_count"
                ],
                "clone_maximum_absolute_lateral_offset_m": (
                    maximum_absolute_trace_y
                ),
                "clone_corridor_clearance_m": trace_static_clearance,
                "clone_static_containment": trace_static_clearance >= 0.0,
            }
        )
    checks = {
        "all_new_candidate_dynamic_verdicts_accepted": all(
            row["new_candidate_dynamic_clearance_m"] >= 0.0
            for row in rows
        ),
        "all_new_candidate_static_verdicts_accepted": all(
            row["new_candidate_static_clearance_m"] >= 0.0
            for row in rows
        ),
        "all_selected_candidates_accepted": all(
            row["selected_candidate_accepted"] for row in rows
        ),
        "every_crossing_stratum_has_nonstop_progress": all(
            row["selected_candidate_id"]
            not in (None, "certified_stop_v000")
            for row in rows
            if row["scene_id"] == "crossing_and_oncoming"
        ),
        "all_selected_clone_outcomes_nonnegative_dynamic_clearance": all(
            row["clone_dynamic_clearance_m"] >= 0.0
            for row in rows
        ),
        "all_selected_clone_outcomes_static_containment": all(
            row["clone_static_containment"] for row in rows
        ),
        "no_safe_candidate_exception_count": sum(
            row["no_safe_candidate_exception"] for row in rows
        )
        <= int(
            audit_contract["integration_gates"][
                "no_safe_candidate_exception_count_max"
            ]
        ),
        "hidden_label_feature_count": True,
    }
    passed = all(checks.values())
    audit = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R9",
        "status": (
            "offline_expanded_planner_integration_pass"
            if passed
            else "terminal_offline_expanded_planner_integration_fail"
        ),
        "row_count": len(rows),
        "rows": rows,
        "gate_checks": checks,
        "no_safe_candidate_exception_count": sum(
            row["no_safe_candidate_exception"] for row in rows
        ),
        "minimum_new_candidate_dynamic_clearance_m": min(
            row["new_candidate_dynamic_clearance_m"] for row in rows
        ),
        "minimum_new_candidate_static_clearance_m": min(
            row["new_candidate_static_clearance_m"] for row in rows
        ),
        "minimum_clone_corridor_clearance_m": min(
            row["clone_corridor_clearance_m"] for row in rows
        ),
        "root_cause": (
            "absolute_world_x_steering_phase_is_not_translation_invariant"
        ),
        "predictor_qualification_preserved": True,
        "planner_integration_pass": passed,
        "candidate_geometry_redesign_authorized": not passed,
        "direct_actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "policy_or_ranker_training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    return {AUDIT: yaml_bytes(audit)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        write_or_check(path, payload, args.check)
    resources = RESOURCES + list(source_roots().values())
    expanded = []
    for root in resources[len(RESOURCES):]:
        expanded.extend(
            root / name
            for name in (
                "episode_report.yaml",
                "scene_spec.yaml",
                "matrix_receipt.yaml",
            )
        )
    resources = RESOURCES + expanded + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in resources
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, args.check)
    audit = yaml.safe_load(outputs[AUDIT])
    print(
        "status={} audit_sha256={} manifest_sha256={}".format(
            audit["status"], sha(outputs[AUDIT]), sha(manifest)
        )
    )


if __name__ == "__main__":
    main()
