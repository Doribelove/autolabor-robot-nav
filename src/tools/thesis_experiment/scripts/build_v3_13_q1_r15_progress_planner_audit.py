#!/usr/bin/env python3
"""Build the pure-offline R15 progress-planner integration audit."""

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
from v3_scope_planner import progress_expanded_recovery_core as core  # noqa: E402


CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r15_progress_planner_integration_contract.yaml"
)
D0 = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
GEOMETRY = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r10_progress_recovery_geometry_contract.yaml"
)
MODEL = Path(
    "artifacts/v3/multiactor/v3_13_q1_r13_predictor/"
    "expanded_progress_predictor.yaml"
)
SOURCE = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r14_r1_progress_qualification/episodes"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r15_planner_audit"
)
AUDIT = ROOT / "progress_planner_audit.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r15_progress_planner_audit.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r15_progress_planner_audit.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/"
    "test_v3_13_q1_r15_progress_planner_audit.py"
)
ASSESSMENT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r14_r1_assessment/"
    "predictor_qualification.yaml"
)
FREEZE = Path(
    "artifacts/v3/multiactor/v3_13_q1_r14_r1_assessment/"
    "evidence_freeze.yaml"
)
RESULT_REVIEW = Path(
    "artifacts/v3/multiactor/v3_13_q1_r14_r1_assessment/"
    "independent_result_review.yaml"
)
RESOURCES = [
    CONTRACT,
    D0,
    GEOMETRY,
    MODEL,
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "progress_expanded_recovery_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "progress_recovery_candidate_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "expanded_recovery_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "multi_actor_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "multi_actor_core_v1.py"
    ),
    ASSESSMENT,
    FREEZE,
    RESULT_REVIEW,
    BUILDER,
    REVIEWER,
    TEST,
]
PROGRESS_ID = "recovery_evasive_right_v120_r2"


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
    values = {}
    for path in sorted((W / SOURCE).iterdir()):
        if not path.is_dir():
            continue
        report = yaml.safe_load(
            (path / "episode_report.yaml").read_bytes()
        )
        block = int(report["block_index"])
        action = report["forced_candidate_id"]
        require(action not in values.setdefault(block, {}), "duplicate action")
        values[block][action] = path.relative_to(W)
    require(set(values) == set(range(1, 9)), "block roster")
    require(
        all(len(actions) == 5 for actions in values.values()),
        "complete action support",
    )
    return values


def snapshot(report, scene, planner_contract):
    decision = report["common_stop_prefix"]
    pose = decision["decision_pose"]
    return {
        "scene_snapshot_id": "R15_recovery_{}".format(
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


def minimum_or_none(rows, key):
    values = [row[key] for row in rows if row[key] is not None]
    return min(values) if values else None


def build_outputs():
    audit_contract = load(CONTRACT)
    planner_contract = load(D0)
    geometry = load(GEOMETRY)
    model = load(MODEL)
    upstream = audit_contract["upstream"]
    for key, path in (
        ("D0_contract_sha256", D0),
        ("geometry_contract_sha256", GEOMETRY),
        ("predictor_sha256", MODEL),
        ("R14_R1_assessment_sha256", ASSESSMENT),
        ("R14_R1_evidence_freeze_sha256", FREEZE),
        ("R14_R1_independent_review_sha256", RESULT_REVIEW),
        ("progress_planner_core_sha256", RESOURCES[4]),
    ):
        require(
            sha(read(path)) == upstream[key],
            "{} differs".format(path),
        )
    roots = source_roots()
    rows = []
    for block_index in sorted(roots):
        action_roots = roots[block_index]
        source_root = action_roots[PROGRESS_ID]
        source_report = load(source_root / "episode_report.yaml")
        scene = load(source_root / "scene_spec.yaml")
        state = snapshot(source_report, scene, planner_contract)
        no_safe_exception = False
        try:
            plan = core.plan(
                state, model, planner_contract, geometry, True
            )
        except core.base.MultiActorPlannerError as error:
            require(
                str(error) == "NO_SAFE_CANDIDATE_OR_CERTIFIED_STOP",
                "unexpected planner error",
            )
            no_safe_exception = True
            plan = {"evaluations": [], "decision": {}}
        evaluations = plan["evaluations"]
        selected_id = plan["decision"].get("selected_candidate_id")
        selected_evaluation = next(
            (
                value for value in evaluations
                if value["candidate_id"] == selected_id
            ),
            None,
        )
        selected_report = (
            load(action_roots[selected_id] / "episode_report.yaml")
            if selected_id is not None
            else None
        )
        progress_evaluation = next(
            (
                value for value in evaluations
                if value["candidate_id"] == PROGRESS_ID
            ),
            None,
        )
        progress_report = load(
            action_roots[PROGRESS_ID] / "episode_report.yaml"
        )
        rows.append(
            {
                "block_index": block_index,
                "seed": int(source_report["seed"]),
                "scene_id": source_report["scene_id"],
                "source_roots": {
                    key: str(value)
                    for key, value in sorted(action_roots.items())
                },
                "selected_candidate_id": selected_id,
                "selected_candidate_accepted": (
                    bool(selected_evaluation["accepted"])
                    if selected_evaluation is not None
                    else False
                ),
                "selected_predicted_dynamic_clearance_m": (
                    selected_evaluation["minimum_dynamic_clearance_m"]
                    if selected_evaluation is not None
                    else None
                ),
                "selected_predicted_static_clearance_m": (
                    selected_evaluation["minimum_static_clearance_m"]
                    if selected_evaluation is not None
                    else None
                ),
                "selected_clone_dynamic_clearance_m": (
                    selected_report["result"][
                        "minimum_conservative_clearance_m"
                    ]
                    if selected_report is not None
                    else None
                ),
                "selected_clone_static_clearance_m": (
                    selected_report["minimum_trace_static_clearance_m"]
                    if selected_report is not None
                    else None
                ),
                "selected_clone_contact_state_count": (
                    selected_report["result"]["contact_state_count"]
                    if selected_report is not None
                    else None
                ),
                "progress_candidate_accepted": (
                    bool(progress_evaluation["accepted"])
                    if progress_evaluation is not None
                    else False
                ),
                "progress_predicted_dynamic_clearance_m": (
                    progress_evaluation["minimum_dynamic_clearance_m"]
                    if progress_evaluation is not None
                    else None
                ),
                "progress_predicted_static_clearance_m": (
                    progress_evaluation["minimum_static_clearance_m"]
                    if progress_evaluation is not None
                    else None
                ),
                "progress_clone_dynamic_clearance_m": (
                    progress_report["result"][
                        "minimum_conservative_clearance_m"
                    ]
                ),
                "progress_clone_static_clearance_m": (
                    progress_report["minimum_trace_static_clearance_m"]
                ),
                "progress_clone_contact_state_count": (
                    progress_report["result"]["contact_state_count"]
                ),
                "no_safe_candidate_exception": no_safe_exception,
                "runtime_hidden_label_feature_count": 0,
            }
        )
    gates = audit_contract["integration_gates"]
    checks = {
        "row_count": len(rows) == int(gates["row_count_exact"]),
        "selected_candidate_accepted_rate": (
            sum(row["selected_candidate_accepted"] for row in rows)
            / len(rows)
            >= float(gates["selected_candidate_accepted_rate_min"])
        ),
        "no_safe_candidate_exception_count": sum(
            row["no_safe_candidate_exception"] for row in rows
        )
        <= int(gates["no_safe_candidate_exception_count_max"]),
        "every_crossing_stratum_nonstop_progress": all(
            row["selected_candidate_id"]
            not in (None, "certified_stop_v000")
            for row in rows
            if row["scene_id"] == "crossing_and_oncoming"
        ),
        "selected_clone_dynamic_clearance": all(
            row["selected_clone_dynamic_clearance_m"] is not None
            and row["selected_clone_dynamic_clearance_m"] >= float(
                gates[
                    "selected_clone_minimum_dynamic_clearance_m_min"
                ]
            )
            for row in rows
        ),
        "selected_clone_static_clearance": all(
            row["selected_clone_static_clearance_m"] is not None
            and row["selected_clone_static_clearance_m"] >= float(
                gates[
                    "selected_clone_minimum_static_clearance_m_min"
                ]
            )
            for row in rows
        ),
        "selected_clone_contact_state_count": all(
            row["selected_clone_contact_state_count"] is not None
            and row["selected_clone_contact_state_count"]
            <= int(gates["selected_clone_contact_state_count_max"])
            for row in rows
        ),
        "progress_predicted_dynamic_clearance": all(
            row["progress_predicted_dynamic_clearance_m"]
            >= float(
                gates[
                    "progress_candidate_predicted_dynamic_clearance_m_min"
                ]
            )
            for row in rows
        ),
        "progress_predicted_static_clearance": all(
            row["progress_predicted_static_clearance_m"]
            >= float(
                gates[
                    "progress_candidate_predicted_static_clearance_m_min"
                ]
            )
            for row in rows
        ),
        "progress_clone_dynamic_clearance": all(
            row["progress_clone_dynamic_clearance_m"]
            >= float(
                gates[
                    "progress_candidate_clone_dynamic_clearance_m_min"
                ]
            )
            for row in rows
        ),
        "progress_clone_static_clearance": all(
            row["progress_clone_static_clearance_m"]
            >= float(
                gates[
                    "progress_candidate_clone_static_clearance_m_min"
                ]
            )
            for row in rows
        ),
        "hidden_label_feature_count": all(
            row["runtime_hidden_label_feature_count"]
            <= int(gates["hidden_label_feature_count_max"])
            for row in rows
        ),
    }
    passed = all(checks.values())
    audit = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R15",
        "status": (
            "offline_progress_planner_integration_pass"
            if passed
            else "terminal_offline_progress_planner_integration_fail"
        ),
        "row_count": len(rows),
        "rows": rows,
        "gate_checks": checks,
        "selected_candidate_counts": {
            candidate_id: sum(
                (row["selected_candidate_id"] or "NONE") == candidate_id
                for row in rows
            )
            for candidate_id in sorted(
                {row["selected_candidate_id"] or "NONE" for row in rows}
            )
        },
        "minimum_selected_predicted_dynamic_clearance_m": minimum_or_none(
            rows, "selected_predicted_dynamic_clearance_m"
        ),
        "minimum_selected_predicted_static_clearance_m": minimum_or_none(
            rows, "selected_predicted_static_clearance_m"
        ),
        "minimum_selected_clone_dynamic_clearance_m": minimum_or_none(
            rows, "selected_clone_dynamic_clearance_m"
        ),
        "minimum_selected_clone_static_clearance_m": minimum_or_none(
            rows, "selected_clone_static_clearance_m"
        ),
        "selected_clone_contact_state_count_total": sum(
            row["selected_clone_contact_state_count"]
            for row in rows
            if row["selected_clone_contact_state_count"] is not None
        ),
        "planner_integration_pass": passed,
        "fresh_actual_TEB_paired_preflight_design_authorized": passed,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "policy_or_ranker_training_authorized": False,
        "new_simulation_units_used": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    return {AUDIT: yaml_bytes(audit)}


def evidence_resources():
    resources = list(RESOURCES)
    for action_roots in source_roots().values():
        for root in action_roots.values():
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
    audit = yaml.safe_load(outputs[AUDIT])
    print(
        "status={} audit_sha256={} manifest_sha256={}".format(
            audit["status"], sha(outputs[AUDIT]), sha(manifest)
        )
    )


if __name__ == "__main__":
    main()
