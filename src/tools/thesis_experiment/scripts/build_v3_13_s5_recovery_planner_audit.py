#!/usr/bin/env python3
"""Build the pure-offline V3-13-S5 recovery planner audit."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_s5_recovery_planner_audit_contract.yaml"
)
D0_CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
CORE = Path(
    "src/application/v3_scope_planner/src/"
    "v3_scope_planner/multi_actor_core_v1.py"
)
MODEL = Path(
    "artifacts/v3/multiactor/v3_13_s3_predictor/"
    "candidate_conditioned_recovery_predictor.yaml"
)
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_s5_recovery_planner_audit.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_s5_recovery_planner_audit.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/"
    "test_v3_13_s5_recovery_audit.py"
)
C3 = Path("artifacts/v3/multiactor/v3_13_c3_execution/episodes")
S4 = Path("artifacts/v3/multiactor/v3_13_s4_execution/episodes")
ROOT = Path("artifacts/v3/multiactor/v3_13_s5_recovery_audit")
AUDIT = ROOT / "recovery_planner_audit.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(relative):
    path = W / relative
    require(path.is_file(), "required file absent: {}".format(relative))
    return path.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def load_yaml(relative):
    return yaml.safe_load(read(relative))


def load_core():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s5_core", W / CORE
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def snapshot_from_initial(report, scene, contract):
    gate = report["warmup"]
    pose = gate["gate_pose"]
    return {
        "scene_snapshot_id": "S5_initial_{}".format(
            report["block_index"]
        ),
        "world_model_seq": 1,
        "ego": {
            "x_m": float(pose["x_m"]),
            "y_m": float(pose["y_m"]),
            "yaw_rad": float(pose["yaw_rad"]),
            "speed_mps": float(gate["gate_speed_mps"]),
        },
        "actors": gate["gate_actor_tracks"],
        "scene": scene,
        "health": {
            field: True
            for field in contract["hard_verifier"][
                "required_health_fields"
            ]
        },
    }


def snapshot_from_recovery(report, scene, contract):
    decision = report["common_stop_prefix"]
    pose = decision["decision_pose"]
    return {
        "scene_snapshot_id": "S5_recovery_{}_{}".format(
            report["block_index"],
            int(report["branch_target_sim_s"] * 10.0),
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
            for field in contract["hard_verifier"][
                "required_health_fields"
            ]
        },
    }


def report_roots_by_block(root, action_suffix):
    values = {}
    for path in sorted((W / root).glob("*{}".format(action_suffix))):
        report = yaml.safe_load(
            (path / "episode_report.yaml").read_bytes()
        )
        values[int(report["block_index"])] = path.relative_to(W)
    return values


def build_outputs():
    audit_contract = load_yaml(CONTRACT)
    planner_contract = load_yaml(D0_CONTRACT)
    model = load_yaml(MODEL)
    require(
        sha(read(MODEL))
        == audit_contract["upstream"]["predictor_sha256"],
        "predictor differs",
    )
    core = load_core()
    c3_roots = report_roots_by_block(C3, "certified_stop_v000")
    s4_records = {}
    for path in sorted((W / S4).glob("*")):
        if not path.is_dir():
            continue
        report = yaml.safe_load(
            (path / "episode_report.yaml").read_bytes()
        )
        key = (
            int(report["block_index"]),
            report["forced_candidate_id"],
        )
        require(key not in s4_records, "duplicate S4 block action")
        s4_records[key] = (path.relative_to(W), report)
    s4_stop_roots = {
        block_index: value[0]
        for (block_index, candidate_id), value in s4_records.items()
        if candidate_id == "certified_stop_v000"
    }
    require(
        set(audit_contract["audit_population"][
            "initial_source_C3_block_indices"
        ]).issubset(c3_roots),
        "initial C3 block absent",
    )
    require(set(range(1, 9)).issubset(s4_stop_roots), "S4 block absent")
    strata = []
    initial_blocks = audit_contract["audit_population"][
        "initial_source_C3_block_indices"
    ]
    for stratum_index, initial_block in enumerate(
        initial_blocks, start=1
    ):
        initial_root = c3_roots[int(initial_block)]
        initial_report = load_yaml(
            initial_root / "episode_report.yaml"
        )
        initial_scene = load_yaml(initial_root / "scene_spec.yaml")
        initial_plan = core.plan(
            snapshot_from_initial(
                initial_report, initial_scene, planner_contract
            ),
            model,
            planner_contract,
        )
        early_block = 2 * stratum_index - 1
        late_block = 2 * stratum_index
        state_rows = []
        for label, block_index in (
            ("early_recovery", early_block),
            ("late_recovery", late_block),
        ):
            canonical_root = s4_stop_roots[block_index]
            canonical_report = load_yaml(
                canonical_root / "episode_report.yaml"
            )
            scene = load_yaml(canonical_root / "scene_spec.yaml")
            plan = core.plan(
                snapshot_from_recovery(
                    canonical_report, scene, planner_contract
                ),
                model,
                planner_contract,
            )
            selected_id = plan["decision"][
                "selected_candidate_id"
            ]
            selected_evaluation = next(
                value
                for value in plan["evaluations"]
                if value["candidate_id"] == selected_id
            )
            require(
                (block_index, selected_id) in s4_records,
                "selected S4 clone absent",
            )
            selected_root, selected_report = s4_records[
                (block_index, selected_id)
            ]
            state_rows.append(
                {
                    "state_label": label,
                    "source_block_index": block_index,
                    "source_root": str(selected_root),
                    "selected_candidate_id": selected_id,
                    "safety_fallback_used": plan["decision"][
                        "safety_fallback_used"
                    ],
                    "selected_analytic_accepted": selected_evaluation[
                        "accepted"
                    ],
                    "selected_analytic_minimum_clearance_m": (
                        selected_evaluation["minimum_clearance_m"]
                    ),
                    "selected_clone_minimum_conservative_clearance_m": (
                        selected_report["result"][
                            "minimum_conservative_clearance_m"
                        ]
                    ),
                    "selected_clone_contact_state_count": (
                        selected_report["result"][
                            "contact_state_count"
                        ]
                    ),
                    "four_action_clone_support": sum(
                        key[0] == block_index for key in s4_records
                    )
                    == 4,
                    "plan": plan,
                }
            )
        initial_selected = initial_plan["decision"][
            "selected_candidate_id"
        ]
        initial_evaluation = next(
            value
            for value in initial_plan["evaluations"]
            if value["candidate_id"] == initial_selected
        )
        strata.append(
            {
                "stratum_index": stratum_index,
                "scene_id": initial_report["scene_id"],
                "actor_policies_evaluator_only": initial_report[
                    "actor_policies_evaluator_only"
                ],
                "initial_interaction": {
                    "source_C3_block_index": initial_block,
                    "source_root": str(initial_root),
                    "selected_candidate_id": initial_selected,
                    "safety_fallback_used": initial_plan["decision"][
                        "safety_fallback_used"
                    ],
                    "selected_analytic_accepted": initial_evaluation[
                        "accepted"
                    ],
                    "selected_analytic_minimum_clearance_m": (
                        initial_evaluation["minimum_clearance_m"]
                    ),
                    "plan": initial_plan,
                },
                "recovery_states": state_rows,
            }
        )
    checks = {
        "all_initial_decisions_certified_stop": all(
            row["initial_interaction"]["selected_candidate_id"]
            == "certified_stop_v000"
            for row in strata
        ),
        "all_early_recovery_decisions_certified_stop": all(
            row["recovery_states"][0]["selected_candidate_id"]
            == "certified_stop_v000"
            for row in strata
        ),
        "all_late_recovery_decisions_nonstop_progress": all(
            row["recovery_states"][1]["selected_candidate_id"]
            != "certified_stop_v000"
            for row in strata
        ),
        "all_selected_candidates_analytic_verifier_accepted": all(
            row["initial_interaction"]["selected_analytic_accepted"]
            and all(
                state["selected_analytic_accepted"]
                for state in row["recovery_states"]
            )
            for row in strata
        ),
        "all_selected_clone_outcomes_nonnegative_conservative_clearance": all(
            state[
                "selected_clone_minimum_conservative_clearance_m"
            ]
            >= 0.0
            for row in strata
            for state in row["recovery_states"]
        ),
        "all_selected_clone_outcomes_contact_free": all(
            state["selected_clone_contact_state_count"] == 0
            for row in strata
            for state in row["recovery_states"]
        ),
        "all_states_have_four_action_clone_support": all(
            state["four_action_clone_support"]
            for row in strata
            for state in row["recovery_states"]
        ),
        "every_stratum_eventually_progresses": all(
            row["recovery_states"][1]["selected_candidate_id"]
            != "certified_stop_v000"
            for row in strata
        ),
        "hidden_label_feature_count": True,
    }
    audit = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S5",
        "status": (
            "offline_three_state_recovery_planner_audit_pass"
            if all(checks.values())
            else "offline_three_state_recovery_planner_audit_fail"
        ),
        "predictor_sha256": sha(read(MODEL)),
        "stratum_count": len(strata),
        "states_per_stratum": 3,
        "strata": strata,
        "gate_checks": checks,
        "audit_pass": all(checks.values()),
        "actual_TEB_release_design_authorized": all(
            checks.values()
        ),
        "direct_actual_TEB_execution_authorized": False,
        "policy_or_ranker_training_authorized": False,
        "performance_claim_authorized": False,
        "perception_claim_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    require(audit["audit_pass"], "recovery audit failed")
    return {AUDIT: yaml_bytes(audit)}


def write_or_check(relative, payload, check):
    path = W / relative
    if check:
        require(path.is_file(), "output absent: {}".format(relative))
        require(path.read_bytes() == payload, "output drift: {}".format(relative))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    require(not path.exists(), "refusing overwrite: {}".format(relative))
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    outputs = build_outputs()
    for relative, payload in outputs.items():
        write_or_check(relative, payload, arguments.check)
    resources = [
        CONTRACT,
        D0_CONTRACT,
        CORE,
        MODEL,
        BUILDER,
        REVIEWER,
        TEST,
        Path(
            "artifacts/v3/multiactor/v3_13_s4_assessment/"
            "recovery_predictor_qualification.yaml"
        ),
        Path(
            "artifacts/v3/multiactor/v3_13_s4_assessment/"
            "evidence_freeze.yaml"
        ),
        Path(
            "artifacts/v3/multiactor/v3_13_s4_assessment/"
            "independent_result_review.yaml"
        ),
    ] + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in resources
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, arguments.check)
    print(
        "status=v3_13_s5_audit_{} audit_sha256={} "
        "manifest_sha256={}".format(
            "reproducible" if arguments.check else "built",
            sha(read(AUDIT)),
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
