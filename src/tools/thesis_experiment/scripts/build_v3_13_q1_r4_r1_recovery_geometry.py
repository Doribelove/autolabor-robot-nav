#!/usr/bin/env python3
"""Build the runtime-radius-aligned V3-13-Q1-R4-R1 audit."""

import argparse
import copy
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BASE_BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r4_recovery_geometry.py"
)
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r4_r1_recovery_candidate_geometry_contract.yaml"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r4_r1_recovery_geometry"
)
AUDIT = ROOT / "candidate_geometry_audit.yaml"
CONSISTENCY = ROOT / "cross_layer_consistency_audit.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r4_r1_recovery_geometry.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r4_r1_recovery_geometry.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/"
    "test_v3_13_q1_r4_r1_recovery_geometry.py"
)
R4_RESOURCES = [
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_q1_r4_recovery_candidate_geometry_contract.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r4_recovery_geometry/"
        "candidate_geometry_audit.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r4_recovery_geometry/"
        "evidence_manifest.sha256"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r4_recovery_geometry/"
        "independent_review.yaml"
    ),
]


def load_base():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r4_base_builder", BASE_BUILDER
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_or_check(path, payload, check):
    target = W / path
    if check:
        if not target.is_file() or target.read_bytes() != payload:
            raise RuntimeError("output drift: {}".format(path))
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        if target.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
        return
    temporary = target.with_suffix(target.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(target))


def build_outputs():
    base = load_base()
    base.CONTRACT = CONTRACT
    base.ROOT = ROOT
    base.AUDIT = AUDIT
    base.MANIFEST = MANIFEST
    raw = base.build_outputs()
    contract = base.load_yaml(CONTRACT)
    for key, path in (
        ("r4_contract_sha256", R4_RESOURCES[0]),
        ("r4_audit_sha256", R4_RESOURCES[1]),
        ("r4_manifest_sha256", R4_RESOURCES[2]),
        ("r4_review_sha256", R4_RESOURCES[3]),
    ):
        base.require(
            base.sha(base.read(path)) == contract["upstream"][key],
            "{} differs".format(path),
        )

    audit = yaml.safe_load(raw[AUDIT])
    d0 = base.load_yaml(base.D0)
    model = base.load_yaml(base.MODEL)
    snapshot = base.late_snapshot()
    candidate, local = (
        base.recovery_candidate_core.generate_recovery_candidate(
            snapshot, d0, contract, True
        )
    )
    local["hard_verifier"]["combined_actor_ego_radius_m"] = float(
        contract["offline_design_replay"][
            "clone_measurement_combined_radius_m"
        ]
    )
    candidate_id = contract["candidate_extension"]["candidate_id"]
    surrogate = copy.deepcopy(model)
    for heads in surrogate["heads"].values():
        heads[candidate_id] = copy.deepcopy(
            heads["spatial_right_v120"]
        )
    runtime_evaluation = base.multi_actor_core_v1.evaluate_candidate(
        snapshot, candidate, surrogate, local
    )
    gates = contract["offline_geometry_gates"]
    runtime_check = runtime_evaluation[
        "minimum_dynamic_clearance_m"
    ] >= float(
        gates["R2_runtime_radius_surrogate_dynamic_clearance_m_min"]
    )
    audit["stage"] = "V3-13-Q1-R4-R1"
    audit["status"] = (
        "runtime_radius_aligned_recovery_geometry_closure_pass"
        if audit["all_offline_geometry_gates_pass"] and runtime_check
        else "runtime_radius_aligned_recovery_geometry_closure_fail"
    )
    audit["supersedes_unexecuted_candidate_id"] = (
        "recovery_evasive_right_v120"
    )
    audit["runtime_radius_surrogate_evaluation"] = {
        "combined_actor_ego_radius_m": local["hard_verifier"][
            "combined_actor_ego_radius_m"
        ],
        "candidate_specific_support_or_qualification": False,
        "accepted": runtime_evaluation["accepted"],
        "rejection_codes": runtime_evaluation["rejection_codes"],
        "minimum_dynamic_clearance_m": runtime_evaluation[
            "minimum_dynamic_clearance_m"
        ],
        "minimum_static_clearance_m": runtime_evaluation[
            "minimum_static_clearance_m"
        ],
        "minimum_clearance_m": runtime_evaluation[
            "minimum_clearance_m"
        ],
    }
    audit["checks"]["runtime_radius_surrogate_clearance"] = runtime_check
    audit["all_offline_geometry_gates_pass"] = all(
        audit["checks"].values()
    )
    audit["fresh_non_evidence_preflight_design_authorized"] = audit[
        "all_offline_geometry_gates_pass"
    ]
    audit_payload = base.yaml_bytes(audit)

    diagnosis = contract["superseded_design_diagnosis"]
    expected_equivalent = (
        float(diagnosis["r4_planner_surrogate_clearance_m"])
        - float(diagnosis["radius_gap_m"])
    )
    consistency_checks = {
        "R4_bytes_preserved": all(
            base.sha(base.read(path))
            == contract["upstream"][key]
            for key, path in (
                ("r4_contract_sha256", R4_RESOURCES[0]),
                ("r4_audit_sha256", R4_RESOURCES[1]),
                ("r4_manifest_sha256", R4_RESOURCES[2]),
                ("r4_review_sha256", R4_RESOURCES[3]),
            )
        ),
        "radius_gap_arithmetic": abs(
            float(
                diagnosis[
                    "r4_clone_radius_equivalent_surrogate_clearance_m"
                ]
            )
            - expected_equivalent
        )
        <= 1.0e-12,
        "R4_clone_radius_equivalent_is_negative": float(
            diagnosis[
                "r4_clone_radius_equivalent_surrogate_clearance_m"
            ]
        )
        < 0.0,
        "R4_candidate_was_not_executed": diagnosis[
            "r4_candidate_not_executed"
        ]
        is True,
        "fresh_seed_unconsumed": diagnosis[
            "seed7336_not_consumed_by_diagnosis"
        ]
        is True,
    }
    consistency = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R4-D1",
        "status": (
            "cross_layer_clearance_mismatch_audit_pass"
            if all(consistency_checks.values())
            else "cross_layer_clearance_mismatch_audit_fail"
        ),
        "planner_combined_radius_m": diagnosis[
            "r4_planner_combined_radius_m"
        ],
        "clone_measurement_combined_radius_m": diagnosis[
            "clone_measurement_combined_radius_m"
        ],
        "radius_gap_m": diagnosis["radius_gap_m"],
        "R4_planner_surrogate_clearance_m": diagnosis[
            "r4_planner_surrogate_clearance_m"
        ],
        "R4_clone_radius_equivalent_surrogate_clearance_m": diagnosis[
            "r4_clone_radius_equivalent_surrogate_clearance_m"
        ],
        "root_cause": diagnosis["root_cause"],
        "checks": consistency_checks,
        "R4_fresh_preflight_design_authorization_superseded": True,
        "simulation_executed": False,
        "new_seed_consumed": False,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    consistency_payload = base.yaml_bytes(consistency)
    resources = [
        CONTRACT,
        base.D0,
        base.MODEL,
        base.R2_DIAGNOSTIC,
        base.R2_EPISODE,
        base.R3_FREEZE,
        base.CORE,
        BUILDER,
        REVIEWER,
        TEST,
    ] + R4_RESOURCES
    manifest = "".join(
        "{}  {}\n".format(base.sha(base.read(path)), path)
        for path in resources
    ).encode("utf-8")
    manifest += "{}  {}\n".format(
        base.sha(consistency_payload), CONSISTENCY
    ).encode("utf-8")
    manifest += "{}  {}\n".format(
        base.sha(audit_payload), AUDIT
    ).encode("utf-8")
    return {
        CONSISTENCY: consistency_payload,
        AUDIT: audit_payload,
        MANIFEST: manifest,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        write_or_check(path, payload, args.check)
    audit = yaml.safe_load(outputs[AUDIT])
    print(
        "status={} audit_sha256={} manifest_sha256={}".format(
            audit["status"],
            hashlib.sha256(outputs[AUDIT]).hexdigest(),
            hashlib.sha256(outputs[MANIFEST]).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
