#!/usr/bin/env python3
"""Build the zero-authority V3-13-Q1-R4 recovery geometry audit."""

import argparse
import copy
import hashlib
import importlib.util
import math
import os
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
sys.path.insert(
    0, str(W / "src/application/v3_scope_planner/src")
)
from v3_scope_planner import multi_actor_core  # noqa: E402
from v3_scope_planner import multi_actor_core_v1  # noqa: E402
from v3_scope_planner import recovery_candidate_core  # noqa: E402

CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r4_recovery_candidate_geometry_contract.yaml"
)
D0 = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
MODEL = Path(
    "artifacts/v3/multiactor/v3_13_s3_predictor/"
    "candidate_conditioned_recovery_predictor.yaml"
)
R2_DIAGNOSTIC = Path(
    "artifacts/v3/multiactor/v3_13_q1_r2_diagnostic/"
    "offline_root_cause.yaml"
)
R2_EPISODE = Path(
    "artifacts/v3/multiactor/v3_13_q1_r2_diagnostic/episodes/"
    "001_01_7331_v3_13_analytic/episode_report.yaml"
)
R3_FREEZE = Path(
    "artifacts/v3/multiactor/v3_13_q1_r3_assessment/"
    "evidence_freeze.yaml"
)
CORE = Path(
    "src/application/v3_scope_planner/src/v3_scope_planner/"
    "recovery_candidate_core.py"
)
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r4_recovery_geometry.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r4_recovery_geometry.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/"
    "test_v3_13_q1_r4_recovery_geometry.py"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r4_recovery_geometry"
)
AUDIT = ROOT / "candidate_geometry_audit.yaml"
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


def load_yaml(relative):
    return yaml.safe_load(read(relative))


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def write_or_check(relative, payload, check):
    path = W / relative
    if check:
        require(path.is_file(), "output absent: {}".format(relative))
        require(path.read_bytes() == payload, "output drift: {}".format(relative))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        require(path.read_bytes() == payload, "refusing overwrite: {}".format(relative))
        return
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def late_snapshot():
    report = load_yaml(R2_EPISODE)
    return next(
        row["snapshot"]
        for row in report["planning_diagnostics"]
        if row["label"] == "late_recovery"
    )


def build_outputs():
    contract = load_yaml(CONTRACT)
    d0 = load_yaml(D0)
    model = load_yaml(MODEL)
    for key, path in (
        ("d0_contract_sha256", D0),
        ("s3_predictor_sha256", MODEL),
        ("r2_diagnostic_sha256", R2_DIAGNOSTIC),
        ("r2_episode_sha256", R2_EPISODE),
        ("r3_negative_freeze_sha256", R3_FREEZE),
    ):
        require(
            sha(read(path)) == contract["upstream"][key],
            "{} differs".format(path),
        )
    for key, path in (
        ("source_urdf_sha256", Path(contract["hardware_envelope"][
            "source_urdf_path"
        ])),
        ("source_gazebo_xacro_sha256", Path(contract[
            "hardware_envelope"
        ]["source_gazebo_xacro_path"])),
    ):
        require(
            sha(read(path)) == contract["hardware_envelope"][key],
            "{} differs".format(path),
        )
    require(
        contract["authority"]["ros_or_gazebo_execution_authorized"] is False,
        "unexpected simulation authority",
    )
    require(
        contract["authority"]["training_authorized"] is False,
        "unexpected training authority",
    )

    snapshot = late_snapshot()
    legacy = multi_actor_core.generate_candidate_set(snapshot, d0)
    initial = recovery_candidate_core.generate_candidate_set(
        snapshot, d0, contract, False
    )
    recovery = recovery_candidate_core.generate_candidate_set(
        snapshot, d0, contract, True
    )
    candidate, local = (
        recovery_candidate_core.generate_recovery_candidate(
            snapshot, d0, contract, True
        )
    )
    legacy_changed = sum(
        left != right
        for left, right in zip(
            legacy["candidates"], recovery["candidates"][:4]
        )
    )
    failures = list(
        multi_actor_core.validate_candidate(candidate, local)
    )
    points = candidate["points"]
    observed = {
        "maximum_acceleration_mps2": max(
            point["longitudinal_acceleration_mps2"]
            for point in points
        ),
        "maximum_steering_angle_rad": max(
            abs(point["steering_angle_rad"]) for point in points
        ),
        "maximum_steering_rate_radps": max(
            abs(point["steering_rate_radps"]) for point in points
        ),
        "maximum_curvature_inv_m": max(
            abs(point["curvature_inv_m"]) for point in points
        ),
        "maximum_absolute_lateral_offset_m": max(
            abs(point["y_m"]) for point in points
        ),
        "terminal_progress_m": (
            points[-1]["x_m"] - points[0]["x_m"]
        ),
        "terminal_lateral_offset_m": (
            points[-1]["y_m"] - points[0]["y_m"]
        ),
    }

    surrogate = copy.deepcopy(model)
    candidate_id = contract["candidate_extension"]["candidate_id"]
    for heads in surrogate["heads"].values():
        heads[candidate_id] = copy.deepcopy(
            heads["spatial_right_v120"]
        )
    evaluation = multi_actor_core_v1.evaluate_candidate(
        snapshot, candidate, surrogate, local
    )
    gates = contract["offline_geometry_gates"]
    checks = {
        "exactly_one_candidate_appended": (
            recovery["extension_count"] == 1
            and len(recovery["candidates"]) == 5
        ),
        "legacy_candidate_digests_unchanged": (
            legacy_changed
            <= int(gates["legacy_candidate_digest_change_count_max"])
        ),
        "initial_context_has_no_extension": (
            initial["extension_count"]
            <= int(gates["initial_context_extension_count_max"])
            and initial["candidates"] == legacy["candidates"]
        ),
        "kinematic_validation": (
            len(failures)
            <= int(gates["kinematic_validation_failure_count_max"])
        ),
        "acceleration_bound": observed["maximum_acceleration_mps2"]
        <= float(gates["maximum_acceleration_mps2_max"]) + 1.0e-12,
        "steering_angle_bound": observed[
            "maximum_steering_angle_rad"
        ]
        <= float(gates["maximum_steering_angle_rad_max"]) + 1.0e-12,
        "steering_rate_bound": observed[
            "maximum_steering_rate_radps"
        ]
        <= float(gates["maximum_steering_rate_radps_max"]) + 1.0e-12,
        "curvature_bound": observed["maximum_curvature_inv_m"]
        <= float(gates["maximum_curvature_inv_m_max"]) + 1.0e-12,
        "M2_physical_curvature_bound": observed[
            "maximum_curvature_inv_m"
        ]
        <= recovery_candidate_core.physical_curvature_limit(contract)
        + 1.0e-12,
        "surrogate_dynamic_clearance": evaluation[
            "minimum_dynamic_clearance_m"
        ]
        >= float(gates["R2_surrogate_dynamic_clearance_m_min"]),
        "corridor_static_clearance": evaluation[
            "minimum_static_clearance_m"
        ]
        >= float(gates["R2_corridor_static_clearance_m_min"]),
        "safety_and_ranker_nonfactors_unchanged": (
            local["hard_verifier"] == d0["hard_verifier"]
            and local["analytic_ranker"] == d0["analytic_ranker"]
        ),
    }
    audit = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R4",
        "status": (
            "offline_recovery_candidate_geometry_closure_pass"
            if all(checks.values())
            else "offline_recovery_candidate_geometry_closure_fail"
        ),
        "source_snapshot_id": snapshot["scene_snapshot_id"],
        "candidate_id": candidate_id,
        "candidate_digest": candidate["candidate_digest"],
        "legacy_candidate_set_digest": legacy[
            "candidate_set_digest"
        ],
        "recovery_candidate_set_digest": recovery[
            "candidate_set_digest"
        ],
        "legacy_candidate_change_count": legacy_changed,
        "validation_failures": failures,
        "observed_geometry": observed,
        "hardware_curvature_limit_inv_m": (
            recovery_candidate_core.physical_curvature_limit(contract)
        ),
        "R2_surrogate_evaluation": {
            "surrogate_head": "S3_assertive_spatial_right_v120",
            "candidate_specific_support_or_qualification": False,
            "accepted": evaluation["accepted"],
            "rejection_codes": evaluation["rejection_codes"],
            "minimum_dynamic_clearance_m": evaluation[
                "minimum_dynamic_clearance_m"
            ],
            "minimum_static_clearance_m": evaluation[
                "minimum_static_clearance_m"
            ],
            "minimum_clearance_m": evaluation[
                "minimum_clearance_m"
            ],
        },
        "checks": checks,
        "all_offline_geometry_gates_pass": all(checks.values()),
        "simulation_evidence_created": False,
        "new_seed_consumed": False,
        "new_candidate_predictor_qualified": False,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "policy_or_ranker_training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    audit_payload = yaml_bytes(audit)
    resources = [
        CONTRACT,
        D0,
        MODEL,
        R2_DIAGNOSTIC,
        R2_EPISODE,
        R3_FREEZE,
        Path(contract["hardware_envelope"]["source_urdf_path"]),
        Path(contract["hardware_envelope"][
            "source_gazebo_xacro_path"
        ]),
        CORE,
        BUILDER,
        REVIEWER,
        TEST,
    ]
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in resources
    ).encode("utf-8")
    return {
        AUDIT: audit_payload,
        MANIFEST: manifest,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for relative, payload in outputs.items():
        write_or_check(relative, payload, args.check)
    audit = yaml.safe_load(outputs[AUDIT])
    print(
        "status={} audit_sha256={} manifest_sha256={}".format(
            audit["status"],
            sha(outputs[AUDIT]),
            sha(outputs[MANIFEST]),
        )
    )


if __name__ == "__main__":
    main()
