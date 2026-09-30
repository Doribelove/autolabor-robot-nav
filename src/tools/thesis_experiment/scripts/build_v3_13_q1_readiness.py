#!/usr/bin/env python3
"""Build immutable preflight or qualification readiness for V3-13-Q1."""

import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_actual_teb_multiactor_qualification_contract.yaml"
)
D0 = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
PREDICTOR = Path(
    "artifacts/v3/multiactor/v3_13_s3_predictor/"
    "candidate_conditioned_recovery_predictor.yaml"
)
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_readiness.py"
)
READINESS_REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_readiness.py"
)
RUNNER = Path(
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_actual_teb_matrix.py"
)
PREFLIGHT_REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_preflight_result.py"
)
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_actual_teb_qualification.py"
)
RESULT_REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_actual_teb_result.py"
)
COMMON_RESOURCES = [
    CONTRACT,
    D0,
    PREDICTOR,
    BUILDER,
    READINESS_REVIEWER,
    RUNNER,
    PREFLIGHT_REVIEWER,
    Path(
        "src/tools/thesis_experiment/scripts/"
        "v3_13_multiactor_design.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_c1_multiactor_clones.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_actual_teb_fair_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_clone_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/"
        "v3_scope_planner/multi_actor_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/"
        "v3_scope_planner/multi_actor_core_v1.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/"
        "v3_scope_planner/execution_core.py"
    ),
    Path(
        "src/simulation/m2_gazebo/launch/"
        "m2_v3_13_actual_teb_multiactor_episode.launch"
    ),
    Path("src/simulation/m2_gazebo/launch/spawn_m2.launch"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("devel/lib/libv3_reactive_actor_plugin.so"),
    Path("src/simulation/m2_gazebo/src/m2_ackermann_plugin.cpp"),
    Path("devel/lib/libm2_ackermann_plugin.so"),
    Path(
        "src/navigation_arena/forks/navigation/local_planner/teb/"
        "src/teb_local_planner_ros.cpp"
    ),
    Path("devel/lib/libteb_local_planner.so"),
    Path("src/simulation/m2_gazebo/config/fixed_teb.yaml"),
    Path("src/simulation/m2_gazebo/config/move_base.yaml"),
    Path("src/simulation/m2_gazebo/config/costmap_common.yaml"),
    Path("src/simulation/m2_gazebo/config/local_costmap.yaml"),
    Path("src/simulation/m2_gazebo/config/global_costmap.yaml"),
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
    Path(
        "artifacts/v3/multiactor/v3_13_s5_recovery_audit/"
        "recovery_planner_audit.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_s5_recovery_audit/"
        "evidence_manifest.sha256"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_s5_recovery_audit/"
        "independent_review.yaml"
    ),
]
ROOTS = {
    "preflight": Path(
        "artifacts/v3/multiactor/v3_13_q1_preflight_readiness"
    ),
    "qualification": Path(
        "artifacts/v3/multiactor/v3_13_q1_readiness"
    ),
}


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


def qualification_rows(contract):
    rows = []
    sequence = 0
    block_index = 0
    seeds = contract["execution_design"]["qualification"]["seeds"]
    for scene in contract["scene_population"]:
        for policies in scene["actor_policy_pairs"]:
            seed = int(seeds[block_index])
            block_index += 1
            methods = ["v3_13_analytic", "actual_teb"]
            if block_index % 2 == 0:
                methods.reverse()
            for method in methods:
                sequence += 1
                rows.append(
                    {
                        "sequence": sequence,
                        "block_index": block_index,
                        "seed": seed,
                        "scene_id": scene["scene_id"],
                        "actor_policies_evaluator_only": list(
                            policies
                        ),
                        "method": method,
                        "attempt": 1,
                    }
                )
    return rows


def preflight_rows(contract):
    design = contract["execution_design"]["preflight"]
    rows = []
    for sequence, method in enumerate(design["methods"], start=1):
        rows.append(
            {
                "sequence": sequence,
                "block_index": 1,
                "seed": int(design["seed"]),
                "scene_id": design["scene_id"],
                "actor_policies_evaluator_only": list(
                    design["actor_policies_evaluator_only"]
                ),
                "method": method,
                "attempt": 1,
            }
        )
    return rows


def preflight_evidence_resources():
    root = Path("artifacts/v3/multiactor/v3_13_q1_preflight")
    resources = [
        root / "exact_schedule.yaml",
        root / "attempt_journal.jsonl",
        root / "collection_summary.yaml",
        root / "independent_result_review.yaml",
    ]
    for episode in sorted((W / root / "episodes").iterdir()):
        require(episode.is_dir(), "unexpected preflight entry")
        relative = episode.relative_to(W)
        resources.extend(
            relative / name
            for name in (
                "world.sdf",
                "scene_spec.yaml",
                "actor_specs.yaml",
                "episode_report.yaml",
                "matrix_receipt.yaml",
                "roslaunch.log",
            )
        )
    return resources


def resources_for(phase):
    resources = list(COMMON_RESOURCES)
    if phase == "qualification":
        resources.extend([ASSESSOR, RESULT_REVIEWER])
        resources.extend(preflight_evidence_resources())
    return resources


def write_or_check(relative, payload, check):
    path = W / relative
    if check:
        require(path.is_file(), "output absent: {}".format(relative))
        require(
            path.read_bytes() == payload,
            "output drift: {}".format(relative),
        )
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    require(not path.exists(), "refusing overwrite: {}".format(relative))
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def build_outputs(phase):
    contract = load_yaml(CONTRACT)
    require(
        sha(read(PREDICTOR))
        == contract["upstream"]["S3_predictor_sha256"],
        "predictor differs",
    )
    if phase == "preflight":
        rows = preflight_rows(contract)
        expected_units = 2
        authorized_seeds = [7300, 7300]
        canonical_output = (
            "artifacts/v3/multiactor/v3_13_q1_preflight"
        )
        status = "unique_non_evidence_preflight_release"
    else:
        preflight_summary = load_yaml(
            Path(
                "artifacts/v3/multiactor/v3_13_q1_preflight/"
                "collection_summary.yaml"
            )
        )
        preflight_review = load_yaml(
            Path(
                "artifacts/v3/multiactor/v3_13_q1_preflight/"
                "independent_result_review.yaml"
            )
        )
        require(
            preflight_review["preflight_pass"] is True,
            "preflight did not pass",
        )
        require(
            preflight_review["collection_summary_sha256"]
            == sha(
                read(
                    Path(
                        "artifacts/v3/multiactor/"
                        "v3_13_q1_preflight/collection_summary.yaml"
                    )
                )
            ),
            "preflight review binding differs",
        )
        require(
            preflight_summary["phase"] == "preflight",
            "preflight phase differs",
        )
        rows = qualification_rows(contract)
        expected_units = 32
        authorized_seeds = list(range(7301, 7317))
        canonical_output = (
            "artifacts/v3/multiactor/v3_13_q1_execution"
        )
        status = "unique_fresh_multiactor_qualification_release"
    require(len(rows) == expected_units, "schedule row count")
    require(
        [int(row["seed"]) for row in rows] == authorized_seeds,
        "seed roster differs",
    )
    used = {int(row["seed"]) for row in rows}
    require(
        not used.intersection(
            set(range(6801, 7259)).union(set(range(7601, 7617)))
        ),
        "historical V3 seed reused",
    )
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": (
            "V3-13-Q1-PREFLIGHT"
            if phase == "preflight"
            else "V3-13-Q1"
        ),
        "phase": phase,
        "status": "exact_schedule_frozen",
        "row_count": len(rows),
        "paired_blocks": len(rows) // 2,
        "rows": rows,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    resources = resources_for(phase)
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": schedule["stage"],
        "phase": phase,
        "status": "closed_dependency_freeze",
        "resource_count": len(resources),
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in resources
        ],
        "credential_environment_scrub_required": True,
    }
    schedule_payload = yaml_bytes(schedule)
    dependencies_payload = yaml_bytes(dependencies)
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": schedule["stage"],
        "phase": phase,
        "status": status,
        "contract_sha256": sha(read(CONTRACT)),
        "predictor_sha256": sha(read(PREDICTOR)),
        "schedule_sha256": sha(schedule_payload),
        "dependency_freeze_sha256": sha(dependencies_payload),
        "canonical_output_root": canonical_output,
        "authorized_units": expected_units,
        "authorized_seed_range": (
            [7300, 7300]
            if phase == "preflight"
            else [7301, 7316]
        ),
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "preflight_excluded_from_claim_evidence": phase == "preflight",
        "execution_authorized": True,
        "formal_execution_authorized": False,
        "policy_or_ranker_training_authorized": False,
        "perception_stage_execution_authorized": False,
        "hardware_device_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    root = ROOTS[phase]
    return {
        root / "exact_schedule.yaml": schedule_payload,
        root / "dependency_freeze.yaml": dependencies_payload,
        root / "execution_release.yaml": yaml_bytes(release),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--phase", choices=sorted(ROOTS), required=True
    )
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    outputs = build_outputs(arguments.phase)
    resources = resources_for(arguments.phase)
    for relative, payload in outputs.items():
        write_or_check(relative, payload, arguments.check)
    root = ROOTS[arguments.phase]
    manifest_path = root / "evidence_manifest.sha256"
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in resources + list(outputs)
    ).encode("utf-8")
    write_or_check(manifest_path, manifest, arguments.check)
    print(
        "status=v3_13_q1_{}_readiness_{} release_sha256={} "
        "manifest_sha256={}".format(
            arguments.phase,
            "reproducible" if arguments.check else "built",
            sha(read(root / "execution_release.yaml")),
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
