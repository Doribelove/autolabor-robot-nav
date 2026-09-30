#!/usr/bin/env python3
"""Build the immutable fresh-seed R8-R2 qualification release."""

import argparse
import ast
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r8_readiness.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r8_frozen_builder", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r8_r2_fresh_runner_recovery_contract.yaml"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_q1_r8_r2_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
R1_REVIEW = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r8_r1_terminal_failure_review.yaml"
)
ORIGINAL_RUNNER = Path(
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r8_recovery_qualification.py"
)
R1_EXECUTION = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r8_r1_recovery_qualification"
)
RESOURCES = BASE.RESOURCES + [
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_q1_r8_r1_runner_repair_contract.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r8_r1_readiness/"
        "execution_release.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r8_r1_readiness/"
        "independent_review.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/"
        "v3_13_q1_r8_r1_recovery_qualification/exact_schedule.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/"
        "v3_13_q1_r8_r1_recovery_qualification/attempt_journal.jsonl"
    ),
    R1_REVIEW,
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r8_r1_terminal_failure.py"
    ),
    CONTRACT,
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r8_r2_recovery_qualification.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_13_q1_r8_r2_recovery_qualification.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r8_r2_qualification_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_13_q1_r8_r2_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r8_r2_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_13_q1_r8_r2_fresh_recovery.py"
    ),
]


def verify_original_runner_call():
    tree = ast.parse(BASE.read(ORIGINAL_RUNNER))
    definitions = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "run_episode"
    ]
    BASE.require(len(definitions) == 1, "run_episode definition count")
    BASE.require(
        len(definitions[0].args.args) == 4,
        "run_episode definition arity",
    )
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "run_episode"
    ]
    BASE.require(len(calls) == 1, "run_episode call count")
    BASE.require(
        len(calls[0].args) == 4 and not calls[0].keywords,
        "run_episode call arity",
    )


def build_outputs():
    contract = yaml.safe_load(BASE.read(CONTRACT))
    inheritance = contract["failure_inheritance"]
    BASE.require(
        BASE.sha(BASE.read(R1_REVIEW))
        == inheritance["r8_r1_terminal_review_sha256"],
        "R8-R1 terminal review differs",
    )
    review = yaml.safe_load(BASE.read(R1_REVIEW))
    BASE.require(
        review["consumed_units"] == 1
        and review["completed_units"] == 0
        and review["forfeited_units"] == 39,
        "R8-R1 accounting differs",
    )
    BASE.require(
        not (R1_EXECUTION / "episodes").exists(),
        "R8-R1 semantic episode unexpectedly exists",
    )
    verify_original_runner_call()
    rows = BASE.schedule_rows(contract)
    BASE.require(len(rows) == 40, "schedule row count")
    seeds = {row["seed"] for row in rows}
    BASE.require(seeds == set(range(7633, 7641)), "fresh seed roster")
    BASE.require(
        not seeds.intersection(set(range(7601, 7633))),
        "historical seed reused",
    )
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R8-R2",
        "status": "exact_fresh_recovery_schedule_frozen",
        "row_count": 40,
        "matched_context_blocks": 8,
        "rows": rows,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R8-R2",
        "status": "closed_fresh_recovery_dependency_freeze",
        "resource_count": len(RESOURCES),
        "resources": [
            {"path": str(path), "sha256": BASE.sha(BASE.read(path))}
            for path in RESOURCES
        ],
        "original_runner_four_argument_definition_and_call_verified": True,
        "semantic_gate_or_model_change_count": 0,
        "r8_r1_retry_or_resume": False,
    }
    schedule_payload = BASE.yaml_bytes(schedule)
    dependency_payload = BASE.yaml_bytes(dependencies)
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R8-R2",
        "status": "unique_fresh_recovery_predictor_qualification_release",
        "contract_sha256": BASE.sha(BASE.read(CONTRACT)),
        "predictor_sha256": contract["upstream"]["predictor_sha256"],
        "schedule_sha256": BASE.sha(schedule_payload),
        "dependency_freeze_sha256": BASE.sha(dependency_payload),
        "r8_r1_terminal_review_sha256": inheritance[
            "r8_r1_terminal_review_sha256"
        ],
        "canonical_output_root": (
            "artifacts/v3/multiactor/"
            "v3_13_q1_r8_r2_recovery_qualification"
        ),
        "authorized_units": 40,
        "authorized_seed_range": [7633, 7640],
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "execution_authorized": True,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    return {
        SCHEDULE: schedule_payload,
        DEPENDENCIES: dependency_payload,
        RELEASE: BASE.yaml_bytes(release),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        BASE.write_or_check(path, payload, args.check)
    manifest = "".join(
        "{}  {}\n".format(BASE.sha(BASE.read(path)), path)
        for path in RESOURCES + list(outputs)
    ).encode("utf-8")
    BASE.write_or_check(MANIFEST, manifest, args.check)
    print(
        "release_sha256={} manifest_sha256={}".format(
            BASE.sha(outputs[RELEASE]), BASE.sha(manifest)
        )
    )


if __name__ == "__main__":
    main()
