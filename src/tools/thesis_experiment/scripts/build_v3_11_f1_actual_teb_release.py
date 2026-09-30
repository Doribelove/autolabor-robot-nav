#!/usr/bin/env python3
"""Build the immutable V3-11-F1 formal schedule, authority, and release."""

import hashlib
import os
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_11_f1_actual_teb_formal_contract.yaml"
)
QUALIFICATION_CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_11_a0_actual_teb_fair_baseline_contract.yaml"
)
QUALIFICATION_ASSESSMENT_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_q1_r1_assessment/"
    "qualification_assessment.yaml"
)
QUALIFICATION_REVIEW_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_q1_r1_assessment/"
    "independent_result_review.yaml"
)
QUALIFICATION_TERMINAL_FREEZE_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_q1_r1_assessment/"
    "terminal_freeze.yaml"
)
SCHEDULE_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_release/exact_schedule.yaml"
)
DEPENDENCY_FREEZE_RELATIVE = Path(
    "artifacts/v3/actual_teb/v3_11_f1_release/dependency_freeze.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_11_f1_actual_teb_simulation_authorization.yaml"
)
RELEASE_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_11_f1_actual_teb_formal_release.yaml"
)
EXPECTED_QUALIFICATION_CONTRACT_SHA256 = (
    "387aff179de115f5a2e748ee119473add9c5b4e8bd488349f1a9a905ea705000"
)
EXPECTED_QUALIFICATION_ASSESSMENT_SHA256 = (
    "d90f44ff5fde177b764bc71c4f42c2c60ebb2a3403d42651983de334e02adbbe"
)
EXPECTED_QUALIFICATION_REVIEW_SHA256 = (
    "2d7674f52e33f6a002d4719404e9b8b0ab2ea3f59787d7f914dd8b6467bd4c3e"
)
EXPECTED_QUALIFICATION_TERMINAL_FREEZE_SHA256 = (
    "6b8fe2a9b9148b5225a5a61f5cb4a48cc8bb6a91a176f8db0c0dc06cabea6113"
)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read_bytes(path):
    require(path.is_file(), "required file absent: {}".format(path))
    return path.read_bytes()


def sha256_file(path):
    return hashlib.sha256(read_bytes(path)).hexdigest()


def load_yaml(path):
    return yaml.safe_load(read_bytes(path))


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def write_once(path, value):
    require(not path.exists(), "immutable output exists: {}".format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = yaml_bytes(value)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))
    return hashlib.sha256(payload).hexdigest()


def expected_rows(contract):
    formal = contract["formal_preregistration"]
    methods = list(contract["methods"]["identifiers"])
    rows = []
    sequence = 0
    for zero_index, seed in enumerate(
        range(int(formal["seed_start"]), int(formal["seed_end"]) + 1)
    ):
        stratum = formal["strata"][zero_index % len(formal["strata"])]
        order = methods if zero_index % 2 == 0 else list(reversed(methods))
        for method in order:
            sequence += 1
            rows.append(
                {
                    "sequence": sequence,
                    "block_index": zero_index + 1,
                    "seed": seed,
                    "scene_id": stratum["scene_id"],
                    "actor_policy": stratum["actor_policy"],
                    "method": method,
                }
            )
    return rows


def main():
    contract_path = WORKSPACE / CONTRACT_RELATIVE
    qualification_contract_path = WORKSPACE / QUALIFICATION_CONTRACT_RELATIVE
    assessment_path = WORKSPACE / QUALIFICATION_ASSESSMENT_RELATIVE
    review_path = WORKSPACE / QUALIFICATION_REVIEW_RELATIVE
    terminal_freeze_path = WORKSPACE / QUALIFICATION_TERMINAL_FREEZE_RELATIVE
    require(
        sha256_file(qualification_contract_path)
        == EXPECTED_QUALIFICATION_CONTRACT_SHA256,
        "qualification contract changed",
    )
    require(
        sha256_file(assessment_path)
        == EXPECTED_QUALIFICATION_ASSESSMENT_SHA256,
        "qualification assessment changed",
    )
    require(
        sha256_file(review_path) == EXPECTED_QUALIFICATION_REVIEW_SHA256,
        "qualification review changed",
    )
    require(
        sha256_file(terminal_freeze_path)
        == EXPECTED_QUALIFICATION_TERMINAL_FREEZE_SHA256,
        "qualification terminal freeze changed",
    )
    qualification_contract = load_yaml(qualification_contract_path)
    assessment = load_yaml(assessment_path)
    review = load_yaml(review_path)
    require(assessment["overall_pass"] is True, "qualification did not pass")
    require(
        review["all_assessment_gates_pass"] is True,
        "independent qualification review did not pass",
    )
    contract = load_yaml(contract_path)
    require(contract["stage"] == "V3-11-F1", "formal contract stage differs")
    require(
        contract["authority"]["formal_execution_authorized"] is True,
        "formal execution is not authorized",
    )
    require(
        contract["authority"]["training_authorized"] is False,
        "training boundary differs",
    )
    require(
        contract["formal_preregistration"]["statistics"]
        == qualification_contract["future_formal_preregistration"],
        "formal statistics or thresholds differ from qualification preregistration",
    )
    rows = expected_rows(contract)
    require(len(rows) == 60, "formal schedule must contain 60 rows")
    require(
        len(
            {
                (
                    row["seed"],
                    row["scene_id"],
                    row["actor_policy"],
                    row["method"],
                )
                for row in rows
            }
        )
        == 60,
        "formal schedule identity is not unique",
    )
    schedule = {
        "schema_version": "1.0",
        "stage": "V3-11-F1",
        "phase": "formal",
        "experimental_unit": "paired scene-policy-seed block",
        "paired_block_count": 30,
        "row_count": 60,
        "attempts_per_identity": 1,
        "retry_or_resume_authorized": False,
        "rows": rows,
    }
    schedule_sha = write_once(WORKSPACE / SCHEDULE_RELATIVE, schedule)

    resources = []
    for relative in contract["release_resource_roster"]:
        path = WORKSPACE / relative
        resources.append(
            {
                "path": relative,
                "sha256": sha256_file(path),
                "size_bytes": path.stat().st_size,
            }
        )
    dependency_freeze = {
        "schema_version": "1.0",
        "stage": "V3-11-F1",
        "status": "formal_dependency_freeze_pass",
        "resource_count": len(resources),
        "resources": resources,
    }
    dependency_sha = write_once(
        WORKSPACE / DEPENDENCY_FREEZE_RELATIVE, dependency_freeze
    )
    contract_sha = sha256_file(contract_path)
    authorization = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-11-F1",
        "status": "explicit_user_fresh_simulation_authorization",
        "user_instruction_date": "2026-07-25",
        "authorized_scope": (
            "one-time fresh actual TebLocalPlannerROS fair paired formal "
            "simulation after passing qualification"
        ),
        "contract_path": str(CONTRACT_RELATIVE),
        "contract_sha256": contract_sha,
        "schedule_path": str(SCHEDULE_RELATIVE),
        "schedule_sha256": schedule_sha,
        "qualification_assessment_sha256": (
            EXPECTED_QUALIFICATION_ASSESSMENT_SHA256
        ),
        "qualification_review_sha256": EXPECTED_QUALIFICATION_REVIEW_SHA256,
        "authorized_units": 60,
        "authorized_seed_start": 7101,
        "authorized_seed_end": 7130,
        "attempts_per_identity": 1,
        "retry_or_resume_authorized": False,
        "execution_root": (
            "artifacts/v3/actual_teb/v3_11_f1_execution"
        ),
        "formal_execution_authorized": True,
        "simulation_only": True,
        "training_authorized": False,
        "hardware_device_access_authorized": False,
        "real_vehicle_authorized": False,
        "stable_workspace_write_authorized": False,
    }
    authorization_sha = write_once(
        WORKSPACE / AUTHORIZATION_RELATIVE, authorization
    )
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-11-F1",
        "status": "immutable_fresh_formal_execution_release",
        "contract_path": str(CONTRACT_RELATIVE),
        "contract_sha256": contract_sha,
        "authorization_path": str(AUTHORIZATION_RELATIVE),
        "authorization_sha256": authorization_sha,
        "schedule_path": str(SCHEDULE_RELATIVE),
        "schedule_sha256": schedule_sha,
        "dependency_freeze_path": str(DEPENDENCY_FREEZE_RELATIVE),
        "dependency_freeze_sha256": dependency_sha,
        "qualification_assessment_sha256": (
            EXPECTED_QUALIFICATION_ASSESSMENT_SHA256
        ),
        "qualification_review_sha256": EXPECTED_QUALIFICATION_REVIEW_SHA256,
        "qualification_terminal_freeze_sha256": (
            EXPECTED_QUALIFICATION_TERMINAL_FREEZE_SHA256
        ),
        "execution_runner_path": (
            "src/tools/thesis_experiment/scripts/"
            "run_v3_11_f1_actual_teb_formal.py"
        ),
        "execution_root": "artifacts/v3/actual_teb/v3_11_f1_execution",
        "authorized_units": 60,
        "paired_blocks": 30,
        "attempts_per_identity": 1,
        "retry_or_resume_authorized": False,
        "threshold_change_after_release_authorized": False,
        "selective_rerun_authorized": False,
        "training_steps": 0,
        "simulation_only": True,
        "formal_assessment_required_before_claim": True,
    }
    release_sha = write_once(WORKSPACE / RELEASE_RELATIVE, release)
    print(
        "status={} rows={} contract_sha256={} schedule_sha256={} "
        "authorization_sha256={} dependency_freeze_sha256={} "
        "release_sha256={}".format(
            release["status"],
            len(rows),
            contract_sha,
            schedule_sha,
            authorization_sha,
            dependency_sha,
            release_sha,
        )
    )


if __name__ == "__main__":
    main()
