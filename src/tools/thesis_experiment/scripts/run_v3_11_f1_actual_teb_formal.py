#!/usr/bin/env python3
"""Execute the one-time fresh V3-11-F1 actual-TEB paired matrix."""

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
QUALIFICATION_RUNNER = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "run_v3_11_actual_teb_matrix.py"
)
CONTRACT_PATH = (
    WORKSPACE
    / "config/thesis_experiments/v3/"
    "v3_11_f1_actual_teb_formal_contract.yaml"
)
AUTHORIZATION_PATH = (
    WORKSPACE
    / "experiments/manifests/v3/"
    "v3_11_f1_actual_teb_simulation_authorization.yaml"
)
RELEASE_PATH = (
    WORKSPACE
    / "experiments/manifests/v3/"
    "v3_11_f1_actual_teb_formal_release.yaml"
)
SCHEDULE_PATH = (
    WORKSPACE
    / "artifacts/v3/actual_teb/v3_11_f1_release/"
    "exact_schedule.yaml"
)
EXPECTED_OUTPUT_ROOT = (
    WORKSPACE / "artifacts/v3/actual_teb/v3_11_f1_execution"
)


def load_runner():
    spec = importlib.util.spec_from_file_location(
        "v3_11_qualification_runner", QUALIFICATION_RUNNER
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


def validate_release(release_sha256):
    contract = load_yaml(CONTRACT_PATH)
    authorization = load_yaml(AUTHORIZATION_PATH)
    release = load_yaml(RELEASE_PATH)
    schedule = load_yaml(SCHEDULE_PATH)
    observed_release_sha = sha256_file(RELEASE_PATH)
    require(
        observed_release_sha == release_sha256,
        "caller release hash differs from immutable release",
    )
    require(release["stage"] == "V3-11-F1", "release stage differs")
    require(
        release["status"] == "immutable_fresh_formal_execution_release",
        "release status differs",
    )
    require(
        release["contract_sha256"] == sha256_file(CONTRACT_PATH),
        "release contract hash differs",
    )
    require(
        release["authorization_sha256"] == sha256_file(AUTHORIZATION_PATH),
        "release authorization hash differs",
    )
    require(
        release["schedule_sha256"] == sha256_file(SCHEDULE_PATH),
        "release schedule hash differs",
    )
    dependency_path = WORKSPACE / release["dependency_freeze_path"]
    require(
        release["dependency_freeze_sha256"] == sha256_file(dependency_path),
        "release dependency freeze hash differs",
    )
    dependency_freeze = load_yaml(dependency_path)
    require(
        dependency_freeze["status"] == "formal_dependency_freeze_pass",
        "dependency closure status differs",
    )
    for resource in dependency_freeze["resources"]:
        path = WORKSPACE / resource["path"]
        require(
            sha256_file(path) == resource["sha256"],
            "released dependency changed: {}".format(resource["path"]),
        )
        require(
            path.stat().st_size == resource["size_bytes"],
            "released dependency size changed: {}".format(resource["path"]),
        )
    require(
        authorization["status"] == "explicit_user_fresh_simulation_authorization",
        "authorization status differs",
    )
    require(
        authorization["formal_execution_authorized"] is True,
        "formal simulation is not authorized",
    )
    require(
        authorization["training_authorized"] is False,
        "training boundary differs",
    )
    require(
        authorization["hardware_device_access_authorized"] is False,
        "hardware boundary differs",
    )
    require(
        authorization["real_vehicle_authorized"] is False,
        "real-vehicle boundary differs",
    )
    require(
        authorization["contract_sha256"] == sha256_file(CONTRACT_PATH),
        "authorization contract hash differs",
    )
    require(
        authorization["schedule_sha256"] == sha256_file(SCHEDULE_PATH),
        "authorization schedule hash differs",
    )
    rows = expected_rows(contract)
    require(schedule["stage"] == "V3-11-F1", "schedule stage differs")
    require(schedule["phase"] == "formal", "schedule phase differs")
    require(schedule["row_count"] == 60, "schedule row count differs")
    require(schedule["rows"] == rows, "schedule derivation differs")
    require(release["authorized_units"] == len(rows), "release budget differs")
    require(authorization["authorized_units"] == len(rows), "authority budget differs")
    require(release["attempts_per_identity"] == 1, "attempt budget differs")
    require(
        release["retry_or_resume_authorized"] is False,
        "retry/resume must remain forbidden",
    )
    require(
        contract["authority"]["formal_execution_authorized"] is True,
        "contract does not authorize formal execution",
    )
    require(
        contract["authority"]["training_authorized"] is False,
        "contract training boundary differs",
    )
    return contract, authorization, release, schedule, observed_release_sha


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    arguments = parser.parse_args()
    output_root = Path(arguments.output_root).resolve()
    require(
        output_root == EXPECTED_OUTPUT_ROOT.resolve(),
        "output root differs from released destination",
    )
    require(
        not output_root.exists(),
        "output root exists; retry/resume/overwrite is forbidden",
    )
    contract, authorization, release, schedule, release_sha = validate_release(
        arguments.release_sha256
    )
    qrunner = load_runner()
    output_root.mkdir(parents=True)
    journal_path = output_root / "attempt_journal.jsonl"
    journal_path.touch(exist_ok=False)
    qrunner.append_jsonl(
        journal_path,
        {
            "event": "execution_opened",
            "stage": "V3-11-F1",
            "phase": "formal",
            "release_sha256": release_sha,
            "contract_sha256": sha256_file(CONTRACT_PATH),
            "authorization_sha256": sha256_file(AUTHORIZATION_PATH),
            "schedule_sha256": sha256_file(SCHEDULE_PATH),
            "authorized_units": len(schedule["rows"]),
            "lookahead_m": float(contract["execution"]["lookahead_m"]),
            "training_steps": 0,
        },
    )
    receipts = []
    scenes = {
        item["scene_id"]: item for item in contract["scene_population"]
    }
    for row in schedule["rows"]:
        episode_root = output_root / "episodes" / (
            "{:03d}_{}_{}_{}_{}".format(
                row["sequence"],
                row["seed"],
                row["scene_id"],
                row["actor_policy"],
                row["method"],
            )
        )
        qrunner.append_jsonl(
            journal_path,
            {
                "event": "unit_consumed",
                "sequence": row["sequence"],
                "row": row,
                "attempt": 1,
            },
        )
        try:
            receipt = qrunner.run_episode(
                row,
                scenes[row["scene_id"]],
                episode_root,
                float(contract["execution"]["lookahead_m"]),
                25000 + 2 * row["sequence"],
            )
        except Exception as error:
            qrunner.append_jsonl(
                journal_path,
                {
                    "event": "terminal_failure",
                    "sequence": row["sequence"],
                    "row": row,
                    "error": "{}:{}".format(type(error).__name__, error),
                    "consumed_units": row["sequence"],
                    "remaining_units_forfeited": (
                        len(schedule["rows"]) - row["sequence"]
                    ),
                    "retry_or_resume_authorized": False,
                },
            )
            raise
        receipt_path = episode_root / "matrix_receipt.yaml"
        qrunner.atomic_yaml(receipt_path, receipt)
        receipts.append(receipt)
        report = receipt["episode_report"]
        qrunner.append_jsonl(
            journal_path,
            {
                "event": "unit_complete",
                "sequence": row["sequence"],
                "row": row,
                "matrix_receipt_sha256": sha256_file(receipt_path),
                "episode_pass": report["episode_pass"],
                "goal_reached": report["result"]["goal_reached"],
                "restricted_elapsed_s": report["result"][
                    "restricted_elapsed_s"
                ],
                "minimum_conservative_clearance_m": report["result"][
                    "minimum_conservative_clearance_m"
                ],
            },
        )
        print(
            "[V3-11-F1] {}/{} {} {} {} pass={} goal={} t={:.3f}s "
            "c={:.3f}m".format(
                row["sequence"],
                len(schedule["rows"]),
                row["scene_id"],
                row["actor_policy"],
                row["method"],
                report["episode_pass"],
                report["result"]["goal_reached"],
                float(report["result"]["restricted_elapsed_s"]),
                float(
                    report["result"]["minimum_conservative_clearance_m"]
                ),
            ),
            flush=True,
        )
    summary = qrunner.summarize(
        receipts,
        "formal",
        sha256_file(CONTRACT_PATH),
        float(contract["execution"]["lookahead_m"]),
    )
    summary.update(
        {
            "stage": "V3-11-F1",
            "release_sha256": release_sha,
            "authorization_sha256": sha256_file(AUTHORIZATION_PATH),
            "schedule_sha256": sha256_file(SCHEDULE_PATH),
            "qualification_review_sha256": release[
                "qualification_review_sha256"
            ],
            "formal_assessment_complete": False,
            "formal_performance_claim_authorized": False,
        }
    )
    summary_path = output_root / "matrix_summary.yaml"
    qrunner.atomic_yaml(summary_path, summary)
    qrunner.append_jsonl(
        journal_path,
        {
            "event": "execution_complete",
            "authorized_units": len(schedule["rows"]),
            "consumed_units": len(schedule["rows"]),
            "completed_units": len(schedule["rows"]),
            "forfeited_units": 0,
            "summary_sha256": sha256_file(summary_path),
            "formal_assessment_complete": False,
        },
    )
    print(
        "[V3-11-F1] execution_complete units={} summary_sha256={}".format(
            len(schedule["rows"]), sha256_file(summary_path)
        ),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
