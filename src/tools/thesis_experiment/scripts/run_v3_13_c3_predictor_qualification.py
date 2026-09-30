#!/usr/bin/env python3
"""Execute the unique V3-13-C3 fresh clone qualification."""

import argparse
import importlib.util
import json
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
SCRIPT_DIR = W / "src/tools/thesis_experiment/scripts"
sys.path.insert(0, str(SCRIPT_DIR))
import run_v3_13_c1_multiactor_clones as c1_runner


CONTRACT = (
    W
    / "config/thesis_experiments/v3/"
    "v3_13_c3_predictor_qualification_contract.yaml"
)
SCENE_CONTRACT = (
    W
    / "config/thesis_experiments/v3/"
    "v3_13_c1_multiactor_clone_contract.yaml"
)
READINESS = W / "artifacts/v3/multiactor/v3_13_c3_readiness"
CANONICAL_ROOT = W / "artifacts/v3/multiactor/v3_13_c3_execution"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--review-sha256", required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    arguments = parser.parse_args()
    release_payload = c1_runner.read_bytes(
        READINESS / "execution_release.yaml"
    )
    review_payload = c1_runner.read_bytes(
        READINESS / "independent_review.yaml"
    )
    if c1_runner.sha256_bytes(release_payload) != arguments.release_sha256:
        raise RuntimeError("release SHA differs")
    if c1_runner.sha256_bytes(review_payload) != arguments.review_sha256:
        raise RuntimeError("review SHA differs")
    release = yaml.safe_load(release_payload)
    review = yaml.safe_load(review_payload)
    if not release["execution_authorized"] or not review["execution_ready"]:
        raise RuntimeError("C3 is not execution ready")
    if CANONICAL_ROOT.exists():
        raise RuntimeError("canonical C3 root exists; retry forbidden")
    schedule = yaml.safe_load(
        c1_runner.read_bytes(READINESS / "exact_schedule.yaml")
    )
    rows = schedule["rows"]
    if len(rows) != 64:
        raise RuntimeError("schedule unit count differs")
    scene_contract = yaml.safe_load(
        c1_runner.read_bytes(SCENE_CONTRACT)
    )
    scenes = {
        item["scene_id"]: item
        for item in scene_contract["scene_population"]
    }
    CANONICAL_ROOT.mkdir(parents=True)
    c1_runner.atomic_yaml(
        CANONICAL_ROOT / "exact_schedule.yaml", schedule
    )
    journal = CANONICAL_ROOT / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    c1_runner.append_jsonl(
        journal,
        {
            "event": "execution_opened",
            "stage": "V3-13-C3",
            "release_sha256": arguments.release_sha256,
            "authorized_units": 64,
        },
    )
    receipts = []
    for row in rows:
        root = CANONICAL_ROOT / "episodes" / (
            "{:03d}_{:02d}_{}_{}_{}".format(
                row["sequence"],
                row["block_index"],
                row["seed"],
                row["scene_id"],
                row["forced_candidate_id"],
            )
        )
        c1_runner.append_jsonl(
            journal,
            {
                "event": "unit_consumed",
                "sequence": row["sequence"],
                "row": row,
            },
        )
        try:
            receipt = c1_runner.run_episode(
                row,
                scenes[row["scene_id"]],
                root,
                30000 + 2 * row["sequence"],
            )
        except Exception as error:
            c1_runner.append_jsonl(
                journal,
                {
                    "event": "terminal_failure",
                    "sequence": row["sequence"],
                    "row": row,
                    "error": "{}:{}".format(type(error).__name__, error),
                    "consumed_units": row["sequence"],
                    "remaining_units_forfeited": 64 - row["sequence"],
                },
            )
            raise
        receipts.append(receipt)
        report = receipt["episode_report"]
        c1_runner.append_jsonl(
            journal,
            {
                "event": "unit_complete",
                "sequence": row["sequence"],
                "row": row,
                "episode_report_sha256": receipt[
                    "episode_report_sha256"
                ],
                "collection_pass": report[
                    "episode_collection_pass"
                ],
            },
        )
        print(
            "[V3-13-C3] {}/64 scene={} action={} pass={} samples={}".format(
                row["sequence"],
                row["scene_id"],
                row["forced_candidate_id"],
                report["episode_collection_pass"],
                report["result"]["trace_sample_count"],
            ),
            flush=True,
        )
    raw_summary = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-C3",
        "status": "fresh_raw_clone_execution_complete",
        "release_sha256": arguments.release_sha256,
        "authorized_units": 64,
        "consumed_units": 64,
        "completed_units": 64,
        "forfeited_units": 0,
        "all_collection_pass": all(
            receipt["episode_report"]["episode_collection_pass"]
            for receipt in receipts
        ),
        "training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    c1_runner.atomic_yaml(
        CANONICAL_ROOT / "raw_execution_summary.yaml", raw_summary
    )
    c1_runner.append_jsonl(
        journal,
        {
            "event": "execution_complete",
            "consumed_units": 64,
            "completed_units": 64,
            "forfeited_units": 0,
            "all_collection_pass": raw_summary[
                "all_collection_pass"
            ],
        },
    )
    print("status=fresh_raw_clone_execution_complete")
    return 0


if __name__ == "__main__":
    sys.exit(main())

