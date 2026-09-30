#!/usr/bin/env python3
"""Run the exact fresh V3-13-S4 recovery qualification matrix once."""

import argparse
import json
from pathlib import Path
import sys

import yaml

import run_v3_13_s2_sequential_clones as s2


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = (
    W
    / "config/thesis_experiments/v3/"
    "v3_13_s4_recovery_predictor_qualification_contract.yaml"
)
S2_CONTRACT = (
    W
    / "config/thesis_experiments/v3/"
    "v3_13_s2_synchronized_sequential_clone_contract.yaml"
)
SCENE_CONTRACT = s2.SCENE_CONTRACT
READINESS = W / "artifacts/v3/multiactor/v3_13_s4_readiness"
RELEASE = READINESS / "execution_release.yaml"
REVIEW = READINESS / "independent_review.yaml"
SCHEDULE = READINESS / "exact_schedule.yaml"
OUTPUT = W / "artifacts/v3/multiactor/v3_13_s4_execution"


def collection_summary(receipts, contract_sha):
    s2_contract = yaml.safe_load(s2.read_bytes(S2_CONTRACT))
    base = s2.summarize(receipts, s2_contract, contract_sha)
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S4",
        "status": (
            "fresh_recovery_qualification_collection_pass"
            if base["collection_gate_pass"]
            else "fresh_recovery_qualification_collection_gate_fail"
        ),
        "contract_sha256": contract_sha,
        "authorized_units": 32,
        "consumed_units": len(receipts),
        "completed_units": len(receipts),
        "forfeited_units": 0,
        "matched_context_blocks": base["matched_context_blocks"],
        "complete_action_support_rate": base[
            "complete_action_support_rate"
        ],
        "maximum_branch_clock_absolute_error_s": base[
            "maximum_branch_clock_absolute_error_s"
        ],
        "maximum_decision_ego_x_range_m": base[
            "maximum_decision_ego_x_range_m"
        ],
        "maximum_decision_actor_coordinate_range_m": base[
            "maximum_decision_actor_coordinate_range_m"
        ],
        "maximum_decision_actor_speed_range_mps": base[
            "maximum_decision_actor_speed_range_mps"
        ],
        "contact_state_count_retained": base[
            "contact_state_count_retained"
        ],
        "gate_checks": base["gate_checks"],
        "collection_gate_pass": base["collection_gate_pass"],
        "blocks": base["blocks"],
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--contract-sha256", required=True)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--review-sha256", required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    arguments = parser.parse_args()
    if Path(arguments.output_root).resolve() != OUTPUT.resolve():
        raise RuntimeError("canonical output root differs")
    if OUTPUT.exists():
        raise RuntimeError("output exists; retry or resume forbidden")
    release_payload = s2.read_bytes(RELEASE)
    review_payload = s2.read_bytes(REVIEW)
    if s2.sha(release_payload) != arguments.release_sha256:
        raise RuntimeError("release SHA differs")
    if s2.sha(review_payload) != arguments.review_sha256:
        raise RuntimeError("review SHA differs")
    release = yaml.safe_load(release_payload)
    review = yaml.safe_load(review_payload)
    if not release["execution_authorized"]:
        raise RuntimeError("execution not authorized")
    if int(release["authorized_units"]) != 32:
        raise RuntimeError("authorized units differ")
    if review["execution_release_sha256"] != arguments.release_sha256:
        raise RuntimeError("review does not bind release")
    contract_payload = s2.read_bytes(CONTRACT)
    contract_sha = s2.sha(contract_payload)
    if arguments.contract_sha256 != contract_sha:
        raise RuntimeError("contract SHA differs")
    if release["contract_sha256"] != contract_sha:
        raise RuntimeError("release does not bind contract")
    contract = yaml.safe_load(contract_payload)
    if contract["authority"]["training_authorized"]:
        raise RuntimeError("training boundary differs")
    rows = yaml.safe_load(s2.read_bytes(SCHEDULE))["rows"]
    if len(rows) != 32:
        raise RuntimeError("schedule row count differs")
    scenes = {
        item["scene_id"]: item
        for item in yaml.safe_load(
            s2.read_bytes(SCENE_CONTRACT)
        )["scene_population"]
    }
    OUTPUT.mkdir(parents=True)
    s2.atomic_yaml(
        OUTPUT / "exact_schedule.yaml",
        {
            "schema_version": "1.0",
            "stage": "V3-13-S4",
            "contract_sha256": contract_sha,
            "row_count": len(rows),
            "rows": rows,
        },
    )
    journal = OUTPUT / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    s2.append_jsonl(
        journal,
        {
            "event": "execution_opened",
            "stage": "V3-13-S4",
            "authorized_units": 32,
            "contract_sha256": contract_sha,
        },
    )
    receipts = []
    for row in rows:
        s2.append_jsonl(
            journal,
            {
                "event": "unit_consumed",
                "sequence": row["sequence"],
                "row": row,
            },
        )
        root = OUTPUT / "episodes" / (
            "{:03d}_{:02d}_{}_{}_{}".format(
                row["sequence"],
                row["block_index"],
                row["seed"],
                row["branch_time_label"],
                row["forced_candidate_id"],
            )
        )
        try:
            receipt = s2.run_episode(
                row,
                scenes[row["scene_id"]],
                root,
                30300 + 2 * row["sequence"],
            )
        except Exception as error:
            s2.append_jsonl(
                journal,
                {
                    "event": "terminal_failure",
                    "sequence": row["sequence"],
                    "row": row,
                    "error": "{}:{}".format(type(error).__name__, error),
                    "remaining_units_forfeited": 32 - row["sequence"],
                },
            )
            raise
        receipt["stage"] = "V3-13-S4"
        s2.atomic_yaml(root / "matrix_receipt.yaml", receipt)
        receipts.append(receipt)
        report = receipt["episode_report"]
        s2.append_jsonl(
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
                "trace_samples": report["result"][
                    "trace_sample_count"
                ],
            },
        )
        print(
            "[V3-13-S4] {}/32 block={} target={:.1f}s action={} "
            "pass={} clock={:.4f}s".format(
                row["sequence"],
                row["block_index"],
                row["branch_target_sim_s"],
                row["forced_candidate_id"],
                report["episode_collection_pass"],
                report["common_stop_prefix"][
                    "branch_clock_absolute_error_s"
                ],
            ),
            flush=True,
        )
    summary = collection_summary(receipts, contract_sha)
    s2.atomic_yaml(OUTPUT / "collection_summary.yaml", summary)
    s2.append_jsonl(
        journal,
        {
            "event": "execution_complete",
            "authorized_units": 32,
            "consumed_units": 32,
            "completed_units": 32,
            "forfeited_units": 0,
            "collection_gate_pass": summary["collection_gate_pass"],
            "summary_sha256": s2.sha(
                s2.read_bytes(OUTPUT / "collection_summary.yaml")
            ),
        },
    )
    print(
        "status={} summary_sha256={}".format(
            summary["status"],
            s2.sha(s2.read_bytes(OUTPUT / "collection_summary.yaml")),
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
