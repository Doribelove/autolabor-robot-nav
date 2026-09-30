#!/usr/bin/env python3
"""Run exact 40-unit R19-R6 tolerance-repaired support matrix."""

import argparse
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
R4_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r19_r4_prefix_aligned_support.py"
)
R5_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r19_r5_r1_propensity_preflight.py"
)
SPEC = importlib.util.spec_from_file_location("r19_r6_r4_base", R4_PATH)
R4 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R4)
SPEC = importlib.util.spec_from_file_location("r19_r6_r5_base", R5_PATH)
R5 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R5)
COMMON = R4.COMMON
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r6_tolerance_repaired_support_contract.yaml"
)
READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r19_r6_readiness"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_r6_tolerance_repaired_support"
)


def run_episode(row, scene, root, ros_port):
    receipt = R5.run_episode(row, scene, root, ros_port)
    receipt["stage"] = "V3-13-Q1-R19-R6"
    COMMON.atomic_yaml(root / "matrix_receipt.yaml", receipt)
    return receipt


def summarize(receipts, contract_sha, contract):
    value = R4.R2.summarize(receipts, contract_sha, contract)
    value["stage"] = "V3-13-Q1-R19-R6"
    value["status"] = (
        "fresh_tolerance_repaired_five_action_support_collection_pass"
        if value["collection_gate_pass"]
        else "fresh_tolerance_repaired_five_action_support_collection_fail"
    )
    value["runtime_component_stage"] = "V3-13-Q1-R19-R5"
    value["predictor_fit_authorized"] = False
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--contract-sha256", required=True)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--review-sha256", required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    args = parser.parse_args()
    if Path(args.output_root).resolve() != OUTPUT.resolve():
        raise RuntimeError("canonical output differs")
    if OUTPUT.exists():
        raise RuntimeError("output exists; retry forbidden")
    payloads = [
        COMMON.read_bytes(CONTRACT),
        COMMON.read_bytes(READINESS / "execution_release.yaml"),
        COMMON.read_bytes(READINESS / "independent_review.yaml"),
    ]
    for payload, expected in zip(
        payloads,
        (args.contract_sha256, args.release_sha256, args.review_sha256),
    ):
        if COMMON.sha(payload) != expected:
            raise RuntimeError("release binding differs")
    release = yaml.safe_load(payloads[1])
    review = yaml.safe_load(payloads[2])
    if (
        not release["execution_authorized"]
        or review["execution_release_sha256"] != args.release_sha256
    ):
        raise RuntimeError("execution not independently released")
    schedule = yaml.safe_load(
        COMMON.read_bytes(READINESS / "exact_schedule.yaml")
    )
    rows = schedule["rows"]
    if len(rows) != 40:
        raise RuntimeError("schedule row count")
    scenes_doc = yaml.safe_load(COMMON.read_bytes(COMMON.SCENE_CONTRACT))
    scenes = {s["scene_id"]: s for s in scenes_doc["scene_families"]}
    contract = yaml.safe_load(payloads[0])
    OUTPUT.mkdir(parents=True)
    COMMON.atomic_yaml(OUTPUT / "exact_schedule.yaml", {
        "schema_version": "1.0", "stage": "V3-13-Q1-R19-R6",
        "contract_sha256": args.contract_sha256,
        "row_count": 40, "rows": rows,
    })
    journal = OUTPUT / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    COMMON.append_jsonl(journal, {
        "event": "execution_opened", "authorized_units": 40,
        "contract_sha256": args.contract_sha256,
    })
    receipts = []
    for row in rows:
        COMMON.append_jsonl(journal, {
            "event": "unit_consumed", "sequence": row["sequence"],
            "row": row,
        })
        root = OUTPUT / "episodes" / (
            "{:03d}_{:02d}_{}_{}".format(
                row["sequence"], row["block_index"], row["seed"],
                row["forced_candidate_id"],
            )
        )
        try:
            receipt = run_episode(
                row, scenes[row["scene_id"]], root,
                32200 + 2 * row["sequence"],
            )
        except Exception as error:
            COMMON.append_jsonl(journal, {
                "event": "terminal_failure", "sequence": row["sequence"],
                "row": row,
                "error": "{}:{}".format(type(error).__name__, error),
                "remaining_units_forfeited": 40 - row["sequence"],
            })
            raise
        receipts.append(receipt)
        report = receipt["episode_report"]
        COMMON.append_jsonl(journal, {
            "event": "unit_complete", "sequence": row["sequence"],
            "row": row,
            "episode_report_sha256": receipt["episode_report_sha256"],
            "collection_pass": report["episode_collection_pass"],
        })
        print(
            "[V3-13-Q1-R19-R6] {}/40 block={} action={} pass={} "
            "ego={:.6f} dynamic={:.3f} guard={}".format(
                row["sequence"], row["block_index"],
                row["forced_candidate_id"],
                report["episode_collection_pass"],
                report["common_stop_prefix"]["decision_pose"]["x_m"],
                report["result"]["minimum_conservative_clearance_m"],
                report["terminal_guard_activation_count"],
            ),
            flush=True,
        )
    summary = summarize(receipts, COMMON.sha(payloads[0]), contract)
    COMMON.atomic_yaml(OUTPUT / "collection_summary.yaml", summary)
    COMMON.append_jsonl(journal, {
        "event": "execution_complete", "consumed_units": 40,
        "completed_units": 40, "forfeited_units": 0,
        "collection_gate_pass": summary["collection_gate_pass"],
        "summary_sha256": COMMON.sha(
            COMMON.read_bytes(OUTPUT / "collection_summary.yaml")
        ),
    })
    print(
        "status={} summary_sha256={}".format(
            summary["status"],
            COMMON.sha(COMMON.read_bytes(OUTPUT / "collection_summary.yaml")),
        )
    )


if __name__ == "__main__":
    main()
