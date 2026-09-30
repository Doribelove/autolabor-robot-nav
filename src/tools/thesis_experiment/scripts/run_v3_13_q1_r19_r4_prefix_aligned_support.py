#!/usr/bin/env python3
"""Run exact 40-unit R19-R4 prefix-aligned support matrix."""

import argparse
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
R2_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r19_r2_boundary_atomic_support.py"
)
R3_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r19_r3_prefix_matching_preflight.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r19_r4_r2_base", R2_PATH
)
R2 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R2)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r19_r4_r3_base", R3_PATH
)
R3RUN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R3RUN)
COMMON = R2.COMMON
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r4_prefix_aligned_support_contract.yaml"
)
READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r19_r4_readiness"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_r4_prefix_aligned_support"
)


def run_episode(row, scene, root, ros_port):
    receipt = R3RUN.run_episode(row, scene, root, ros_port)
    receipt["stage"] = "V3-13-Q1-R19-R4"
    COMMON.atomic_yaml(root / "matrix_receipt.yaml", receipt)
    return receipt


def summarize(receipts, contract_sha, contract):
    value = R2.summarize(receipts, contract_sha, contract)
    value["stage"] = "V3-13-Q1-R19-R4"
    value["status"] = (
        "fresh_prefix_aligned_five_action_support_collection_pass"
        if value["collection_gate_pass"]
        else "fresh_prefix_aligned_five_action_support_collection_fail"
    )
    value["runtime_component_stage"] = "V3-13-Q1-R19-R3"
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
    contract_payload = COMMON.read_bytes(CONTRACT)
    release_payload = COMMON.read_bytes(
        READINESS / "execution_release.yaml"
    )
    review_payload = COMMON.read_bytes(
        READINESS / "independent_review.yaml"
    )
    for payload, expected, label in (
        (contract_payload, args.contract_sha256, "contract"),
        (release_payload, args.release_sha256, "release"),
        (review_payload, args.review_sha256, "review"),
    ):
        if COMMON.sha(payload) != expected:
            raise RuntimeError("{} SHA differs".format(label))
    release = yaml.safe_load(release_payload)
    review = yaml.safe_load(review_payload)
    if not release["execution_authorized"]:
        raise RuntimeError("execution not authorized")
    if review["execution_release_sha256"] != args.release_sha256:
        raise RuntimeError("review release differs")
    rows = yaml.safe_load(
        COMMON.read_bytes(READINESS / "exact_schedule.yaml")
    )["rows"]
    if len(rows) != 40:
        raise RuntimeError("schedule row count")
    d0 = yaml.safe_load(COMMON.read_bytes(COMMON.SCENE_CONTRACT))
    scenes = {
        scene["scene_id"]: scene for scene in d0["scene_families"]
    }
    contract = yaml.safe_load(contract_payload)
    OUTPUT.mkdir(parents=True)
    COMMON.atomic_yaml(
        OUTPUT / "exact_schedule.yaml",
        {
            "schema_version": "1.0",
            "stage": "V3-13-Q1-R19-R4",
            "contract_sha256": args.contract_sha256,
            "row_count": 40,
            "rows": rows,
        },
    )
    journal = OUTPUT / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    COMMON.append_jsonl(
        journal,
        {
            "event": "execution_opened",
            "authorized_units": 40,
            "contract_sha256": args.contract_sha256,
        },
    )
    receipts = []
    for row in rows:
        COMMON.append_jsonl(
            journal,
            {
                "event": "unit_consumed",
                "sequence": row["sequence"],
                "row": row,
            },
        )
        root = OUTPUT / "episodes" / (
            "{:03d}_{:02d}_{}_{}".format(
                row["sequence"],
                row["block_index"],
                row["seed"],
                row["forced_candidate_id"],
            )
        )
        try:
            receipt = run_episode(
                row,
                scenes[row["scene_id"]],
                root,
                31900 + 2 * row["sequence"],
            )
        except Exception as error:
            COMMON.append_jsonl(
                journal,
                {
                    "event": "terminal_failure",
                    "sequence": row["sequence"],
                    "row": row,
                    "error": "{}:{}".format(
                        type(error).__name__, error
                    ),
                    "remaining_units_forfeited": 40
                    - row["sequence"],
                },
            )
            raise
        receipts.append(receipt)
        report = receipt["episode_report"]
        COMMON.append_jsonl(
            journal,
            {
                "event": "unit_complete",
                "sequence": row["sequence"],
                "row": row,
                "episode_report_sha256": receipt[
                    "episode_report_sha256"
                ],
                "collection_pass": report["episode_collection_pass"],
            },
        )
        print(
            "[V3-13-Q1-R19-R4] {}/40 block={} action={} pass={} "
            "ego={:.6f} dynamic={:.3f} guard={}".format(
                row["sequence"],
                row["block_index"],
                row["forced_candidate_id"],
                report["episode_collection_pass"],
                report["common_stop_prefix"][
                    "decision_pose"
                ]["x_m"],
                report["result"]["minimum_conservative_clearance_m"],
                report["terminal_guard_activation_count"],
            ),
            flush=True,
        )
    summary = summarize(receipts, args.contract_sha256, contract)
    COMMON.atomic_yaml(OUTPUT / "collection_summary.yaml", summary)
    COMMON.append_jsonl(
        journal,
        {
            "event": "execution_complete",
            "consumed_units": 40,
            "completed_units": 40,
            "forfeited_units": 0,
            "collection_gate_pass": summary["collection_gate_pass"],
            "summary_sha256": COMMON.sha(
                COMMON.read_bytes(OUTPUT / "collection_summary.yaml")
            ),
        },
    )
    print(
        "status={} summary_sha256={}".format(
            summary["status"],
            COMMON.sha(
                COMMON.read_bytes(OUTPUT / "collection_summary.yaml")
            ),
        )
    )


if __name__ == "__main__":
    main()
