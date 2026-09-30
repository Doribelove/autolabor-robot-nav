#!/usr/bin/env python3
"""Replay and independently review the non-evidence Q1 preflight."""

import hashlib
import importlib.util
import json
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
RUNNER_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_actual_teb_matrix.py"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_q1_preflight")
REVIEW = ROOT / "independent_result_review.yaml"


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_preflight_runner", RUNNER_PATH
    )
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    rows = yaml.safe_load(
        runner.read_bytes(W / ROOT / "exact_schedule.yaml")
    )["rows"]
    episodes = sorted((W / ROOT / "episodes").iterdir())
    if len(rows) != 2 or len(episodes) != 2:
        raise RuntimeError("preflight cardinality differs")
    receipts = []
    for row, episode in zip(rows, episodes):
        receipt = yaml.safe_load(
            runner.read_bytes(episode / "matrix_receipt.yaml")
        )
        if receipt["row"] != row:
            raise RuntimeError("receipt row differs")
        for field, name in (
            ("episode_report_sha256", "episode_report.yaml"),
            ("world_sha256", "world.sdf"),
            ("scene_spec_sha256", "scene_spec.yaml"),
            ("actor_specs_sha256", "actor_specs.yaml"),
            ("roslaunch_log_sha256", "roslaunch.log"),
        ):
            if receipt[field] != runner.sha(
                runner.read_bytes(episode / name)
            ):
                raise RuntimeError("{} hash differs".format(name))
        if receipt["timed_out"]:
            raise RuntimeError("preflight timed out")
        receipts.append(receipt)
    events = [
        json.loads(line)
        for line in runner.read_bytes(
            W / ROOT / "attempt_journal.jsonl"
        )
        .decode("utf-8")
        .splitlines()
        if line.strip()
    ]
    if [event["event"] for event in events] != [
        "execution_opened",
        "unit_consumed",
        "unit_complete",
        "unit_consumed",
        "unit_complete",
        "execution_complete",
    ]:
        raise RuntimeError("journal sequence differs")
    contract_sha = runner.sha(runner.read_bytes(runner.CONTRACT))
    summary = runner.summarize(receipts, "preflight", contract_sha)
    summary_path = W / ROOT / "collection_summary.yaml"
    if runner.read_bytes(summary_path) != runner.yaml_bytes(summary):
        raise RuntimeError("preflight summary replay differs")
    reports = {
        receipt["row"]["method"]: receipt["episode_report"]
        for receipt in receipts
    }
    v3 = reports["v3_13_analytic"]
    teb = reports["actual_teb"]
    decisions = v3["result"]["decisions"]
    chain = [
        decision["selected_candidate_id"] for decision in decisions
    ]
    preflight_pass = (
        summary["episode_pass_count"] == 2
        and summary["goal_success_count"] == 2
        and summary["complete_paired_blocks"] == 1
        and summary["all_reports_finite"]
        and summary[
            "maximum_paired_gate_ego_x_difference_m"
        ]
        <= 0.03
        and summary[
            "maximum_paired_gate_actor_coordinate_difference_m"
        ]
        <= 0.05
        and summary[
            "maximum_paired_gate_actor_speed_difference_mps"
        ]
        <= 0.05
        and chain
        == [
            "certified_stop_v000",
            "certified_stop_v000",
            "straight_v120",
        ]
        and teb["result"]["local_plan_count"] > 0
    )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-PREFLIGHT",
        "status": (
            "non_evidence_actual_teb_multiactor_preflight_pass"
            if preflight_pass
            else "non_evidence_actual_teb_multiactor_preflight_fail"
        ),
        "collection_summary_sha256": runner.sha(
            runner.read_bytes(summary_path)
        ),
        "episode_report_sha256": {
            receipt["row"]["method"]: receipt[
                "episode_report_sha256"
            ]
            for receipt in receipts
        },
        "v3_decision_chain": chain,
        "actual_teb_local_plan_count": teb["result"][
            "local_plan_count"
        ],
        "preflight_pass": preflight_pass,
        "included_in_qualification_or_performance_evidence": False,
        "qualification_execution_authorized": False,
        "policy_or_ranker_training_authorized": False,
        "perception_claim_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = runner.yaml_bytes(value)
    review_path = W / REVIEW
    if review_path.exists():
        raise RuntimeError("preflight result review exists")
    review_path.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )
    if not preflight_pass:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
