#!/usr/bin/env python3
"""Independent replay of the V3-13-S2 non-evidence preflight."""

import hashlib
import importlib.util
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
SCRIPT_DIR = W / "src/tools/thesis_experiment/scripts"
sys.path.insert(0, str(SCRIPT_DIR))
RUNNER_PATH = SCRIPT_DIR / "run_v3_13_s2_preflight.py"
OUTPUT = W / "artifacts/v3/multiactor/v3_13_s2_preflight"
REVIEW = OUTPUT / "independent_review.yaml"


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s2_preflight_runner", RUNNER_PATH
    )
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    contract = yaml.safe_load(runner.read_bytes(runner.CONTRACT))
    schedule = yaml.safe_load(
        runner.read_bytes(OUTPUT / "preflight_schedule.yaml")
    )["rows"]
    expected_rows = runner.rows(contract)
    if schedule != expected_rows:
        raise RuntimeError("preflight schedule differs")
    receipts = []
    for row, root in zip(
        schedule,
        sorted((OUTPUT / "episodes").iterdir()),
    ):
        receipt = yaml.safe_load(
            runner.read_bytes(root / "matrix_receipt.yaml")
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
            if receipt[field] != sha(runner.read_bytes(root / name)):
                raise RuntimeError("{} hash differs".format(name))
        receipts.append(receipt)
    replay = runner.summarize(receipts, contract)
    result_payload = runner.read_bytes(OUTPUT / "preflight_result.yaml")
    if result_payload != runner.yaml_bytes(replay):
        raise RuntimeError("preflight result replay differs")
    if not replay["preflight_pass"]:
        raise RuntimeError("preflight did not pass")
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S2-PREFLIGHT",
        "status": "v3_13_s2_independent_preflight_result_review_pass",
        "preflight_result_sha256": sha(result_payload),
        "all_four_attempt_one_reports_replayed": True,
        "paired_clock_coordinate_speed_gates_replayed": True,
        "evidence_units_created": 0,
        "output_excluded_from_fit_and_claim_evidence": True,
        "future_evidence_release_design_authorized": True,
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = runner.yaml_bytes(value)
    if REVIEW.exists():
        raise RuntimeError("preflight result review exists")
    runner.atomic_bytes(REVIEW, payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], sha(payload)
        )
    )


if __name__ == "__main__":
    main()
