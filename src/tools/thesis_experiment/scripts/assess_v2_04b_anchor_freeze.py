#!/usr/bin/env python3
"""Generate the fail-closed final V2-04B Anchor freeze assessment."""

import argparse
from pathlib import Path

import yaml

from thesis_experiment.v2_04b_freeze import assess_anchor_freeze, write_freeze_assessment


WORKSPACE = Path("/home/robot/robot_ws_base_rl")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.workspace.resolve()
    contract_path = root / "config/thesis_experiments/v2/typed_transaction_calibration_contract.yaml"
    plan_path = root / "artifacts/v2/calibration/v2_04b_anchor_screen_plan.yaml"
    progress_path = root / "artifacts/v2/calibration/v2_04b_batch_progress.yaml"
    output = args.output or root / "artifacts/v2/calibration/v2_04b_final_freeze_assessment.yaml"
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    plan = yaml.safe_load(plan_path.read_text(encoding="utf-8"))
    progress = yaml.safe_load(progress_path.read_text(encoding="utf-8"))
    report = assess_anchor_freeze(contract, plan, progress, progress_path)
    write_freeze_assessment(report, output)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
