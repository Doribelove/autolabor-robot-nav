#!/usr/bin/env python3
"""Apply the preregistered V2-04C selection and write a frozen simulation bank."""

import argparse
from pathlib import Path

import yaml

from thesis_experiment.v2_04c import (
    freeze_v2_04c, validate_v2_04c_contract, write_frozen_outputs,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path("/home/robot/robot_ws_base_rl"))
    parser.add_argument("--progress", type=Path, default=None)
    parser.add_argument("--qualification", type=Path, default=None)
    args = parser.parse_args()
    root = args.workspace.resolve()
    contract_path = root / "config/thesis_experiments/v2/v2_04c_refinement_contract.yaml"
    plan_path = root / "artifacts/v2/calibration/v2_04c/v2_04c_refinement_plan.yaml"
    progress_path = (args.progress or
        root / "artifacts/v2/calibration/v2_04c/refinement/v2_04c_batch_progress.yaml"
    ).resolve()
    qualification_path = (args.qualification or
        root / "artifacts/v2/calibration/v2_04c/v2_04c_ttc_qualification_r2.yaml"
    ).resolve()
    bank_path = root / "src/application/teb_mode_manager/config/v2_04c_anchor_bank_frozen.yaml"
    report_path = root / "artifacts/v2/calibration/v2_04c/v2_04c_freeze_report.yaml"
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    validate_v2_04c_contract(contract, root, True)
    plan = yaml.safe_load(plan_path.read_text(encoding="utf-8"))
    progress = yaml.safe_load(progress_path.read_text(encoding="utf-8"))
    qualification = yaml.safe_load(qualification_path.read_text(encoding="utf-8"))
    source_bank = root / contract["frozen_inputs"]["candidate_anchor_bank"]["path"]
    result = freeze_v2_04c(contract, plan, progress, qualification, source_bank)
    write_frozen_outputs(result, bank_path, report_path)
    print(bank_path)
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
