#!/usr/bin/env python3
"""Assess the preregistered V2-04C TTC qualification gate."""

import argparse
import hashlib
from pathlib import Path

import yaml

from thesis_experiment.v2_04c import (
    assess_ttc_qualification,
    validate_v2_04c_contract,
    validate_v2_04c_r2_amendment,
    validate_v2_04c_r3_amendment,
)


def _validate_r4(amendment, root):
    if (
        amendment.get("stage") != "V2-04C-Q-R4"
        or amendment.get("simulation_only") is not True
        or amendment.get("runtime_ready") is not False
        or amendment.get("training_allowed") is not False
    ):
        raise ValueError("V2-04C R4 boundary drifted")
    for group in ("failed_r3_evidence", "fusion_resources"):
        for item in amendment[group].values():
            if not isinstance(item, dict):
                continue
            path = root / item["path"]
            if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
                raise ValueError("V2-04C R4 resource hash drifted")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path("/home/robot/robot_ws_base_rl"))
    parser.add_argument("--progress", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--stage", default="V2-04C-Q")
    parser.add_argument("--amendment", type=Path, default=None)
    args = parser.parse_args()
    root = args.workspace.resolve()
    contract_path = root / "config/thesis_experiments/v2/v2_04c_refinement_contract.yaml"
    progress_path = (args.progress or
        root / "artifacts/v2/calibration/v2_04c/qualification/v2_04c_q_batch_progress.yaml"
    ).resolve()
    output = (args.output or
        root / "artifacts/v2/calibration/v2_04c/v2_04c_ttc_qualification.yaml"
    ).resolve()
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    validate_v2_04c_contract(contract, root, True)
    progress = yaml.safe_load(progress_path.read_text(encoding="utf-8"))
    report = assess_ttc_qualification(contract, progress, progress_path)
    report["stage"] = args.stage
    if args.amendment is not None:
        amendment_path = args.amendment.resolve()
        amendment = yaml.safe_load(amendment_path.read_text(encoding="utf-8"))
        if args.stage == "V2-04C-Q-R4":
            _validate_r4(amendment, root)
        elif args.stage == "V2-04C-Q-R3":
            validate_v2_04c_r3_amendment(amendment, root, True)
        else:
            validate_v2_04c_r2_amendment(amendment, root, True)
        report["amendment"] = {
            "path": str(amendment_path),
            "sha256": hashlib.sha256(amendment_path.read_bytes()).hexdigest(),
        }
    output.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    print(output)
    return 0 if report["decision"]["start_refinement"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
