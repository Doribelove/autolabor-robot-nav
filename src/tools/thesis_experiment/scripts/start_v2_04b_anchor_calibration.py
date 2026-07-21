#!/usr/bin/env python3
"""Validate and materialize the calibration-only V2-04B Anchor screen."""

import argparse
from pathlib import Path

from thesis_experiment.v2_04b_calibration import (
    build_anchor_calibration_plan,
    write_anchor_calibration_plan,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument(
        "--contract",
        default="config/thesis_experiments/v2/typed_transaction_calibration_contract.yaml",
    )
    parser.add_argument(
        "--output",
        default="artifacts/v2/calibration/v2_04b_anchor_screen_plan.yaml",
    )
    args = parser.parse_args()
    root = args.workspace.resolve()
    destination = (root / args.output).resolve()
    destination.relative_to((root / "artifacts/v2").resolve())
    plan = build_anchor_calibration_plan(root / args.contract, root)
    write_anchor_calibration_plan(plan, destination)
    print(destination)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
