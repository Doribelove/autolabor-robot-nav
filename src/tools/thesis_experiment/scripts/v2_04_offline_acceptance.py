#!/usr/bin/env python3
"""Run deterministic V2-04 Anchor/action transaction acceptance."""

import argparse
from pathlib import Path

from thesis_experiment.v2_04_acceptance import run_v2_04_acceptance, write_v2_04_acceptance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument(
        "--anchor-bank",
        default="src/application/teb_mode_manager/config/v2_04_anchor_bank_candidate.yaml",
    )
    parser.add_argument(
        "--output",
        default="artifacts/v2/component_acceptance/v2_04_rule_loop_acceptance.yaml",
    )
    args = parser.parse_args()
    report = run_v2_04_acceptance(args.workspace / args.anchor_bank)
    write_v2_04_acceptance(report, args.workspace / args.output)
    print(args.workspace / args.output)
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
