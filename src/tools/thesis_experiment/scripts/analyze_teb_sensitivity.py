#!/usr/bin/env python3
"""Analyze T07 calibration CSVs and emit an evidence-backed A_TEB YAML."""

import argparse
import json
from pathlib import Path
import sys

import yaml

from thesis_experiment.calibration import (
    CalibrationError, analyze_sensitivity, build_mapping_document,
    load_observations, validate_frozen_mapping,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="calibration observation/episode CSV files")
    parser.add_argument("--mapping-version", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--report-json", help="optional detailed paired-sensitivity report")
    parser.add_argument("--min-pairs", type=int, default=2)
    parser.add_argument("--min-sign-consistency", type=float, default=0.75)
    parser.add_argument("--min-abs-sensitivity", type=float, default=1e-9)
    parser.add_argument("--top-k-per-eta", type=int, default=3)
    parser.add_argument("--freeze", action="store_true",
                        help="fail closed unless all observations are paired and evidenced")
    args = parser.parse_args(argv)
    try:
        observations = load_observations(args.inputs)
        report = analyze_sensitivity(
            observations, min_pairs=args.min_pairs,
            min_sign_consistency=args.min_sign_consistency,
            min_abs_sensitivity=args.min_abs_sensitivity,
            top_k_per_eta=args.top_k_per_eta,
        )
        document = build_mapping_document(report, args.mapping_version, freeze=args.freeze)
        if args.freeze:
            validate_frozen_mapping(document)
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(yaml.safe_dump(document, sort_keys=False, allow_unicode=True), encoding="utf-8")
        if args.report_json:
            report_path = Path(args.report_json)
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(document["sha256"])
        return 0
    except (CalibrationError, OSError, yaml.YAMLError) as exc:
        print("T07 calibration rejected: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
