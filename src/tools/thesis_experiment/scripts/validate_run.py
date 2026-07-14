#!/usr/bin/env python3
"""Validate a T06 run manifest and its referenced artifacts."""

import argparse
import json
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from thesis_experiment.run_artifacts import RunValidationError, RunValidator  # noqa: E402
from thesis_experiment.schema import SchemaValidationError  # noqa: E402


def _default_schema(name):
    workspace = PACKAGE_ROOT.parents[2]
    return workspace / "docs" / "thesis_experiment" / "schemas" / name


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Validate T06 CSV schemas, references, checksums, IDs, and termination semantics."
    )
    parser.add_argument("manifest", help="Path to the concrete run manifest YAML")
    parser.add_argument(
        "--episode-schema",
        default=str(_default_schema("episode_metrics_schema.csv")),
        help="Frozen episode metric schema CSV",
    )
    parser.add_argument(
        "--step-schema",
        default=str(_default_schema("step_metrics_schema.csv")),
        help="Frozen step metric schema CSV",
    )
    args = parser.parse_args(argv)
    try:
        report = RunValidator(args.episode_schema, args.step_schema).validate(args.manifest)
    except (RunValidationError, SchemaValidationError, OSError) as exc:
        print("INVALID: {}".format(exc), file=sys.stderr)
        return 1
    print(json.dumps(report, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
