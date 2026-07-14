#!/usr/bin/env python3
"""Validate all T08 bundles and produce the unified paired baseline report."""

import argparse
import csv
import sys
from pathlib import Path

import yaml

from thesis_experiment.baseline_evaluator import (
    evaluate_baselines, load_baseline_contract, read_episode_rows,
)
from thesis_experiment.run_artifacts import (
    RunValidator, sha256_file, write_checksums, write_episode_csv, write_step_csv,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
DEFAULT_RUNS = (
    "t08_fixed_dwa_seed42", "t08_teb_default_seed42",
    "t08_teb_tuned_seed42", "t08_rule_teb_seed42",
)


def _read_csvs(paths):
    rows = []
    for path in paths:
        with Path(path).open("r", encoding="utf-8", newline="") as handle:
            rows.extend(dict(row) for row in csv.DictReader(handle))
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", default=str(
        WORKSPACE / "config/thesis_experiments/t08_baselines.yaml"))
    parser.add_argument("--artifact-root", default=str(WORKSPACE / "artifacts/t08"))
    parser.add_argument("--output-dir", default=str(WORKSPACE / "artifacts/t08/evaluation"))
    args = parser.parse_args(argv)
    root, output = Path(args.artifact_root), Path(args.output_dir)
    manifests = [root / name / "run_manifest.yaml" for name in DEFAULT_RUNS]
    episode_paths = [path.parent / "episodes.csv" for path in manifests]
    step_paths = [path.parent / "steps.csv" for path in manifests]
    episode_schema = WORKSPACE / "docs/thesis_experiment/schemas/episode_metrics_schema.csv"
    step_schema = WORKSPACE / "docs/thesis_experiment/schemas/step_metrics_schema.csv"
    validator = RunValidator(episode_schema, step_schema)
    validations = [validator.validate(path) for path in manifests]
    contract = load_baseline_contract(args.contract)
    rows = read_episode_rows(episode_paths)
    report = evaluate_baselines(rows, contract)
    report.update({
        "task": "T08", "simulation_only": True, "formal_experiment": False,
        "real_vehicle_use_forbidden": True,
        "contract_path": str(Path(args.contract)),
        "contract_sha256": sha256_file(args.contract),
        "source_bundles": [
            {"manifest": str(path), "manifest_sha256": sha256_file(path), "validation": validation}
            for path, validation in zip(manifests, validations)
        ],
        "passed": all(item["valid"] for item in validations) and report["complete_matrix"],
    })
    output.mkdir(parents=True, exist_ok=True)
    combined_episodes = output / "episodes.csv"
    combined_steps = output / "steps.csv"
    report_path = output / "t08_evaluation_report.yaml"
    checksum_path = output / "checksums.sha256"
    write_episode_csv(combined_episodes, rows, episode_schema)
    write_step_csv(combined_steps, _read_csvs(step_paths), step_schema)
    report_path.write_text(yaml.safe_dump(report, allow_unicode=True, sort_keys=False),
                           encoding="utf-8")
    write_checksums(checksum_path, [combined_episodes, combined_steps, report_path], output)
    print(yaml.safe_dump({"passed": report["passed"], "episode_count": report["episode_count"],
                         "report": str(report_path)}, sort_keys=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
