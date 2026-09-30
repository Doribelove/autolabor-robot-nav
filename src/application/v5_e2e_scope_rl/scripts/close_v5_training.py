#!/usr/bin/env python3
"""Close and hash the exact 1.8 M-step V5-02-R1 training schedule."""

import argparse
import hashlib
import json
from pathlib import Path

import torch


SCHEDULE = {
    "asym_seed9101": 300000,
    "asym_seed9102": 300000,
    "asym_seed9103": 300000,
    "asym_seed9104": 300000,
    "symmetric_seed9101": 150000,
    "symmetric_seed9102": 150000,
    "no_prior_seed9103": 150000,
    "no_prior_seed9104": 150000,
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--training-root", type=Path, required=True)
    parser.add_argument("--release", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output_directory.mkdir(parents=True, exist_ok=False)
    identity_rows = []
    file_bindings = {}
    for identity, expected_steps in SCHEDULE.items():
        directory = arguments.training_root / identity
        summary_path = directory / "training_summary.json"
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        final_actor_path = directory / "actor_final.pt"
        actor_checkpoint = torch.load(final_actor_path, map_location="cpu", weights_only=False)
        privileged_names = [
            name for name in actor_checkpoint["state_dict"] if "privileg" in name.lower()
        ]
        row = {
            "identity": identity,
            "expected_steps": expected_steps,
            "observed_steps": int(summary["global_steps"]),
            "completed_episodes": int(summary["completed_episodes"]),
            "goal_count": int(summary["goal_count"]),
            "contact_count": int(summary["contact_count"]),
            "mask_violation_count": int(summary["mask_violation_count"]),
            "minimum_physical_clearance_m": float(summary["minimum_physical_clearance_m"]),
            "wall_time_s": float(summary["wall_time_s"]),
            "actor_checkpoint_contains_privileged_modules": actor_checkpoint[
                "contains_privileged_modules"
            ],
            "privileged_parameter_names": privileged_names,
            "pass": (
                summary["status"] == "training_complete"
                and summary["global_steps"] == expected_steps
                and summary["contact_count"] == 0
                and summary["mask_violation_count"] == 0
                and actor_checkpoint["contains_privileged_modules"] is False
                and not privileged_names
            ),
        }
        identity_rows.append(row)
        for path in sorted(directory.rglob("*")):
            if path.is_file():
                file_bindings[str(path.relative_to(arguments.training_root))] = sha256(path)
    report = {
        "status": (
            "v5_02_r1_training_closure_pass"
            if all(row["pass"] for row in identity_rows)
            else "v5_02_r1_training_closure_fail"
        ),
        "release_path": str(arguments.release),
        "release_sha256": sha256(arguments.release),
        "scheduled_environment_steps": 1800000,
        "observed_environment_steps": sum(row["observed_steps"] for row in identity_rows),
        "identity_rows": identity_rows,
        "all_training_contacts": sum(row["contact_count"] for row in identity_rows),
        "all_training_mask_violations": sum(
            row["mask_violation_count"] for row in identity_rows
        ),
        "eligible_budget_remaining": 0,
        "retries_or_extensions_authorized": False,
        "pass": all(row["pass"] for row in identity_rows),
    }
    report_path = arguments.output_directory / "training_closure.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (arguments.output_directory / "training_file_bindings.json").write_text(
        json.dumps(file_bindings, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    raise SystemExit(0 if report["pass"] else 2)


if __name__ == "__main__":
    main()

