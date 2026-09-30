#!/usr/bin/env python3
"""Deterministically replay all P1-E1 raw resources and freeze four-gate results."""

import argparse
import json
import os
from pathlib import Path
import tempfile

import yaml

from thesis_experiment.v2_04g_r6_i1_p1_e1_replay import build_terminal_assessment


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SCHEDULE = WORKSPACE / (
    "artifacts/v2/integration/v2_04g_r6_i1/p1_r1_offline_readiness/"
    "v2_04g_p1_r1_exact_schedule.yaml"
)
OUTPUT_ROOT = WORKSPACE / "artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution"


def _atomic(path, payload):
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".tmp.", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument(
        "--journal-root",
        default="artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution/journals",
    )
    args = parser.parse_args()
    schedule = yaml.safe_load(SCHEDULE.read_text(encoding="utf-8"))
    normalized, replay, assessment = build_terminal_assessment(
        schedule, Path(args.journal_root), args.release_sha256, WORKSPACE
    )
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    _atomic(
        OUTPUT_ROOT / "normalized_terminal_matrix.json",
        (json.dumps(normalized, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8"),
    )
    _atomic(
        OUTPUT_ROOT / "raw_replay_report.yaml",
        yaml.safe_dump(replay, sort_keys=False).encode("utf-8"),
    )
    _atomic(
        OUTPUT_ROOT / "p1_e1_four_gate_assessment.yaml",
        yaml.safe_dump(assessment, sort_keys=False).encode("utf-8"),
    )
    print(yaml.safe_dump({
        "raw_replay_status": replay["status"],
        "assessment_status": assessment["assessment_status"],
        "four_gate_decision": assessment["four_gate_decision"],
        "claim_authorized": assessment["claim_authorized"],
    }, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
