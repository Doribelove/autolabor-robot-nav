#!/usr/bin/env python3
"""Freeze the incomplete terminal V3-05-A0 execution without inference."""

import hashlib
import json
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/formal/v3_05_a0"
EXECUTION = ROOT / "execution"
PARTIAL = ROOT / "v3_05_a0_partial_evidence_freeze.sha256"
TERMINAL = ROOT / "v3_05_a0_terminal_freeze.sha256"
FAILURE = ROOT / "v3_05_a0_terminal_failure_record.yaml"
JOURNAL = EXECUTION / "v3_05_a0_attempt_journal.jsonl"


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.write_bytes(payload)


def main():
    events = [
        json.loads(line)
        for line in JOURNAL.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    completed = [row for row in events if row["event"] == "unit_complete"]
    failure = events[-1]
    if (
        len(completed) != 205
        or failure["event"] != "terminal_failure"
        or failure["sequence"] != 206
        or failure["remaining_units_forfeited"] != 64
    ):
        raise RuntimeError("terminal accounting differs")
    excluded = {PARTIAL.resolve(), TERMINAL.resolve()}
    files = [
        path
        for path in sorted(EXECUTION.rglob("*"))
        if path.is_file() and path.resolve() not in excluded
    ]
    partial = (
        "\n".join(
            "{}  {}".format(sha(path), path.relative_to(W).as_posix())
            for path in files
        )
        + "\n"
    ).encode("utf-8")
    write_once(PARTIAL, partial)
    terminal_resources = [
        W
        / "artifacts/v3/formal/v3_05_a0/offline_readiness/"
        "v3_05_a0_execution_release.yaml",
        W
        / "artifacts/v3/formal/v3_05_a0/offline_readiness/"
        "v3_05_a0_independent_release_review.yaml",
        W
        / "artifacts/v3/formal/v3_05_a0/offline_readiness/"
        "v3_05_a0_readiness_freeze.sha256",
        W / "experiments/manifests/v3/v3_05_a0_simulation_authorization.yaml",
        FAILURE,
        PARTIAL,
        W
        / "src/tools/thesis_experiment/scripts/"
        "freeze_v3_05_a0_failure.py",
    ]
    terminal = (
        "\n".join(
            "{}  {}".format(sha(path), path.relative_to(W).as_posix())
            for path in terminal_resources
        )
        + "\n"
    ).encode("utf-8")
    write_once(TERMINAL, terminal)
    print(
        "completed=205 consumed=206 forfeited=64 files={} "
        "partial_sha256={} terminal_sha256={}".format(
            len(files),
            hashlib.sha256(partial).hexdigest(),
            hashlib.sha256(terminal).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
