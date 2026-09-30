#!/usr/bin/env python3
"""Deterministically review the exact Arena Q1 release before execution."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import tempfile
from pathlib import Path

import yaml

import run_qualification as runner


def write_atomic_json(path, payload):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=runner.DEFAULT_OUTPUT)
    parser.add_argument("--release-sha256", required=True)
    args = parser.parse_args()

    output_root = runner.require_safe_output_root(args.root)
    release_path = output_root / "q1_execution_release.yaml"
    review_path = output_root / "q1_release_review.json"
    if review_path.exists():
        raise RuntimeError("Q1_RELEASE_REVIEW_ALREADY_EXISTS")
    if (output_root / "q1").exists():
        raise RuntimeError("Q1_EXECUTION_EXISTS_BEFORE_RELEASE_REVIEW")
    if not release_path.is_file():
        raise RuntimeError("Q1_RELEASE_ABSENT")
    observed_release_sha256 = runner.sha256(release_path)
    if observed_release_sha256 != args.release_sha256:
        raise RuntimeError("Q1_RELEASE_REVIEW_SHA_ARGUMENT_MISMATCH")

    release, verified_release_sha256 = runner.verify_release_resources(
        release_path, expected_release_sha256=observed_release_sha256
    )
    if verified_release_sha256 != observed_release_sha256:
        raise RuntimeError("Q1_RELEASE_SINGLE_READ_SHA_MISMATCH")
    expected_release = runner.release_payload("ARENA-PCT-Q1")
    if release != expected_release:
        raise RuntimeError("Q1_RELEASE_EXACT_PAYLOAD_MISMATCH")
    schedule = release.get("schedule", [])
    identities = [
        (int(row["seed"]), str(row["scene"]), str(method))
        for row in schedule
        for method in row["method_order"]
    ]
    expected_identities = [
        (int(row["seed"]), str(row["scene"]), str(method))
        for row in runner.QUALIFICATION_SCHEDULE
        for method in row["order"]
    ]
    if identities != expected_identities or len(set(identities)) != 12:
        raise RuntimeError("Q1_RELEASE_SCHEDULE_INVALID")
    if (
        release.get("stage") != "ARENA-PCT-Q1"
        or not release.get("qualification_only")
        or release.get("formal_result") is not False
        or not release.get("execution_authorized_by_current_user_instruction")
        or release.get("execution_protocol")
        != "two_phase_prepare_review_then_execute"
    ):
        raise RuntimeError("Q1_RELEASE_SEMANTICS_INVALID")

    with tempfile.TemporaryDirectory(prefix="arena_q1_env_review_") as directory:
        environment = runner.runtime_environment(
            Path(directory), 16900, 16901
        )
    credential_like_names = sorted(
        name
        for name in environment
        if any(
            token in name.upper()
            for token in ("KEY", "TOKEN", "SECRET", "PASSWORD", "CREDENTIAL")
        )
    )
    forbidden_path_fragments = (
        "/home/robot/arena_ws",
        "/home/robot/catkin_ws",
        "/home/robot/robot_ws/",
        "ActorCollisionsPlugin/build",
    )
    external_workspace_leaks = sorted(
        "{}={}".format(name, value)
        for name, value in environment.items()
        if any(fragment in str(value) for fragment in forbidden_path_fragments)
    )
    if credential_like_names or external_workspace_leaks:
        raise RuntimeError(
            "Q1_CHILD_ENVIRONMENT_INVALID:{}:{}".format(
                credential_like_names, external_workspace_leaks
            )
        )

    category_counts = {
        name: len(release.get(name, []))
        for name in ("resources", "system_resources", "hospital_assets")
    }
    roster_payload = json.dumps(
        {
            name: release.get(name, [])
            for name in ("resources", "system_resources", "hospital_assets")
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    roster_sha256 = hashlib.sha256(roster_payload).hexdigest()
    source_world_hash_pass = (
        runner.sha256(runner.SOURCE_WORLD)
        == runner.EXPECTED_SOURCE_WORLD_SHA256
    )
    if not source_world_hash_pass:
        raise RuntimeError("Q1_SOURCE_WORLD_HASH_INVALID_AT_REVIEW")
    review = {
        "schema_version": "1.0",
        "stage": "ARENA-PCT-Q1-RELEASE-REVIEW",
        "status": "arena_q1_release_preflight_pass",
        "execution_ready": True,
        "formal_result": False,
        "qualification_only": True,
        "release_path": str(release_path.relative_to(runner.ROOT)),
        "release_sha256": observed_release_sha256,
        "reviewer_source_sha256": runner.sha256(Path(__file__).resolve()),
        "exact_release_payload_match": True,
        "exact_resource_roster_match": True,
        "resource_category_counts": category_counts,
        "resource_roster_sha256": roster_sha256,
        "episode_identities": len(identities),
        "resource_count": sum(
            len(release.get(name, []))
            for name in ("resources", "system_resources", "hospital_assets")
        ),
        "credential_like_child_environment_names": credential_like_names,
        "external_workspace_path_leaks": external_workspace_leaks,
        "source_world_sha256": runner.sha256(runner.SOURCE_WORLD),
        "source_world_hash_pass": source_world_hash_pass,
        "q1_execution_absent_at_review": True,
    }
    write_atomic_json(review_path, review)
    print(
        json.dumps(
            {**review, "release_review_sha256": runner.sha256(review_path)},
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
