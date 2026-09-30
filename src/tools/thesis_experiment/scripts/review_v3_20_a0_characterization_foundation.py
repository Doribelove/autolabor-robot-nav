#!/usr/bin/env python3
"""Independent review of V3-20 failure-inclusive release readiness."""
import argparse
import hashlib
import os
from collections import Counter
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_20_a0_characterization_foundation"
OUT = ROOT / "independent_release_review.yaml"
FREEZE = ROOT / "terminal_freeze.sha256"


def sha(data): return hashlib.sha256(data).hexdigest()


def atomic(path, data, check):
    if check:
        if not path.is_file() or path.read_bytes() != data: raise RuntimeError("{} differs".format(path))
        return
    if path.exists(): raise RuntimeError("{} exists".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream: stream.write(data); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    manifest = ROOT / "foundation_manifest.sha256"; release_path = ROOT / "execution_release.yaml"; auth_path = ROOT / "simulation_authorization.yaml"; schedule_path = ROOT / "formal_exact_schedule.yaml"
    release = yaml.safe_load(release_path.read_bytes()); auth = yaml.safe_load(auth_path.read_bytes()); schedule = yaml.safe_load(schedule_path.read_bytes())
    contract = yaml.safe_load((W / "config/thesis_experiments/v3/v3_20_a0_failure_inclusive_characterization_contract.yaml").read_bytes())
    q1 = yaml.safe_load((W / "artifacts/v3/thesis/v3_19_q1_execution/qualification_assessment.yaml").read_bytes())
    entries = [line.split("  ", 1) for line in manifest.read_text().splitlines() if line]
    per_block = Counter(row["block_index"] for row in schedule["rows"]); per_stratum = Counter((row["scene_id"], row["actor_policy"]) for row in schedule["rows"] if row["method"] == "v3_spatial_safe")
    checks = {
        "manifest_rehash": all((W / path).is_file() and sha((W / path).read_bytes()) == digest for digest, path in entries),
        "v3_19_negative_preserved": q1["qualification_pass"] is False and q1["formal_execution_authorized"] is False,
        "no_retired_seed_reuse": min(row["seed"] for row in schedule["rows"]) == 8101 and max(row["seed"] for row in schedule["rows"]) == 8130,
        "complete_balanced_120_schedule": len(schedule["rows"]) == 120 and set(per_block.values()) == {4} and set(per_stratum.values()) == {5},
        "four_methods_each_block": all({row["method"] for row in schedule["rows"] if row["block_index"] == block} == set(contract["frozen_methods"]) for block in per_block),
        "failure_inclusion_not_success_selection": contract["inclusion"]["every_consumed_identity_retained"] is True and release["minimum_task_success_execution_gate"] is False,
        "runtime_bindings_rehash": all(sha((W / item["path"]).read_bytes()) == item["sha256"] for item in release["runtime_bindings"]),
        "simulation_authority_bounded": auth["execution_authorized"] is True and auth["training_authorized"] is False and auth["hardware_access_authorized"] is False and auth["real_vehicle_authorized"] is False,
        "exploratory_claim_scope_explicit": contract["claim_boundary"]["study_kind"] == "exploratory_followup_characterization" and contract["statistics"]["no_positive_effect_gate"] is True,
    }
    value = {
        "schema_version": "1.0", "stage": "V3-20-A0",
        "status": "v3_20_a0_independent_release_review_pass" if all(checks.values()) else "v3_20_a0_independent_release_review_fail",
        "checks": checks, "release_sha256": sha(release_path.read_bytes()), "manifest_sha256": sha(manifest.read_bytes()),
        "execution_ready": all(checks.values()), "formal_units": 120,
        "training_authorized": False, "hardware_access_authorized": False, "real_vehicle_authorized": False,
    }
    body = yaml.safe_dump(value, sort_keys=False).encode("utf-8"); atomic(OUT, body, args.check)
    paths = (manifest, release_path, auth_path, schedule_path, OUT)
    freeze = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in paths).encode("utf-8"); atomic(FREEZE, freeze, args.check)
    print("status={} release_sha256={} review_sha256={}".format(value["status"], value["release_sha256"], sha(body)))


if __name__ == "__main__": main()

