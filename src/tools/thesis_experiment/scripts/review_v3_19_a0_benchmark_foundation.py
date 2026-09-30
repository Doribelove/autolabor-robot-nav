#!/usr/bin/env python3
"""Independent offline review of V3-19 preregistration and release."""
import argparse
import hashlib
import os
from collections import Counter
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_19_a0_benchmark_foundation"
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
    manifest = ROOT / "foundation_manifest.sha256"
    release = yaml.safe_load((ROOT / "execution_release.yaml").read_bytes())
    auth = yaml.safe_load((ROOT / "simulation_authorization.yaml").read_bytes())
    contract = yaml.safe_load((W / "config/thesis_experiments/v3/v3_19_a0_four_method_benchmark_contract.yaml").read_bytes())
    schedules = {phase: yaml.safe_load((ROOT / "{}_exact_schedule.yaml".format(phase)).read_bytes()) for phase in ("qualification", "formal")}
    entries = [line.split("  ", 1) for line in manifest.read_text().splitlines() if line]
    methods = set(contract["methods"]["identifiers"])
    schedule_checks = {}
    for phase, schedule in schedules.items():
        identities = [(row["seed"], row["scene_id"], row["actor_policy"], row["method"], row["attempt"]) for row in schedule["rows"]]
        per_block = Counter(row["block_index"] for row in schedule["rows"])
        schedule_checks[phase] = len(identities) == len(set(identities)) and set(per_block.values()) == {4} and {row["method"] for row in schedule["rows"]} == methods
    checks = {
        "manifest_rehash": all((W / path).is_file() and sha((W / path).read_bytes()) == digest for digest, path in entries),
        "four_roles_and_single_factor": len(methods) == 4 and contract["methods"]["ablation"]["single_factor"] == "candidate_conditioning",
        "fresh_nonoverlapping_seed_ranges": contract["evaluation"]["qualification"]["seed_start"] == 7901 and contract["evaluation"]["formal"]["seed_start"] == 8001,
        "exact_balanced_schedules": all(schedule_checks.values()) and len(schedules["qualification"]["rows"]) == 24 and len(schedules["formal"]["rows"]) == 120,
        "release_bindings_rehash": all(sha((W / item["path"]).read_bytes()) == item["sha256"] for item in release["runtime_bindings"]),
        "qualification_before_formal": release["qualification_required_before_formal"] is True,
        "explicit_simulation_only_authority": auth["execution_authorized"] is True and auth["training_authorized"] is False and auth["hardware_access_authorized"] is False and auth["real_vehicle_authorized"] is False,
        "claims_bounded": contract["claim_boundary"]["perception_or_ood_claim_authorized"] is False and contract["claim_boundary"]["formal_safety_claim_authorized"] is False,
    }
    value = {
        "schema_version": "1.0", "stage": "V3-19-A0",
        "status": "v3_19_a0_independent_release_review_pass" if all(checks.values()) else "v3_19_a0_independent_release_review_fail",
        "checks": checks, "schedule_checks": schedule_checks,
        "release_sha256": sha((ROOT / "execution_release.yaml").read_bytes()),
        "manifest_sha256": sha(manifest.read_bytes()),
        "simulation_execution_ready": all(checks.values()),
        "training_authorized": False, "hardware_access_authorized": False, "real_vehicle_authorized": False,
    }
    body = yaml.safe_dump(value, sort_keys=False).encode("utf-8"); atomic(OUT, body, args.check)
    paths = (manifest, ROOT / "execution_release.yaml", ROOT / "simulation_authorization.yaml", ROOT / "qualification_exact_schedule.yaml", ROOT / "formal_exact_schedule.yaml", ROOT / "v3_19_no_candidate_conditioning_model.yaml", OUT)
    freeze = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in paths).encode("utf-8"); atomic(FREEZE, freeze, args.check)
    print("status={} release_sha256={} review_sha256={}".format(value["status"], value["release_sha256"], sha(body)))


if __name__ == "__main__": main()

