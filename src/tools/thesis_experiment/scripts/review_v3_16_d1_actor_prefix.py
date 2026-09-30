#!/usr/bin/env python3
"""Independent review of the V3-16-D1 offline attribution."""

import argparse
import hashlib
import os
from pathlib import Path
import yaml

W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/multiactor/v3_16_d1_actor_prefix_attribution"
OUT = ROOT / "independent_review.yaml"
FREEZE = ROOT / "terminal_freeze.sha256"
def sha(payload): return hashlib.sha256(payload).hexdigest()

def atomic(path, payload, check):
    if check:
        if not path.is_file() or path.read_bytes() != payload: raise RuntimeError("{} differs".format(path))
        return
    if path.exists(): raise RuntimeError("{} exists".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream: stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))

def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    report_path = ROOT / "attribution_report.yaml"; dep_path = ROOT / "dependency_freeze.yaml"; manifest_path = ROOT / "evidence_manifest.sha256"
    report = yaml.safe_load(report_path.read_bytes()); deps = yaml.safe_load(dep_path.read_bytes())
    checks = {
        "dependency_roster_rehashes": all(sha((W / x["path"]).read_bytes()) == x["sha256"] for x in deps["resources"]),
        "all_binding_and_ego_checks_pass": report["checks"]["all_40_receipts_have_action_binding"] and report["checks"]["ego_pairing_gate_passed"],
        "gate_coordinate_is_small": report["max_gate_actor_coordinate_range_m"] < 0.01,
        "branch_coordinate_and_speed_fail": report["max_branch_actor_coordinate_range_m"] > 0.05 and report["max_branch_actor_speed_range_mps"] > 0.05,
        "failed_blocks_exact": report["failed_coordinate_blocks"] == [1, 3, 6] and report["failed_speed_blocks"] == [1],
        "next_factor_is_single_and_postbranch": report["preregistered_next_single_factor"] == {"name": "postbranch_response_enable_gate", "response_enable_sim_s": 12.55, "branch_target_sim_s": 12.5},
        "zero_authority": not report["execution_authorized"] and not report["training_authorized"] and not report["hardware_accessed"] and not report["real_vehicle_used"],
    }
    value = {"schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-16-D1", "status": "v3_16_d1_independent_attribution_review_pass" if all(checks.values()) else "v3_16_d1_independent_attribution_review_fail", "attribution_report_sha256": sha(report_path.read_bytes()), "evidence_manifest_sha256": sha(manifest_path.read_bytes()), "checks": checks, "execution_authorized": False, "hardware_accessed": False, "real_vehicle_used": False}
    payload = yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8"); atomic(OUT, payload, args.check)
    freeze = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in (report_path, dep_path, manifest_path, OUT)).encode("utf-8"); atomic(FREEZE, freeze, args.check)
    print("status={} review_sha256={} terminal_freeze_sha256={}".format(value["status"], sha(payload), sha(freeze)))

if __name__ == "__main__": main()

