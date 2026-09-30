#!/usr/bin/env python3
"""Independent terminal review of the V3-14-S1 assessment and evidence seal."""

import argparse
import hashlib
import os
from pathlib import Path
import yaml

W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/multiactor/v3_14_s1_multiaction_support"
OUT = ROOT / "independent_result_review.yaml"
def sha(payload): return hashlib.sha256(payload).hexdigest()

def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    assessment_path = ROOT / "assessment.yaml"; manifest_path = ROOT / "evidence_manifest.sha256"
    assessment = yaml.safe_load(assessment_path.read_bytes())
    lines = [line.split("  ", 1) for line in manifest_path.read_text().splitlines() if line]
    checks = {
        "assessment_pass": assessment["status"] == "v3_14_s1_deterministic_result_assessment_pass",
        "manifest_all_resources_rehash": all(sha((W / path).read_bytes()) == digest for digest, path in lines),
        "binding_replay_pass": assessment["checks"]["all_binding_replays_pass"],
        "scientific_count_closed": assessment["scientific_episode_count"] in (0, 40),
        "downstream_authority_scoped": not assessment["actual_TEB_execution_authorized"] and not assessment["perception_execution_authorized"] and not assessment["training_authorized"],
        "no_hardware": not assessment["hardware_accessed"] and not assessment["real_vehicle_used"],
    }
    report = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-14-S1",
        "status": "v3_14_s1_independent_result_review_pass" if all(checks.values()) else "v3_14_s1_independent_result_review_fail",
        "assessment_sha256": sha(assessment_path.read_bytes()), "evidence_manifest_sha256": sha(manifest_path.read_bytes()),
        "terminal_status": assessment["terminal_status"], "checks": checks,
        "predictor_fit_authorized": all(checks.values()) and assessment["predictor_fit_authorized"],
        "actual_TEB_execution_authorized": False, "perception_execution_authorized": False, "training_authorized": False,
        "hardware_accessed": False, "real_vehicle_used": False,
    }
    payload = yaml.safe_dump(report, sort_keys=False, allow_unicode=True).encode("utf-8")
    if args.check:
        if not OUT.is_file() or OUT.read_bytes() != payload: raise RuntimeError("review differs")
    else:
        if OUT.exists(): raise RuntimeError("review exists")
        temporary = OUT.with_name(OUT.name + ".tmp")
        with temporary.open("wb") as stream: stream.write(payload); stream.flush(); os.fsync(stream.fileno())
        os.replace(str(temporary), str(OUT))
    print("status={} review_sha256={}".format(report["status"], sha(payload)))

if __name__ == "__main__": main()

