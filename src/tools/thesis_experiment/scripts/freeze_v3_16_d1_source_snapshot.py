#!/usr/bin/env python3
"""Preserve exact pre-V3-16 implementation bytes referenced by D1."""

import argparse
import hashlib
import os
from pathlib import Path

import yaml

W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/multiactor/v3_16_d1_actor_prefix_attribution/source_snapshot"
SOURCE = W / "src/tools/thesis_experiment/scripts/freeze_v3_16_d1_source_snapshot.py"
INPUTS = (
    W / "src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp",
    W / "src/tools/thesis_experiment/scripts/v3_13_multiactor_design.py",
)
NAMES = ("v3_reactive_actor_plugin_pre_v3_16.cpp", "v3_13_multiactor_design_pre_v3_16.py")
REPORT = ROOT / "snapshot_report.yaml"
MANIFEST = ROOT / "snapshot_manifest.sha256"
def sha(payload): return hashlib.sha256(payload).hexdigest()

def atomic(path, payload, check):
    if check:
        if not path.is_file() or path.read_bytes() != payload: raise RuntimeError("{} differs".format(path))
        return
    if path.exists(): raise RuntimeError("{} exists".format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("wb") as stream: stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(tmp), str(path))

def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    copies = [(ROOT / name, path.read_bytes()) for name, path in zip(NAMES, INPUTS)]
    report = {"schema_version": "1.0", "stage": "V3-16-D1-R1", "status": "pre_factor_source_snapshot_closed", "resources": [{"source_path": str(path.relative_to(W)), "source_sha256": sha(payload), "snapshot_path": str(target.relative_to(W)), "snapshot_sha256": sha(payload)} for (target, payload), path in zip(copies, INPUTS)], "execution_authorized": False, "hardware_accessed": False, "real_vehicle_used": False}
    report_payload = yaml.safe_dump(report, sort_keys=False).encode("utf-8")
    manifest_payload = ("".join("{}  {}\n".format(sha(payload), target.relative_to(W)) for target, payload in copies) + "{}  {}\n".format(sha(report_payload), REPORT.relative_to(W)) + "{}  {}\n".format(sha(SOURCE.read_bytes()), SOURCE.relative_to(W))).encode("utf-8")
    for path, payload in copies + [(REPORT, report_payload), (MANIFEST, manifest_payload)]: atomic(path, payload, args.check)
    print("status=pre_factor_source_snapshot_closed report_sha256={} manifest_sha256={}".format(sha(report_payload), sha(manifest_payload)))

if __name__ == "__main__": main()

