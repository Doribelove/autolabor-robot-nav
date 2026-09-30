#!/usr/bin/env python3
"""Independent review of the terminal V3-20 infrastructure failure."""
import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl"); ROOT = W / "artifacts/v3/thesis/v3_20_f1_execution"; OUT = ROOT / "independent_terminal_review.yaml"; FREEZE = ROOT / "terminal_freeze.sha256"
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
    parser=argparse.ArgumentParser();parser.add_argument("--check",action="store_true");args=parser.parse_args();manifest=ROOT/"terminal_evidence_manifest.sha256";assessment_path=ROOT/"terminal_failure_assessment.yaml";assessment=yaml.safe_load(assessment_path.read_bytes());entries=[line.split("  ",1) for line in manifest.read_text().splitlines() if line]
    checks={
        "manifest_rehash":all((W/path).is_file() and sha((W/path).read_bytes())==digest for digest,path in entries),
        "budget_accounting":assessment["consumed_units"]==assessment["completed_units"]+assessment["terminal_failed_units"] and assessment["authorized_units"]==assessment["consumed_units"]+assessment["forfeited_units"],
        "port_inside_ephemeral_range":assessment["linux_ephemeral_port_range"][0]<=assessment["failed_port"]<=assessment["linux_ephemeral_port_range"][1],
        "semantic_execution_not_claimed":assessment["failure_before_semantic_execution"] is True and assessment["formal_or_inferential_result"] is False,
        "nonresume_and_seed_retirement":assessment["retry_resume_replacement_authorized"] is False and assessment["seed_range_8101_8130_reuse_forbidden"] is True,
        "completed_evidence_safe_and_integral":assessment["all_completed_execution_integrity_pass"] is True and assessment["completed_contact_count"]==0,
    }
    value={"schema_version":"1.0","stage":"V3-20-F1","status":"v3_20_f1_independent_terminal_failure_review_pass" if all(checks.values()) else "v3_20_f1_independent_terminal_failure_review_fail","checks":checks,"assessment_sha256":sha(assessment_path.read_bytes()),"manifest_sha256":sha(manifest.read_bytes()),"formal_result":False,"execution_authorized":False,"training_authorized":False,"hardware_accessed":False,"real_vehicle_used":False}
    body=yaml.safe_dump(value,sort_keys=False).encode("utf-8");atomic(OUT,body,args.check);paths=(manifest,assessment_path,ROOT/"exact_schedule.yaml",ROOT/"attempt_journal.jsonl",OUT);freeze="".join("{}  {}\n".format(sha(path.read_bytes()),path.relative_to(W)) for path in paths).encode("utf-8");atomic(FREEZE,freeze,args.check);print("status={} review_sha256={} freeze_sha256={}".format(value["status"],sha(body),sha(freeze)))
if __name__=="__main__":main()

