#!/usr/bin/env python3
"""Deterministically assess and seal V3-16-S1 raw evidence."""
import argparse,hashlib,importlib.util,os
from pathlib import Path
import yaml
W=Path("/home/robot/robot_ws_base_rl");ROOT=W/"artifacts/v3/multiactor/v3_16_s1_response_gated_support";OUT=ROOT/"assessment.yaml";MANIFEST=ROOT/"evidence_manifest.sha256"
def sha(b):return hashlib.sha256(b).hexdigest()
def atomic(p,b,check):
 if check:
  if not p.is_file() or p.read_bytes()!=b:raise RuntimeError("{} differs".format(p))
  return
 if p.exists():raise RuntimeError("{} exists".format(p))
 q=p.with_name(p.name+".tmp")
 with q.open("wb") as s:s.write(b);s.flush();os.fsync(s.fileno())
 os.replace(str(q),str(p))
def main():
 p=argparse.ArgumentParser();p.add_argument("--check",action="store_true");a=p.parse_args();schedule=yaml.safe_load((ROOT/"exact_schedule.yaml").read_bytes());terminal=yaml.safe_load((ROOT/"terminal_result.yaml").read_bytes());contract_path=W/"config/thesis_experiments/v3/v3_16_s1_response_gated_support_contract.yaml";contract=yaml.safe_load(contract_path.read_bytes())
 spec=importlib.util.spec_from_file_location("v3_16_assess_binding",W/"src/tools/thesis_experiment/src/thesis_experiment/v3_14_action_binding.py");binding=importlib.util.module_from_spec(spec);spec.loader.exec_module(binding);allowed=tuple(contract["population"]["actions"])
 resources=[contract_path,ROOT/"exact_schedule.yaml",ROOT/"attempt_journal.jsonl",ROOT/"preflight_summary.yaml",ROOT/"terminal_result.yaml"]
 if (ROOT/"evidence_summary.yaml").is_file():resources.append(ROOT/"evidence_summary.yaml")
 receipts=[]
 for row in schedule["rows"]:
  paths=list((ROOT/"episodes").glob("{:03d}_*/matrix_receipt.yaml".format(row["sequence"])))
  if len(paths)!=1:break
  path=paths[0];r=yaml.safe_load(path.read_bytes());binding.verify_runtime_report_binding(row,r["episode_report"],allowed)
  if r["row"]!=row or not r["scientific_inclusion_binding_pass"] or not r["response_gate_binding_pass"] or r["episode_report"]["single_factor"]["treatment"]!=12.55:raise RuntimeError("receipt identity")
  receipts.append(r);resources.extend([path,path.parent/"episode_report.yaml",path.parent/"world.sdf",path.parent/"scene_spec.yaml",path.parent/"actor_specs.yaml",path.parent/"roslaunch.log"])
 expected=terminal["completed_units"];summary=yaml.safe_load((ROOT/"evidence_summary.yaml").read_bytes()) if (ROOT/"evidence_summary.yaml").is_file() else None
 checks={"receipt_count_matches_terminal":len(receipts)==expected,"all_action_response_bindings_replay":len(receipts)==expected,"all_raw_report_hashes_match":all(r["episode_report_sha256"]==sha((resources[5+6*i+1]).read_bytes()) for i,r in enumerate(receipts)),"preflight_excluded":terminal["scientific_episode_count"] in (0,40),"evidence_gate_consistent":summary is None or terminal["evidence_collection_gate_pass"]==summary["collection_gate_pass"],"no_hardware":not terminal["hardware_accessed"] and not terminal["real_vehicle_used"]}
 value={"schema_version":"1.0","architecture_generation":"v3","stage":"V3-16-S1","status":"v3_16_s1_deterministic_result_assessment_pass" if all(checks.values()) else "v3_16_s1_deterministic_result_assessment_fail","terminal_status":terminal["status"],"completed_units":len(receipts),"scientific_episode_count":terminal["scientific_episode_count"],"evidence_collection_gate_pass":terminal.get("evidence_collection_gate_pass",False),"checks":checks,"predictor_fit_authorized":all(checks.values()) and terminal.get("predictor_fit_authorized",False),"actual_TEB_execution_authorized":False,"perception_execution_authorized":False,"training_authorized":False,"hardware_accessed":False,"real_vehicle_used":False};b=yaml.safe_dump(value,sort_keys=False).encode();m="".join("{}  {}\n".format(sha(x.read_bytes()),x.relative_to(W)) for x in resources).encode();atomic(OUT,b,a.check);atomic(MANIFEST,m,a.check);print("status={} assessment_sha256={} manifest_sha256={}".format(value["status"],sha(b),sha(m)))
if __name__=="__main__":main()

