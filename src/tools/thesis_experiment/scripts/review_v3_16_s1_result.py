#!/usr/bin/env python3
"""Independent terminal review of V3-16-S1."""
import argparse,hashlib,os
from pathlib import Path
import yaml
W=Path("/home/robot/robot_ws_base_rl");ROOT=W/"artifacts/v3/multiactor/v3_16_s1_response_gated_support";OUT=ROOT/"independent_result_review.yaml";FREEZE=ROOT/"terminal_freeze.sha256"
def sha(b):return hashlib.sha256(b).hexdigest()
def atomic(p,b,check):
 if check:
  if not p.is_file() or p.read_bytes()!=b:raise RuntimeError("{} differs".format(p))
  return
 if p.exists():raise RuntimeError("{} exists".format(p))
 q=p.with_name(p.name+".tmp");q.write_bytes(b);os.replace(str(q),str(p))
def main():
 p=argparse.ArgumentParser();p.add_argument("--check",action="store_true");a=p.parse_args();ap=ROOT/"assessment.yaml";mp=ROOT/"evidence_manifest.sha256";assessment=yaml.safe_load(ap.read_bytes());entries=[x.split("  ",1) for x in mp.read_text().splitlines() if x]
 checks={"assessment_pass":assessment["status"]=="v3_16_s1_deterministic_result_assessment_pass","manifest_rehash":all(sha((W/path).read_bytes())==digest for digest,path in entries),"binding_replay":assessment["checks"]["all_action_response_bindings_replay"],"scientific_count_closed":assessment["scientific_episode_count"] in (0,40),"authority_scoped":not assessment["actual_TEB_execution_authorized"] and not assessment["perception_execution_authorized"] and not assessment["training_authorized"],"no_hardware":not assessment["hardware_accessed"] and not assessment["real_vehicle_used"]}
 value={"schema_version":"1.0","stage":"V3-16-S1","status":"v3_16_s1_independent_result_review_pass" if all(checks.values()) else "v3_16_s1_independent_result_review_fail","assessment_sha256":sha(ap.read_bytes()),"evidence_manifest_sha256":sha(mp.read_bytes()),"terminal_status":assessment["terminal_status"],"checks":checks,"predictor_fit_authorized":all(checks.values()) and assessment["predictor_fit_authorized"],"actual_TEB_execution_authorized":False,"perception_execution_authorized":False,"training_authorized":False,"hardware_accessed":False,"real_vehicle_used":False};b=yaml.safe_dump(value,sort_keys=False).encode();atomic(OUT,b,a.check);freeze="".join("{}  {}\n".format(sha(x.read_bytes()),x.relative_to(W)) for x in (ap,mp,OUT)).encode();atomic(FREEZE,freeze,a.check);print("status={} review_sha256={} terminal_freeze_sha256={}".format(value["status"],sha(b),sha(freeze)))
if __name__=="__main__":main()

