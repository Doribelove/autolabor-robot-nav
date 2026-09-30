#!/usr/bin/env python3
import argparse,hashlib,importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl");BASE=W/"src/tools/thesis_experiment/scripts/assess_v3_09_q1_qualification.py";MODULE=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_q1_r1.py"
def load(p,n):
 s=importlib.util.spec_from_file_location(n,p);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
b=load(BASE,"q1r1assessbase");b.MODULE=MODULE
def assess(v):
 schedule=v.load_yaml_relative(W,v.SCHEDULE_RELATIVE);correct=0
 for row in schedule["rows"]:
  raw=v.UNTRANSLATED_LOAD(W,v.EXECUTION_RELATIVE/"episodes"/row["unit_id"]/"episode_receipt.yaml");correct+=raw["stage"]=="V3-09-Q1-R1"
 v.require(correct==36,"Q1-R1 receipt stages differ");value=b.assess(v);value["stage"]="V3-09-Q1-R1";value["metrics"]["receipt_stage_identity_rate"]=correct/36.;value["gates"]["stage_identity_pass"]=correct==36;value["overall_pass"]=all(value["gates"].values());value["qualified_for_formal_evaluation"]=value["overall_pass"];value["status"]="paired_performance_qualification_pass" if value["overall_pass"] else "paired_performance_qualification_valid_negative";return value
def main():
 ap=argparse.ArgumentParser();ap.add_argument("--check",action="store_true");args=ap.parse_args();v=load(MODULE,"q1r1assessmodule");value=assess(v);evidence=value.pop("_evidence");payload=v.BASE_YAML_BYTES(value)
 if args.check:v.require((W/v.ASSESSMENT_RELATIVE).read_bytes()==payload,"Q1-R1 assessment differs")
 else:
  b.write_once(W/v.ASSESSMENT_RELATIVE,payload);freeze=("\\n".join("{}  {}".format(v.sha256_bytes(v.read_file_once(W/x)),x) for x in evidence+[v.ASSESSMENT_RELATIVE])+"\\n").encode();b.write_once(W/v.EVIDENCE_FREEZE_RELATIVE,freeze)
 print("status={} qualified={} effect={:.6f} assessment_sha256={}".format(value["status"],value["qualified_for_formal_evaluation"],value["metrics"]["point_relative_reduction"],hashlib.sha256(payload).hexdigest()))
if __name__=="__main__":main()
