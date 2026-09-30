#!/usr/bin/env python3
import argparse,hashlib,importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); BASE=W/"src/tools/thesis_experiment/scripts/assess_v3_07_b0_data.py"; MODULE=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0_r1.py"
def load(p,n):
    s=importlib.util.spec_from_file_location(n,p); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
def assess(module):
    b=load(BASE,"r1_base_assessor"); b.MODULE=MODULE; value=b.assess(module); value["stage"]="V3-07-B0-R1"; return value
def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--check",action="store_true"); args=ap.parse_args(); b=load(BASE,"r1_assessor_io"); m=load(MODULE,"r1_assessor_module"); value=assess(m); evidence=value.pop("_evidence"); payload=m.yaml_bytes(value); out=W/m.DATA_QUALIFICATION_RELATIVE
    if args.check: m.require(out.is_file() and out.read_bytes()==payload,"R1 assessment differs")
    else:
        b.write_once(out,payload); freeze=("\n".join("{}  {}".format(m.sha256_bytes(m.read_file_once(W/x)),x) for x in evidence+[m.DATA_QUALIFICATION_RELATIVE])+"\n").encode(); b.write_once(W/m.EVIDENCE_FREEZE_RELATIVE,freeze)
    print("status={} qualified={} assessment_sha256={}".format(value["status"],value["qualified_for_frozen_supervised_model_fit"],hashlib.sha256(payload).hexdigest()))
if __name__=="__main__": main()
