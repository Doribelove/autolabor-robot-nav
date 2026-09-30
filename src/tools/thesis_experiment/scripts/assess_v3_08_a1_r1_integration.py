#!/usr/bin/env python3
import argparse
import hashlib
import importlib.util
from pathlib import Path
W = Path("/home/robot/robot_ws_base_rl")
import sys
PACKAGE = W / "src/application/v3_scope_planner/src"
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))
from v3_scope_planner import spatial_core_r1
BASE = W / "src/tools/thesis_experiment/scripts/assess_v3_08_a1_integration.py"
MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_08_a1_r1.py"
def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path); value=importlib.util.module_from_spec(spec); spec.loader.exec_module(value); return value
base=load(BASE,"v3_08_a1_r1_assessment_base"); base.MODULE=MODULE; base.spatial_core=spatial_core_r1
def assess(v):
    value=base.assess(v); value["stage"]="V3-08-A1-R1"; return value
def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--check",action="store_true"); args=parser.parse_args()
    v=load(MODULE,"v3_08_a1_r1_assessment_module"); value=assess(v); evidence=value.pop("_evidence"); assessment_bytes=v.yaml_bytes(value)
    if args.check: v.require((W/v.ASSESSMENT_RELATIVE).read_bytes()==assessment_bytes,"R1 assessment differs")
    else:
        base.write_once(W/v.ASSESSMENT_RELATIVE,assessment_bytes)
        freeze=("\\n".join("{}  {}".format(v.sha256_bytes(v.read_file_once(W/item)),item) for item in evidence+[v.ASSESSMENT_RELATIVE])+"\\n").encode()
        base.write_once(W/v.EVIDENCE_FREEZE_RELATIVE,freeze)
    print("status={} qualified={} assessment_sha256={}".format(value["status"],value["qualified_for_fresh_performance_evaluation"],hashlib.sha256(assessment_bytes).hexdigest()))
if __name__=="__main__": main()
