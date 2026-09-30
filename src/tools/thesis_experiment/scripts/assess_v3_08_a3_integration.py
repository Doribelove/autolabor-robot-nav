#!/usr/bin/env python3
import argparse
import hashlib
import importlib.util
from pathlib import Path
import sys
W=Path("/home/robot/robot_ws_base_rl"); PACKAGE=W/"src/application/v3_scope_planner/src"
if str(PACKAGE) not in sys.path: sys.path.insert(0,str(PACKAGE))
from v3_scope_planner import spatial_core_r1
BASE=W/"src/tools/thesis_experiment/scripts/assess_v3_08_a1_integration.py"; MODULE=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_08_a3.py"
def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path); value=importlib.util.module_from_spec(spec); spec.loader.exec_module(value); return value
base=load(BASE,"v3_08_a3_assessment_base"); base.MODULE=MODULE; base.spatial_core=spatial_core_r1
def assess(v):
    contract=v.load_yaml_relative(W,v.CONTRACT_RELATIVE); schedule=v.load_yaml_relative(W,v.SCHEDULE_RELATIVE); correct=0
    for row in schedule["rows"]:
        path=v.EXECUTION_RELATIVE/"episodes"/row["unit_id"]/"episode_receipt.yaml"; raw=v.RAW_LOAD(W,path); correct+=raw["stage"]=="V3-08-A3"
    v.require(correct==18,"A3 raw receipt stage identity differs")
    value=base.assess(v); value["stage"]="V3-08-A3"; value["metrics"]["receipt_stage_identity_rate"]=correct/18.0; value["gates"]["receipt_stage_identity_pass"]=correct==18
    value["overall_pass"]=all(value["gates"].values()); value["qualified_for_fresh_performance_evaluation"]=value["overall_pass"]
    value["status"]="spatial_planner_simulation_integration_pass" if value["overall_pass"] else "spatial_planner_simulation_integration_fail"
    return value
def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--check",action="store_true"); args=parser.parse_args(); v=load(MODULE,"v3_08_a3_assessment_module"); value=assess(v); evidence=value.pop("_evidence"); assessment_bytes=v.BASE_YAML_BYTES(value)
    if args.check: v.require((W/v.ASSESSMENT_RELATIVE).read_bytes()==assessment_bytes,"A3 assessment differs")
    else:
        base.write_once(W/v.ASSESSMENT_RELATIVE,assessment_bytes); freeze=("\\n".join("{}  {}".format(v.sha256_bytes(v.read_file_once(W/item)),item) for item in evidence+[v.ASSESSMENT_RELATIVE])+"\\n").encode(); base.write_once(W/v.EVIDENCE_FREEZE_RELATIVE,freeze)
    print("status={} qualified={} assessment_sha256={}".format(value["status"],value["qualified_for_fresh_performance_evaluation"],hashlib.sha256(assessment_bytes).hexdigest()))
if __name__=="__main__": main()
