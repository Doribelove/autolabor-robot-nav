#!/usr/bin/env python3
import hashlib
import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); ASSESSOR=W/"src/tools/thesis_experiment/scripts/assess_v3_08_a3_integration.py"
def load(path,name):
    spec=importlib.util.spec_from_file_location(name,path); value=importlib.util.module_from_spec(spec); spec.loader.exec_module(value); return value
def main():
    a=load(ASSESSOR,"v3_08_a3_result_review"); v=a.load(a.MODULE,"v3_08_a3_result_module"); value=a.assess(v); value.pop("_evidence"); assessment_bytes=v.BASE_YAML_BYTES(value); v.require((W/v.ASSESSMENT_RELATIVE).read_bytes()==assessment_bytes,"A3 replay differs"); passed=value["overall_pass"] is True
    review={"schema_version":"1.0","architecture_generation":"v3","stage":"V3-08-A3","status":("v3_08_a3_independent_integration_review_pass" if passed else "v3_08_a3_independent_integration_review_valid_negative"),"assessment_sha256":hashlib.sha256(assessment_bytes).hexdigest(),"evidence_freeze_sha256":v.sha256_bytes(v.read_file_once(W/v.EVIDENCE_FREEZE_RELATIVE)),"qualified_for_fresh_performance_evaluation":passed,"receipt_stage_identity_pass":value["gates"]["receipt_stage_identity_pass"],"performance_claim_authorized":False,"runtime_ready":False,"real_vehicle_authorized":False}
    review_bytes=v.BASE_YAML_BYTES(review); a.base.write_once(W/v.INDEPENDENT_RESULT_REVIEW_RELATIVE,review_bytes); resources=[v.ASSESSMENT_RELATIVE,v.EVIDENCE_FREEZE_RELATIVE,v.INDEPENDENT_RESULT_REVIEW_RELATIVE]; terminal=("\\n".join("{}  {}".format(v.sha256_bytes(v.read_file_once(W/item)),item) for item in resources)+"\\n").encode(); a.base.write_once(W/v.TERMINAL_FREEZE_RELATIVE,terminal); print("status={} review_sha256={}".format(review["status"],hashlib.sha256(review_bytes).hexdigest()))
if __name__=="__main__": main()
