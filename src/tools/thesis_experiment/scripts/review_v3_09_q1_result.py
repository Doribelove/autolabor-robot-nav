#!/usr/bin/env python3
import hashlib,importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); A=W/"src/tools/thesis_experiment/scripts/assess_v3_09_q1_qualification.py"
def load(p,n):
    s=importlib.util.spec_from_file_location(n,p); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
def main():
    a=load(A,"q1review"); v=a.load(a.MODULE,"q1reviewmodule"); value=a.assess(v); value.pop("_evidence"); payload=v.BASE_YAML_BYTES(value); v.require((W/v.ASSESSMENT_RELATIVE).read_bytes()==payload,"Q1 replay differs"); passed=value["overall_pass"] is True
    review={"schema_version":"1.0","architecture_generation":"v3","stage":"V3-09-Q1","status":"v3_09_q1_independent_qualification_review_pass" if passed else "v3_09_q1_independent_qualification_review_valid_negative","assessment_sha256":hashlib.sha256(payload).hexdigest(),"evidence_freeze_sha256":v.sha256_bytes(v.read_file_once(W/v.EVIDENCE_FREEZE_RELATIVE)),"qualified_for_formal_evaluation":passed,"performance_claim_authorized":False,"actual_teb_comparison":False,"runtime_ready":False,"real_vehicle_authorized":False}
    rb=v.BASE_YAML_BYTES(review); a.write_once(W/v.INDEPENDENT_RESULT_REVIEW_RELATIVE,rb); resources=[v.ASSESSMENT_RELATIVE,v.EVIDENCE_FREEZE_RELATIVE,v.INDEPENDENT_RESULT_REVIEW_RELATIVE]; terminal=("\\n".join("{}  {}".format(v.sha256_bytes(v.read_file_once(W/x)),x) for x in resources)+"\\n").encode(); a.write_once(W/v.TERMINAL_FREEZE_RELATIVE,terminal); print("status={} review_sha256={}".format(review["status"],hashlib.sha256(rb).hexdigest()))
if __name__=="__main__": main()
