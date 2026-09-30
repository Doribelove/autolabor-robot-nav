#!/usr/bin/env python3
import hashlib,importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); A=W/"src/tools/thesis_experiment/scripts/assess_v3_07_b0_data.py"
def load(p,n):
    s=importlib.util.spec_from_file_location(n,p); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
def main():
    a=load(A,"b0datareview"); m=a.load(a.MODULE,"b0datareviewmodule"); value=a.assess(m); value.pop("_evidence"); payload=m.yaml_bytes(value)
    m.require((W/m.DATA_QUALIFICATION_RELATIVE).read_bytes()==payload,"B0 data differs"); passed=value["overall_pass"] is True
    review={"schema_version":"1.0","architecture_generation":"v3","stage":"V3-07-B0","status":("v3_07_b0_independent_data_review_pass" if passed else "v3_07_b0_independent_data_review_fail"),"assessment_sha256":hashlib.sha256(payload).hexdigest(),"evidence_freeze_sha256":m.sha256_bytes(m.read_file_once(W/m.EVIDENCE_FREEZE_RELATIVE)),"qualified_for_frozen_supervised_model_fit":passed,"held_out_test_accessed":False,"training_authorized":passed,"reinforcement_learning_authorized":False,"real_vehicle_authorized":False}
    rp=m.yaml_bytes(review); a.write_once(W/m.DATA_REVIEW_RELATIVE,rp); print("status={} review_sha256={}".format(review["status"],hashlib.sha256(rp).hexdigest()))
if __name__=="__main__": main()
