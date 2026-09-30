#!/usr/bin/env python3
import hashlib,importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); F=W/"src/tools/thesis_experiment/scripts/fit_v3_07_b0_predictor.py"
def load(p,n):
    s=importlib.util.spec_from_file_location(n,p); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
def main():
    f=load(F,"b0modelreview"); m=f.load(f.MODULE,"b0modelreviewmodule"); model,report=f.build(m); mb=m.yaml_bytes(model); rb=m.yaml_bytes(report)
    m.require((W/m.MODEL_RELATIVE).read_bytes()==mb and (W/m.CALIBRATION_RELATIVE).read_bytes()==rb,"B0 model differs")
    review={"schema_version":"1.0","architecture_generation":"v3","stage":"V3-07-B0","status":"v3_07_b0_independent_model_review_pass","model_sha256":hashlib.sha256(mb).hexdigest(),"calibration_report_sha256":hashlib.sha256(rb).hexdigest(),"training_rows":report["training_rows"],"calibration_rows":report["calibration_rows"],"eligible_head_count":report["eligible_head_count"],"held_out_test_accessed":False,"runtime_ready":False,"real_vehicle_authorized":False}
    payload=m.yaml_bytes(review); f.write_once(W/m.MODEL_REVIEW_RELATIVE,payload)
    paths=[m.DATA_QUALIFICATION_RELATIVE,m.EVIDENCE_FREEZE_RELATIVE,m.DATA_REVIEW_RELATIVE,m.MODEL_RELATIVE,m.CALIBRATION_RELATIVE,m.MODEL_FREEZE_RELATIVE,m.MODEL_REVIEW_RELATIVE]
    terminal=("\n".join("{}  {}".format(m.sha256_bytes(m.read_file_once(W/p)),p) for p in paths)+"\n").encode(); f.write_once(W/m.TERMINAL_FREEZE_RELATIVE,terminal)
    print("status={} review_sha256={}".format(review["status"],hashlib.sha256(payload).hexdigest()))
if __name__=="__main__": main()
