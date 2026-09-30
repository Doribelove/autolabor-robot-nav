#!/usr/bin/env python3
import hashlib,importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); BASE=W/"src/tools/thesis_experiment/scripts/fit_v3_07_b0_predictor.py"; MODULE=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0_r1.py"
def load(p,n):
    s=importlib.util.spec_from_file_location(n,p); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
def build(module):
    b=load(BASE,"r1_base_fitter"); b.MODULE=MODULE; model,report=b.build(module); model["stage"]="V3-07-B0-R1"; model["model_id"]="v3_07_b0_r1_observed_mode_spatial_candidate_linear_1"; report["stage"]="V3-07-B0-R1"; return model,report
def main():
    b=load(BASE,"r1_fit_io"); m=load(MODULE,"r1_fit_module"); model,report=build(m); mb=m.yaml_bytes(model); rb=m.yaml_bytes(report); b.write_once(W/m.MODEL_RELATIVE,mb); b.write_once(W/m.CALIBRATION_RELATIVE,rb); freeze=("{}  {}\n{}  {}\n".format(hashlib.sha256(mb).hexdigest(),m.MODEL_RELATIVE,hashlib.sha256(rb).hexdigest(),m.CALIBRATION_RELATIVE)).encode(); b.write_once(W/m.MODEL_FREEZE_RELATIVE,freeze); print("model_sha256={} radius_m={}".format(hashlib.sha256(mb).hexdigest(),model["calibration"]["radius_m"]))
if __name__=="__main__": main()
