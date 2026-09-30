#!/usr/bin/env python3
"""Fit frozen V3-07-B0 mode/candidate linear predictor and conformal radius."""

import hashlib
import importlib.util
import math
from pathlib import Path
import numpy as np

W=Path("/home/robot/robot_ws_base_rl"); MODULE=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0.py"
def load(p,n):
    s=importlib.util.spec_from_file_location(n,p); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
def features(metrics):
    c=float(metrics["actor_conflict_coordinate_at_observation_m"]); v=float(metrics["actor_observed_speed_mps"])
    return [1.0,c,v,c*c,v*v,c*v]
def write_once(path,payload):
    if path.exists():
        if path.read_bytes()!=payload: raise RuntimeError("refusing overwrite")
    else: path.parent.mkdir(parents=True,exist_ok=True); path.write_bytes(payload)
def build(m):
    c=m.load_yaml_relative(W,m.CONTRACT_RELATIVE); schedule=m.load_yaml_relative(W,m.SCHEDULE_RELATIVE)
    assessment=m.load_yaml_relative(W,m.DATA_QUALIFICATION_RELATIVE); review=m.load_yaml_relative(W,m.DATA_REVIEW_RELATIVE)
    m.require(assessment["qualified_for_frozen_supervised_model_fit"] is True and review["qualified_for_frozen_supervised_model_fit"] is True,"B0 data not qualified")
    train={}; calibration=[]
    for row in schedule["rows"]:
        receipt=m.load_yaml_relative(W,m.EXECUTION_RELATIVE/"attempts"/row["unit_id"]/"attempt_receipt.yaml"); metrics=receipt["metrics_preview"]; key=(metrics["observed_mode"],metrics["candidate_id"])
        sample=(features(metrics),[float(x) for x in metrics["target_actor_conflict_coordinates_m"]])
        if row["role_evaluator_only"]=="training": train.setdefault(key,[]).append(sample)
        else: calibration.append((key,)+sample)
    lam=float(c["predictor_preregistration"]["ridge_lambda"]); horizons=["t1_5","t2_5","t3_5","t4_5"]; heads={}
    for key,samples in sorted(train.items()):
        X=np.asarray([x for x,_ in samples],dtype=float); Y=np.asarray([y for _,y in samples],dtype=float)
        penalty=np.eye(X.shape[1])*lam; penalty[0,0]=0.0
        coefficients=np.linalg.solve(X.T@X+penalty,X.T@Y)
        mode,candidate=key; heads.setdefault(mode,{})[candidate]={h:{"coefficients":[float(x) for x in coefficients[:,i]],"training_rows":len(samples)} for i,h in enumerate(horizons)}
    scores=[]; residuals=[]
    for key,x,y in calibration:
        mode,candidate=key; predictions=[]
        for h in horizons:
            coef=heads[mode][candidate][h]["coefficients"]; predictions.append(sum(a*b for a,b in zip(coef,x)))
        errors=[abs(a-b) for a,b in zip(y,predictions)]; scores.append(max(errors)); residuals.extend(errors)
    coverage=float(c["predictor_preregistration"]["conformal_coverage"]); rank=min(len(scores),int(math.ceil((len(scores)+1)*coverage))); radius=sorted(scores)[rank-1]
    model={"schema_version":"1.0","architecture_generation":"v3","stage":"V3-07-B0","model_id":"v3_07_b0_observed_mode_spatial_candidate_linear_1","status":"frozen_supervised_candidate_predictor","feature_names":c["predictor_preregistration"]["scalar_features"],"horizons_s":c["predictor_preregistration"]["target_horizons_s"],"supported_action_heads":heads,"unsupported_action_policy":{"assertive":["spatial_left_v120"],"behavior":"fail_closed_to_certified_stop"},"calibration":{"method":"split_conformal_max_horizon_absolute_residual","coverage":coverage,"calibration_rows":len(scores),"rank":rank,"radius_m":float(radius)},"reinforcement_learning_steps":0,"held_out_test_accessed":False}
    calibration_report={"schema_version":"1.0","architecture_generation":"v3","stage":"V3-07-B0","status":"spatial_predictor_split_conformal_calibration_complete","training_rows":sum(len(v) for v in train.values()),"calibration_rows":len(scores),"eligible_head_count":len(train),"calibration_radius_m":float(radius),"calibration_score_max_m":max(scores),"absolute_residual_mean_m":sum(residuals)/len(residuals),"absolute_residual_max_m":max(residuals),"calibration_empirical_coverage":sum(s<=radius+1e-12 for s in scores)/len(scores),"held_out_test_accessed":False}
    return model,calibration_report
def main():
    m=load(MODULE,"b0fitmodule"); model,report=build(m); mb=m.yaml_bytes(model); rb=m.yaml_bytes(report); write_once(W/m.MODEL_RELATIVE,mb); write_once(W/m.CALIBRATION_RELATIVE,rb)
    freeze=("{}  {}\n{}  {}\n".format(hashlib.sha256(mb).hexdigest(),m.MODEL_RELATIVE,hashlib.sha256(rb).hexdigest(),m.CALIBRATION_RELATIVE)).encode(); write_once(W/m.MODEL_FREEZE_RELATIVE,freeze)
    print("model_sha256={} radius_m={}".format(hashlib.sha256(mb).hexdigest(),model["calibration"]["radius_m"]))
if __name__=="__main__": main()
