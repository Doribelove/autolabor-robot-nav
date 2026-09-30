#!/usr/bin/env python3
"""Assess V3-07-B0 safe-overlap predictor data."""

import argparse
import csv
import hashlib
import importlib.util
import json
import math
from collections import Counter
from pathlib import Path

W=Path("/home/robot/robot_ws_base_rl")
MODULE=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0.py"
def load(path,name):
    s=importlib.util.spec_from_file_location(name,path); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
def write_once(path,payload):
    if path.exists():
        if path.read_bytes()!=payload: raise RuntimeError("refusing overwrite")
    else: path.write_bytes(payload)
def trace(path):
    with path.open(newline="") as f: return [{k:float(v) for k,v in r.items()} for r in csv.DictReader(f)]
def assess(m):
    c=m.load_yaml_relative(W,m.CONTRACT_RELATIVE); p=m.load_yaml_relative(W,m.PREREGISTRATION_RELATIVE); m.validate_contract(c,p)
    schedule=m.load_yaml_relative(W,m.SCHEDULE_RELATIVE); m.require(schedule==m.exact_schedule(c),"B0 schedule differs")
    journal=[json.loads(x) for x in (W/m.JOURNAL_RELATIVE).read_text().splitlines() if x.strip()]
    m.require(journal[-1]["event"]=="execution_complete" and len([x for x in journal if x["event"]=="unit_complete"])==50,"B0 journal incomplete")
    rules=c["data_qualification_gates"]; radius=float(rules["conservative_combined_radius_m"])
    counts=Counter(); mode_match=0; prefix_error=0.; min_clear=float("inf"); violations=0; nonfinite=0
    evidence=[m.JOURNAL_RELATIVE]
    for row in schedule["rows"]:
        root=m.EXECUTION_RELATIVE/"attempts"/row["unit_id"]; rp=root/"attempt_receipt.yaml"; rawp=root/"raw_pose_trace.csv"; wp=root/"world.sdf"
        receipt=m.load_yaml_relative(W,rp); raw=m.read_file_once(W/rawp)
        m.require(receipt["row"]==row and receipt["raw_csv_sha256"]==m.sha256_bytes(raw),"B0 receipt differs")
        rows=trace(W/rawp); metrics=m.trace_metrics(rows,row)
        m.require(receipt["metrics_preview"]==metrics,"B0 metrics preview differs")
        mode_match+=metrics["observed_mode"]==row["actor_policy_evaluator_only"]
        counts[(row["role_evaluator_only"],metrics["observed_mode"],row["candidate_id_evaluator_only"])]+=1
        prefix_error=max(prefix_error,math.hypot(m.interpolate(rows,"ego_x_m",.5)+2.6,m.interpolate(rows,"ego_y_m",.5)))
        clearance=min(math.hypot(x["ego_x_m"]-x["actor_x_m"],x["ego_y_m"]-x["actor_y_m"])-radius for x in rows)
        min_clear=min(min_clear,clearance); violations+=clearance<0
        nonfinite+=sum(not math.isfinite(v) for v in [metrics["actor_conflict_coordinate_at_observation_m"],metrics["actor_observed_speed_mps"],clearance]+metrics["target_actor_conflict_coordinates_m"])
        evidence.extend((rp,rawp,wp))
    eligible=[("cautious","certified_stop_v000"),("cautious","spatial_left_v120"),("hesitant","certified_stop_v000"),("hesitant","spatial_left_v120"),("assertive","certified_stop_v000")]
    metrics={"completed_rows":50,"training_rows":sum(v for (role,_,_),v in counts.items() if role=="training"),"calibration_rows":sum(v for (role,_,_),v in counts.items() if role=="calibration"),"common_prefix_position_error_max_m":prefix_error,"observed_mode_matches_evaluator_rate":mode_match/50.0,"eligible_cell_count":len(eligible),"minimum_training_rows_per_eligible_cell":min(counts[("training",mode,candidate)] for mode,candidate in eligible),"minimum_calibration_rows_per_eligible_cell":min(counts[("calibration",mode,candidate)] for mode,candidate in eligible),"assertive_spatial_row_count":sum(v for (role,mode,candidate),v in counts.items() if mode=="assertive" and candidate=="spatial_left_v120"),"minimum_conservative_clearance_m":min_clear,"conservative_clearance_violation_count":violations,"nonfinite_metric_count":nonfinite}
    gates={"completed_pass":metrics["completed_rows"]==rules["completed_rows"],"split_pass":metrics["training_rows"]==rules["training_rows"] and metrics["calibration_rows"]==rules["calibration_rows"],"prefix_pass":prefix_error<=float(rules["common_prefix_position_error_max_m"]),"mode_pass":metrics["observed_mode_matches_evaluator_rate"]>=float(rules["observed_mode_matches_evaluator_rate_min"]),"cell_count_pass":metrics["eligible_cell_count"]==rules["eligible_cell_count"],"training_cell_pass":metrics["minimum_training_rows_per_eligible_cell"]>=rules["training_rows_per_eligible_cell_min"],"calibration_cell_pass":metrics["minimum_calibration_rows_per_eligible_cell"]>=rules["calibration_rows_per_eligible_cell_min"],"assertive_fail_closed_pass":metrics["assertive_spatial_row_count"]<=rules["assertive_spatial_row_count_max"],"clearance_pass":violations<=rules["conservative_clearance_violation_count_max"],"finite_pass":nonfinite<=rules["nonfinite_metric_count_max"]}
    passed=all(gates.values())
    return {"schema_version":"1.0","architecture_generation":"v3","stage":"V3-07-B0","status":("spatial_predictor_data_qualification_pass" if passed else "spatial_predictor_data_qualification_fail"),"formal_result":False,"execution":{"authorized_units":50,"consumed_units":50,"completed_units":50,"retry_units":0},"metrics":metrics,"gates":gates,"qualified_for_frozen_supervised_model_fit":passed,"overall_pass":passed,"held_out_test_accessed":False,"_evidence":evidence}
def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--check",action="store_true"); args=ap.parse_args(); m=load(MODULE,"b0assessmodule"); value=assess(m); evidence=value.pop("_evidence"); payload=m.yaml_bytes(value); out=W/m.DATA_QUALIFICATION_RELATIVE
    if args.check: m.require(out.is_file() and out.read_bytes()==payload,"B0 data assessment differs")
    else:
        write_once(out,payload); freeze=("\n".join("{}  {}".format(m.sha256_bytes(m.read_file_once(W/x)),x) for x in evidence+[m.DATA_QUALIFICATION_RELATIVE])+"\n").encode(); write_once(W/m.EVIDENCE_FREEZE_RELATIVE,freeze)
    print("status={} qualified={} assessment_sha256={}".format(value["status"],value["qualified_for_frozen_supervised_model_fit"],hashlib.sha256(payload).hexdigest()))
if __name__=="__main__": main()
