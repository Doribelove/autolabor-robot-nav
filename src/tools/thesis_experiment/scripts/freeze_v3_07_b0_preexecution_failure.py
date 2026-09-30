#!/usr/bin/env python3
"""Freeze V3-07-B0 pre-execution schedule-key validation failure."""

import hashlib
from pathlib import Path
import yaml
W=Path("/home/robot/robot_ws_base_rl")
ROOT=Path("artifacts/v3/prediction/v3_07_b0")
REPORT=ROOT/"v3_07_b0_preexecution_failure_report.yaml"
FREEZE=ROOT/"v3_07_b0_terminal_freeze.sha256"
RESOURCES=[
    Path("config/thesis_experiments/v3/v3_07_b0_spatial_predictor_data_contract.yaml"),
    Path("experiments/manifests/v3/v3_07_b0_spatial_predictor_data_preregistration.yaml"),
    Path("experiments/manifests/v3/v3_07_b0_simulation_authorization.yaml"),
    ROOT/"offline_readiness/v3_07_b0_execution_release.yaml",
    ROOT/"offline_readiness/v3_07_b0_independent_release_review.yaml",
    ROOT/"offline_readiness/v3_07_b0_readiness_freeze.sha256",
]
def read(p): return (W/p).read_bytes()
def sha(b): return hashlib.sha256(b).hexdigest()
def dump(v): return yaml.safe_dump(v,sort_keys=False,allow_unicode=True,width=1000000).encode()
def write_once(p,b):
    t=W/p
    if t.exists():
        if t.read_bytes()!=b: raise RuntimeError("refusing overwrite")
    else: t.write_bytes(b)
def main():
    if (W/ROOT/"execution").exists(): raise RuntimeError("B0 execution root unexpectedly exists")
    report={"schema_version":"1.0","architecture_generation":"v3","stage":"V3-07-B0","status":"terminal_preexecution_schedule_key_validation_failure","failure_class":"pre_execution_contract_runner_compatibility","confirmed_cause":"The inherited runner requires experimental_unit.schedule_rows while the frozen B0 contract exposed only experimental_unit.exact_rows.","authorized_units":50,"consumed_units":0,"completed_units":0,"forfeited_units":0,"execution_root_present":False,"gazebo_started":False,"outcome_accessed":False,"authorization_reusable":False,"qualified_for_model_fit":False,"simulation_authorized":False,"training_authorized":False,"real_vehicle_authorized":False,"repair_boundary":{"new_stage_release_review_and_authorization_required":True,"only_schedule_rows_compatibility_alias_may_change":True,"data_support_model_and_gates_must_remain_frozen":True,"fresh_stage_identities_required":True},"evidence_resources":[{"path":p.as_posix(),"sha256":sha(read(p)),"size_bytes":len(read(p))} for p in RESOURCES]}
    payload=dump(report); write_once(REPORT,payload)
    paths=RESOURCES+[REPORT]; freeze=("\n".join("{}  {}".format(sha(read(p)),p) for p in paths)+"\n").encode(); write_once(FREEZE,freeze)
    print("status={} report_sha256={} terminal_freeze_sha256={}".format(report["status"],sha(payload),sha(freeze)))
if __name__=="__main__": main()
