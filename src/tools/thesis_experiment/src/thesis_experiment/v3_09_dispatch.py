"""Method dispatcher for V3-09 paired spatial versus straight qualification."""

import copy
from pathlib import Path
import sys
import yaml

W=Path("/home/robot/robot_ws_base_rl"); PACKAGE=W/"src/application/v3_scope_planner/src"
if str(PACKAGE) not in sys.path: sys.path.insert(0,str(PACKAGE))
from v3_scope_planner import core,spatial_core_r1

BASELINE_PLANNER=yaml.safe_load((W/"artifacts/v3/planner/v3_03_c5/v3_03_c5_effective_planner_contract.yaml").read_bytes())
BASELINE_MODEL=yaml.safe_load((W/"artifacts/v3/prediction/v3_02_d1/model/v3_02_d1_candidate_predictor.yaml").read_bytes())
digest=core.digest

def _baseline_plan(snapshot):
    result=core.plan(snapshot,BASELINE_MODEL,BASELINE_PLANNER)
    desired=next(x for x in result["evaluations"] if x["candidate_id"]=="feasible_v120")
    stop_id=BASELINE_PLANNER["candidate_generator"]["certified_stop_candidate_id"]; stop=next(x for x in result["evaluations"] if x["candidate_id"]==stop_id)
    selected=desired if desired["accepted"] else stop
    if not selected["accepted"]: raise RuntimeError("baseline lacks accepted stop")
    value=copy.deepcopy(result)
    decision={"schema_version":"1.0","scene_snapshot_id":snapshot["scene_snapshot_id"],"candidate_set_digest":result["candidate_set"]["candidate_set_digest"],"selected_candidate_id":selected["candidate_id"],"policy_kind":"fixed_v120_shielded","policy_artifact_digest":core.digest({"method":"fixed_v120_shielded","desired_candidate_id":"feasible_v120","fallback_candidate_id":stop_id}),"safety_fallback_used":not desired["accepted"],"fallback_reason":"FIXED_CANDIDATE_REJECTED_CERTIFIED_STOP" if not desired["accepted"] else ""}
    decision["decision_digest"]=core.digest(decision); value["decision"]=decision
    return value
def plan(snapshot,model,planner):
    if "_fixed_v120_shielded_" in snapshot["scene_snapshot_id"]: return _baseline_plan(snapshot)
    return spatial_core_r1.plan(snapshot,model,planner)
def actor_cleared(snapshot,planner):
    return spatial_core_r1.actor_cleared(snapshot,planner)
