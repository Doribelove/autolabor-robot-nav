import json
import math
import sys
from pathlib import Path
import yaml
W=Path("/home/robot/robot_ws_base_rl"); PACKAGE=W/"src/application/v3_scope_planner/src"
if str(PACKAGE) not in sys.path: sys.path.insert(0,str(PACKAGE))
from v3_scope_planner import spatial_core,spatial_core_r1
def test_r1_changes_only_nonfinite_rejected_diagnostic():
    planner=yaml.safe_load((W/"artifacts/v3/planner/v3_08_a0/v3_08_a0_effective_planner_contract.yaml").read_text())
    model=yaml.safe_load((W/"artifacts/v3/prediction/v3_07_b0_r1/model/v3_07_b0_r1_spatial_candidate_predictor.yaml").read_text())
    snapshot={"scene_snapshot_id":"r1","world_model_seq":1,"snapshot_age_s":0.0,"predictor_latency_ms":0.0,"verifier_latency_ms":0.0,"ego":{"x_m":-2.6,"y_m":0.0,"yaw_rad":0.0,"speed_mps":0.8},"actor":{"coordinate_m":-2.6,"speed_mps":1.35,"path_angle_deg":90.0,"monotone_coordinate_certified":True},"health":{"localization_ok":True,"transform_ok":True,"actor_track_ok":True,"command_chain_ok":True}}
    old=spatial_core.plan(snapshot,model,planner); new=spatial_core_r1.plan(snapshot,model,planner)
    assert old["decision"]==new["decision"] and old["candidate_set"]==new["candidate_set"]
    for left,right in zip(old["evaluations"],new["evaluations"]):
        assert left["candidate_id"]==right["candidate_id"] and left["accepted"]==right["accepted"] and left["rejection_codes"]==right["rejection_codes"]
    assert any(not math.isfinite(x["minimum_clearance_m"]) for x in old["evaluations"])
    assert all(math.isfinite(x["minimum_clearance_m"]) for x in new["evaluations"])
    json.dumps(new,allow_nan=False)
