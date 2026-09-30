import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); M=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0.py"
def test_v3_07_b0_frozen_support_and_split():
    s=importlib.util.spec_from_file_location("b0test",M); m=importlib.util.module_from_spec(s); s.loader.exec_module(m)
    c=m.load_yaml_relative(W,m.CONTRACT_RELATIVE); p=m.load_yaml_relative(W,m.PREREGISTRATION_RELATIVE); m.validate_contract(c,p)
    schedule=m.exact_schedule(c)
    assert len(schedule["rows"])==50
    assert sum(r["role_evaluator_only"]=="training" for r in schedule["rows"])==30
    assert sum(r["role_evaluator_only"]=="calibration" for r in schedule["rows"])==20
    assert not any(r["actor_policy_evaluator_only"]=="assertive" and r["candidate_id_evaluator_only"]=="spatial_left_v120" for r in schedule["rows"])
