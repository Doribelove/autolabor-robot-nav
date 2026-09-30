import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl")
MODULE=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_a1.py"
def load():
    s=importlib.util.spec_from_file_location("v3_07_a1_tested",MODULE); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
def test_v3_07_a1_schedule_and_mirrored_waypoints():
    m=load(); c=m.load_yaml_relative(W,m.CONTRACT_RELATIVE); p=m.load_yaml_relative(W,m.PREREGISTRATION_RELATIVE)
    m.validate_contract(c,p); schedule=m.exact_schedule(c)
    assert len(schedule["rows"])==36 and len({r["unit_id"] for r in schedule["rows"]})==36
    base={r["candidate_id_evaluator_only"]:r for r in schedule["rows"] if r["block_id"]=="ident_6541" and r["actor_policy_evaluator_only"]=="cautious"}
    left=m.ego_waypoints(c,base["spatial_left_v120"]); right=m.ego_waypoints(c,base["spatial_right_v120"])
    assert len(left)==len(right)
    assert max(abs(a[1][1]+b[1][1]) for a,b in zip(left,right))<1e-12
    assert max(abs(a[1][0]-b[1][0]) for a,b in zip(left,right))<1e-12
