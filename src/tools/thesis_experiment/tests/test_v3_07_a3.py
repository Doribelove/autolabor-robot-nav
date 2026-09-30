import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); M=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_a3.py"
def load():
    s=importlib.util.spec_from_file_location("a3test",M); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); return m
def test_v3_07_a3_schedule_and_mode_gate():
    m=load(); c=m.load_yaml_relative(W,m.CONTRACT_RELATIVE); p=m.load_yaml_relative(W,m.PREREGISTRATION_RELATIVE); m.validate_contract(c,p)
    schedule=m.exact_schedule(c); assert len(schedule["rows"])==48
    assert sum(r["phase"]=="probe" for r in schedule["rows"])==18
    assert sum(r["phase"]=="certified_stop" for r in schedule["rows"])==18
    assert sum(r["phase"]=="conditional_spatial_left" for r in schedule["rows"])==12
    gate=c["observed_mode_gate"]
    assert m.infer_mode(0.15,gate)=="cautious"
    assert m.infer_mode(0.32,gate)=="hesitant"
    assert m.infer_mode(1.35,gate)=="assertive"
