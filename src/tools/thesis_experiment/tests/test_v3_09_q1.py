import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); M=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_q1.py"
def test_q1_paired_counterbalanced_schedule():
    s=importlib.util.spec_from_file_location("q1test",M); m=importlib.util.module_from_spec(s); s.loader.exec_module(m); c=m.load_yaml_relative(W,m.CONTRACT_RELATIVE); p=m.load_yaml_relative(W,m.PREREGISTRATION_RELATIVE); m.validate_contract(c,p); schedule=m.exact_schedule(c)
    assert len(schedule["rows"])==36 and sorted({x["seed"] for x in schedule["rows"]})==list(range(6711,6717))
    assert {x["method"] for x in schedule["rows"]}=={"v3_spatial_safe","fixed_v120_shielded"}
    assert c["scene"]["actor"]["endpoint_after_conflict_m"]==3.5
