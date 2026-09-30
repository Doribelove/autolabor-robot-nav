import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); M=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0_r1.py"
def test_v3_07_b0_r1_is_schedule_alias_only():
    s=importlib.util.spec_from_file_location("r1test",M); m=importlib.util.module_from_spec(s); s.loader.exec_module(m)
    c=m.load_yaml_relative(W,m.CONTRACT_RELATIVE); p=m.load_yaml_relative(W,m.PREREGISTRATION_RELATIVE); m.validate_contract(c,p); schedule=m.exact_schedule(c)
    assert c["experimental_unit"]["schedule_rows"]==c["experimental_unit"]["exact_rows"]==50
    assert len(schedule["rows"])==50 and sorted({r["seed"] for r in schedule["rows"]})==list(range(6641,6651))
