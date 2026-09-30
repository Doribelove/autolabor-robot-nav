import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); M=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_a2.py"
def test_v3_07_a2_is_duration_only_fresh_repair():
    s=importlib.util.spec_from_file_location("a2test",M); m=importlib.util.module_from_spec(s); s.loader.exec_module(m)
    c=m.load_yaml_relative(W,m.CONTRACT_RELATIVE); p=m.load_yaml_relative(W,m.PREREGISTRATION_RELATIVE); m.validate_contract(c,p)
    schedule=m.exact_schedule(c)
    assert len(schedule["rows"])==36
    assert sorted({r["seed"] for r in schedule["rows"]})==[6561,6562,6563,6564]
    assert c["single_factor_repair"]["qualification_gates_changed"] is False
