import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl");M=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_f1.py"
def test_f1_fresh_formal_matrix_and_statistics():
 s=importlib.util.spec_from_file_location("f1test",M);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);c=m.load_yaml_relative(W,m.CONTRACT_RELATIVE);p=m.load_yaml_relative(W,m.PREREGISTRATION_RELATIVE);m.validate_contract(c,p);schedule=m.exact_schedule(c);assert len(schedule["rows"])==180 and sorted({x["seed"] for x in schedule["rows"]})==list(range(6801,6831));assert c["formal_statistics"]["point_relative_reduction_min"]==.10 and c["formal_statistics"]["one_sided_lower_relative_reduction_min"]==.05
