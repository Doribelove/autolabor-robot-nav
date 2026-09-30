import importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl");M=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_09_q1_r1.py"
def test_q1_r1_changes_only_shared_replanning_period():
 s=importlib.util.spec_from_file_location("q1r1test",M);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);c=m.load_yaml_relative(W,m.CONTRACT_RELATIVE);p=m.load_yaml_relative(W,m.PREREGISTRATION_RELATIVE);m.validate_contract(c,p);schedule=m.exact_schedule(c);assert len(schedule["rows"])==36 and c["closed_loop"]["replanning_period_s"]==.25 and c["closed_loop"]["maximum_decisions_per_episode"]==48 and sorted({x["seed"] for x in schedule["rows"]})==list(range(6721,6727))
