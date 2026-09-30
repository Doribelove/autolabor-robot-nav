import importlib.util
from pathlib import Path
import yaml
W=Path("/home/robot/robot_ws_base_rl"); M=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_08_a3.py"
def test_a3_duration_only_and_receipt_stage_repair():
    spec=importlib.util.spec_from_file_location("v3_08_a3_test",M); module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    contract=module.load_yaml_relative(W,module.CONTRACT_RELATIVE); prereg=module.load_yaml_relative(W,module.PREREGISTRATION_RELATIVE); module.validate_contract(contract,prereg); schedule=module.exact_schedule(contract)
    assert len(schedule["rows"])==18 and sorted({x["seed"] for x in schedule["rows"]})==list(range(6691,6697))
    assert contract["closed_loop"]["maximum_episode_duration_s"]==12.5 and contract["closed_loop"]["maximum_decisions_per_episode"]==24
    encoded=module.yaml_bytes({"stage":"V3-08-A1","status":"unit_complete"}); assert yaml.safe_load(encoded)["stage"]=="V3-08-A3"
