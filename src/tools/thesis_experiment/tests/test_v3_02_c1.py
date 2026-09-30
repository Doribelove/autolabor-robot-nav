import importlib.util
from pathlib import Path
import tempfile


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PATH = WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_c1.py"
SPEC = importlib.util.spec_from_file_location("v3_02_c1", PATH)
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)


def test_common_prefix_schedule_and_world():
    contract = V3.load_yaml_relative(WORKSPACE, V3.CONTRACT_RELATIVE)
    prereg = V3.load_yaml_relative(WORKSPACE, V3.PREREGISTRATION_RELATIVE)
    V3.validate_contract(contract, prereg)
    schedule = V3.exact_schedule(contract)
    first, second = schedule["rows"][0], schedule["rows"][1]
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        world_a = V3.render_world(contract, first, root / "a.csv", root / "a.done")
        world_b = V3.render_world(contract, second, root / "b.csv", root / "b.done")
    assert "<time>0.500000000</time>" in world_a
    assert "<x>-2.600000000</x>" in world_a
    assert "<time>0.500000000</time>" in world_b
    assert first["ego_speed_mps"] != second["ego_speed_mps"]
