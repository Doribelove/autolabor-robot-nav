import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_c3.py"
SPEC = importlib.util.spec_from_file_location("v3_06_c3_test", MODULE)
V = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(V)


def test_fresh_qualification_schedule():
    schedule = V.exact_schedule(V.effective_contract())
    assert schedule["row_count"] == 36
    assert schedule["seeds"] == [6521, 6522, 6523, 6524, 6525, 6526]
    assert schedule["methods"] == ["sparse_control", "minimal_v040"]
