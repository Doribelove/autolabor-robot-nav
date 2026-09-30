import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
SCRIPT = W / "src/tools/thesis_experiment/scripts/build_v3_07_a0_spatial_candidate_foundation.py"


def test_v3_07_a0_spatial_candidates_are_feasible_but_not_execution_ready():
    spec = importlib.util.spec_from_file_location("v3_07_a0_tested", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    left, right, report = module.build()
    assert left["candidate_digest"] != right["candidate_digest"]
    assert report["overall_pass"] is True
    assert report["execution_ready"] is False
    assert report["claim"]["interaction_prediction_qualified"] is False
