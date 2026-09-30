import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_06_c0_low_speed_candidate.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_06_c0_test_builder", BUILDER_PATH
)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)


def outputs():
    return BUILDER.build()


def test_candidate_foundation_is_active_and_fail_closed():
    value = yaml.safe_load(outputs()[BUILDER.REPORT])
    assert value["overall_pass"]
    assert value["metrics"]["newly_selected_nonstop_decision_count"] > 0
    assert value["metrics"]["selected_candidate_accepted_rate"] == 1.0
    assert value["metrics"]["certified_stop_present_rate"] == 1.0


def test_only_target_speed_roster_changes():
    value = yaml.safe_load(outputs()[BUILDER.PLANNER])
    assert value["candidate_generator"]["target_speeds_mps"] == [
        0.0,
        0.2,
        0.4,
        0.6,
        0.8,
        1.0,
        1.2,
    ]
    amendment = value["candidate_generator"]["low_speed_candidate_amendment"]
    assert not amendment["hard_verifier_changed"]
    assert not amendment["predictor_changed"]
    assert not amendment["certified_stop_changed"]
    assert not amendment["ranker_changed"]
