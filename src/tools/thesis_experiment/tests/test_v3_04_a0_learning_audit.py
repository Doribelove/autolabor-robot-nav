import importlib.util
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_04_a0_learning_audit.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_04_a0_test_builder", BUILDER_PATH
)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)


def test_learning_audit_stops_without_training():
    outputs = BUILDER.build()
    report = yaml.safe_load(outputs[BUILDER.REPORT])
    assert report["overall_pass"]
    assert not report["learning_activation_observations"][
        "all_learning_activation_requirements_pass"
    ]
    assert report["metrics"]["training_steps"] == 0
    assert report["metrics"]["learned_model_artifact_count"] == 0
    assert report["disposition"]["analytic_ranker_retained"]


def test_analytic_ranker_has_zero_immediate_progress_regret():
    report = yaml.safe_load(BUILDER.build()[BUILDER.REPORT])
    assert report["metrics"]["multi_safe_action_decision_count"] > 0
    assert report["metrics"]["immediate_progress_regret_max_m"] == 0.0
    assert report["metrics"]["analytic_maximum_progress_rate"] == 1.0


def test_cloned_data_does_not_claim_sequential_support():
    report = yaml.safe_load(BUILDER.build()[BUILDER.REPORT])
    assert report["metrics"]["d1_cloned_row_count"] == 135
    assert report["metrics"]["d1_cloned_propensity_min"] == 0.2
    assert not report["metrics"][
        "d1_sequential_closed_loop_transition_support"
    ]
