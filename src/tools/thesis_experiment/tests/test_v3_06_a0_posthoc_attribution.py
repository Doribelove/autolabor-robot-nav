import importlib.util
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "build_v3_06_a0_posthoc_attribution.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_06_a0_test_builder", BUILDER_PATH
)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)


def report():
    return yaml.safe_load(BUILDER.build()[BUILDER.REPORT])


def test_posthoc_attribution_preserves_formal_negative_result():
    value = report()
    assert value["overall_pass"]
    assert value["upstream_formal_result_preserved"]
    assert not value["claim"]["new_performance_improvement"]
    assert value["analysis_kind"] == "posthoc_pure_offline_descriptive"


def test_failure_rosters_and_pairing_are_exact():
    value = report()
    metrics = value["metrics"]
    assert metrics["receipt_count"] == 180
    assert metrics["paired_episode_count"] == 90
    assert metrics["v3_failure_count"] == 6
    assert metrics["fixed_v120_failure_count"] == 8
    assert metrics["paired_outcome_partition"]["fixed_v120_only_success"] == 0


def test_attribution_keeps_safety_and_learning_frozen():
    value = report()
    direction = value["next_single_factor_direction"]
    assert not direction["hard_verifier_changed"]
    assert not direction["certified_stop_changed"]
    assert not direction["learning_activated"]
    assert all(not item for item in value["authority"].values())
