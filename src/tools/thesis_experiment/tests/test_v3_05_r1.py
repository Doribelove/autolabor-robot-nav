import importlib.util
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/build_v3_05_r1_port_repair.py"
)
SPEC = importlib.util.spec_from_file_location("v3_05_r1_builder", BUILDER_PATH)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)


def test_port_repair_is_unique_and_below_ephemeral_range():
    report = yaml.safe_load(BUILDER.build()[BUILDER.REPORT])
    assert report["overall_pass"]
    assert report["metrics"]["allocated_port_count"] == 10530
    assert report["metrics"]["unique_port_rate"] == 1.0
    assert report["metrics"]["below_ephemeral_rate"] == 1.0
    assert report["metrics"]["maximum_allocated_port"] == 25538


def test_historical_failure_port_is_identified():
    report = yaml.safe_load(BUILDER.build()[BUILDER.REPORT])
    assert report["metrics"]["historical_failure_port"] == 35600
    assert report["metrics"]["historical_failure_port_in_ephemeral_range"]
