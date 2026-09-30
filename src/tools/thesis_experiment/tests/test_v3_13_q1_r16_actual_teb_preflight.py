#!/usr/bin/env python3
"""Static tests for the R16 actual-TEB paired preflight."""

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r16_actual_teb_paired_preflight_contract.yaml"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_progress_actual_teb_preflight.launch"
)
RUNTIME = W / (
    "src/application/v3_scope_planner/scripts/"
    "v3_scope_progress_actual_teb_preflight_episode.py"
)
RUNNER = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r16_actual_teb_preflight.py"
)


def file_sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_actual_teb_identity_is_exact():
    baseline = yaml.safe_load(CONTRACT.read_bytes())["methods"][
        "primary_baseline"
    ]
    assert baseline["planner_plugin"] == (
        "teb_local_planner/TebLocalPlannerROS"
    )
    assert baseline["max_vel_x_mps"] == 1.2
    assert file_sha(W / baseline["config_path"]) == baseline[
        "config_sha256"
    ]
    assert file_sha(W / baseline["built_plugin_path"]) == baseline[
        "built_plugin_sha256"
    ]
    assert file_sha(W / baseline["source_path"]) == baseline[
        "source_sha256"
    ]


def test_fresh_pair_and_no_formal_claim():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    population = contract["preflight_population"]
    assert population["seed"] == 7666
    assert population["method_order"] == [
        "actual_teb", "v3_13_analytic"
    ]
    assert population["exact_units"] == 2
    assert population[
        "included_in_qualification_or_performance_evidence"
    ] is False
    assert contract["authority"][
        "qualification_execution_authorized"
    ] is False
    assert contract["authority"]["training_authorized"] is False


def test_launch_binds_real_teb_and_progress_runtime():
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"
    text = LAUNCH.read_text(encoding="utf-8")
    assert "move_base" in text
    assert "fixed_teb.yaml" in text
    assert "include_dynamic_obstacles" in text
    assert "v3_scope_progress_actual_teb_preflight_episode.py" in text
    assert "geometry_contract_path" in text
    assert "/cmd_vel_navigation /cmd_vel" in text


def test_progress_extension_is_recovery_only():
    text = RUNTIME.read_text(encoding="utf-8")
    assert "if not recovery:" in text
    assert "return super().plan_v3(label, recovery)" in text
    assert "progress_expanded_recovery_core.plan" in text


def test_actual_teb_performance_is_retained_not_an_integration_gate():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["integration_gates"][
        "actual_teb_performance_outcome_retained_without_pass_requirement"
    ] is True
    text = RUNNER.read_text(encoding="utf-8")
    assert "actual_teb_outcome_retained_without_pass_requirement" in text
    assert '"actual_teb_local_plan"' in text
    assert '"actual_teb_command"' in text
