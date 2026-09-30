#!/usr/bin/env python3

import importlib.util
from pathlib import Path

import yaml


ROOT = Path("/home/robot/robot_ws_base_rl")
CORE_PATH = (
    ROOT
    / "src/application/v3_scope_planner/src/"
    "v3_scope_planner/multi_actor_core_v1.py"
)
CONTRACT_PATH = (
    ROOT
    / "config/thesis_experiments/v3/"
    "v3_13_c2_predictor_fit_contract.yaml"
)
CONTRACT = yaml.safe_load(CONTRACT_PATH.read_text(encoding="utf-8"))


def load_core():
    spec = importlib.util.spec_from_file_location(
        "v3_13_c2_core_test", CORE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_feature_order_has_no_hidden_policy_label():
    text = " ".join(CONTRACT["model"]["feature_order"])
    assert "policy" not in text
    assert "hidden" not in text
    assert CONTRACT["data_scope"]["hidden_policy_runtime_input"] is False


def test_contextual_features_are_zero_at_handoff():
    core = load_core()
    actor = {
        "coordinate_speed_mps": 0.32,
        "coordinate_m": -3.0,
        "path_origin_x_m": 2.0,
    }
    assert core.contextual_features(actor, 0.0) == [0.0] * 7


def test_training_boundary_is_predictor_only():
    authority = CONTRACT["authority"]
    assert authority["deterministic_offline_predictor_fit_authorized"]
    assert not authority["simulation_execution_authorized"]
    assert not authority["policy_or_ranker_training_authorized"]
    assert not authority["hardware_device_access_authorized"]
    assert not authority["real_vehicle_authorized"]


def test_internal_cv_is_not_fresh_qualification():
    boundary = CONTRACT["future_qualification_boundary"]
    assert boundary["internal_cross_validation_is_not_fresh_qualification"]
    assert boundary["future_predictor_qualification_requires_new_release"]

