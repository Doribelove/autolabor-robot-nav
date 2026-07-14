from pathlib import Path

import yaml

from thesis_experiment.t12_replay import build_runtime, evaluate_replay


ROOT = Path("/home/robot/robot_ws_base_rl")


def test_t12_config_is_read_only_and_builds_runtime():
    path = ROOT / "config/thesis_experiments/t12_shadow.yaml"
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert config["mode"] == "shadow"
    assert config["read_only"] is True
    assert config["allow_motion"] is False
    assert config["allow_parameter_write"] is False
    runtime = build_runtime(config, ROOT)
    assert runtime.config.ema_alpha == 0.35


def test_t12_replay_is_side_effect_free_and_reduces_action_magnitude():
    report, decisions = evaluate_replay(
        ROOT / "config/thesis_experiments/t12_shadow.yaml", ROOT)
    assert report["allow_motion"] is False
    assert report["allow_parameter_write"] is False
    assert report["mean_projected_l1"] <= report["mean_candidate_l1"]
    assert report["mean_action_l1_reduction_after_smoothing_projection"] >= 0.0
    assert all(not row["write_allowed"] and not row["motion_allowed"]
               for row in decisions)
