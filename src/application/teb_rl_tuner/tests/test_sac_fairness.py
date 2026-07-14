from pathlib import Path

import pytest
import yaml

from teb_rl_tuner.sac_fairness import SacFairnessError, load_and_validate_sac_pair


ROOT = Path(__file__).resolve().parents[4]


def test_checked_in_t09_t10_pair_differs_only_in_action_parameterization():
    result = load_and_validate_sac_pair(
        ROOT / "config/thesis_experiments/t09_sac.yaml",
        ROOT / "config/thesis_experiments/t10_direct_theta_sac.yaml",
    )
    assert result["status"] == "valid"
    assert result["observation_dimension"] == 254
    assert result["semantic_action_dimension"] == 5
    assert result["direct_action_dimension"] == 9


def test_fairness_rejects_training_budget_drift(tmp_path):
    semantic_path = ROOT / "config/thesis_experiments/t09_sac.yaml"
    direct = yaml.safe_load(
        (ROOT / "config/thesis_experiments/t10_direct_theta_sac.yaml").read_text()
    )
    direct["training"]["gamma"] = 0.9
    changed = tmp_path / "direct.yaml"
    changed.write_text(yaml.safe_dump(direct), encoding="utf-8")
    with pytest.raises(SacFairnessError, match="training"):
        load_and_validate_sac_pair(semantic_path, changed)
