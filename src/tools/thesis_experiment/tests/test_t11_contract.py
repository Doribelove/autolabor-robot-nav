from pathlib import Path

import pytest
import yaml

from thesis_experiment.t11_contract import T11ContractError, validate_t11_contract


ROOT = Path(__file__).resolve().parents[4]
CONFIG = ROOT / "config/thesis_experiments/t11_formal.yaml"


def test_checked_in_t11_contract_is_frozen_multi_scene_multi_seed():
    result = validate_t11_contract(CONFIG, ROOT)
    assert result["status"] == "valid"
    assert result["training_seed_count"] == 5
    assert result["evaluation_seed_count"] == 10
    assert result["scene_counts"] == {
        "train": 5, "validation": 3, "test_id": 4, "test_ood": 3,
    }


def test_t11_contract_rejects_test_based_model_selection(tmp_path):
    data = yaml.safe_load(CONFIG.read_text())
    data["evaluation"]["no_test_checkpoint_selection"] = False
    changed = tmp_path / "t11.yaml"
    changed.write_text(yaml.safe_dump(data), encoding="utf-8")
    with pytest.raises(T11ContractError, match="test checkpoint"):
        validate_t11_contract(changed, ROOT)
