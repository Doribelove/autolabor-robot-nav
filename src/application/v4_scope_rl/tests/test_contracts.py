from pathlib import Path

import pytest

from v4_scope_rl.contracts import ContractError, load_contract, validate_contract


ROOT = Path(__file__).resolve().parents[4]
CONTRACT = ROOT / "src/application/v4_scope_rl/config/v4_00_architecture_contract.yaml"


def test_foundation_contract_is_valid():
    value = load_contract(CONTRACT)
    assert value["scope"]["real_vehicle_forbidden"] is True
    assert value["decision_process"]["actions"][0]["name"] == "certified_stop"


def test_contract_rejects_real_vehicle_authority():
    value = load_contract(CONTRACT)
    value["scope"]["real_vehicle_forbidden"] = False
    with pytest.raises(ContractError, match="REAL_VEHICLE"):
        validate_contract(value)
