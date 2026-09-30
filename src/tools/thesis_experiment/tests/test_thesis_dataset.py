import importlib.util
from pathlib import Path

import pytest


PATH = Path(__file__).parents[1] / "src/thesis_experiment/thesis_dataset.py"
SPEC = importlib.util.spec_from_file_location("thesis_dataset_under_test", PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def valid_row():
    row = {field: None for field in MODULE.REQUIRED_FIELDS}
    row.update({
        "schema_version": "1.0", "stage": "TEST", "sequence": 1,
        "block_id": "b1", "attempt": 1, "seed": 1, "scene_id": "scene",
        "actor_policy": "cautious", "method": "method", "method_role": "main",
        "simulation_only": True, "completed": True, "terminal_reason": "completed",
        "retry_used": False, "resume_used": False, "goal_success": True,
        "contact_count": 0, "collision": False, "emergency_count": 0,
        "safety_fallback_used": False, "selected_accepted": True,
        "certified_stop_available": True, "scientific_inclusion": True,
        "statistical_unit": "block", "oracle_track_used": True,
        "training_steps": 0, "hardware_accessed": False, "real_vehicle_used": False,
    })
    return row


def test_schema_row_accepts_explicit_null_measurements():
    assert MODULE.validate_row(valid_row()) is True


def test_schema_rejects_missing_field():
    row = valid_row()
    row.pop("report_sha256")
    with pytest.raises(ValueError, match="missing fields"):
        MODULE.validate_row(row)


def test_schema_rejects_nonfinite_and_hardware():
    row = valid_row()
    row["restricted_time_s"] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        MODULE.validate_row(row)
    row = valid_row()
    row["hardware_accessed"] = True
    with pytest.raises(ValueError, match="hardware"):
        MODULE.validate_row(row)

