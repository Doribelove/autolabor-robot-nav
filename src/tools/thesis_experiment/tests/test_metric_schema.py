from pathlib import Path

import pytest

from teb_rl_tuner import load_yaml_mapping
from thesis_experiment import (
    SchemaValidationError,
    load_metric_schema,
    validate_metric_record,
    validate_run_manifest,
)


ROOT = Path(__file__).resolve().parents[4]
SCHEMA_DIR = ROOT / "docs/thesis_experiment/schemas"


def test_repository_metric_schemas_are_well_formed():
    episode = load_metric_schema(SCHEMA_DIR / "episode_metrics_schema.csv")
    step = load_metric_schema(SCHEMA_DIR / "step_metrics_schema.csv")
    assert len(episode) == 43
    assert len(step) == 57
    assert sum(field.required for field in episode) > 30
    assert sum(field.required for field in step) > 40


def test_required_and_enum_fields_are_checked():
    schema = load_metric_schema(SCHEMA_DIR / "episode_metrics_schema.csv")
    with pytest.raises(SchemaValidationError, match="Missing required field"):
        validate_metric_record({}, schema)

    record = {field.name: "x" for field in schema if field.required}
    record["algorithm"] = "not-an-algorithm"
    with pytest.raises(SchemaValidationError, match="algorithm"):
        validate_metric_record(record, schema)


def test_json_string_fields_are_checked():
    schema = load_metric_schema(SCHEMA_DIR / "step_metrics_schema.csv")
    field = next(item for item in schema if item.name == "theta_candidate_json")
    with pytest.raises(SchemaValidationError, match="JSON"):
        validate_metric_record({field.name: "not-json"}, (field,))
    validate_metric_record({field.name: '{"max_vel_x": 1.0}'}, (field,))


def test_run_manifest_template_is_safe_and_structurally_valid():
    data = load_yaml_mapping(ROOT / "docs/thesis_experiment/templates/run_manifest.template.yaml")
    validate_run_manifest(data, allow_placeholders=True)
    assert data["safety"]["allow_motion"] is False
    assert data["safety"]["allow_parameter_write"] is False


def test_non_real_mode_cannot_enable_permissions():
    data = load_yaml_mapping(ROOT / "docs/thesis_experiment/templates/run_manifest.template.yaml")
    data["mode"] = "shadow"
    data["safety"]["allow_motion"] = True
    with pytest.raises(SchemaValidationError, match="cannot enable"):
        validate_run_manifest(data, allow_placeholders=True)
