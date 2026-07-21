"""CSV metric-schema and run-manifest validation without ROS side effects."""

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Sequence, Tuple


SCHEMA_COLUMNS = ("field", "type", "unit_or_values", "required", "description")
SUPPORTED_TYPES = ("string", "enum", "int", "float", "bool")
RUN_MODES = ("gazebo", "bag_replay", "shadow", "real_closed_loop")


class SchemaValidationError(ValueError):
    """Raised when a metric schema, row, or run manifest is invalid."""


@dataclass(frozen=True)
class MetricField:
    name: str
    value_type: str
    unit_or_values: str
    required: bool
    description: str

    @property
    def enum_values(self) -> Tuple[str, ...]:
        if self.value_type != "enum":
            return ()
        return tuple(item for item in self.unit_or_values.split("|") if item)


def load_metric_schema(path: Any) -> Tuple[MetricField, ...]:
    """Load and validate a CSV schema definition."""

    source = Path(path)
    try:
        with source.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            if tuple(reader.fieldnames or ()) != SCHEMA_COLUMNS:
                raise SchemaValidationError(
                    "{} columns must be {}".format(source, list(SCHEMA_COLUMNS))
                )
            rows = list(reader)
    except OSError as exc:
        raise SchemaValidationError("Cannot read schema {}: {}".format(source, exc))

    if not rows:
        raise SchemaValidationError("Schema is empty: {}".format(source))
    names = set()
    fields = []
    for line_number, row in enumerate(rows, start=2):
        name = row["field"].strip()
        value_type = row["type"].strip()
        required_text = row["required"].strip().lower()
        if not name or name in names:
            raise SchemaValidationError("Invalid or duplicate field at line {}".format(line_number))
        if value_type not in SUPPORTED_TYPES:
            raise SchemaValidationError("Unsupported type {} at line {}".format(value_type, line_number))
        if required_text not in ("true", "false"):
            raise SchemaValidationError("required must be true/false at line {}".format(line_number))
        if value_type == "enum" and not row["unit_or_values"].strip():
            raise SchemaValidationError("enum field {} has no values".format(name))
        names.add(name)
        fields.append(
            MetricField(
                name=name,
                value_type=value_type,
                unit_or_values=row["unit_or_values"].strip(),
                required=required_text == "true",
                description=row["description"].strip(),
            )
        )
    return tuple(fields)


def _is_missing(value: Any) -> bool:
    return value is None or value == ""


def _validate_scalar(field: MetricField, value: Any) -> None:
    if field.value_type == "string":
        if not isinstance(value, str):
            raise SchemaValidationError("{} must be a string".format(field.name))
        if field.unit_or_values == "json":
            try:
                json.loads(value)
            except (TypeError, ValueError) as exc:
                raise SchemaValidationError("{} must contain JSON: {}".format(field.name, exc))
    elif field.value_type == "enum":
        if value not in field.enum_values:
            raise SchemaValidationError(
                "{} must be one of {}".format(field.name, list(field.enum_values))
            )
    elif field.value_type == "int":
        if isinstance(value, bool):
            raise SchemaValidationError("{} must be an int".format(field.name))
        try:
            converted = int(value)
        except (TypeError, ValueError):
            raise SchemaValidationError("{} must be an int".format(field.name))
        if isinstance(value, float) and converted != value:
            raise SchemaValidationError("{} must be an int".format(field.name))
    elif field.value_type == "float":
        if isinstance(value, bool):
            raise SchemaValidationError("{} must be a float".format(field.name))
        try:
            float(value)
        except (TypeError, ValueError):
            raise SchemaValidationError("{} must be a float".format(field.name))
    elif field.value_type == "bool":
        if value not in (True, False, "true", "false", "True", "False", "0", "1", 0, 1):
            raise SchemaValidationError("{} must be a bool".format(field.name))


def validate_metric_record(
    record: Mapping[str, Any],
    schema: Sequence[MetricField],
    allow_extra: bool = False,
) -> Mapping[str, Any]:
    """Validate one episode or step record against its loaded schema."""

    schema_names = {field.name for field in schema}
    extra = sorted(set(record) - schema_names)
    if extra and not allow_extra:
        raise SchemaValidationError("Unexpected fields: {}".format(", ".join(extra)))
    for field in schema:
        value = record.get(field.name)
        if _is_missing(value):
            if field.required:
                raise SchemaValidationError("Missing required field: {}".format(field.name))
            continue
        _validate_scalar(field, value)
    return record


def _require_mapping(data: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = data.get(key)
    if not isinstance(value, dict):
        raise SchemaValidationError("run_manifest.{} must be a mapping".format(key))
    return value


def _require_keys(data: Mapping[str, Any], keys: Iterable[str], context: str) -> None:
    missing = [key for key in keys if key not in data]
    if missing:
        raise SchemaValidationError("{} missing {}".format(context, ", ".join(sorted(missing))))


def validate_run_manifest(
    data: Mapping[str, Any],
    allow_placeholders: bool = False,
) -> Mapping[str, Any]:
    """Validate run-manifest structure and safety invariants."""

    _require_keys(
        data,
        ("schema_version", "run_id", "mode", "source", "configuration", "topics", "safety", "artifacts", "completion"),
        "run_manifest",
    )
    if str(data["schema_version"]) != "1.0":
        raise SchemaValidationError("run_manifest.schema_version must be 1.0")
    if data["mode"] not in RUN_MODES:
        raise SchemaValidationError("run_manifest.mode must be one of {}".format(list(RUN_MODES)))
    for key in ("source", "configuration", "topics", "artifacts", "completion"):
        _require_mapping(data, key)
    safety = _require_mapping(data, "safety")
    _require_keys(safety, ("allow_motion", "allow_parameter_write"), "run_manifest.safety")
    if not isinstance(safety["allow_motion"], bool) or not isinstance(safety["allow_parameter_write"], bool):
        raise SchemaValidationError("run_manifest safety permissions must be bool")
    if data["mode"] != "real_closed_loop" and (
        safety["allow_motion"] or safety["allow_parameter_write"]
    ):
        raise SchemaValidationError("Non-closed-loop modes cannot enable motion or parameter writes")
    if not allow_placeholders:
        for key in ("run_id", "created_at_utc", "algorithm", "scene_id", "scene_split"):
            if _is_missing(data.get(key)):
                raise SchemaValidationError("run_manifest.{} cannot be empty".format(key))
    return data
