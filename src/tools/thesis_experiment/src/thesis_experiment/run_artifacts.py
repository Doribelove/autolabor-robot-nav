"""Deterministic T06 run artifacts and offline validation.

This module intentionally has no ROS dependencies.  It writes the two frozen
metric CSV formats, a YAML run manifest, and sha256sum-compatible checksums.
Rosbags are referenced by URI only; this module never copies them into an
artifact bundle.
"""

import csv
import hashlib
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import yaml

from .schema import (
    MetricField,
    SchemaValidationError,
    load_metric_schema,
    validate_metric_record,
    validate_run_manifest,
)


CHECKSUM_NAME = "checksums.sha256"
TERMINATED_REASONS = frozenset(
    (
        "goal",
        "collision",
        "planner_failure",
        "sensor_fault",
        "tf_fault",
        "interface_fault",
        "emergency_stop",
    )
)
TRUNCATED_REASONS = frozenset(("timeout", "operator_stop", "infrastructure_fault"))


class RunValidationError(ValueError):
    """Raised when a run bundle violates the T06 reproducibility contract."""


def _normalise_csv_value(value: Any) -> Any:
    if isinstance(value, bool):
        return "true" if value else "false"
    return "" if value is None else value


def _check_finite(record: Mapping[str, Any], schema: Sequence[MetricField]) -> None:
    for field in schema:
        value = record.get(field.name)
        if field.value_type != "float" or value in (None, ""):
            continue
        try:
            finite = math.isfinite(float(value))
        except (TypeError, ValueError):
            finite = False
        if not finite:
            raise SchemaValidationError("{} must be finite".format(field.name))


def write_metric_csv(
    path: Any,
    records: Iterable[Mapping[str, Any]],
    schema: Sequence[MetricField],
) -> Path:
    """Validate and atomically write records in the schema's exact field order."""

    destination = Path(path)
    if destination.suffix.lower() == ".bag" or destination.name.endswith(".bag.active"):
        raise RunValidationError("A rosbag cannot be used as a CSV artifact")
    destination.parent.mkdir(parents=True, exist_ok=True)
    rows = list(records)
    for index, row in enumerate(rows, start=1):
        try:
            validate_metric_record(row, schema)
            _check_finite(row, schema)
        except SchemaValidationError as exc:
            raise RunValidationError("record {}: {}".format(index, exc))

    temporary = destination.with_name(destination.name + ".tmp")
    names = [field.name for field in schema]
    try:
        with temporary.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=names, extrasaction="raise")
            writer.writeheader()
            for row in rows:
                writer.writerow({name: _normalise_csv_value(row.get(name)) for name in names})
        temporary.replace(destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


def write_step_csv(path: Any, records: Iterable[Mapping[str, Any]], schema_path: Any) -> Path:
    return write_metric_csv(path, records, load_metric_schema(schema_path))


def write_episode_csv(path: Any, records: Iterable[Mapping[str, Any]], schema_path: Any) -> Path:
    return write_metric_csv(path, records, load_metric_schema(schema_path))


def write_run_manifest(path: Any, manifest: Mapping[str, Any]) -> Path:
    """Validate and atomically write a concrete run manifest."""

    try:
        validate_run_manifest(manifest, allow_placeholders=False)
    except SchemaValidationError as exc:
        raise RunValidationError(str(exc))
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8") as handle:
            yaml.safe_dump(dict(manifest), handle, allow_unicode=True, sort_keys=False)
        temporary.replace(destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


def sha256_file(path: Any, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        while True:
            chunk = handle.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def write_checksums(path: Any, files: Iterable[Any], base_dir: Optional[Any] = None) -> Path:
    """Write deterministic GNU sha256sum-style entries.

    Paths are stored relative to ``base_dir`` (the checksum file directory by
    default). External rosbag files may be hashed and referenced, but are never
    copied into the bundle.
    """

    destination = Path(path).resolve()
    root = Path(base_dir).resolve() if base_dir is not None else destination.parent
    entries = []
    for item in files:
        source = Path(item).resolve()
        if not source.is_file():
            raise RunValidationError("Checksum input is not a file: {}".format(source))
        try:
            relative = source.relative_to(root)
        except ValueError:
            raise RunValidationError("Checksum input must be within {}: {}".format(root, source))
        if source == destination:
            raise RunValidationError("Checksum file cannot checksum itself")
        entries.append((relative.as_posix(), sha256_file(source)))
    names = [name for name, _ in entries]
    if len(names) != len(set(names)):
        raise RunValidationError("Duplicate checksum input")
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8", newline="\n") as handle:
            for name, digest in sorted(entries):
                handle.write("{}  {}\n".format(digest, name))
        temporary.replace(destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


def _load_yaml(path: Path) -> Mapping[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as handle:
            value = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as exc:
        raise RunValidationError("Cannot read manifest {}: {}".format(path, exc))
    if not isinstance(value, dict):
        raise RunValidationError("Manifest must contain a YAML mapping: {}".format(path))
    return value


def _resolve_reference(manifest_path: Path, value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise RunValidationError("Missing artifact reference: {}".format(label))
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = manifest_path.parent / candidate
    candidate = candidate.resolve()
    if not candidate.is_file():
        raise RunValidationError("Referenced artifact does not exist ({}): {}".format(label, candidate))
    return candidate


def _read_csv(path: Path, schema: Sequence[MetricField]) -> List[Dict[str, str]]:
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            expected = [field.name for field in schema]
            if reader.fieldnames != expected:
                raise RunValidationError(
                    "{} header does not match frozen schema order".format(path)
                )
            rows = list(reader)
    except OSError as exc:
        raise RunValidationError("Cannot read {}: {}".format(path, exc))
    for line_number, row in enumerate(rows, start=2):
        try:
            validate_metric_record(row, schema)
            _check_finite(row, schema)
        except SchemaValidationError as exc:
            raise RunValidationError("{}:{}: {}".format(path, line_number, exc))
    return rows


def _parse_bool(value: Any, label: str) -> bool:
    if value in (True, "true", "True", "1", 1):
        return True
    if value in (False, "false", "False", "0", 0):
        return False
    raise RunValidationError("{} is not a bool".format(label))


def _validate_termination(row: Mapping[str, Any]) -> None:
    episode = row["episode_id"]
    reason = row["termination_reason"]
    terminated = _parse_bool(row["terminated"], "{}.terminated".format(episode))
    truncated = _parse_bool(row["truncated"], "{}.truncated".format(episode))
    success = _parse_bool(row["success"], "{}.success".format(episode))
    collision = _parse_bool(row["collision"], "{}.collision".format(episode))
    if terminated == truncated:
        raise RunValidationError(
            "Episode {} must set exactly one of terminated/truncated".format(episode)
        )
    if (reason in TERMINATED_REASONS) != terminated:
        raise RunValidationError("Episode {} termination_reason is inconsistent".format(episode))
    if (reason in TRUNCATED_REASONS) != truncated:
        raise RunValidationError("Episode {} truncation reason is inconsistent".format(episode))
    if success != (reason == "goal"):
        raise RunValidationError("Episode {} success must mean goal".format(episode))
    if collision != (reason == "collision"):
        raise RunValidationError("Episode {} collision flag is inconsistent".format(episode))


def _load_checksums(path: Path, root: Path) -> Dict[Path, str]:
    entries: Dict[Path, str] = {}
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise RunValidationError("Cannot read checksum file {}: {}".format(path, exc))
    for line_number, line in enumerate(lines, start=1):
        if not line.strip():
            continue
        parts = line.split(None, 1)
        if len(parts) != 2 or len(parts[0]) != 64:
            raise RunValidationError("Invalid checksum line {}:{}".format(path, line_number))
        digest, name = parts[0].lower(), parts[1].lstrip("* ")
        if any(char not in "0123456789abcdef" for char in digest):
            raise RunValidationError("Invalid SHA256 at {}:{}".format(path, line_number))
        candidate = (root / name).resolve()
        try:
            candidate.relative_to(root.resolve())
        except ValueError:
            raise RunValidationError("Checksum path escapes run directory: {}".format(name))
        if candidate in entries:
            raise RunValidationError("Duplicate checksum entry: {}".format(name))
        entries[candidate] = digest
    if not entries:
        raise RunValidationError("Checksum file is empty: {}".format(path))
    return entries


class RunValidator:
    """Validate one completed run and return a compact machine-readable report."""

    def __init__(self, episode_schema_path: Any, step_schema_path: Any):
        self.episode_schema = load_metric_schema(episode_schema_path)
        self.step_schema = load_metric_schema(step_schema_path)

    def validate(self, manifest_path: Any) -> Dict[str, Any]:
        manifest_file = Path(manifest_path).resolve()
        manifest = _load_yaml(manifest_file)
        try:
            validate_run_manifest(manifest, allow_placeholders=False)
        except SchemaValidationError as exc:
            raise RunValidationError(str(exc))
        artifacts = manifest.get("artifacts")
        if not isinstance(artifacts, dict):
            raise RunValidationError("run_manifest.artifacts must be a mapping")
        episode_file = _resolve_reference(manifest_file, artifacts.get("episode_csv"), "episode_csv")
        step_file = _resolve_reference(manifest_file, artifacts.get("step_log"), "step_log")
        checksum_file = _resolve_reference(
            manifest_file, artifacts.get("checksums_file"), "checksums_file"
        )
        episodes = _read_csv(episode_file, self.episode_schema)
        steps = _read_csv(step_file, self.step_schema)
        run_id = str(manifest["run_id"])
        episode_ids = set()
        for row in episodes:
            if row["run_id"] != run_id:
                raise RunValidationError("Episode run_id does not match manifest")
            episode_id = row["episode_id"]
            if episode_id in episode_ids:
                raise RunValidationError("Duplicate episode_id: {}".format(episode_id))
            episode_ids.add(episode_id)
            for field in ("algorithm", "scene_id", "scene_split"):
                allowed_values = manifest.get(field + "s")
                matches = (row[field] in [str(value) for value in allowed_values]
                           if isinstance(allowed_values, list) else
                           row[field] == str(manifest[field]))
                if not matches:
                    raise RunValidationError(
                        "Episode {} {} does not match manifest".format(episode_id, field)
                    )
            _validate_termination(row)
        last: Dict[str, Tuple[int, int]] = {}
        for row in steps:
            if row["run_id"] != run_id:
                raise RunValidationError("Step run_id does not match manifest")
            episode_id = row["episode_id"]
            if episode_id not in episode_ids:
                raise RunValidationError("Step references unknown episode_id: {}".format(episode_id))
            current = (int(row["step_id"]), int(row["config_seq"]))
            previous = last.get(episode_id)
            if previous is not None and current[0] <= previous[0]:
                raise RunValidationError("step_id is not strictly increasing for {}".format(episode_id))
            if previous is not None and current[1] < previous[1]:
                raise RunValidationError("config_seq decreases for {}".format(episode_id))
            last[episode_id] = current
        checksums = _load_checksums(checksum_file, checksum_file.parent)
        for checked_path, expected in checksums.items():
            if not checked_path.is_file():
                raise RunValidationError("Checksum references missing file: {}".format(checked_path))
            if sha256_file(checked_path) != expected:
                raise RunValidationError("Checksum mismatch: {}".format(checked_path))

        local_artifacts = [episode_file, step_file]
        for key in ("stdout_log", "failure_index"):
            if artifacts.get(key) not in (None, ""):
                local_artifacts.append(_resolve_reference(manifest_file, artifacts[key], key))
        for referenced in local_artifacts:
            expected = checksums.get(referenced)
            if expected is None:
                raise RunValidationError("Missing checksum entry: {}".format(referenced))

        configuration = manifest.get("configuration")
        if not isinstance(configuration, dict):
            raise RunValidationError("run_manifest.configuration must be a mapping")
        for key, value in configuration.items():
            if not key.endswith("_path") or value in (None, ""):
                continue
            referenced = _resolve_reference(manifest_file, value, "configuration.{}".format(key))
            sha_key = key[:-5] + "_sha256"
            expected = configuration.get(sha_key)
            if not isinstance(expected, str) or len(expected) != 64:
                raise RunValidationError("Missing SHA256 for configuration.{}".format(key))
            if sha256_file(referenced) != expected.lower():
                raise RunValidationError("Configuration checksum mismatch: {}".format(referenced))
        return {
            "valid": True,
            "run_id": run_id,
            "episode_count": len(episodes),
            "step_count": len(steps),
            "verified_checksum_count": len(checksums),
        }
