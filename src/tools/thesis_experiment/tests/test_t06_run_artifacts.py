import csv
import copy
from pathlib import Path

import pytest
import yaml

from thesis_experiment.run_artifacts import (
    RunValidationError,
    RunValidator,
    write_checksums,
    write_episode_csv,
    write_run_manifest,
    write_step_csv,
)
from thesis_experiment.schema import load_metric_schema


ROOT = Path(__file__).resolve().parents[4]
EPISODE_SCHEMA_PATH = ROOT / "docs/thesis_experiment/schemas/episode_metrics_schema.csv"
STEP_SCHEMA_PATH = ROOT / "docs/thesis_experiment/schemas/step_metrics_schema.csv"
TEMPLATE_PATH = ROOT / "docs/thesis_experiment/templates/run_manifest.template.yaml"


def _record(schema_path):
    result = {}
    for field in load_metric_schema(schema_path):
        if not field.required:
            continue
        if field.value_type == "string":
            result[field.name] = "{}-value".format(field.name)
            if field.unit_or_values == "json":
                result[field.name] = "{}"
        elif field.value_type == "enum":
            result[field.name] = field.enum_values[0]
        elif field.value_type == "int":
            result[field.name] = 0
        elif field.value_type == "float":
            result[field.name] = 0.0
        elif field.value_type == "bool":
            result[field.name] = False
    return result


def _episode(run_id="run-1", episode_id="episode-1"):
    row = _record(EPISODE_SCHEMA_PATH)
    row.update(
        run_id=run_id,
        episode_id=episode_id,
        algorithm="TEB-Default",
        scene_id="scene-1",
        scene_split="test_id",
        success=True,
        collision=False,
        terminated=True,
        truncated=False,
        termination_reason="goal",
    )
    return row


def _step(run_id="run-1", episode_id="episode-1", step_id=0, config_seq=0):
    row = _record(STEP_SCHEMA_PATH)
    row.update(run_id=run_id, episode_id=episode_id, step_id=step_id, config_seq=config_seq)
    return row


def _manifest(directory, run_id="run-1"):
    data = yaml.safe_load(TEMPLATE_PATH.read_text(encoding="utf-8"))
    data.update(
        run_id=run_id,
        created_at_utc="2026-07-12T00:00:00Z",
        algorithm="TEB-Default",
        scene_id="scene-1",
        scene_split="test_id",
    )
    data["artifacts"].update(
        episode_csv="episodes.csv",
        step_log="steps.csv",
        checksums_file="checksums.sha256",
        rosbag="file:///data/external/run-1.bag",
    )
    return data


def _bundle(tmp_path, episodes=None, steps=None):
    episodes = episodes if episodes is not None else [_episode()]
    steps = steps if steps is not None else [_step()]
    episode_file = write_episode_csv(tmp_path / "episodes.csv", episodes, EPISODE_SCHEMA_PATH)
    step_file = write_step_csv(tmp_path / "steps.csv", steps, STEP_SCHEMA_PATH)
    write_checksums(tmp_path / "checksums.sha256", [episode_file, step_file])
    manifest_file = write_run_manifest(tmp_path / "run.yaml", _manifest(tmp_path))
    return manifest_file


def test_writers_use_frozen_field_order_and_validator_accepts_bundle(tmp_path):
    manifest = _bundle(tmp_path)
    with (tmp_path / "steps.csv").open(newline="", encoding="utf-8") as handle:
        assert next(csv.reader(handle)) == [field.name for field in load_metric_schema(STEP_SCHEMA_PATH)]
    report = RunValidator(EPISODE_SCHEMA_PATH, STEP_SCHEMA_PATH).validate(manifest)
    assert report == {
        "valid": True,
        "run_id": "run-1",
        "episode_count": 1,
        "step_count": 1,
        "verified_checksum_count": 2,
    }


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_writer_rejects_non_finite_float(tmp_path, bad_value):
    row = _step()
    row["reward_total"] = bad_value
    with pytest.raises(RunValidationError, match="must be finite"):
        write_step_csv(tmp_path / "steps.csv", [row], STEP_SCHEMA_PATH)


def test_validator_rejects_checksum_tampering(tmp_path):
    manifest = _bundle(tmp_path)
    with (tmp_path / "steps.csv").open("a", encoding="utf-8") as handle:
        handle.write("tampered\n")
    with pytest.raises(RunValidationError, match="(header|checksum|Missing required|Unexpected)"):
        RunValidator(EPISODE_SCHEMA_PATH, STEP_SCHEMA_PATH).validate(manifest)


def test_validator_rejects_duplicate_episode_and_nonmonotonic_steps(tmp_path):
    with pytest.raises(RunValidationError, match="Duplicate episode_id"):
        manifest = _bundle(tmp_path / "duplicate", [_episode(), _episode()], [_step()])
        RunValidator(EPISODE_SCHEMA_PATH, STEP_SCHEMA_PATH).validate(manifest)
    steps = [_step(step_id=1, config_seq=2), _step(step_id=1, config_seq=3)]
    with pytest.raises(RunValidationError, match="step_id is not strictly increasing"):
        manifest = _bundle(tmp_path / "monotonic", [_episode()], steps)
        RunValidator(EPISODE_SCHEMA_PATH, STEP_SCHEMA_PATH).validate(manifest)


def test_validator_rejects_inconsistent_termination(tmp_path):
    episode = _episode()
    episode.update(success=False, terminated=False, truncated=True)
    manifest = _bundle(tmp_path, [episode], [_step()])
    with pytest.raises(RunValidationError, match="termination_reason is inconsistent"):
        RunValidator(EPISODE_SCHEMA_PATH, STEP_SCHEMA_PATH).validate(manifest)


def test_rosbag_is_uri_only_and_cannot_be_csv_destination(tmp_path):
    manifest = _bundle(tmp_path)
    assert RunValidator(EPISODE_SCHEMA_PATH, STEP_SCHEMA_PATH).validate(manifest)["valid"]
    with pytest.raises(RunValidationError, match="rosbag"):
        write_step_csv(tmp_path / "accidental.bag", [_step()], STEP_SCHEMA_PATH)


def test_validator_accepts_declared_multi_scene_matrix_and_rejects_undeclared_scene(tmp_path):
    first = _episode(episode_id="episode-1")
    second = _episode(episode_id="episode-2")
    second["scene_id"] = "scene-2"
    episode_file = write_episode_csv(
        tmp_path / "episodes.csv", [first, second], EPISODE_SCHEMA_PATH
    )
    step_file = write_step_csv(
        tmp_path / "steps.csv", [_step(episode_id="episode-1"),
                                  _step(episode_id="episode-2")], STEP_SCHEMA_PATH
    )
    write_checksums(tmp_path / "checksums.sha256", [episode_file, step_file])
    manifest_data = _manifest(tmp_path)
    manifest_data["scene_id"] = "paired-matrix"
    manifest_data["scene_ids"] = ["scene-1", "scene-2"]
    manifest = write_run_manifest(tmp_path / "run.yaml", manifest_data)
    report = RunValidator(EPISODE_SCHEMA_PATH, STEP_SCHEMA_PATH).validate(manifest)
    assert report["episode_count"] == 2

    manifest_data["scene_ids"] = ["scene-1"]
    write_run_manifest(tmp_path / "run.yaml", manifest_data)
    with pytest.raises(RunValidationError, match="scene_id does not match"):
        RunValidator(EPISODE_SCHEMA_PATH, STEP_SCHEMA_PATH).validate(tmp_path / "run.yaml")
