import csv
import copy

import pytest

from thesis_experiment.calibration import (
    ETA_ORDER, THETA_ORDER, CalibrationError, analyze_sensitivity,
    build_mapping_document, canonical_sha256, load_observations,
    validate_frozen_mapping,
)


FIELDS = ["scene_id", "seed", "theta_name", "direction", "delta"] + list(ETA_ORDER)


def _write(path, omit=None, unstable=False, all_theta=False):
    rows = []
    theta_names = THETA_ORDER if all_theta else ("max_vel_x",)
    for theta_index, theta in enumerate(theta_names, 1):
        for scene, seed, factor in (("open", "7", 1.0), ("narrow", "7", -1.0 if unstable else 2.0)):
            for direction, delta in (("minus", -0.1), ("baseline", 0.0), ("plus", 0.1)):
                if omit == (scene, seed, direction):
                    continue
                row = {"scene_id": scene, "seed": seed, "theta_name": theta,
                       "direction": direction, "delta": delta}
                for index, eta in enumerate(ETA_ORDER, 1):
                    row[eta] = index + theta_index * factor * delta
                rows.append(row)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def test_pairs_same_seed_and_builds_sparse_candidate(tmp_path):
    source = tmp_path / "observations.csv"
    _write(source)
    report = analyze_sensitivity(load_observations([source]), min_pairs=2, top_k_per_eta=1)
    item = report["evidence_by_entry"]["max_vel_x.speed"]
    assert item["paired_scene_seed_count"] == 2
    assert item["sign_consistency"] == 1.0
    assert item["paired_scene_seeds"][0]["source_records"][0]["path"] == str(source.resolve())
    assert report["matrix"][0] == [1.0] * 5
    assert all(value == 0.0 for row in report["matrix"][1:] for value in row)


def test_numeric_direction_and_parameter_aliases(tmp_path):
    source = tmp_path / "aliases.csv"
    _write(source)
    text = source.read_text(encoding="utf-8")
    text = text.replace("theta_name", "parameter", 1).replace("direction,delta", "direction,normalized_delta", 1)
    text = text.replace(",minus,", ",-1,").replace(",baseline,", ",0,").replace(",plus,", ",1,")
    source.write_text(text, encoding="utf-8")
    assert len(load_observations([source])) == 6


def test_missing_pair_is_reported_and_freeze_fails_closed(tmp_path):
    source = tmp_path / "incomplete.csv"
    _write(source, omit=("narrow", "7", "plus"))
    report = analyze_sensitivity(load_observations([source]), min_pairs=1)
    assert report["incomplete_pairs"][0]["missing"] == ["plus"]
    with pytest.raises(CalibrationError, match="incomplete"):
        build_mapping_document(report, "A_TEB_v1", freeze=True)


def test_unstable_sign_is_rejected_not_explained_away(tmp_path):
    source = tmp_path / "unstable.csv"
    _write(source, unstable=True)
    report = analyze_sensitivity(load_observations([source]), min_pairs=2,
                                 min_sign_consistency=0.75)
    item = report["evidence_by_entry"]["max_vel_x.clearance"]
    assert not item["accepted"]
    assert item["rejection_reason"] == "unstable_sign"
    assert report["matrix"][0][ETA_ORDER.index("clearance")] == 0.0


def test_frozen_hash_and_source_hash_are_verified(tmp_path):
    source = tmp_path / "complete.csv"
    _write(source, all_theta=True)
    report = analyze_sensitivity(load_observations([source]), min_pairs=2, top_k_per_eta=1)
    document = build_mapping_document(report, "A_TEB_v1", freeze=True)
    assert document["sha256"] == canonical_sha256(document)
    assert validate_frozen_mapping(document) is document
    tampered = copy.deepcopy(document)
    tampered["matrix"][0][0] = -1.0
    with pytest.raises(CalibrationError, match="SHA256"):
        validate_frozen_mapping(tampered)
    source.write_text(source.read_text(encoding="utf-8") + "\n", encoding="utf-8")
    with pytest.raises(CalibrationError, match="source SHA256"):
        validate_frozen_mapping(document)


def test_freeze_rejects_missing_entry_evidence(tmp_path):
    source = tmp_path / "complete.csv"
    _write(source, all_theta=True)
    report = analyze_sensitivity(load_observations([source]), min_pairs=2)
    del report["evidence_by_entry"]["max_vel_x.speed"]
    with pytest.raises(CalibrationError, match="45 matrix entries"):
        build_mapping_document(report, "A_TEB_v1", freeze=True)


def test_freeze_rejects_unscanned_parameters(tmp_path):
    source = tmp_path / "only_one_theta.csv"
    _write(source)
    report = analyze_sensitivity(load_observations([source]), min_pairs=2)
    with pytest.raises(CalibrationError, match="unscanned"):
        build_mapping_document(report, "A_TEB_v1", freeze=True)
