"""Fail-closed offline sensitivity analysis for the T07 A_TEB mapping.

The input scores are *oriented*: a larger score must mean "more of" the
corresponding semantic eta.  This module deliberately does not infer those
scores from rewards, because doing so would turn a missing calibration into an
undocumented modelling assumption.
"""

import csv
import hashlib
import json
import math
from pathlib import Path
from statistics import median


THETA_ORDER = (
    "max_vel_x", "max_vel_theta", "acc_lim_x", "acc_lim_theta",
    "min_obstacle_dist", "inflation_dist", "weight_obstacle",
    "weight_viapoint", "weight_optimaltime",
)
ETA_ORDER = (
    "speed", "obstacle_conservatism", "clearance", "path_tracking", "smoothness",
)
IDENTITY_FIELDS = ("scene_id", "seed", "direction")


class CalibrationError(ValueError):
    """Raised when calibration evidence is absent, ambiguous, or invalid."""


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(document):
    """Hash a mapping independent of YAML formatting and its sha256 field."""
    payload = dict(document)
    payload.pop("sha256", None)
    try:
        encoded = json.dumps(
            payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError) as exc:
        raise CalibrationError("mapping is not canonically serializable: {}".format(exc))
    return hashlib.sha256(encoded).hexdigest()


def _finite(value, context):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise CalibrationError("{} must be numeric".format(context))
    if not math.isfinite(number):
        raise CalibrationError("{} must be finite".format(context))
    return number


def load_observations(paths):
    """Load dedicated observations or calibration CSVs with explicit scores.

    Required columns are scene_id, seed, theta_name, direction, delta and the
    five ETA_ORDER score columns.  Existing episode CSVs may use
    ``path_efficiency`` for speed and ``min_obstacle_distance`` for clearance;
    the other semantic scores remain explicit to avoid fabricating evidence.
    """
    aliases = {"speed": ("speed", "path_efficiency"),
               "clearance": ("clearance", "min_obstacle_distance")}
    result = []
    for path_value in paths:
        path = Path(path_value).resolve()
        try:
            handle = path.open("r", encoding="utf-8", newline="")
        except OSError as exc:
            raise CalibrationError("cannot read {}: {}".format(path, exc))
        with handle:
            reader = csv.DictReader(handle)
            fields = set(reader.fieldnames or ())
            missing_identity = [name for name in IDENTITY_FIELDS if name not in fields]
            if missing_identity:
                raise CalibrationError("{} missing calibration columns: {}".format(
                    path, ", ".join(missing_identity)))
            theta_field = "theta_name" if "theta_name" in fields else (
                "parameter" if "parameter" in fields else None)
            delta_field = "delta" if "delta" in fields else (
                "normalized_delta" if "normalized_delta" in fields else None)
            if theta_field is None or delta_field is None:
                raise CalibrationError(
                    "{} requires theta_name/parameter and delta/normalized_delta; "
                    "physical theta values cannot establish normalized sensitivity".format(path))
            score_fields = {}
            for eta in ETA_ORDER:
                candidates = aliases.get(eta, (eta,))
                # An explicit oriented score wins when the same CSV also keeps
                # its raw metric (for example clearance plus
                # min_obstacle_distance) for auditability.
                if eta in fields:
                    matches = [eta]
                else:
                    matches = [name for name in candidates if name in fields]
                if len(matches) != 1:
                    raise CalibrationError(
                        "{} requires exactly one oriented score column for {} ({})".format(
                            path, eta, ", ".join(candidates)))
                score_fields[eta] = matches[0]
            for line, raw in enumerate(reader, 2):
                theta = (raw.get(theta_field) or "").strip()
                direction_raw = (raw.get("direction") or "").strip().lower()
                direction = {"-1": "minus", "0": "baseline", "+1": "plus", "1": "plus"}.get(
                    direction_raw, direction_raw)
                if theta not in THETA_ORDER:
                    raise CalibrationError("{}:{} unknown theta_name {}".format(path, line, theta))
                if direction not in ("minus", "baseline", "plus"):
                    raise CalibrationError("{}:{} direction must be minus/baseline/plus".format(path, line))
                row = {
                    "scene_id": (raw.get("scene_id") or "").strip(),
                    "seed": (raw.get("seed") or "").strip(),
                    "theta_name": theta,
                    "direction": direction,
                    "delta": _finite(raw.get(delta_field), "{}:{} normalized delta".format(path, line)),
                    "source_file": str(path),
                    "source_line": line,
                }
                if not row["scene_id"] or not row["seed"]:
                    raise CalibrationError("{}:{} scene_id and seed are required".format(path, line))
                expected_sign = {"minus": -1, "baseline": 0, "plus": 1}[direction]
                actual_sign = (row["delta"] > 0) - (row["delta"] < 0)
                if actual_sign != expected_sign:
                    raise CalibrationError("{}:{} delta sign disagrees with direction".format(path, line))
                row["scores"] = {eta: _finite(raw.get(score_fields[eta]),
                                                    "{}:{} {}".format(path, line, eta))
                                 for eta in ETA_ORDER}
                result.append(row)
    if not result:
        raise CalibrationError("no calibration observations")
    return result


def _sign(value, epsilon):
    return 1 if value > epsilon else (-1 if value < -epsilon else 0)


def _validate_entry_evidence(item, key):
    required = ("paired_scene_seed_count", "sign_consistency", "median_sensitivity",
                "accepted", "rejection_reason", "paired_scene_seeds")
    if not isinstance(item, dict) or any(name not in item for name in required):
        raise CalibrationError("incomplete evidence for {}".format(key))
    try:
        pair_count = int(item["paired_scene_seed_count"])
    except (TypeError, ValueError):
        raise CalibrationError("invalid paired count for {}".format(key))
    consistency = _finite(item["sign_consistency"], "{} sign_consistency".format(key))
    _finite(item["median_sensitivity"], "{} median_sensitivity".format(key))
    if pair_count < 0 or not 0.0 <= consistency <= 1.0:
        raise CalibrationError("invalid evidence statistics for {}".format(key))
    if not isinstance(item["accepted"], bool) or not isinstance(item["paired_scene_seeds"], list):
        raise CalibrationError("invalid evidence types for {}".format(key))
    if item["accepted"] and item["rejection_reason"] is not None:
        raise CalibrationError("accepted evidence {} has a rejection reason".format(key))
    if not item["accepted"] and not item["rejection_reason"]:
        raise CalibrationError("rejected evidence {} lacks a reason".format(key))
    return pair_count


def analyze_sensitivity(rows, min_pairs=2, min_sign_consistency=0.75,
                        min_abs_sensitivity=1e-9, top_k_per_eta=3):
    """Pair +/- observations and construct an evidence-backed sparse matrix."""
    if min_pairs < 1 or not 0.5 <= min_sign_consistency <= 1.0:
        raise CalibrationError("invalid stability thresholds")
    if top_k_per_eta < 1:
        raise CalibrationError("top_k_per_eta must be positive")
    groups = {}
    sources = set()
    for row in rows:
        key = (str(row["scene_id"]), str(row["seed"]), str(row["theta_name"]))
        direction = str(row["direction"]).lower()
        if key not in groups:
            groups[key] = {}
        if direction in groups[key]:
            raise CalibrationError("duplicate {} observation for {}".format(direction, key))
        groups[key][direction] = row
        if row.get("source_file"):
            sources.add(str(row["source_file"]))

    paired = []
    incomplete = []
    sensitivities = {theta: {eta: [] for eta in ETA_ORDER} for theta in THETA_ORDER}
    for key in sorted(groups):
        observations = groups[key]
        missing = [direction for direction in ("minus", "baseline", "plus")
                   if direction not in observations]
        if missing:
            incomplete.append({"scene_id": key[0], "seed": key[1], "theta_name": key[2],
                               "missing": missing})
            continue
        minus, baseline, plus = (observations[name] for name in ("minus", "baseline", "plus"))
        span = float(plus["delta"]) - float(minus["delta"])
        if span <= 0.0:
            raise CalibrationError("non-positive perturbation span for {}".format(key))
        pair_item = {"scene_id": key[0], "seed": key[1], "theta_name": key[2],
                     "minus_delta": float(minus["delta"]), "plus_delta": float(plus["delta"]),
                     "source_lines": sorted(value for value in set([
                         minus.get("source_line"), baseline.get("source_line"), plus.get("source_line")
                     ]) if value is not None),
                     "source_records": [
                         {"direction": direction,
                          "path": observation.get("source_file"),
                          "line": observation.get("source_line")}
                         for direction, observation in (
                             ("minus", minus), ("baseline", baseline), ("plus", plus))
                     ]}
        pair_item["sensitivity"] = {}
        for eta in ETA_ORDER:
            # Baseline is required for drift/outlier auditing even though the
            # central difference itself uses the symmetric endpoints.
            value = (float(plus["scores"][eta]) - float(minus["scores"][eta])) / span
            pair_item["sensitivity"][eta] = value
            sensitivities[key[2]][eta].append(value)
        paired.append(pair_item)

    evidence = {}
    accepted_by_eta = {eta: [] for eta in ETA_ORDER}
    for theta in THETA_ORDER:
        for eta in ETA_ORDER:
            values = sensitivities[theta][eta]
            signs = [_sign(value, min_abs_sensitivity) for value in values]
            nonzero_signs = [value for value in signs if value]
            resolved_values = [value for value in values
                               if abs(value) >= min_abs_sensitivity]
            consistency = (max(nonzero_signs.count(-1), nonzero_signs.count(1)) /
                           float(len(nonzero_signs))) if nonzero_signs else 0.0
            # A semantic can be inactive in a non-relevant geometry (for
            # example obstacle conservatism in a clear turn). Keep those zero
            # pairs in the evidence count, but estimate direction/magnitude
            # only from resolved effects instead of letting two exact zeros
            # erase a repeatable obstacle-scene response.
            estimate = median(resolved_values) if resolved_values else 0.0
            reason = None
            if len(values) < min_pairs:
                reason = "insufficient_paired_scene_seeds"
            elif nonzero_signs and consistency < min_sign_consistency:
                reason = "unstable_sign"
            elif not nonzero_signs or abs(estimate) < min_abs_sensitivity:
                reason = "no_resolved_effect"
            item = {
                "paired_scene_seed_count": len(values),
                "resolved_effect_pair_count": len(resolved_values),
                "sign_consistency": consistency,
                "median_sensitivity": estimate,
                "accepted": reason is None,
                "rejection_reason": reason,
                "paired_scene_seeds": [
                    {"scene_id": pair["scene_id"], "seed": pair["seed"],
                     "source_records": pair["source_records"]}
                    for pair in paired if pair["theta_name"] == theta
                ],
            }
            evidence["{}.{}".format(theta, eta)] = item
            if reason is None:
                accepted_by_eta[eta].append((abs(estimate), theta, estimate))

    matrix = [[0.0 for _ in ETA_ORDER] for _ in THETA_ORDER]
    for eta_index, eta in enumerate(ETA_ORDER):
        ranked = sorted(accepted_by_eta[eta], key=lambda value: (-value[0], value[1]))
        selected = ranked[:top_k_per_eta]
        scale = max((item[0] for item in selected), default=0.0)
        for _, theta, estimate in selected:
            theta_index = THETA_ORDER.index(theta)
            matrix[theta_index][eta_index] = round(estimate / scale, 8)
        for _, theta, _ in ranked[top_k_per_eta:]:
            item = evidence["{}.{}".format(theta, eta)]
            item["accepted"] = False
            item["rejection_reason"] = "sparsity_top_k"

    return {
        "analysis_version": "t07-central-difference-v1",
        "thresholds": {"min_pairs": min_pairs, "min_sign_consistency": min_sign_consistency,
                       "min_abs_sensitivity": min_abs_sensitivity,
                       "top_k_per_eta": top_k_per_eta},
        "source_files": sorted(sources),
        "paired_observations": paired,
        "incomplete_pairs": incomplete,
        "evidence_by_entry": evidence,
        "matrix": matrix,
    }


def build_mapping_document(report, mapping_version, freeze=False):
    """Build a candidate or frozen mapping; freezing rechecks source evidence."""
    if not mapping_version or str(mapping_version).strip().upper() == "TBD":
        raise CalibrationError("mapping_version must be explicit")
    matrix = report.get("matrix")
    if not isinstance(matrix, list) or len(matrix) != len(THETA_ORDER):
        raise CalibrationError("matrix must have 9 rows")
    numeric_matrix = []
    for row_index, row in enumerate(matrix):
        if not isinstance(row, list) or len(row) != len(ETA_ORDER):
            raise CalibrationError("matrix row {} must have 5 columns".format(row_index))
        numeric_matrix.append([_finite(value, "matrix[{}]".format(row_index)) for value in row])
    evidence = report.get("evidence_by_entry")
    expected_keys = {"{}.{}".format(theta, eta) for theta in THETA_ORDER for eta in ETA_ORDER}
    if not isinstance(evidence, dict) or set(evidence) != expected_keys:
        raise CalibrationError("every one of the 45 matrix entries requires evidence")
    source_paths = report.get("source_files")
    if not source_paths:
        raise CalibrationError("source calibration files are required")
    source_records = []
    for value in sorted(set(source_paths)):
        path = Path(value).resolve()
        if not path.is_file():
            raise CalibrationError("source calibration file is unavailable: {}".format(path))
        source_records.append({"path": str(path), "sha256": sha256_file(path)})
    if freeze:
        if report.get("incomplete_pairs"):
            raise CalibrationError("cannot freeze with incomplete +/-/baseline pairs")
        required_pairs = int(report.get("thresholds", {}).get("min_pairs", 1))
        for theta_index, theta in enumerate(THETA_ORDER):
            for eta_index, eta in enumerate(ETA_ORDER):
                item = evidence["{}.{}".format(theta, eta)]
                nonzero = numeric_matrix[theta_index][eta_index] != 0.0
                key = "{}.{}".format(theta, eta)
                if _validate_entry_evidence(item, key) < required_pairs:
                    raise CalibrationError("cannot freeze unscanned entry {}.{}".format(theta, eta))
                if nonzero and not item.get("accepted"):
                    raise CalibrationError("nonzero entry {}.{} lacks accepted evidence".format(theta, eta))
                if not nonzero and item.get("accepted"):
                    raise CalibrationError("accepted entry {}.{} was silently zeroed".format(theta, eta))
        for eta_index, eta in enumerate(ETA_ORDER):
            if not any(row[eta_index] != 0.0 for row in numeric_matrix):
                raise CalibrationError("semantic column {} has no evidenced mapping".format(eta))
    document = {
        "schema_version": "1.0",
        "mapping_version": str(mapping_version),
        "status": "frozen" if freeze else "calibration_candidate",
        "created_from": {"source_files": source_records,
                         "analysis_version": report.get("analysis_version")},
        "eta_order": list(ETA_ORDER),
        "theta_order": list(THETA_ORDER),
        "normalization": "normalized_theta_to_minus_one_plus_one",
        "matrix": numeric_matrix,
        "evidence": {"by_matrix_entry": evidence,
                     "incomplete_pairs": report.get("incomplete_pairs", []),
                     "thresholds": report.get("thresholds", {})},
        "frozen": bool(freeze),
        "sha256": None,
    }
    document["sha256"] = canonical_sha256(document)
    return document


def validate_frozen_mapping(document, verify_sources=True):
    """Validate hash, dimensions, evidence and optionally source-file hashes."""
    if not isinstance(document, dict) or not document.get("frozen") or document.get("status") != "frozen":
        raise CalibrationError("mapping is not frozen")
    if not document.get("mapping_version") or str(document.get("mapping_version")).upper() == "TBD":
        raise CalibrationError("frozen mapping has no explicit mapping_version")
    claimed = document.get("sha256")
    if not claimed or claimed != canonical_sha256(document):
        raise CalibrationError("canonical mapping SHA256 mismatch")
    # Reuse the structural/evidence checks without changing the document hash.
    evidence = document.get("evidence", {}).get("by_matrix_entry")
    report = {"matrix": document.get("matrix"), "evidence_by_entry": evidence,
              "source_files": [item.get("path") for item in document.get("created_from", {}).get("source_files", [])],
              "incomplete_pairs": document.get("evidence", {}).get("incomplete_pairs", [])}
    expected_keys = {"{}.{}".format(theta, eta) for theta in THETA_ORDER for eta in ETA_ORDER}
    if not isinstance(evidence, dict) or set(evidence) != expected_keys:
        raise CalibrationError("every one of the 45 matrix entries requires evidence")
    matrix = report["matrix"]
    if not isinstance(matrix, list) or len(matrix) != 9 or any(not isinstance(row, list) or len(row) != 5 for row in matrix):
        raise CalibrationError("frozen matrix must be numeric 9x5")
    for row_index, row in enumerate(matrix):
        for column_index, value in enumerate(row):
            _finite(value, "matrix[{}][{}]".format(row_index, column_index))
            item = evidence["{}.{}".format(THETA_ORDER[row_index], ETA_ORDER[column_index])]
            _validate_entry_evidence(
                item, "{}.{}".format(THETA_ORDER[row_index], ETA_ORDER[column_index]))
            if float(value) != 0.0 and not item.get("accepted"):
                raise CalibrationError("nonzero frozen entry lacks accepted evidence")
            if float(value) == 0.0 and item.get("accepted"):
                raise CalibrationError("accepted frozen entry was silently zeroed")
    if report["incomplete_pairs"]:
        raise CalibrationError("frozen mapping contains incomplete pairs")
    required_pairs = int(document.get("evidence", {}).get("thresholds", {}).get("min_pairs", 1))
    for key, item in evidence.items():
        if _validate_entry_evidence(item, key) < required_pairs:
            raise CalibrationError("frozen mapping contains an unscanned entry")
    for eta_index, eta in enumerate(ETA_ORDER):
        if not any(float(row[eta_index]) != 0.0 for row in matrix):
            raise CalibrationError("frozen semantic column {} is empty".format(eta))
    if verify_sources:
        records = document.get("created_from", {}).get("source_files", [])
        if not records:
            raise CalibrationError("frozen mapping has no source hashes")
        source_paths = set()
        for item in records:
            if not isinstance(item, dict) or not item.get("path") or not item.get("sha256"):
                raise CalibrationError("invalid source hash record")
            if sha256_file(item["path"]) != item["sha256"]:
                raise CalibrationError("source SHA256 mismatch: {}".format(item["path"]))
            source_paths.add(str(Path(item["path"]).resolve()))
        for key, item in evidence.items():
            for pair in item["paired_scene_seeds"]:
                if not isinstance(pair, dict) or not isinstance(pair.get("source_records"), list):
                    raise CalibrationError("{} lacks source-record evidence".format(key))
                for record in pair["source_records"]:
                    path = str(Path(record.get("path", "")).resolve())
                    if path not in source_paths or not isinstance(record.get("line"), int):
                        raise CalibrationError("{} references unaudited source evidence".format(key))
    return document
