"""Deterministic, ROS-free assessment of one terminal R6-I5 stage report.

The assessor accepts caller-supplied exact hashes for the frozen
preregistration and dynamic stage report.  It opens each selected workspace
file once through a component-wise no-follow chain, hashes and parses the same
bytes, and directly replays every completed journal and raw-evidence binding.
It neither imports ROS nor creates execution state.
"""

from dataclasses import dataclass, replace
import hashlib
import hmac
import json
import math
import os
from pathlib import Path
import stat
from typing import Any, Dict, Mapping, Optional, Tuple

import yaml


STAGE = "V2-04G-R6-I5"
WORKSPACE_ROOT = Path("/home/robot/robot_ws_base_rl")
PREREGISTRATION_PATH = (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_execution_preregistration.yaml"
)
EXPECTED_PREREGISTRATION_SHA256 = (
    "602fc1044fb9e3e8ac284e77cadcbaced95c5edfb9721df0da26f876cc42c073"
)
STAGE_REPORT_PATH = (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/"
    "v2_04g_r6_i5_stage_report.yaml"
)
EXECUTION_REPORT_PATH = (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/"
    "v2_04g_r6_i5_execution_report.yaml"
)
ATTEMPTS_ROOT = (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/attempts"
)
JOURNALS_ROOT = (
    "artifacts/v2/integration/v2_04g_r6_i1/r6_i5_execution/journals"
)
CANONICAL_AUTHORIZATION_PATH = (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_bounded_simulation_authorization.yaml"
)
EXPECTED_AUTHORIZATION_SHA256 = (
    "bc59820b0140b50503657966d735511a8007d9ec8e14f3f2cf237791ff170592"
)
CANONICAL_RELEASE_PATH = (
    "experiments/manifests/v2/integration/"
    "v2_04g_r6_i5_execution_release.yaml"
)
EXPECTED_SCHEDULE_SHA256 = (
    "b52d00a2dc0c1f2edf149d30120451ea836fc1d0589109a1016dc48e9a9d5402"
)
MAX_RESOURCE_BYTES = 64 * 1024 * 1024

EXPECTED_SCHEDULE = [
    {
        "sequence": 1,
        "profile_id": "r6_semantics_legacy_control",
        "scene_id": "v2-04g-r6-i5-dynamic-conflict-single-s5161",
        "seed": 5161,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none",
    },
    {
        "sequence": 2,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r6-i5-dynamic-conflict-single-s5161",
        "seed": 5161,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none_iff_finite_ttc",
    },
    {
        "sequence": 3,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r6-i5-dynamic-conflict-multi-s5162",
        "seed": 5162,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none_iff_finite_ttc",
    },
    {
        "sequence": 4,
        "profile_id": "r6_semantics_legacy_control",
        "scene_id": "v2-04g-r6-i5-dynamic-conflict-multi-s5162",
        "seed": 5162,
        "attempt": 1,
        "expected_ttc_status": "OBSERVED_CONFLICT",
        "expected_overlay_semantics": "non_none",
    },
    {
        "sequence": 5,
        "profile_id": "r6_semantics_legacy_control",
        "scene_id": "v2-04g-r6-i5-dynamic-semantic-clear-s5163",
        "seed": 5163,
        "attempt": 1,
        "expected_ttc_status": "NO_CONFLICT_IN_HORIZON",
        "expected_overlay_semantics": "legacy_non_none_identifiability",
    },
    {
        "sequence": 6,
        "profile_id": "r6_semantics_circle_contact",
        "scene_id": "v2-04g-r6-i5-dynamic-semantic-clear-s5163",
        "seed": 5163,
        "attempt": 1,
        "expected_ttc_status": "NO_CONFLICT_IN_HORIZON",
        "expected_overlay_semantics": "none_iff_no_finite_ttc",
    },
]

IDENTITY_FIELDS = {"stage", "profile_id", "scene_id", "seed", "attempt"}
RAW_FILENAMES = {
    "activation": "activation.yaml",
    "evaluation": "evaluation.yaml",
    "trace": "trace.csv",
    "clearance": "clearance.yaml",
    "process_log": "process.log",
    "teardown_receipt": "teardown_receipt.yaml",
}
PARSED_RAW_LABELS = frozenset(
    {"activation", "evaluation", "clearance", "teardown_receipt"}
)
COMPLETED_JOURNAL_FIELDS = {
    "schema_version",
    "stage",
    "identity",
    "status",
    "lifecycle_phase",
    "resume_forbidden",
    "active_identity",
    "downstream_authorized",
    "evidence_binding",
    "startup_profile_sha256",
    "scene_snapshot",
    "pre_spawn_scene_verification",
    "post_episode_scene_verification",
    "launch_stop_authorization",
}


class R6I5AssessmentError(ValueError):
    """Raised when an assessment input fails closed."""


class _StrictYamlLoader(yaml.SafeLoader):
    def flatten_mapping(self, node):
        for key_node, _ in node.value:
            if key_node.tag == "tag:yaml.org,2002:merge":
                raise R6I5AssessmentError("YAML merge keys are forbidden")
        return super().flatten_mapping(node)


def _construct_unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        if key_node.tag == "tag:yaml.org,2002:merge":
            raise R6I5AssessmentError("YAML merge keys are forbidden")
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in result
        except TypeError as exc:
            raise R6I5AssessmentError("YAML mapping key is not hashable") from exc
        if duplicate:
            raise R6I5AssessmentError("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_StrictYamlLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


@dataclass(frozen=True)
class FileSnapshot:
    declared_path: str
    path: Path
    sha256: str
    size_bytes: int
    payload: bytes
    document: Optional[Mapping[str, Any]]


def _require(condition, message):
    if not condition:
        raise R6I5AssessmentError(message)


def _exact(actual, expected):
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            _exact(actual[key], expected[key]) for key in expected
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            _exact(left, right) for left, right in zip(actual, expected)
        )
    return actual == expected


def _hex_digest(value, label):
    _require(
        type(value) is str
        and len(value) == 64
        and value == value.lower()
        and all(character in "0123456789abcdef" for character in value),
        label + " must be a lowercase SHA256",
    )
    return value


def _closed_mapping(value, fields, label):
    _require(isinstance(value, Mapping), label + " must be a mapping")
    _require(
        set(value) == set(fields),
        "{} keys drifted; missing={} extra={}".format(
            label,
            sorted(set(fields) - set(value)),
            sorted(set(value) - set(fields)),
        ),
    )
    return value


def _validate_data_tree(value, label):
    if isinstance(value, Mapping):
        for key, child in value.items():
            _require(type(key) is str, label + " contains a non-string key")
            _validate_data_tree(child, "{}.{}".format(label, key))
        return
    if isinstance(value, list):
        for index, child in enumerate(value):
            _validate_data_tree(child, "{}[{}]".format(label, index))
        return
    _require(
        value is None or type(value) in {str, bool, int, float},
        label + " contains an unsupported scalar",
    )
    if type(value) is float:
        _require(math.isfinite(value), label + " contains a non-finite float")


def canonical_document_sha256(value):
    _validate_data_tree(value, "canonical document")
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _relative_parts(declared_path):
    _require(type(declared_path) is str and declared_path, "path is empty")
    _require("\x00" not in declared_path, "path contains NUL")
    declared = Path(declared_path)
    _require(not declared.is_absolute(), "workspace path must be relative")
    _require(
        declared.as_posix() == declared_path,
        "workspace path is not normalized POSIX form",
    )
    _require(
        declared.parts
        and all(part not in {"", ".", ".."} for part in declared.parts),
        "workspace path contains traversal",
    )
    return declared.parts


def _read_descriptor_once(descriptor, declared_path):
    before = os.fstat(descriptor)
    _require(stat.S_ISREG(before.st_mode), declared_path + " is not regular")
    _require(before.st_size <= MAX_RESOURCE_BYTES, declared_path + " is too large")
    chunks = []
    remaining = MAX_RESOURCE_BYTES + 1
    while remaining:
        chunk = os.read(descriptor, min(1024 * 1024, remaining))
        if not chunk:
            break
        chunks.append(chunk)
        remaining -= len(chunk)
    payload = b"".join(chunks)
    after = os.fstat(descriptor)
    fields = (
        "st_dev",
        "st_ino",
        "st_mode",
        "st_nlink",
        "st_size",
        "st_mtime_ns",
        "st_ctime_ns",
    )
    _require(
        len(payload) == before.st_size
        and all(getattr(before, field) == getattr(after, field) for field in fields),
        declared_path + " changed during its single read",
    )
    return payload


def _read_workspace_bytes_once(workspace, declared_path):
    root = Path(workspace)
    _require(root.is_absolute() and root == root.resolve(), "workspace is not canonical")
    _require(root.is_dir() and not root.is_symlink(), "workspace root is unsafe")
    _require(hasattr(os, "O_NOFOLLOW"), "O_NOFOLLOW is required")
    parts = _relative_parts(declared_path)
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    directory_flags = flags | os.O_DIRECTORY | os.O_NOFOLLOW
    descriptors = []
    try:
        current = os.open(str(root), directory_flags)
        descriptors.append(current)
        for component in parts[:-1]:
            current = os.open(component, directory_flags, dir_fd=current)
            descriptors.append(current)
        descriptor = os.open(parts[-1], flags | os.O_NOFOLLOW, dir_fd=current)
        descriptors.append(descriptor)
        return _read_descriptor_once(descriptor, declared_path)
    except OSError as exc:
        raise R6I5AssessmentError(
            "cannot safely open {}: {}".format(declared_path, exc)
        ) from exc
    finally:
        for descriptor in reversed(descriptors):
            try:
                os.close(descriptor)
            except OSError:
                pass


def _parse_yaml(payload, label):
    try:
        document = yaml.load(payload.decode("utf-8"), Loader=_StrictYamlLoader)
    except R6I5AssessmentError:
        raise
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise R6I5AssessmentError(
            "cannot strictly parse {}: {}".format(label, exc)
        ) from exc
    _require(isinstance(document, Mapping), label + " must contain one mapping")
    _validate_data_tree(document, label)
    return document


class _SnapshotCache:
    def __init__(self, workspace):
        self.workspace = Path(workspace)
        self.items = {}  # type: Dict[str, FileSnapshot]

    def file(self, declared_path, parse_yaml=False):
        snapshot = self.items.get(declared_path)
        if snapshot is None:
            payload = _read_workspace_bytes_once(self.workspace, declared_path)
            snapshot = FileSnapshot(
                declared_path=declared_path,
                path=self.workspace / declared_path,
                sha256=hashlib.sha256(payload).hexdigest(),
                size_bytes=len(payload),
                payload=payload,
                document=None,
            )
            self.items[declared_path] = snapshot
        if parse_yaml and snapshot.document is None:
            snapshot = replace(
                snapshot,
                document=_parse_yaml(snapshot.payload, declared_path),
            )
            self.items[declared_path] = snapshot
        return snapshot


def _identity(schedule_row):
    return {
        "stage": STAGE,
        "profile_id": schedule_row["profile_id"],
        "scene_id": schedule_row["scene_id"],
        "seed": schedule_row["seed"],
        "attempt": schedule_row["attempt"],
    }


def _validate_identity(actual, expected, label):
    _closed_mapping(actual, IDENTITY_FIELDS, label)
    _require(_exact(dict(actual), expected), label + " drifted")


def _confined_path(declared_path, root, label):
    _relative_parts(declared_path)
    try:
        Path(declared_path).relative_to(Path(root))
    except ValueError as exc:
        raise R6I5AssessmentError(label + " leaves its canonical root") from exc
    return declared_path


def _row_path_sha(row, label):
    _closed_mapping(row, {"path", "sha256"}, label)
    _relative_parts(row["path"])
    _hex_digest(row["sha256"], label + ".sha256")
    return row


def _absolute_attempt_path(workspace, raw, attempt_root, label):
    _require(type(raw) is str and raw, label + " path is missing")
    path = Path(raw)
    _require(path.is_absolute(), label + " path must be absolute")
    try:
        relative = path.relative_to(Path(workspace))
    except ValueError as exc:
        raise R6I5AssessmentError(label + " path leaves workspace") from exc
    declared = relative.as_posix()
    _confined_path(declared, attempt_root, label)
    return declared


def _validate_readiness(identity, activation, evaluation, minimum, persisted):
    for label, document in (("activation", activation), ("evaluation", evaluation)):
        for key in IDENTITY_FIELDS:
            _require(
                key in document
                and type(document[key]) is type(identity[key])
                and document[key] == identity[key],
                "{} identity {} drifted".format(label, key),
            )
    _require(activation.get("all_hard_gates_pass") is True, "activation gates failed")
    counts = {}
    for label, document in (("activation", activation), ("evaluation", evaluation)):
        for field in ("tracker_message_count", "context_message_count"):
            value = document.get(field)
            _require(
                type(value) is int and value >= minimum,
                "{}.{} is below readiness minimum".format(label, field),
            )
            counts["{}_{}".format(label, field)] = value
    expected = {
        "identity": identity,
        "minimum_message_count": minimum,
        "direct_counts": counts,
        "pass": True,
    }
    _require(_exact(persisted, expected), "journal readiness binding drifted")
    return expected


def _semantic_observation(schedule_row, evaluation):
    _require(
        evaluation.get("ttc_status") == schedule_row["expected_ttc_status"],
        "evaluation TTC status drifted",
    )
    for key, expected in {
        "formal_result": False,
        "runtime_ready": False,
        "training_used": False,
        "runtime_policy_manifest_access": False,
        "runtime_scene_labels_available": False,
    }.items():
        _require(
            key in evaluation and _exact(evaluation[key], expected),
            "evaluation {} drifted".format(key),
        )
    finite = evaluation.get("finite_ttc_sample_count")
    _require(type(finite) is int and finite >= 0, "finite TTC count is invalid")
    overlay = evaluation.get("context_overlay_sample_counts")
    _require(isinstance(overlay, Mapping), "overlay counts are missing")
    non_none = 0
    for key, value in overlay.items():
        _require(type(key) is str and key, "overlay label is invalid")
        _require(type(value) is int and value >= 0, "overlay count is invalid")
        if key != "NONE":
            non_none += value
    role = schedule_row["expected_overlay_semantics"]
    if role in {"non_none", "non_none_iff_finite_ttc"}:
        _require(finite > 0 and non_none > 0, "conflict semantic evidence failed")
    elif role == "legacy_non_none_identifiability":
        _require(finite == 0 and non_none > 0, "legacy clear semantics failed")
    elif role == "none_iff_no_finite_ttc":
        _require(finite == 0 and non_none == 0, "aligned clear semantics failed")
    else:
        raise R6I5AssessmentError("unknown overlay semantic role")
    return {
        "finite_ttc_sample_count": finite,
        "non_none_overlay_count": non_none,
    }


def _validate_scene_snapshot(cache, workspace, attempt_root, journal):
    scene = _closed_mapping(
        journal["scene_snapshot"],
        {
            "scene_id",
            "index",
            "source_instance",
            "source_world",
            "snapshot_instance",
            "snapshot_world",
        },
        "journal.scene_snapshot",
    )
    _require(
        scene["scene_id"] == journal["identity"]["scene_id"],
        "journal scene identity drifted",
    )
    snapshot_resources = {}
    for key in ("snapshot_instance", "snapshot_world"):
        row = _closed_mapping(
            scene[key], {"path", "sha256"}, "journal.scene_snapshot." + key
        )
        _hex_digest(row["sha256"], "journal.scene_snapshot." + key + ".sha256")
        declared = _absolute_attempt_path(
            workspace, row["path"], attempt_root, "scene snapshot " + key
        )
        snapshot = cache.file(declared, parse_yaml=False)
        _require(
            hmac.compare_digest(snapshot.sha256, row["sha256"]),
            "scene snapshot hash drifted: " + key,
        )
        snapshot_resources[key] = dict(row)
    expected_pre = {
        "scene_id": scene["scene_id"],
        "verification_phase": "pre_spawn",
        "resources": snapshot_resources,
    }
    expected_post = {
        "scene_id": scene["scene_id"],
        "verification_phase": "post_episode",
        "resources": snapshot_resources,
    }
    _require(
        _exact(journal["pre_spawn_scene_verification"], expected_pre),
        "pre-spawn scene verification drifted",
    )
    _require(
        _exact(journal["post_episode_scene_verification"], expected_post),
        "post-episode scene verification drifted",
    )
    return expected_post


def _validate_completed_attempt(cache, workspace, entry, schedule_row, minimum):
    identity = _identity(schedule_row)
    _validate_identity(entry.get("identity"), identity, "ledger identity")
    _require(entry.get("status") == "evidence_complete", "ledger is not complete")
    _require(entry.get("seed_consumed") is True, "completed seed is not consumed")
    _require(entry.get("evidence_units_consumed") == 1, "completed unit count drifted")
    journal_path = _confined_path(entry.get("journal"), JOURNALS_ROOT, "journal")
    journal_snapshot = cache.file(journal_path, parse_yaml=True)
    journal = _closed_mapping(
        journal_snapshot.document, COMPLETED_JOURNAL_FIELDS, "completed journal"
    )
    _validate_identity(journal["identity"], identity, "journal identity")
    for key, expected in {
        "schema_version": "2.0",
        "stage": STAGE,
        "status": "evidence_complete",
        "lifecycle_phase": "evidence_complete",
        "resume_forbidden": True,
        "active_identity": None,
        "downstream_authorized": False,
    }.items():
        _require(_exact(journal[key], expected), "journal {} drifted".format(key))
    startup_sha = _hex_digest(
        journal["startup_profile_sha256"], "journal startup profile SHA256"
    )
    attempt_name = Path(entry["raw_evidence_root"]).parent.name
    expected_attempt_root = "{}/{}".format(ATTEMPTS_ROOT, attempt_name)
    expected_raw_root = expected_attempt_root + "/raw"
    _require(
        entry["raw_evidence_root"] == expected_raw_root,
        "ledger raw evidence root drifted",
    )
    post_scene = _validate_scene_snapshot(
        cache, workspace, expected_attempt_root, journal
    )

    binding = _closed_mapping(
        journal["evidence_binding"],
        {"identity", "raw_evidence_bound", "readiness_direct_counts", "teardown_restore", "resources"},
        "journal.evidence_binding",
    )
    _validate_identity(binding["identity"], identity, "evidence identity")
    _require(binding["raw_evidence_bound"] is True, "raw evidence is not bound")
    resources = _closed_mapping(binding["resources"], RAW_FILENAMES, "raw resources")
    _require(
        _exact(entry.get("raw_resources"), dict(resources)),
        "ledger and journal raw resources differ",
    )
    snapshots = {}
    for label, filename in RAW_FILENAMES.items():
        row = _row_path_sha(resources[label], "raw resource " + label)
        expected_path = expected_raw_root + "/" + filename
        _require(row["path"] == expected_path, "raw resource path drifted: " + label)
        snapshot = cache.file(row["path"], parse_yaml=label in PARSED_RAW_LABELS)
        _require(
            hmac.compare_digest(snapshot.sha256, row["sha256"]),
            "raw resource hash drifted: " + label,
        )
        snapshots[label] = snapshot

    activation = snapshots["activation"].document
    evaluation = snapshots["evaluation"].document
    clearance = snapshots["clearance"].document
    teardown_receipt = snapshots["teardown_receipt"].document
    readiness = _validate_readiness(
        identity,
        activation,
        evaluation,
        minimum,
        binding["readiness_direct_counts"],
    )
    for key in IDENTITY_FIELDS:
        _require(
            key in clearance
            and type(clearance[key]) is type(identity[key])
            and clearance[key] == identity[key],
            "clearance identity drifted: " + key,
        )
        _require(
            key in teardown_receipt
            and type(teardown_receipt[key]) is type(identity[key])
            and teardown_receipt[key] == identity[key],
            "teardown identity drifted: " + key,
        )
    _require(
        evaluation.get("raw_trace_sha256") == snapshots["trace"].sha256,
        "evaluation raw trace binding drifted",
    )
    semantic = _semantic_observation(schedule_row, evaluation)
    _require(
        _exact(entry.get("semantic_observation"), semantic),
        "ledger semantic observation drifted",
    )
    _require(
        entry.get("expected_ttc_status") == schedule_row["expected_ttc_status"]
        and entry.get("observed_ttc_status") == evaluation["ttc_status"],
        "ledger TTC observation drifted",
    )

    for field in (
        "restore_requested_while_backend_alive",
        "transaction_acknowledged",
        "transaction_readback_match",
        "independent_readback_match",
        "service_response_success",
    ):
        _require(teardown_receipt.get(field) is True, "teardown failed: " + field)
    _require(
        teardown_receipt.get("startup_profile_sha256") == startup_sha
        and teardown_receipt.get("transaction_readback_sha256") == startup_sha
        and teardown_receipt.get("independent_readback_sha256") == startup_sha,
        "teardown profile hashes drifted",
    )
    expected_teardown = {
        "identity": identity,
        "status": "pass",
        "journal_state_path": str(Path(workspace) / journal_path),
        "startup_profile_sha256": startup_sha,
        "two_phase_restore_verified": True,
        "launch_stop_allowed": True,
        "post_episode_scene_verification": post_scene,
    }
    _require(
        _exact(binding["teardown_restore"], expected_teardown),
        "journal teardown binding drifted",
    )
    expected_launch_stop = {
        "launch_stop_allowed": True,
        "identity": identity,
        "teardown_restore": expected_teardown,
    }
    _require(
        _exact(journal["launch_stop_authorization"], expected_launch_stop),
        "journal launch-stop authorization drifted",
    )
    return {
        "sequence": schedule_row["sequence"],
        "identity": identity,
        "status": "evidence_complete",
        "journal": {"path": journal_path, "sha256": journal_snapshot.sha256},
        "raw_resources": {label: dict(resources[label]) for label in sorted(resources)},
        "scene_snapshot_post_episode_verified": True,
        "readiness": readiness,
        "semantic_observation": semantic,
        "teardown_restore_verified": True,
    }


def _validate_stage_boundary(stage, schedule):
    expected_values = {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "evidence_budget_authorized": 6,
        "r5_remaining_units_consumed": 0,
        "r6_i1_forfeited_units_consumed": 0,
        "held_out_5001_5010_accessed": False,
        "retry_count": 0,
        "resume_used": False,
        "attempt_limit_per_identity": 1,
        "planned_identity_count": 6,
        "assessment_complete": False,
        "winner_ranked_or_frozen": False,
    }
    for key, expected in expected_values.items():
        _require(
            key in stage and _exact(stage[key], expected),
            "stage report {} drifted".format(key),
        )
    _require(
        stage.get("status") in {"execution_complete_pending_assessment", "terminal_failure"},
        "stage report is not terminal for assessment",
    )
    release_row = _row_path_sha(stage.get("execution_release"), "stage execution release")
    authorization_row = _row_path_sha(
        stage.get("authorization_envelope"), "stage authorization"
    )
    _require(release_row["path"] == CANONICAL_RELEASE_PATH, "stage release path drifted")
    _require(
        authorization_row
        == {"path": CANONICAL_AUTHORIZATION_PATH, "sha256": EXPECTED_AUTHORIZATION_SHA256},
        "stage authorization binding drifted",
    )
    ledger = stage.get("attempt_ledger")
    _require(isinstance(ledger, list) and len(ledger) == 6, "stage ledger must have six rows")
    for row, expected_row in zip(ledger, schedule):
        _require(
            type(row.get("sequence")) is int
            and row["sequence"] == expected_row["sequence"],
            "ledger sequence drifted",
        )
        _validate_identity(row.get("identity"), _identity(expected_row), "ledger identity")
        _require(row.get("attempt_limit") == 1, "ledger attempt limit drifted")
        _require(row.get("resume_forbidden") is True, "ledger resume boundary drifted")
        _require(
            type(row.get("evidence_units_consumed")) is int
            and row["evidence_units_consumed"] in {0, 1},
            "ledger unit consumption is invalid",
        )
    consumed = sum(row["evidence_units_consumed"] for row in ledger)
    _require(
        type(stage.get("evidence_units_consumed")) is int
        and stage["evidence_units_consumed"] == consumed,
        "stage evidence consumption drifted",
    )
    statuses = [row.get("status") for row in ledger]
    if stage["status"] == "execution_complete_pending_assessment":
        _require(statuses == ["evidence_complete"] * 6, "successful ledger is incomplete")
        _require(consumed == 6, "successful evidence consumption drifted")
        _require(stage.get("terminal_failure") is None, "successful stage has a failure")
        _require(stage.get("unattempted_budget_forfeited") == 0, "successful forfeiture drifted")
    else:
        terminal_indices = [index for index, value in enumerate(statuses) if value == "terminal_failure"]
        _require(len(terminal_indices) == 1, "terminal ledger must have one failure")
        terminal_index = terminal_indices[0]
        _require(
            statuses[:terminal_index] == ["evidence_complete"] * terminal_index,
            "terminal ledger completed prefix drifted",
        )
        _require(
            statuses[terminal_index + 1 :]
            == ["forfeited_unattempted_after_terminal_failure"] * (5 - terminal_index),
            "terminal ledger forfeiture suffix drifted",
        )
        _require(
            isinstance(stage.get("terminal_failure"), Mapping),
            "terminal failure receipt is missing",
        )
        _require(
            stage.get("unattempted_budget_forfeited") == 6 - consumed,
            "terminal forfeited budget drifted",
        )
    return ledger


def build_assessment(
    workspace,
    preregistration_path,
    caller_preregistration_sha256,
    stage_report_path,
    caller_stage_report_sha256,
):
    """Build a deterministic pass/fail report without writing any file."""

    root = Path(workspace)
    _require(root.is_absolute() and root == root.resolve(), "workspace is not canonical")
    _require(preregistration_path == PREREGISTRATION_PATH, "preregistration path drifted")
    _require(stage_report_path == STAGE_REPORT_PATH, "stage report path drifted")
    preregistration_sha = _hex_digest(
        caller_preregistration_sha256, "caller preregistration SHA256"
    )
    stage_report_sha = _hex_digest(caller_stage_report_sha256, "caller stage report SHA256")
    _require(
        preregistration_sha == EXPECTED_PREREGISTRATION_SHA256,
        "caller preregistration SHA256 is not frozen I5 authority",
    )
    cache = _SnapshotCache(root)
    preregistration = cache.file(PREREGISTRATION_PATH, parse_yaml=True)
    stage_report = cache.file(STAGE_REPORT_PATH, parse_yaml=True)
    _require(
        hmac.compare_digest(preregistration.sha256, preregistration_sha),
        "preregistration trust-anchor hash mismatch",
    )
    _require(
        hmac.compare_digest(stage_report.sha256, stage_report_sha),
        "stage report trust-anchor hash mismatch",
    )
    prereg = preregistration.document
    _require(prereg.get("stage") == STAGE, "preregistration stage drifted")
    _require(prereg.get("execution_authorized") is False, "preregistration became authorizing")
    schedule = prereg.get("schedule")
    _require(_exact(schedule, EXPECTED_SCHEDULE), "preregistration schedule drifted")
    _require(
        canonical_document_sha256(schedule) == EXPECTED_SCHEDULE_SHA256,
        "preregistration schedule SHA256 drifted",
    )
    minimum = prereg.get("readiness_gate", {}).get("minimum_message_count_per_stream")
    _require(type(minimum) is int and minimum > 0, "readiness minimum drifted")
    stage = stage_report.document
    ledger = _validate_stage_boundary(stage, schedule)

    replays = []
    integrity_failures = []
    for entry, schedule_row in zip(ledger, schedule):
        if entry["status"] != "evidence_complete":
            continue
        try:
            replays.append(
                _validate_completed_attempt(cache, root, entry, schedule_row, minimum)
            )
        except R6I5AssessmentError as exc:
            integrity_failures.append(
                {
                    "sequence": schedule_row["sequence"],
                    "identity": _identity(schedule_row),
                    "error": str(exc)[:4096],
                }
            )

    complete = len(replays) == 6 and not integrity_failures
    integration_pass = bool(
        stage["status"] == "execution_complete_pending_assessment" and complete
    )
    if integration_pass:
        status = "simulation_integration_validation_pass"
    elif stage["status"] == "terminal_failure":
        status = "terminal_execution_failure_preserved"
    else:
        status = "simulation_integration_validation_fail"
    result = {
        "schema_version": "1.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "assessment_id": "fam_teb_v2_04g_r6_i5_deterministic_assessment_1",
        "status": status,
        "assessment_result": "pass" if integration_pass else "fail",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "winner_ranked_or_frozen": False,
        "downstream_authorized": False,
        "safety_performance_or_generalization_claimed": False,
        "claim_limit": "fresh_simulation_runtime_evaluator_semantic_integration_only",
        "preregistration": {
            "path": PREREGISTRATION_PATH,
            "sha256": preregistration.sha256,
        },
        "stage_report_input": {
            "path": STAGE_REPORT_PATH,
            "sha256": stage_report.sha256,
        },
        "exact_schedule_sha256": EXPECTED_SCHEDULE_SHA256,
        "planned_identity_count": 6,
        "completed_identity_count": len(replays),
        "evidence_budget_authorized": 6,
        "evidence_units_consumed": stage["evidence_units_consumed"],
        "unattempted_budget_forfeited": stage.get("unattempted_budget_forfeited", 0),
        "all_completed_journals_directly_replayed": (
            len(replays)
            == sum(entry["status"] == "evidence_complete" for entry in ledger)
            and not integrity_failures
        ),
        "attempt_replays": replays,
        "integrity_failures": integrity_failures,
        "ttc_status_matches_preregistration": complete,
        "semantic_schedule_pass": complete,
        "readiness_direct_counts_pass": complete,
        "two_phase_teardown_restore_pass": complete,
        "integration_validation_pass": integration_pass,
    }
    _validate_data_tree(result, "assessment")
    return result


def write_assessment_once(workspace, assessment):
    """Persist one already-built canonical report with exclusive creation."""

    root = Path(workspace)
    _require(root.is_absolute() and root == root.resolve(), "workspace is not canonical")
    _require(isinstance(assessment, Mapping), "assessment must be a mapping")
    _require(
        assessment.get("stage") == STAGE
        and assessment.get("assessment_result") in {"pass", "fail"},
        "assessment boundary drifted",
    )
    _validate_data_tree(assessment, "assessment")
    payload = yaml.safe_dump(
        dict(assessment), sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    target = root / EXECUTION_REPORT_PATH
    target.parent.mkdir(parents=False, exist_ok=True)
    descriptor = os.open(
        str(target),
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
        0o444,
    )
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        directory = os.open(str(target.parent), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        try:
            target.unlink()
        except OSError:
            pass
        raise
    return {
        "path": EXECUTION_REPORT_PATH,
        "sha256": hashlib.sha256(payload).hexdigest(),
    }


__all__ = [
    "EXECUTION_REPORT_PATH",
    "EXPECTED_PREREGISTRATION_SHA256",
    "EXPECTED_SCHEDULE",
    "EXPECTED_SCHEDULE_SHA256",
    "FileSnapshot",
    "PREREGISTRATION_PATH",
    "R6I5AssessmentError",
    "STAGE",
    "STAGE_REPORT_PATH",
    "build_assessment",
    "canonical_document_sha256",
    "write_assessment_once",
]
