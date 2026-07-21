"""ROS-free integrity protocols required before any future R6 execution.

These helpers implement and unit-test the six D1 repairs without authorizing
or running an episode.  A future executor must bind these protocols into its
entrypoint and regenerate a complete dependency closure before authorization.
"""

import copy
from dataclasses import dataclass
import fcntl
import hashlib
import math
import os
from pathlib import Path
import stat
import tempfile
from typing import Dict, Mapping, Optional, Sequence, Tuple

import yaml


RISK_REPAIR_IDS = (
    "D1-RISK-READINESS-DIRECT-COUNTS",
    "D1-RISK-COMPILED-SCENE-TOCTOU",
    "D1-RISK-SIGINT-IN-PROGRESS",
    "D1-RISK-ASSESSMENT-RAW-BINDING",
    "D1-RISK-EXECUTION-HASH-CLOSURE",
    "D1-RISK-TEARDOWN-RESTORE",
)
TERMINAL_STATUSES = {
    "evidence_complete",
    "terminal_failure",
    "terminal_interrupted",
    "terminal_unclean_shutdown",
    "terminal_incomplete",
    "terminal_teardown_failure",
}
IDENTITY_FIELDS = ("stage", "profile_id", "scene_id", "seed", "attempt")
RAW_EVIDENCE_LABELS = {
    "activation",
    "evaluation",
    "trace",
    "clearance",
    "process_log",
    "teardown_receipt",
}
TERMINAL_EVIDENCE_PHASES = (
    "attempt_started",
    "startup_profile_captured",
    "pre_spawn_scene_verified",
    "execution_started",
    "post_episode_scene_verified",
)


class R6IntegrityError(ValueError):
    """Raised when an R6 execution-integrity boundary fails closed."""


class R6TeardownFailure(R6IntegrityError):
    """Raised when startup-profile restoration is not machine-verified."""


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe YAML loader rejecting duplicate mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise R6IntegrityError("duplicate YAML key: {!r}".format(key))
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _require(condition, message):
    if not condition:
        raise R6IntegrityError(message)


def sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def _read_regular_file_once(path, label):
    source = Path(path)
    _require(not source.is_symlink(), "{} is a symlink".format(label))
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(str(source), flags)
    except OSError as exc:
        raise R6IntegrityError(
            "cannot open {} {}: {}".format(label, source, exc)
        ) from exc
    try:
        metadata = os.fstat(descriptor)
        _require(stat.S_ISREG(metadata.st_mode), "{} is not regular".format(label))
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def sha256_file(path):
    return sha256_bytes(_read_regular_file_once(path, "file"))


def _strict_yaml_bytes(payload, label):
    try:
        value = yaml.load(payload.decode("utf-8"), Loader=_UniqueKeyLoader)
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise R6IntegrityError(
            "cannot strictly load {}: {}".format(label, exc)
        ) from exc
    _require(isinstance(value, dict), "{} must contain a mapping".format(label))
    return value


def strict_yaml(path):
    source = Path(path)
    try:
        payload = _read_regular_file_once(source, "YAML")
    except (OSError, R6IntegrityError) as exc:
        raise R6IntegrityError(
            "cannot strictly load {}: {}".format(source, exc)
        ) from exc
    return _strict_yaml_bytes(payload, str(source))


def _declared_workspace_file(root, declared_path, label):
    _require(
        isinstance(declared_path, str) and declared_path,
        "{} path must be a non-empty string".format(label),
    )
    declared = Path(declared_path)
    _require(not declared.is_absolute(), "{} path must be relative".format(label))
    _require(".." not in declared.parts, "{} path contains parent traversal".format(label))
    raw = Path(root) / declared
    _require(not raw.is_symlink(), "{} is a symlink".format(label))
    return _inside(root, raw, label)


def _inside(root, path, label):
    boundary = Path(root).resolve()
    resolved = Path(path).resolve()
    try:
        resolved.relative_to(boundary)
    except ValueError as exc:
        raise R6IntegrityError(
            "{} leaves allowed root: {}".format(label, resolved)
        ) from exc
    return resolved


def _integer(value, label):
    _require(
        type(value) is int and value >= 0,
        "{} must be a non-negative integer".format(label),
    )
    return value


def _canonical_identity(value):
    _require(
        isinstance(value, Mapping) and set(value) == set(IDENTITY_FIELDS),
        "attempt identity must contain exactly {}".format(
            ",".join(IDENTITY_FIELDS)
        ),
    )
    _require(
        all(
            isinstance(value[key], str) and value[key]
            for key in ("stage", "profile_id", "scene_id")
        ),
        "attempt identity strings must be non-empty",
    )
    _integer(value["seed"], "identity.seed")
    _require(
        type(value["attempt"]) is int and value["attempt"] > 0,
        "identity.attempt must be a positive integer",
    )
    return {key: value[key] for key in IDENTITY_FIELDS}


def _identity_exact_equal(actual, expected):
    """Compare all five identity fields without YAML bool/int coercion."""

    return (
        isinstance(actual, Mapping)
        and set(actual) == set(IDENTITY_FIELDS)
        and all(
            type(actual[key]) is type(expected[key])
            and actual[key] == expected[key]
            for key in IDENTITY_FIELDS
        )
    )


def validate_readiness_raw_evidence(
    expected_identity: Mapping,
    activation: Mapping,
    evaluation: Mapping,
    minimum_message_count: int,
):
    """Directly bind identity and hard-check raw tracker/context counts."""

    minimum = _integer(minimum_message_count, "minimum message count")
    _require(minimum > 0, "minimum message count must be positive")
    identity = _canonical_identity(expected_identity)
    for key in IDENTITY_FIELDS:
        _require(
            type(activation.get(key)) is type(identity[key])
            and activation.get(key) == identity[key],
            "activation identity mismatch: {}".format(key),
        )
        _require(
            type(evaluation.get(key)) is type(identity[key])
            and evaluation.get(key) == identity[key],
            "evaluation identity mismatch: {}".format(key),
        )
    counts = {}
    for document_name, document in (
        ("activation", activation),
        ("evaluation", evaluation),
    ):
        for field in ("tracker_message_count", "context_message_count"):
            value = _integer(
                document.get(field), "{}.{}".format(document_name, field)
            )
            _require(
                value >= minimum,
                "{}.{} is below {}".format(document_name, field, minimum),
            )
            counts["{}_{}".format(document_name, field)] = value
    return {
        "identity": identity,
        "minimum_message_count": minimum,
        "direct_counts": counts,
        "pass": True,
    }


_COMPILED_LEASE_TOKEN = object()


@dataclass(frozen=True)
class CompiledSceneLease:
    """Immutable in-memory pair acquired from an index-verified scene."""

    scene_id: str
    index_path: str
    index_sha256: str
    instance_source_path: str
    instance_sha256: str
    instance_bytes: bytes
    world_source_path: str
    world_sha256: str
    world_bytes: bytes
    _token: object


_VERIFIED_SCENE_TOKEN = object()
_MATERIALIZED_SCENE_TOKEN = object()


class MaterializedSceneSnapshot:
    """Opaque attempt-local scene pair produced from a verified lease."""

    def __init__(self, document, token):
        _require(
            token is _MATERIALIZED_SCENE_TOKEN,
            "materialized scene snapshot cannot be constructed directly",
        )
        self._document = copy.deepcopy(document)

    @property
    def scene_id(self):
        return self._document["scene_id"]

    def as_document(self):
        return copy.deepcopy(self._document)


class VerifiedSceneSnapshot:
    """Opaque proof that both attempt-local scene files matched their lease."""

    def __init__(self, scene_id, phase, resources, token):
        _require(
            token is _VERIFIED_SCENE_TOKEN,
            "verified scene snapshot cannot be constructed directly",
        )
        self._scene_id = str(scene_id)
        self._phase = str(phase)
        self._resources = copy.deepcopy(resources)

    @property
    def scene_id(self):
        return self._scene_id

    @property
    def phase(self):
        return self._phase

    def as_document(self):
        return {
            "scene_id": self.scene_id,
            "verification_phase": self.phase,
            "resources": copy.deepcopy(self._resources),
        }


def acquire_compiled_scene_lease(
    workspace,
    index_path,
    expected_index_sha256,
    scene_id,
):
    """Read and bind an exact compiled pair before any launch materialization."""

    root = Path(workspace).resolve()
    raw_index = Path(index_path)
    if not raw_index.is_absolute():
        raw_index = root / raw_index
    _require(not raw_index.is_symlink(), "compiled index is a symlink")
    index = _inside(root, raw_index, "compiled index")
    index_payload = _read_regular_file_once(index, "compiled index")
    _require(
        sha256_bytes(index_payload) == expected_index_sha256,
        "compiled index hash drifted",
    )
    document = _strict_yaml_bytes(index_payload, str(index))
    files = document.get("files")
    _require(isinstance(files, list), "compiled index files must be a list")
    seen = set()
    selected = {}
    for row in files:
        _require(
            isinstance(row, dict) and set(row) == {"path", "sha256"},
            "compiled child declaration drifted",
        )
        child = _declared_workspace_file(root, row["path"], "compiled child")
        _inside(index.parent, child, "compiled child")
        relative = str(child.relative_to(root))
        _require(relative not in seen, "duplicate compiled child path")
        seen.add(relative)
        payload = _read_regular_file_once(child, "compiled child")
        digest = sha256_bytes(payload)
        _require(digest == row["sha256"], "compiled child hash drifted")
        if child.name == scene_id + ".instance.yaml":
            _require("instance" not in selected, "duplicate instance child")
            selected["instance"] = (relative, digest, payload)
        elif child.name == scene_id + ".world":
            _require("world" not in selected, "duplicate world child")
            selected["world"] = (relative, digest, payload)
    _require(
        set(selected) == {"instance", "world"},
        "compiled scene pair is incomplete",
    )
    return CompiledSceneLease(
        scene_id=str(scene_id),
        index_path=str(index.relative_to(root)),
        index_sha256=expected_index_sha256,
        instance_source_path=selected["instance"][0],
        instance_sha256=selected["instance"][1],
        instance_bytes=selected["instance"][2],
        world_source_path=selected["world"][0],
        world_sha256=selected["world"][1],
        world_bytes=selected["world"][2],
        _token=_COMPILED_LEASE_TOKEN,
    )


def _exclusive_write(path, payload):
    target = Path(path)
    descriptor = os.open(str(target), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o444)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException:
        try:
            target.unlink()
        except OSError:
            pass
        raise


def materialize_scene_snapshot(lease: CompiledSceneLease, target_directory):
    """Write content-addressed attempt-local files from verified in-memory bytes."""

    _require(
        isinstance(lease, CompiledSceneLease)
        and lease._token is _COMPILED_LEASE_TOKEN,
        "scene materialization requires an acquired lease",
    )
    _require(
        lease.scene_id
        and Path(lease.scene_id).name == lease.scene_id
        and sha256_bytes(lease.instance_bytes) == lease.instance_sha256
        and sha256_bytes(lease.world_bytes) == lease.world_sha256,
        "compiled scene lease payload drifted",
    )
    target = Path(target_directory)
    target.mkdir(parents=True, exist_ok=False)
    instance = target / "{}__{}.instance.yaml".format(
        lease.scene_id, lease.instance_sha256
    )
    world = target / "{}__{}.world".format(
        lease.scene_id, lease.world_sha256
    )
    _exclusive_write(instance, lease.instance_bytes)
    _exclusive_write(world, lease.world_bytes)
    directory_fd = os.open(str(target), os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)
    snapshot = {
        "scene_id": lease.scene_id,
        "index": {
            "path": lease.index_path,
            "sha256": lease.index_sha256,
        },
        "source_instance": {
            "path": lease.instance_source_path,
            "sha256": lease.instance_sha256,
        },
        "source_world": {
            "path": lease.world_source_path,
            "sha256": lease.world_sha256,
        },
        "snapshot_instance": {
            "path": str(instance),
            "sha256": lease.instance_sha256,
        },
        "snapshot_world": {
            "path": str(world),
            "sha256": lease.world_sha256,
        },
    }
    return MaterializedSceneSnapshot(snapshot, _MATERIALIZED_SCENE_TOKEN)


def revalidate_scene_snapshot(snapshot, phase):
    """Fail if an attempt-local pair drifts before spawn or after execution."""

    _require(
        isinstance(snapshot, MaterializedSceneSnapshot),
        "scene revalidation requires a materialized snapshot token",
    )
    _require(
        phase in {"pre_spawn", "post_episode"},
        "scene verification phase is invalid",
    )
    document = snapshot.as_document()
    verified = {}
    for key in ("snapshot_instance", "snapshot_world"):
        row = document.get(key)
        _require(
            isinstance(row, dict) and set(row) == {"path", "sha256"},
            "{} declaration drifted".format(key),
        )
        path = Path(row["path"])
        _require(path.is_file() and not path.is_symlink(), "{} is unsafe".format(key))
        _require(sha256_file(path) == row["sha256"], "{} hash drifted".format(key))
        verified[key] = dict(row)
    return VerifiedSceneSnapshot(
        snapshot.scene_id, phase, verified, _VERIFIED_SCENE_TOKEN
    )


def _yaml_bytes(value):
    return yaml.safe_dump(value, sort_keys=False).encode("utf-8")


def _fsync_directory(path):
    directory_fd = os.open(str(path), os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def _exclusive_yaml_create(path, value):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    _require(not target.is_symlink(), "attempt journal is a symlink")
    payload = _yaml_bytes(value)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(str(target), flags, 0o600)
    except FileExistsError as exc:
        raise R6IntegrityError("attempt state already exists") from exc
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        _fsync_directory(target.parent)
    except BaseException:
        try:
            target.unlink()
        except OSError:
            pass
        raise


def _atomic_yaml(path, value):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    _require(not target.is_symlink(), "atomic YAML target is a symlink")
    payload = _yaml_bytes(value)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=target.name + ".tmp.", dir=str(target.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(target))
        _fsync_directory(target.parent)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def _journal_lock_path(path):
    target = Path(path)
    return target.with_suffix(target.suffix + ".lock")


def canonical_attempt_state_path(journal_root, identity):
    """Derive one state path from the complete canonical attempt identity."""

    canonical = _canonical_identity(identity)
    root = Path(journal_root)
    _require(not root.is_symlink(), "attempt journal root is a symlink")
    payload = "\0".join(
        str(canonical[key]) for key in IDENTITY_FIELDS
    ).encode("utf-8")
    identity_sha256 = sha256_bytes(payload)
    return root / "attempt_{}.yaml".format(identity_sha256)


def _acquire_journal_lock(path):
    lock_path = _journal_lock_path(path)
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    _require(not lock_path.is_symlink(), "attempt lock is a symlink")
    flags = os.O_RDWR | os.O_CREAT | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(str(lock_path), flags, 0o600)
    try:
        metadata = os.fstat(descriptor)
        _require(stat.S_ISREG(metadata.st_mode), "attempt lock is not regular")
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        os.close(descriptor)
        raise R6IntegrityError("attempt identity is already active") from exc
    except BaseException:
        os.close(descriptor)
        raise
    return descriptor


def _release_journal_lock(descriptor):
    if descriptor is not None:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)


def _seal_orphaned_attempt_locked(path, terminal_evidence=None):
    target = Path(path)
    document = strict_yaml(target)
    status = document.get("status")
    if status in TERMINAL_STATUSES:
        return document
    if terminal_evidence is not None:
        _require(
            isinstance(terminal_evidence, VerifiedTerminalEvidence)
            and _identity_exact_equal(
                terminal_evidence.identity, document.get("identity")
            ),
            "orphan terminal evidence identity mismatch",
        )
        _require(
            document.get("lifecycle_phase") in TERMINAL_EVIDENCE_PHASES
            and terminal_evidence.terminal_phase
            == document.get("lifecycle_phase"),
            "orphan terminal evidence phase does not match persisted lifecycle",
        )
        if document.get("lifecycle_phase") == "post_episode_scene_verified":
            _require(
                terminal_evidence.produced_labels == RAW_EVIDENCE_LABELS,
                "post-episode orphan evidence must bind all raw resources",
            )
        evidence_binding = terminal_evidence.as_document()
    else:
        evidence_binding = {
            "terminal_raw_evidence_declared": False,
            "protocol_violation": (
                "orphan sealed before terminal evidence reconciliation"
            ),
        }
    document.update({
        "status": "terminal_unclean_shutdown",
        "lifecycle_phase": "terminal_unclean_shutdown",
        "resume_forbidden": True,
        "active_identity": None,
        "downstream_authorized": False,
        "evidence_binding": evidence_binding,
    })
    _atomic_yaml(target, document)
    return document


def seal_orphaned_attempt(journal_root, identity, terminal_evidence=None):
    """Seal a persisted non-terminal attempt while holding its lifecycle lock."""

    canonical_identity = _canonical_identity(identity)
    target = canonical_attempt_state_path(journal_root, canonical_identity)
    descriptor = _acquire_journal_lock(target)
    try:
        document = strict_yaml(target)
        _require(
            _identity_exact_equal(
                document.get("identity"), canonical_identity
            ),
            "orphan journal identity drifted",
        )
        return _seal_orphaned_attempt_locked(target, terminal_evidence)
    finally:
        _release_journal_lock(descriptor)


_VALIDATED_BINDING_TOKEN = object()
_VALIDATED_TERMINAL_BINDING_TOKEN = object()
_VERIFIED_TEARDOWN_TOKEN = object()
_STARTUP_PROFILE_TOKEN = object()


class StartupProfileLease:
    """Opaque startup bytes captured by the attempt journal before execution."""

    def __init__(self, identity, journal_state_path, payload, token):
        _require(
            token is _STARTUP_PROFILE_TOKEN,
            "startup profile lease cannot be constructed directly",
        )
        self._token = token
        self._identity = _canonical_identity(identity)
        self._journal_state_path = str(
            Path(journal_state_path).absolute()
        )
        self._payload = bytes(payload)

    @property
    def identity(self):
        return copy.deepcopy(self._identity)

    @property
    def journal_state_path(self):
        return self._journal_state_path

    @property
    def sha256(self):
        return sha256_bytes(self._payload)

    def payload(self):
        return bytes(self._payload)


class VerifiedTeardownRestore:
    """Opaque restore proof required before launch stop or evidence completion."""

    def __init__(self, identity, document, token):
        _require(
            token is _VERIFIED_TEARDOWN_TOKEN,
            "verified teardown cannot be constructed directly",
        )
        self._token = token
        self._identity = _canonical_identity(identity)
        self._document = copy.deepcopy(document)

    @property
    def identity(self):
        return copy.deepcopy(self._identity)

    def as_document(self):
        return copy.deepcopy(self._document)


class VerifiedTerminalEvidence:
    """Opaque explicit produced/not-produced bundle for terminal attempts."""

    def __init__(self, identity, resources, terminal_phase, token):
        _require(
            token is _VALIDATED_TERMINAL_BINDING_TOKEN,
            "terminal evidence cannot be constructed directly",
        )
        self._token = token
        self._identity = copy.deepcopy(identity)
        self._resources = copy.deepcopy(resources)
        self._terminal_phase = terminal_phase

    @property
    def identity(self):
        return copy.deepcopy(self._identity)

    @property
    def terminal_phase(self):
        return self._terminal_phase

    @property
    def produced_labels(self):
        return {
            label
            for label, row in self._resources.items()
            if row.get("status") == "produced"
        }

    def as_document(self):
        return {
            "identity": copy.deepcopy(self._identity),
            "terminal_raw_evidence_declared": True,
            "terminal_phase": self._terminal_phase,
            "resources": copy.deepcopy(self._resources),
        }


class VerifiedAttemptEvidence:
    """Opaque evidence binding constructible only by the validating binder."""

    def __init__(self, identity, resources, readiness, teardown, token):
        _require(
            token is _VALIDATED_BINDING_TOKEN,
            "verified evidence cannot be constructed directly",
        )
        _require(
            isinstance(teardown, VerifiedTeardownRestore),
            "verified teardown token is required",
        )
        _require(
            teardown.identity == _canonical_identity(identity),
            "verified teardown identity does not match evidence",
        )
        self._token = token
        self._identity = copy.deepcopy(identity)
        self._resources = copy.deepcopy(resources)
        self._readiness = copy.deepcopy(readiness)
        self._teardown = teardown

    @property
    def identity(self):
        return copy.deepcopy(self._identity)

    @property
    def startup_profile_sha256(self):
        return self._teardown.as_document()["startup_profile_sha256"]

    @property
    def post_episode_scene_verification(self):
        return self._teardown.as_document()[
            "post_episode_scene_verification"
        ]

    @property
    def verified_teardown(self):
        return self._teardown

    def as_document(self):
        return {
            "identity": copy.deepcopy(self._identity),
            "raw_evidence_bound": True,
            "readiness_direct_counts": copy.deepcopy(self._readiness),
            "teardown_restore": self._teardown.as_document(),
            "resources": copy.deepcopy(self._resources),
        }


class AtomicAttemptJournal:
    """Outer lifecycle context covering start, work, evidence, and teardown."""

    def __init__(self, journal_root, identity):
        self.identity = _canonical_identity(identity)
        self.journal_root = Path(journal_root)
        self.path = canonical_attempt_state_path(
            self.journal_root, self.identity
        )
        self.document = None
        self.completed = False
        self.terminal_evidence = None
        self.startup_profile_lease = None
        self.materialized_scene = None
        self.pre_spawn_scene_verification = None
        self.post_episode_scene_verification = None
        self.launch_stop_authorization = None
        self.lifecycle_phase = None
        self._lock_descriptor = None

    def __enter__(self):
        self._lock_descriptor = _acquire_journal_lock(self.path)
        try:
            if self.path.exists():
                sealed = _seal_orphaned_attempt_locked(self.path)
                raise R6IntegrityError(
                    "attempt state already exists with status {}; resume forbidden".format(
                        sealed.get("status")
                    )
                )
            self.document = {
                "schema_version": "2.0",
                "stage": self.identity.get("stage"),
                "identity": dict(self.identity),
                "status": "attempt_started",
                "lifecycle_phase": "attempt_started",
                "resume_forbidden": True,
                "active_identity": dict(self.identity),
                "downstream_authorized": False,
                "evidence_binding": None,
            }
            self.lifecycle_phase = "attempt_started"
            _exclusive_yaml_create(self.path, self.document)
            return self
        except BaseException:
            _release_journal_lock(self._lock_descriptor)
            self._lock_descriptor = None
            raise

    def capture_startup_profile(self, payload):
        _require(
            self.lifecycle_phase == "attempt_started"
            and self.terminal_evidence is None,
            "startup profile must be captured before execution",
        )
        _require(
            isinstance(payload, bytes) and payload,
            "startup profile bytes are required",
        )
        self.startup_profile_lease = StartupProfileLease(
            self.identity, self.path, payload, _STARTUP_PROFILE_TOKEN
        )
        self.lifecycle_phase = "startup_profile_captured"
        self.document.update({
            "lifecycle_phase": self.lifecycle_phase,
            "startup_profile_sha256": self.startup_profile_lease.sha256,
        })
        _atomic_yaml(self.path, self.document)
        return self.startup_profile_lease

    def bind_scene_snapshot(self, snapshot):
        _require(
            self.lifecycle_phase == "startup_profile_captured"
            and self.terminal_evidence is None,
            "scene snapshot must be bound after startup capture",
        )
        _require(
            isinstance(snapshot, MaterializedSceneSnapshot)
            and snapshot.scene_id == self.identity["scene_id"],
            "materialized scene identity does not match attempt",
        )
        verification = revalidate_scene_snapshot(snapshot, "pre_spawn")
        self.materialized_scene = snapshot
        self.pre_spawn_scene_verification = verification
        self.lifecycle_phase = "pre_spawn_scene_verified"
        self.document.update({
            "lifecycle_phase": self.lifecycle_phase,
            "scene_snapshot": snapshot.as_document(),
            "pre_spawn_scene_verification": verification.as_document(),
        })
        _atomic_yaml(self.path, self.document)
        return verification

    def mark_execution_started(self):
        _require(
            self.lifecycle_phase == "pre_spawn_scene_verified"
            and self.startup_profile_lease is not None
            and self.pre_spawn_scene_verification is not None,
            "execution start requires startup and pre-spawn scene bindings",
        )
        _require(
            self.terminal_evidence is None,
            "execution start requires startup and pre-spawn scene bindings",
        )
        self.lifecycle_phase = "execution_started"
        self.document["lifecycle_phase"] = self.lifecycle_phase
        _atomic_yaml(self.path, self.document)

    def verify_post_episode_scene(self):
        _require(
            self.lifecycle_phase == "execution_started"
            and self.materialized_scene is not None
            and self.terminal_evidence is None,
            "post-episode verification requires an execution-started phase",
        )
        verification = revalidate_scene_snapshot(
            self.materialized_scene, "post_episode"
        )
        self.post_episode_scene_verification = verification
        self.lifecycle_phase = "post_episode_scene_verified"
        self.document.update({
            "lifecycle_phase": self.lifecycle_phase,
            "post_episode_scene_verification": verification.as_document(),
        })
        _atomic_yaml(self.path, self.document)
        return verification

    def complete(self, evidence_binding):
        _require(self.document is not None, "attempt journal is not active")
        _require(not self.completed, "attempt journal is already complete")
        _require(
            self.terminal_evidence is None,
            "successful attempt cannot carry terminal failure evidence",
        )
        _require(
            self.lifecycle_phase == "post_episode_scene_verified",
            "attempt cannot complete before post-episode scene verification",
        )
        _require(
            isinstance(evidence_binding, VerifiedAttemptEvidence),
            "validated evidence binding is required before completion",
        )
        _require(
            evidence_binding.identity == self.identity,
            "evidence binding identity does not match attempt journal",
        )
        _require(
            evidence_binding.startup_profile_sha256
            == self.startup_profile_lease.sha256,
            "evidence startup profile does not match journal capture",
        )
        _require(
            evidence_binding.post_episode_scene_verification
            == self.post_episode_scene_verification.as_document(),
            "evidence scene verification does not match attempt journal",
        )
        _require(
            self.launch_stop_authorization is not None
            and self.launch_stop_authorization["teardown_restore"]
            == evidence_binding.verified_teardown.as_document(),
            "attempt cannot complete before identity-bound launch-stop authorization",
        )
        evidence_document = evidence_binding.as_document()
        _require(
            evidence_document["teardown_restore"].get(
                "two_phase_restore_verified"
            )
            is True,
            "verified teardown is required before completion",
        )
        self.document.update({
            "status": "evidence_complete",
            "lifecycle_phase": "evidence_complete",
            "active_identity": None,
            "evidence_binding": evidence_document,
        })
        _atomic_yaml(self.path, self.document)
        self.completed = True
        self.lifecycle_phase = "evidence_complete"

    def authorize_launch_stop(self, verified_teardown):
        """Authorize stop only for this active post-episode attempt."""

        authorization = authorize_launch_stop(
            verified_teardown, self.identity, self
        )
        self.launch_stop_authorization = copy.deepcopy(authorization)
        self.document["launch_stop_authorization"] = copy.deepcopy(
            authorization
        )
        _atomic_yaml(self.path, self.document)
        return copy.deepcopy(authorization)

    def attach_terminal_evidence(self, evidence_binding):
        _require(self.document is not None, "attempt journal is not active")
        _require(
            isinstance(evidence_binding, VerifiedTerminalEvidence),
            "validated terminal evidence binding is required",
        )
        _require(
            evidence_binding.identity == self.identity,
            "terminal evidence identity does not match attempt journal",
        )
        _require(
            self.terminal_evidence is None,
            "terminal evidence is already attached",
        )
        _require(
            evidence_binding.terminal_phase == self.lifecycle_phase,
            "terminal evidence phase does not match attempt lifecycle",
        )
        if self.lifecycle_phase == "post_episode_scene_verified":
            _require(
                evidence_binding.produced_labels == RAW_EVIDENCE_LABELS,
                "post-episode terminal evidence must bind all raw resources",
            )
        self.terminal_evidence = evidence_binding

    def _terminal_evidence_document(self):
        if self.terminal_evidence is None:
            return {
                "terminal_raw_evidence_declared": False,
                "protocol_violation": (
                    "terminal evidence bundle was not attached"
                ),
            }
        if (
            self.terminal_evidence.terminal_phase != self.lifecycle_phase
            or (
                self.lifecycle_phase == "post_episode_scene_verified"
                and self.terminal_evidence.produced_labels
                != RAW_EVIDENCE_LABELS
            )
        ):
            return {
                "terminal_raw_evidence_declared": False,
                "protocol_violation": (
                    "terminal evidence no longer matches lifecycle phase"
                ),
            }
        return self.terminal_evidence.as_document()

    def __exit__(self, exception_type, exception, traceback):
        try:
            if exception_type is not None:
                interrupted = issubclass(
                    exception_type, (KeyboardInterrupt, SystemExit)
                )
                teardown_failure = issubclass(
                    exception_type, R6TeardownFailure
                )
                status = "terminal_failure"
                if interrupted:
                    status = "terminal_interrupted"
                elif teardown_failure:
                    status = "terminal_teardown_failure"
                self.document.update({
                    "status": status,
                    "lifecycle_phase": status,
                    "active_identity": None,
                    "failure_type": exception_type.__name__,
                    "resume_forbidden": True,
                    "downstream_authorized": False,
                    "evidence_binding": self._terminal_evidence_document(),
                })
                _atomic_yaml(self.path, self.document)
                self.lifecycle_phase = status
                return False
            if not self.completed:
                self.document.update({
                    "status": "terminal_incomplete",
                    "lifecycle_phase": "terminal_incomplete",
                    "active_identity": None,
                    "resume_forbidden": True,
                    "downstream_authorized": False,
                    "evidence_binding": self._terminal_evidence_document(),
                })
                _atomic_yaml(self.path, self.document)
                self.lifecycle_phase = "terminal_incomplete"
            return False
        finally:
            _release_journal_lock(self._lock_descriptor)
            self._lock_descriptor = None


def bind_attempt_raw_evidence(
    workspace,
    artifact_root,
    expected_identity,
    resources,
    minimum_message_count,
    startup_profile_lease,
    post_episode_scene_verification,
):
    """Validate and bind every raw attempt file before journal completion."""

    root = Path(workspace).resolve()
    identity = _canonical_identity(expected_identity)
    _require(
        isinstance(startup_profile_lease, StartupProfileLease)
        and startup_profile_lease.identity == identity,
        "journal-captured startup profile lease is required",
    )
    _require(
        isinstance(post_episode_scene_verification, VerifiedSceneSnapshot)
        and post_episode_scene_verification.phase == "post_episode"
        and post_episode_scene_verification.scene_id == identity["scene_id"],
        "post-episode scene verification token is required",
    )
    raw_evidence_root = Path(artifact_root)
    if not raw_evidence_root.is_absolute():
        raw_evidence_root = root / raw_evidence_root
    _require(
        not raw_evidence_root.is_symlink(),
        "artifact root is a symlink",
    )
    evidence_root = _inside(root, raw_evidence_root, "artifact root")
    _require(evidence_root.is_dir(), "artifact root is not a directory")
    _require(
        set(resources) == RAW_EVIDENCE_LABELS,
        "raw evidence resource set drifted",
    )
    bound = {}
    resolved = {}
    declared_paths = set()
    resolved_paths = set()
    for label in sorted(resources):
        row = resources[label]
        _require(
            isinstance(row, dict) and set(row) == {"path", "sha256"},
            "{} evidence declaration drifted".format(label),
        )
        _require(
            row["path"] not in declared_paths,
            "raw evidence resources alias one declared path",
        )
        declared_paths.add(row["path"])
        path = _declared_workspace_file(root, row["path"], label)
        _inside(evidence_root, path, label)
        _require(
            str(path) not in resolved_paths,
            "raw evidence resources alias one resolved file",
        )
        resolved_paths.add(str(path))
        payload = _read_regular_file_once(path, label)
        digest = sha256_bytes(payload)
        _require(digest == row["sha256"], "{} evidence hash drifted".format(label))
        bound[label] = {"path": row["path"], "sha256": digest}
        resolved[label] = (path, payload)
    inventory = list(evidence_root.rglob("*"))
    _require(
        not any(path.is_symlink() for path in inventory),
        "raw artifact directory contains a symlink",
    )
    actual_files = {
        str(path.resolve())
        for path in inventory
        if path.is_file()
    }
    _require(
        actual_files == resolved_paths,
        "raw artifact directory contains undeclared or omitted files",
    )
    activation = _strict_yaml_bytes(resolved["activation"][1], "activation")
    evaluation = _strict_yaml_bytes(resolved["evaluation"][1], "evaluation")
    clearance = _strict_yaml_bytes(resolved["clearance"][1], "clearance")
    teardown_receipt = _strict_yaml_bytes(
        resolved["teardown_receipt"][1], "teardown receipt"
    )
    for document_name, document in (
        ("activation", activation),
        ("evaluation", evaluation),
        ("clearance", clearance),
        ("teardown_receipt", teardown_receipt),
    ):
        for key in IDENTITY_FIELDS:
            _require(
                type(document.get(key)) is type(identity[key])
                and document.get(key) == identity[key],
                "{} identity mismatch: {}".format(document_name, key),
            )
    readiness = validate_readiness_raw_evidence(
        identity,
        activation,
        evaluation,
        minimum_message_count,
    )
    _require(
        evaluation.get("raw_trace_sha256") == bound["trace"]["sha256"],
        "evaluation does not bind the raw trace",
    )
    teardown = verify_teardown_restore(
        teardown_receipt,
        startup_profile_lease,
        post_episode_scene_verification,
        identity,
    )
    return VerifiedAttemptEvidence(
        identity,
        bound,
        readiness,
        teardown,
        _VALIDATED_BINDING_TOKEN,
    )


def bind_terminal_attempt_evidence(
    workspace,
    artifact_root,
    expected_identity,
    resources,
):
    """Bind explicit produced/not-produced evidence for any terminal failure."""

    root = Path(workspace).resolve()
    identity = _canonical_identity(expected_identity)
    raw_evidence_root = Path(artifact_root)
    if not raw_evidence_root.is_absolute():
        raw_evidence_root = root / raw_evidence_root
    _require(
        not raw_evidence_root.is_symlink(),
        "artifact root is a symlink",
    )
    evidence_root = _inside(root, raw_evidence_root, "artifact root")
    _require(evidence_root.is_dir(), "artifact root is not a directory")
    _require(
        set(resources) == RAW_EVIDENCE_LABELS,
        "terminal evidence resource set drifted",
    )
    bound = {}
    produced_paths = set()
    produced_payloads = {}
    not_produced_phases = set()
    for label in sorted(resources):
        row = resources[label]
        _require(isinstance(row, dict), "{} declaration is invalid".format(label))
        status = row.get("status")
        if status == "not_produced":
            _require(
                set(row) == {"status", "phase", "reason"}
                and isinstance(row["phase"], str)
                and row["phase"] in TERMINAL_EVIDENCE_PHASES
                and isinstance(row["reason"], str)
                and row["reason"],
                "{} not-produced declaration drifted".format(label),
            )
            not_produced_phases.add(row["phase"])
            bound[label] = dict(row)
            continue
        _require(
            status == "produced"
            and set(row) == {"status", "path", "sha256"},
            "{} produced declaration drifted".format(label),
        )
        path = _declared_workspace_file(root, row["path"], label)
        _inside(evidence_root, path, label)
        _require(
            str(path) not in produced_paths,
            "terminal evidence resources alias one file",
        )
        produced_paths.add(str(path))
        payload = _read_regular_file_once(path, label)
        digest = sha256_bytes(payload)
        _require(digest == row["sha256"], "{} evidence hash drifted".format(label))
        bound[label] = {
            "status": "produced",
            "path": row["path"],
            "sha256": digest,
        }
        produced_payloads[label] = payload
    inventory = list(evidence_root.rglob("*"))
    _require(
        not any(path.is_symlink() for path in inventory),
        "terminal artifact directory contains a symlink",
    )
    actual_files = {
        str(path.resolve())
        for path in inventory
        if path.is_file()
    }
    _require(
        actual_files == produced_paths,
        "terminal artifact directory contains undeclared or omitted files",
    )
    for label in ("activation", "evaluation", "clearance", "teardown_receipt"):
        if label not in produced_payloads:
            continue
        document = _strict_yaml_bytes(produced_payloads[label], label)
        for key in IDENTITY_FIELDS:
            _require(
                type(document.get(key)) is type(identity[key])
                and document.get(key) == identity[key],
                "{} identity mismatch: {}".format(label, key),
            )
    if "evaluation" in produced_payloads:
        _require(
            "trace" in produced_payloads,
            "produced evaluation requires produced trace",
        )
        evaluation = _strict_yaml_bytes(
            produced_payloads["evaluation"], "evaluation"
        )
        _require(
            evaluation.get("raw_trace_sha256")
            == bound["trace"]["sha256"],
            "terminal evaluation does not bind raw trace",
        )
    _require(
        len(not_produced_phases) <= 1,
        "terminal not-produced resources declare conflicting lifecycle phases",
    )
    terminal_phase = (
        next(iter(not_produced_phases))
        if not_produced_phases
        else "post_episode_scene_verified"
    )
    if terminal_phase == "post_episode_scene_verified":
        _require(
            set(produced_payloads) == RAW_EVIDENCE_LABELS,
            "post-episode terminal evidence must bind all raw resources",
        )
    return VerifiedTerminalEvidence(
        identity,
        bound,
        terminal_phase,
        _VALIDATED_TERMINAL_BINDING_TOKEN,
    )


def verify_dependency_closure(workspace, manifest, required_paths):
    """Verify an explicit deterministic dependency graph and all file hashes."""

    root = Path(workspace).resolve()
    _require(manifest.get("unresolved") == [], "dependency closure is unresolved")
    files = manifest.get("files")
    edges = manifest.get("edges")
    entrypoints = manifest.get("entrypoints")
    _require(
        isinstance(files, list)
        and isinstance(edges, list)
        and isinstance(entrypoints, list)
        and entrypoints,
        "dependency closure schema drifted",
    )
    records = {}
    _require(
        len(required_paths) == len(set(required_paths)),
        "required dependency paths contain duplicates",
    )
    for row in files:
        _require(
            isinstance(row, dict) and set(row) == {"path", "sha256"},
            "dependency file declaration drifted",
        )
        path = _declared_workspace_file(root, row["path"], "dependency")
        _require(row["path"] not in records, "duplicate dependency path")
        digest = sha256_bytes(_read_regular_file_once(path, "dependency"))
        _require(digest == row["sha256"], "dependency hash drifted")
        records[row["path"]] = digest
    _require(set(records) == set(required_paths), "dependency path set drifted")
    _require(
        all(isinstance(value, str) for value in entrypoints)
        and len(entrypoints) == len(set(entrypoints))
        and set(entrypoints).issubset(records),
        "dependency entrypoint set is invalid",
    )
    allowed_edge_kinds = {
        "candidate_specification",
        "design_reference",
        "frozen_input",
        "future_protocol_input",
        "integrity_protocol",
        "python_import",
    }
    canonical_edges = []
    edge_keys = set()
    adjacency = {path: set() for path in records}
    for edge in edges:
        _require(
            isinstance(edge, dict) and set(edge) == {"from", "to", "kind"},
            "dependency edge declaration drifted",
        )
        _require(
            edge["from"] in records and edge["to"] in records,
            "dependency edge leaves closure",
        )
        _require(
            edge["kind"] in allowed_edge_kinds,
            "dependency edge kind is not allowed",
        )
        key = (edge["from"], edge["to"], edge["kind"])
        _require(key not in edge_keys, "duplicate dependency edge")
        edge_keys.add(key)
        adjacency[edge["from"]].add(edge["to"])
        canonical_edges.append("{}\0{}\0{}".format(*key))
    reachable = set(entrypoints)
    pending = list(entrypoints)
    while pending:
        source = pending.pop()
        for target in adjacency[source]:
            if target not in reachable:
                reachable.add(target)
                pending.append(target)
    _require(
        reachable == set(records),
        "dependency closure contains unreachable files",
    )
    payload = "".join(
        "{} {}\n".format(path, records[path]) for path in sorted(records)
    )
    payload += "".join(
        "entrypoint {}\n".format(path) for path in sorted(entrypoints)
    )
    payload += "".join("{}\n".format(edge) for edge in sorted(canonical_edges))
    return {
        "file_count": len(records),
        "edge_count": len(edges),
        "entrypoints": list(entrypoints),
        "unresolved": [],
        "closure_sha256": hashlib.sha256(payload.encode("utf-8")).hexdigest(),
        "all_hashes_match": True,
    }


def verify_teardown_restore(
    receipt,
    startup_profile_lease,
    post_episode_scene_verification,
    expected_identity,
):
    """Require explicit restore transaction and independent final readback."""

    def teardown_require(condition, message):
        if not condition:
            raise R6TeardownFailure(message)

    teardown_require(isinstance(receipt, dict), "teardown receipt must be a mapping")
    try:
        identity = _canonical_identity(expected_identity)
    except R6IntegrityError as exc:
        raise R6TeardownFailure(str(exc)) from exc
    teardown_require(
        isinstance(startup_profile_lease, StartupProfileLease)
        and startup_profile_lease._token is _STARTUP_PROFILE_TOKEN
        and startup_profile_lease.identity == identity,
        "journal-captured startup profile lease is required",
    )
    teardown_require(
        isinstance(post_episode_scene_verification, VerifiedSceneSnapshot)
        and post_episode_scene_verification.phase == "post_episode"
        and post_episode_scene_verification.scene_id == identity["scene_id"],
        "post-episode scene verification token is required",
    )
    for key in IDENTITY_FIELDS:
        teardown_require(
            type(receipt.get(key)) is type(identity[key])
            and receipt.get(key) == identity[key],
            "teardown receipt identity mismatch: {}".format(key),
        )
    required_true = (
        "restore_requested_while_backend_alive",
        "transaction_acknowledged",
        "transaction_readback_match",
        "independent_readback_match",
    )
    for key in required_true:
        teardown_require(
            receipt.get(key) is True,
            "teardown receipt failed: {}".format(key),
        )
    expected = startup_profile_lease.sha256
    teardown_require(
        receipt.get("startup_profile_sha256") == expected
        and receipt.get("transaction_readback_sha256") == expected
        and receipt.get("independent_readback_sha256") == expected,
        "teardown profile hash/readback mismatch",
    )
    document = {
        "identity": copy.deepcopy(identity),
        "status": "pass",
        "journal_state_path": startup_profile_lease.journal_state_path,
        "startup_profile_sha256": expected,
        "two_phase_restore_verified": True,
        "launch_stop_allowed": True,
        "post_episode_scene_verification": (
            post_episode_scene_verification.as_document()
        ),
    }
    return VerifiedTeardownRestore(
        identity, document, _VERIFIED_TEARDOWN_TOKEN
    )


def authorize_launch_stop(
    verified_teardown, expected_identity, active_journal
):
    """Authorize stop only inside the matching active attempt journal."""

    identity = _canonical_identity(expected_identity)
    _require(
        isinstance(verified_teardown, VerifiedTeardownRestore)
        and verified_teardown._token is _VERIFIED_TEARDOWN_TOKEN,
        "launch stop requires verified teardown",
    )
    _require(
        isinstance(active_journal, AtomicAttemptJournal)
        and active_journal.document is not None
        and not active_journal.completed
        and active_journal._lock_descriptor is not None
        and active_journal.lifecycle_phase == "post_episode_scene_verified",
        "launch stop requires an active post-episode attempt journal",
    )
    _require(
        active_journal.document.get("status") == "attempt_started"
        and active_journal.document.get("lifecycle_phase")
        == "post_episode_scene_verified",
        "launch stop requires persisted active post-episode state",
    )
    _require(
        active_journal.identity == identity
        and verified_teardown.identity == identity,
        "launch stop identity does not match active attempt journal",
    )
    teardown_document = verified_teardown.as_document()
    _require(
        active_journal.startup_profile_lease is not None
        and teardown_document.get("journal_state_path")
        == str(active_journal.path.absolute())
        and teardown_document.get("startup_profile_sha256")
        == active_journal.startup_profile_lease.sha256
        and teardown_document.get("post_episode_scene_verification")
        == active_journal.post_episode_scene_verification.as_document(),
        "launch stop teardown proof does not match active attempt state",
    )
    return {
        "launch_stop_allowed": True,
        "identity": copy.deepcopy(identity),
        "teardown_restore": teardown_document,
    }
