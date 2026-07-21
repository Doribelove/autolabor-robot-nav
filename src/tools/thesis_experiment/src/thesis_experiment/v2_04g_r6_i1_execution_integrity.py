"""Persisted R6-I1 integrity checks used by the offline assessor."""

import hashlib
import json
from pathlib import Path
from typing import Mapping

from . import v2_04g_r6_integrity as design_integrity


class R6I1PersistedIntegrityError(ValueError):
    """Raised when persisted execution evidence cannot be replayed."""


def _require(condition, message):
    if not condition:
        raise R6I1PersistedIntegrityError(message)


def _identity_equal(actual, expected):
    return (
        isinstance(actual, Mapping)
        and set(actual) == set(design_integrity.IDENTITY_FIELDS)
        and all(
            type(actual[key]) is type(expected[key])
            and actual[key] == expected[key]
            for key in design_integrity.IDENTITY_FIELDS
        )
    )


def _inside(root, path, label):
    boundary = Path(root).resolve()
    resolved = Path(path).resolve()
    try:
        resolved.relative_to(boundary)
    except ValueError as exc:
        raise R6I1PersistedIntegrityError(
            "{} leaves workspace".format(label)
        ) from exc
    return resolved


def _resource_file(workspace, raw_root, row, label):
    _require(
        isinstance(row, dict) and set(row) == {"path", "sha256"},
        "{} resource schema drifted".format(label),
    )
    declared = Path(row["path"])
    _require(
        not declared.is_absolute() and ".." not in declared.parts,
        "{} resource path is unsafe".format(label),
    )
    path = _inside(workspace, Path(workspace) / declared, label)
    _inside(raw_root, path, label)
    _require(
        path.is_file() and not path.is_symlink(),
        "{} resource is missing or unsafe".format(label),
    )
    digest = design_integrity.sha256_file(path)
    _require(digest == row["sha256"], "{} resource hash drifted".format(label))
    return path, digest


def _canonical_profile(text, expected_sha256, label):
    _require(isinstance(text, str) and text, "{} profile is missing".format(label))
    try:
        document = json.loads(text)
    except (TypeError, ValueError) as exc:
        raise R6I1PersistedIntegrityError(
            "{} profile JSON is invalid".format(label)
        ) from exc
    canonical = json.dumps(
        document, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    )
    _require(canonical == text, "{} profile JSON is not canonical".format(label))
    digest = hashlib.sha256(text.encode("utf-8")).hexdigest()
    _require(digest == expected_sha256, "{} profile hash drifted".format(label))
    return document


def _validate_teardown(receipt, journal, identity):
    _require(_identity_equal(
        {key: receipt.get(key) for key in design_integrity.IDENTITY_FIELDS},
        identity,
    ), "teardown top-level identity mismatch")
    _require(
        _identity_equal(receipt.get("identity"), identity),
        "teardown nested identity mismatch",
    )
    for key in (
        "restore_requested_while_backend_alive",
        "transaction_acknowledged",
        "transaction_readback_match",
        "independent_readback_match",
        "backend_alive_after_restore",
    ):
        _require(receipt.get(key) is True, "teardown gate failed: " + key)
    expected = journal.get("startup_profile_sha256")
    _require(
        isinstance(expected, str) and len(expected) == 64,
        "journal startup profile hash is missing",
    )
    fields = (
        ("startup_profile", "startup_profile_sha256",
         "startup_profile_canonical_json"),
        ("transaction_ack", "transaction_ack_sha256",
         "transaction_ack_canonical_json"),
        ("transaction_readback", "transaction_readback_sha256",
         "transaction_readback_canonical_json"),
        ("independent_readback", "independent_readback_sha256",
         "independent_readback_canonical_json"),
    )
    for label, hash_key, json_key in fields:
        _require(receipt.get(hash_key) == expected, label + " hash mismatch")
        _canonical_profile(receipt.get(json_key), expected, label)
    _require(
        receipt.get("status") == "pass"
        and receipt.get("service_response_success") is True,
        "teardown receipt is not a passing service result",
    )
    return expected


def _validate_auxiliary(workspace, ledger, identity, startup_sha256):
    required = {
        "initial_readback",
        "transaction_startup",
        "arm_receipt",
    }
    _require(
        required.issubset(ledger),
        "persisted startup/arm provenance is incomplete",
    )
    documents = {}
    for label in sorted(required):
        row = ledger[label]
        path, _ = _resource_file(
            workspace,
            Path(workspace).resolve(),
            row,
            label,
        )
        document = design_integrity.strict_yaml(path)
        _require(
            _identity_equal(document.get("identity"), identity),
            "{} identity mismatch".format(label),
        )
        documents[label] = document
    initial = documents["initial_readback"]
    transaction = documents["transaction_startup"]
    arm = documents["arm_receipt"]
    _require(
        initial.get("startup_profile_sha256") == startup_sha256
        and transaction.get("startup_profile_sha256") == startup_sha256
        and arm.get("startup_profile_sha256") == startup_sha256,
        "startup/transaction/arm profile hashes disagree",
    )
    _canonical_profile(
        initial.get("startup_profile_canonical_json"),
        startup_sha256,
        "initial independent readback",
    )
    _canonical_profile(
        transaction.get("startup_profile_canonical_json"),
        startup_sha256,
        "transaction startup capture",
    )
    _require(
        arm.get("execution_armed") is True
        and arm.get("service_response_success") is True,
        "transaction arm receipt did not pass",
    )
    return True


def _validate_scene_snapshot(journal, identity):
    snapshot = journal.get("scene_snapshot")
    _require(
        isinstance(snapshot, dict)
        and snapshot.get("scene_id") == identity["scene_id"],
        "journal scene snapshot identity drifted",
    )
    for phase in ("pre_spawn_scene_verification",
                  "post_episode_scene_verification"):
        verification = journal.get(phase)
        _require(
            isinstance(verification, dict)
            and verification.get("scene_id") == identity["scene_id"]
            and verification.get("verification_phase")
            == ("pre_spawn" if phase.startswith("pre_") else "post_episode"),
            "{} is missing or drifted".format(phase),
        )
        for row in verification.get("resources", {}).values():
            path = Path(row["path"])
            _require(
                path.is_file()
                and not path.is_symlink()
                and design_integrity.sha256_file(path) == row["sha256"],
                "{} resource drifted".format(phase),
            )
    return True


def validate_persisted_attempt(
    workspace,
    ledger,
    minimum_message_count,
):
    """Replay one canonical journal and its exact raw evidence inventory."""

    root = Path(workspace).resolve()
    identity = ledger.get("identity")
    identity = design_integrity._canonical_identity(identity)
    expected_journal = design_integrity.canonical_attempt_state_path(
        root / ledger["journal_root"], identity
    ).resolve()
    journal_path = (root / ledger["journal"]).resolve()
    _require(
        journal_path == expected_journal,
        "ledger journal path is not canonical",
    )
    journal = design_integrity.strict_yaml(journal_path)
    _require(
        _identity_equal(journal.get("identity"), identity),
        "journal identity mismatch",
    )
    status = journal.get("status")
    raw_root = _inside(root, root / ledger["raw_evidence_root"], "raw root")
    _require(
        raw_root.is_dir() and not raw_root.is_symlink(),
        "raw evidence root is missing or unsafe",
    )
    evidence = journal.get("evidence_binding")
    if status == "evidence_complete":
        _require(
            isinstance(evidence, dict)
            and evidence.get("raw_evidence_bound") is True,
            "complete journal lacks bound raw evidence",
        )
        resources = evidence.get("resources")
        _require(
            isinstance(resources, dict)
            and set(resources) == design_integrity.RAW_EVIDENCE_LABELS,
            "complete raw resource set drifted",
        )
        resolved = {}
        for label, row in resources.items():
            resolved[label] = _resource_file(root, raw_root, row, label)[0]
        actual = {
            path.resolve()
            for path in raw_root.rglob("*")
            if path.is_file()
        }
        _require(
            actual == set(resolved.values()),
            "raw evidence inventory is not exact",
        )
        activation = design_integrity.strict_yaml(resolved["activation"])
        evaluation = design_integrity.strict_yaml(resolved["evaluation"])
        clearance = design_integrity.strict_yaml(resolved["clearance"])
        receipt = design_integrity.strict_yaml(
            resolved["teardown_receipt"]
        )
        readiness = design_integrity.validate_readiness_raw_evidence(
            identity, activation, evaluation, minimum_message_count
        )
        for label, document in (
            ("clearance", clearance),
            ("teardown", receipt),
        ):
            _require(
                all(
                    type(document.get(key)) is type(identity[key])
                    and document.get(key) == identity[key]
                    for key in design_integrity.IDENTITY_FIELDS
                ),
                "{} identity mismatch".format(label),
            )
        _require(
            evaluation.get("raw_trace_sha256")
            == resources["trace"]["sha256"],
            "evaluation trace binding drifted",
        )
        startup_sha = _validate_teardown(receipt, journal, identity)
        _validate_auxiliary(root, ledger, identity, startup_sha)
        _validate_scene_snapshot(journal, identity)
        stop = journal.get("launch_stop_authorization")
        _require(
            isinstance(stop, dict)
            and stop.get("launch_stop_allowed") is True
            and _identity_equal(stop.get("identity"), identity),
            "normal launch-stop authorization is missing",
        )
        return {
            "identity": identity,
            "status": status,
            "raw_resource_count": len(resources),
            "readiness_direct_counts": readiness["direct_counts"],
            "startup_profile_sha256": startup_sha,
            "integrity_pass": True,
        }
    _require(
        status in design_integrity.TERMINAL_STATUSES,
        "journal remains non-terminal",
    )
    _require(
        isinstance(evidence, dict)
        and evidence.get("terminal_raw_evidence_declared") is True,
        "terminal journal lacks explicit raw evidence declaration",
    )
    resources = evidence.get("resources")
    _require(
        isinstance(resources, dict)
        and set(resources) == design_integrity.RAW_EVIDENCE_LABELS,
        "terminal raw resource set drifted",
    )
    produced = set()
    for label, row in resources.items():
        if row.get("status") == "produced":
            normalized = {"path": row.get("path"), "sha256": row.get("sha256")}
            produced.add(
                _resource_file(root, raw_root, normalized, label)[0]
            )
        else:
            _require(
                set(row) == {"status", "phase", "reason"}
                and row["status"] == "not_produced"
                and row["phase"] in design_integrity.TERMINAL_EVIDENCE_PHASES
                and isinstance(row["reason"], str)
                and row["reason"],
                "{} terminal omission is not explicit".format(label),
            )
    actual = {
        path.resolve() for path in raw_root.rglob("*") if path.is_file()
    }
    _require(actual == produced, "terminal raw inventory is not exact")
    return {
        "identity": identity,
        "status": status,
        "raw_resource_count": len(produced),
        "integrity_pass": True,
    }
