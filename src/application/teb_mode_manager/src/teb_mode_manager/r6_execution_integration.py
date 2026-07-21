"""Fail-closed R6 execution-integration helpers.

This module adds no authorization and starts no process.  It preserves the
frozen R2 idempotent typed-transaction behavior during an episode, then
requires an explicit restore transaction and an independent final readback
while the primary backend is still alive.
"""

from dataclasses import dataclass
import hashlib
import json
import math
import threading
from typing import Any, Callable, Dict, Mapping

from .idempotent_typed_teb_transaction import (
    IdempotentTypedTebTransactionBackend,
)
from .typed_teb_transaction import (
    TypedTebTransactionError,
    _extract_values,
)


IDENTITY_FIELDS = ("stage", "profile_id", "scene_id", "seed", "attempt")


class R6ExecutionIntegrationError(TypedTebTransactionError):
    """Raised when the R6 integration boundary cannot be proven."""

    code = "r6_execution_integration_fault"


class R6TwoPhaseRestoreError(R6ExecutionIntegrationError):
    """Machine-readable failure of the explicit teardown transaction."""

    code = "r6_two_phase_restore_failed"

    def __init__(self, message: str, receipt: Mapping[str, Any]):
        super().__init__(message)
        self.receipt = dict(receipt)


def canonical_attempt_identity(value: Mapping[str, Any]) -> Dict[str, Any]:
    """Validate the exact five-field execution identity without coercion."""

    if not isinstance(value, Mapping) or set(value) != set(IDENTITY_FIELDS):
        raise R6ExecutionIntegrationError(
            "attempt identity must contain exactly {}".format(
                ",".join(IDENTITY_FIELDS)
            )
        )
    for field in ("stage", "profile_id", "scene_id"):
        if not isinstance(value[field], str) or not value[field]:
            raise R6ExecutionIntegrationError(
                "attempt identity {} must be a non-empty string".format(field)
            )
    if type(value["seed"]) is not int or value["seed"] < 0:
        raise R6ExecutionIntegrationError(
            "attempt identity seed must be a non-negative integer"
        )
    if type(value["attempt"]) is not int or value["attempt"] <= 0:
        raise R6ExecutionIntegrationError(
            "attempt identity attempt must be a positive integer"
        )
    return {field: value[field] for field in IDENTITY_FIELDS}


def canonical_profile_bytes(bank: Any, values: Mapping[str, Any]) -> bytes:
    """Serialize one complete typed profile for journal/profile hash binding."""

    typed = bank.validate_values(values, "R6 canonical typed profile")
    parameters = []
    for name in bank.parameter_names:
        parameter_type = bank.definitions[name].parameter_type
        value = typed[name]
        if parameter_type == "double":
            value = float(value)
            if not math.isfinite(value):
                raise R6ExecutionIntegrationError(
                    "{} must be finite".format(name)
                )
        parameters.append({
            "name": name,
            "type": parameter_type,
            "value": value,
        })
    document = {
        "schema_version": "2.0",
        "profile_kind": "teb_startup_profile",
        "parameters": parameters,
    }
    return json.dumps(
        document, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


@dataclass(frozen=True)
class StartupProfileCapture:
    """Exact startup bytes published for attempt-journal capture."""

    payload: bytes
    sha256: str

    def as_document(self, identity: Mapping[str, Any]) -> Dict[str, Any]:
        return {
            "schema_version": "2.0",
            "record_type": "r6_startup_profile_capture",
            "identity": canonical_attempt_identity(identity),
            "startup_profile_sha256": self.sha256,
            "startup_profile_canonical_json": self.payload.decode("utf-8"),
        }


class R6TwoPhaseIdempotentTypedTebTransactionBackend(
    IdempotentTypedTebTransactionBackend
):
    """Frozen R2 backend plus an explicit, independently verified teardown."""

    def __init__(self, *args: Any, **kwargs: Any):
        super().__init__(*args, **kwargs)
        self._r6_lock = threading.RLock()
        self._startup_capture = None
        self._teardown_attempted = False
        self._teardown_verified = False
        self._teardown_receipt = None

    @property
    def startup_capture(self) -> StartupProfileCapture:
        if self._startup_capture is None:
            raise R6ExecutionIntegrationError(
                "startup profile has not been captured"
            )
        return self._startup_capture

    @property
    def teardown_verified(self) -> bool:
        return self._teardown_verified

    @property
    def teardown_attempted(self) -> bool:
        return self._teardown_attempted

    @property
    def backend_alive(self) -> bool:
        return not self._closed

    @property
    def teardown_receipt(self) -> Dict[str, Any]:
        if self._teardown_receipt is None:
            raise R6ExecutionIntegrationError(
                "teardown has not produced a receipt"
            )
        return dict(self._teardown_receipt)

    def initialize(self):
        specs = super().initialize()
        payload = canonical_profile_bytes(self.bank, self.startup.values)
        self._startup_capture = StartupProfileCapture(
            payload=payload,
            sha256=sha256_bytes(payload),
        )
        return specs

    def apply(self, requested, now_s):
        with self._r6_lock:
            if self._teardown_attempted:
                raise R6ExecutionIntegrationError(
                    "parameter application is forbidden after teardown begins"
                )
            return super().apply(requested, now_s)

    def _profile_document(self, values: Mapping[str, Any]) -> Dict[str, Any]:
        payload = canonical_profile_bytes(self.bank, values)
        return {
            "sha256": sha256_bytes(payload),
            "canonical_json": payload.decode("utf-8"),
        }

    def restore_startup_two_phase(
        self,
        identity: Mapping[str, Any],
        independent_reader: Callable[[], Mapping[str, Any]],
    ) -> Dict[str, Any]:
        """Restore startup and independently re-read it before launch stop.

        The call is single-shot.  A caller may retrieve the cached receipt
        after the call, but no second restore transaction is allowed.
        """

        canonical_identity = canonical_attempt_identity(identity)
        with self._r6_lock:
            if self._teardown_attempted:
                raise R6ExecutionIntegrationError(
                    "explicit teardown was already attempted"
                )
            if self._closed:
                raise R6ExecutionIntegrationError(
                    "primary typed backend is not alive"
                )
            self._teardown_attempted = True
            startup = self.startup_capture
            receipt = {
                "schema_version": "2.0",
                "record_type": "r6_two_phase_teardown_receipt",
                **canonical_identity,
                "identity": dict(canonical_identity),
                "status": "fail",
                "failure_reason": "",
                "restore_requested_while_backend_alive": True,
                "transaction_acknowledged": False,
                "transaction_readback_match": False,
                "independent_readback_match": False,
                "backend_alive_after_restore": False,
                "startup_profile_sha256": startup.sha256,
                "startup_profile_canonical_json": (
                    startup.payload.decode("utf-8")
                ),
                "transaction_ack_sha256": None,
                "transaction_ack_canonical_json": None,
                "transaction_readback_sha256": None,
                "transaction_readback_canonical_json": None,
                "independent_readback_sha256": None,
                "independent_readback_canonical_json": None,
                "restore_t_request_s": None,
                "restore_t_ack_s": None,
                "restore_t_readback_s": None,
                "restore_t_active_s": None,
                "independent_readback_t_s": None,
            }
            try:
                transaction = self.restore_startup(
                    "r6_explicit_restore_startup"
                )
                acknowledgement = self._profile_document(
                    transaction.acknowledgement.values
                )
                readback = self._profile_document(
                    transaction.readback.values
                )
                receipt.update({
                    "transaction_ack_sha256": acknowledgement["sha256"],
                    "transaction_ack_canonical_json": (
                        acknowledgement["canonical_json"]
                    ),
                    "transaction_readback_sha256": readback["sha256"],
                    "transaction_readback_canonical_json": (
                        readback["canonical_json"]
                    ),
                    "transaction_acknowledged": (
                        acknowledgement["sha256"] == startup.sha256
                    ),
                    "transaction_readback_match": (
                        readback["sha256"] == startup.sha256
                    ),
                    "restore_t_request_s": transaction.t_request_s,
                    "restore_t_ack_s": transaction.t_ack_s,
                    "restore_t_readback_s": transaction.t_readback_s,
                    "restore_t_active_s": transaction.t_active_s,
                })
                independent_raw = independent_reader()
                independent_values = _extract_values(
                    independent_raw, self.bank, "independent final readback"
                )
                independent = self._profile_document(independent_values)
                receipt.update({
                    "independent_readback_sha256": independent["sha256"],
                    "independent_readback_canonical_json": (
                        independent["canonical_json"]
                    ),
                    "independent_readback_match": (
                        independent["sha256"] == startup.sha256
                    ),
                    "independent_readback_t_s": max(
                        float(transaction.t_active_s),
                        float(self._time_source()),
                    ),
                    "backend_alive_after_restore": not self._closed,
                })
                required = (
                    receipt["transaction_acknowledged"],
                    receipt["transaction_readback_match"],
                    receipt["independent_readback_match"],
                    receipt["backend_alive_after_restore"],
                )
                if not all(required):
                    raise R6ExecutionIntegrationError(
                        "startup profile restore/readback evidence mismatched"
                    )
                receipt["status"] = "pass"
                self._teardown_verified = True
            except Exception as exc:
                receipt["failure_reason"] = "{}: {}".format(
                    type(exc).__name__, exc
                )
                self._teardown_receipt = dict(receipt)
                raise R6TwoPhaseRestoreError(
                    "R6 explicit teardown failed: {}".format(exc), receipt
                ) from exc
            self._teardown_receipt = dict(receipt)
            return dict(receipt)

    def close(self) -> None:
        """Close without a second write only after explicit proof succeeded."""

        with self._r6_lock:
            if self._closed:
                return
            if not self._teardown_verified:
                # A non-verified shutdown remains a terminal execution failure.
                # Preserve the frozen backend's best-effort safety restoration,
                # but never turn it into a passing two-phase receipt.
                return super().close()
            self._closed = True
            if hasattr(self._adapter, "close"):
                self._adapter.close()
