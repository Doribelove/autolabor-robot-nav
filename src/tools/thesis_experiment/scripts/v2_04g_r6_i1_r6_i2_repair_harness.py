#!/usr/bin/env python3
"""Offline-only integration harness for the independent R6-I2 repair.

The harness deliberately has no command-line entry point and no ROS or process
launcher imports.  It accepts observation callbacks so unit tests and an
independent integration reviewer can exercise the exact bootstrap ordering
without starting anything.  A future execution runner is outside this review
and would still require a separate authorization.
"""

import math
from pathlib import Path, PurePosixPath
from typing import Callable, Dict, Mapping, Optional, Sequence

from thesis_experiment.v2_04g_r6_i1_r6_i2_bootstrap import (
    R6I2BootstrapError,
    R6I2EnvironmentPolicyError,
    R6I2PositiveClockBarrier,
    assert_credential_safe_environment,
    build_credential_safe_environment,
    credential_safe_log_policy_receipt,
    redact_credential_material,
)


STAGE = "V2-04G-R6-I2"
HARNESS_ID = "v2_04g_r6_i2_offline_bootstrap_integration_review_v1"
XACRO_RUNTIME_BINDING = "package-executable:xacro:xacro"
REQUIRED_CALL_ORDER = (
    "base_spawn",
    "unpause_request",
    "unpause_success_ack",
    "post_ack_positive_clock_1",
    "post_ack_positive_clock_2",
    "service_wait_release",
    "move_base_teb_service_wait",
)


class R6I2RepairHarnessError(RuntimeError):
    """A fail-closed I2 review error carrying a machine-readable receipt."""

    def __init__(self, message: str, receipt: Mapping[str, object]) -> None:
        super().__init__(message)
        self.receipt = dict(receipt)


class _FailClosed(RuntimeError):
    """Internal control-flow error converted to a safe public receipt."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise _FailClosed(message)


def _finite_time(value: object, label: str) -> float:
    try:
        numeric = float(value)
    except (OverflowError, TypeError, ValueError):
        raise _FailClosed("{}_must_be_finite".format(label)) from None
    _require(
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(numeric),
        "{}_must_be_finite".format(label),
    )
    return numeric


def _exact_observation(
    value: object,
    required_keys: Sequence[str],
    event: str,
) -> Mapping[str, object]:
    _require(isinstance(value, Mapping), "{}_observation_not_mapping".format(event))
    _require(
        set(value) == set(required_keys),
        "{}_observation_schema_drifted".format(event),
    )
    _require(
        isinstance(value["diagnostic"], str),
        "{}_diagnostic_not_text".format(event),
    )
    return value


def _canonical_xacro_target(
    dependency_closure: Mapping[str, object],
) -> str:
    _require(
        isinstance(dependency_closure, Mapping),
        "dependency_closure_not_mapping",
    )
    _require(
        dependency_closure.get("execution_authorized") is False,
        "dependency_closure_must_be_non_authorizing",
    )
    _require(
        dependency_closure.get("unresolved") == [],
        "dependency_closure_contains_unresolved_items",
    )
    external = dependency_closure.get("external")
    _require(
        isinstance(external, Mapping),
        "dependency_closure_external_section_missing",
    )
    _require(
        external.get("unresolved") == [],
        "external_dependency_closure_contains_unresolved_items",
    )
    rows = external.get("runtime_bindings")
    _require(
        isinstance(rows, list),
        "external_runtime_bindings_not_list",
    )
    matches = [
        row
        for row in rows
        if isinstance(row, Mapping)
        and row.get("binding") == XACRO_RUNTIME_BINDING
    ]
    _require(
        len(matches) == 1,
        "xacro_runtime_binding_must_resolve_exactly_once",
    )
    row = matches[0]
    target = row.get("target_canonical_path")
    _require(
        isinstance(target, str) and bool(target),
        "xacro_target_canonical_path_missing",
    )
    pure_target = PurePosixPath(target)
    _require(
        pure_target.is_absolute()
        and target == pure_target.as_posix()
        and "." not in pure_target.parts
        and ".." not in pure_target.parts,
        "xacro_target_path_not_canonical",
    )
    bound_paths = row.get("canonical_paths")
    _require(
        isinstance(bound_paths, list)
        and target in bound_paths
        and bound_paths == sorted(set(bound_paths)),
        "xacro_target_not_closed_by_runtime_binding",
    )
    files = external.get("files")
    _require(
        isinstance(files, list),
        "external_dependency_files_not_list",
    )
    target_records = [
        record
        for record in files
        if isinstance(record, Mapping)
        and record.get("canonical_path") == target
    ]
    _require(
        len(target_records) == 1,
        "xacro_target_file_record_must_exist_exactly_once",
    )
    target_record = target_records[0]
    digest = target_record.get("sha256")
    size = target_record.get("size_bytes")
    _require(
        isinstance(digest, str)
        and len(digest) == 64
        and digest == digest.lower()
        and all(character in "0123456789abcdef" for character in digest),
        "xacro_target_sha256_invalid",
    )
    _require(
        isinstance(size, int) and not isinstance(size, bool) and size >= 0,
        "xacro_target_size_invalid",
    )
    return target


def _invoke_offline_observation(
    event: str,
    callback: Callable[..., object],
    arguments: Sequence[object],
    source_environment: Mapping[str, str],
    call_order: list,
) -> object:
    _require(callable(callback), "{}_callback_not_callable".format(event))
    call_order.append(event)
    try:
        return callback(*arguments)
    except Exception as exc:
        safe_detail = redact_credential_material(
            "{}:{}".format(type(exc).__name__, exc),
            source_environment,
        )
        raise _FailClosed(
            "{}_offline_callback_failed:{}".format(event, safe_detail)
        ) from None


def _append_diagnostic(
    event: str,
    observation: Mapping[str, object],
    source_environment: Mapping[str, str],
    diagnostics: list,
) -> None:
    diagnostics.append(
        {
            "event": event,
            "text": redact_credential_material(
                observation["diagnostic"],
                source_environment,
            ),
        }
    )


def _receipt(
    *,
    status: str,
    failure_reason: Optional[str],
    call_order: Sequence[str],
    barrier: Optional[R6I2PositiveClockBarrier],
    xacro_target: Optional[str],
    xacro_exact_match: bool,
    environment_audit: Optional[Mapping[str, object]],
    diagnostics: Sequence[Mapping[str, str]],
    service_readiness: Optional[Mapping[str, bool]],
) -> Dict[str, object]:
    return {
        "schema_version": "1.0",
        "stage": STAGE,
        "harness_id": HARNESS_ID,
        "review_scope": "offline_bootstrap_integration_repair_only",
        "status": status,
        "failure_reason": failure_reason,
        "execution_authorized": False,
        "authorization_created": False,
        "seed_values": [],
        "seed_or_evidence_units_allocated": 0,
        "seed_or_evidence_units_consumed": 0,
        "ros_or_subprocess_started": False,
        "real_vehicle_used": False,
        "training_started": False,
        "callback_mode": "offline_injected_observations_only",
        "required_call_order": list(REQUIRED_CALL_ORDER),
        "observed_call_order": list(call_order),
        "call_order_exact": tuple(call_order) == REQUIRED_CALL_ORDER,
        "xacro_runtime_binding": {
            "binding": XACRO_RUNTIME_BINDING,
            "target_canonical_path": xacro_target,
            "supplied_executable_exact_match": xacro_exact_match,
        },
        "bootstrap_receipt": (
            None if barrier is None else barrier.receipt()
        ),
        "child_environment_audit": (
            None if environment_audit is None else dict(environment_audit)
        ),
        "log_policy": credential_safe_log_policy_receipt(),
        "redacted_diagnostics": list(diagnostics),
        "service_readiness": (
            None if service_readiness is None else dict(service_readiness)
        ),
    }


def review_bootstrap_integration(
    *,
    dependency_closure: Mapping[str, object],
    xacro_executable: str,
    timeout_s: float,
    attempt_root: Path,
    source_environment: Mapping[str, str],
    spawn_base_observation: Callable[..., object],
    unpause_request_observation: Callable[..., object],
    unpause_ack_observation: Callable[..., object],
    clock_observation: Callable[..., object],
    service_wait_observation: Callable[..., object],
) -> Dict[str, object]:
    """Exercise the repaired ordering with offline injected observations.

    Each callback is an observation adapter used by tests or an offline
    reviewer.  This function has no seed argument, does not create an
    authorization, and contains no facility for starting a process.
    """

    call_order = []  # type: list
    diagnostics = []  # type: list
    barrier = None  # type: Optional[R6I2PositiveClockBarrier]
    xacro_target = None  # type: Optional[str]
    xacro_exact_match = False
    environment_audit = None  # type: Optional[Mapping[str, object]]
    service_readiness = None  # type: Optional[Mapping[str, bool]]

    try:
        xacro_target = _canonical_xacro_target(dependency_closure)
        _require(
            isinstance(xacro_executable, str)
            and xacro_executable == xacro_target,
            "xacro_executable_does_not_exactly_match_closed_binding",
        )
        xacro_exact_match = True
        child_environment, environment_audit = (
            build_credential_safe_environment(
                source_environment,
                Path(attempt_root),
            )
        )
        assert_credential_safe_environment(
            child_environment,
            Path(attempt_root),
        )
        barrier = R6I2PositiveClockBarrier(timeout_s=timeout_s)

        spawn = _exact_observation(
            _invoke_offline_observation(
                "base_spawn",
                spawn_base_observation,
                (dict(child_environment), xacro_executable),
                source_environment,
                call_order,
            ),
            ("monotonic_s", "base_returncode", "diagnostic"),
            "base_spawn",
        )
        _append_diagnostic(
            "base_spawn", spawn, source_environment, diagnostics
        )
        spawn_time = _finite_time(spawn["monotonic_s"], "base_spawn_time")
        barrier.mark_base_spawned(spawn_time)
        barrier.observe_base_exit(spawn["base_returncode"], spawn_time)

        request = _exact_observation(
            _invoke_offline_observation(
                "unpause_request",
                unpause_request_observation,
                (),
                source_environment,
                call_order,
            ),
            ("monotonic_s", "base_returncode", "diagnostic"),
            "unpause_request",
        )
        _append_diagnostic(
            "unpause_request", request, source_environment, diagnostics
        )
        request_time = _finite_time(
            request["monotonic_s"], "unpause_request_time"
        )
        barrier.observe_base_exit(request["base_returncode"], request_time)
        barrier.mark_unpause_requested(request_time)

        acknowledgement = _exact_observation(
            _invoke_offline_observation(
                "unpause_success_ack",
                unpause_ack_observation,
                (),
                source_environment,
                call_order,
            ),
            (
                "monotonic_s",
                "base_returncode",
                "service_success",
                "diagnostic",
            ),
            "unpause_success_ack",
        )
        _append_diagnostic(
            "unpause_success_ack",
            acknowledgement,
            source_environment,
            diagnostics,
        )
        acknowledgement_time = _finite_time(
            acknowledgement["monotonic_s"],
            "unpause_acknowledgement_time",
        )
        barrier.observe_base_exit(
            acknowledgement["base_returncode"],
            acknowledgement_time,
        )
        barrier.mark_unpause_acknowledged(
            acknowledgement_time,
            service_success=acknowledgement["service_success"],
        )

        first_clock = _exact_observation(
            _invoke_offline_observation(
                "post_ack_positive_clock_1",
                clock_observation,
                (1,),
                source_environment,
                call_order,
            ),
            (
                "sec",
                "nsec",
                "monotonic_s",
                "base_returncode",
                "diagnostic",
            ),
            "post_ack_positive_clock_1",
        )
        _append_diagnostic(
            "post_ack_positive_clock_1",
            first_clock,
            source_environment,
            diagnostics,
        )
        first_clock_time = _finite_time(
            first_clock["monotonic_s"], "first_clock_time"
        )
        barrier.observe_base_exit(
            first_clock["base_returncode"], first_clock_time
        )
        first_progress = barrier.observe_clock(
            first_clock["sec"],
            first_clock["nsec"],
            first_clock_time,
        )
        _require(
            first_progress is False
            and barrier.receipt()["positive_clock"] is not None,
            "first_post_ack_clock_must_be_positive_and_not_release",
        )

        second_clock = _exact_observation(
            _invoke_offline_observation(
                "post_ack_positive_clock_2",
                clock_observation,
                (2,),
                source_environment,
                call_order,
            ),
            (
                "sec",
                "nsec",
                "monotonic_s",
                "base_returncode",
                "diagnostic",
            ),
            "post_ack_positive_clock_2",
        )
        _append_diagnostic(
            "post_ack_positive_clock_2",
            second_clock,
            source_environment,
            diagnostics,
        )
        second_clock_time = _finite_time(
            second_clock["monotonic_s"], "second_clock_time"
        )
        barrier.observe_base_exit(
            second_clock["base_returncode"], second_clock_time
        )
        _require(
            barrier.observe_clock(
                second_clock["sec"],
                second_clock["nsec"],
                second_clock_time,
            )
            is True,
            "second_post_ack_clock_must_strictly_progress",
        )

        barrier.release_service_wait(
            second_clock_time,
            base_returncode=second_clock["base_returncode"],
        )
        call_order.append("service_wait_release")
        _require(
            barrier.service_wait_allowed is True,
            "service_wait_release_did_not_open_barrier",
        )

        service = _exact_observation(
            _invoke_offline_observation(
                "move_base_teb_service_wait",
                service_wait_observation,
                (dict(child_environment),),
                source_environment,
                call_order,
            ),
            (
                "monotonic_s",
                "base_returncode",
                "move_base_ready",
                "teb_reconfigure_ready",
                "diagnostic",
            ),
            "move_base_teb_service_wait",
        )
        _append_diagnostic(
            "move_base_teb_service_wait",
            service,
            source_environment,
            diagnostics,
        )
        service_time = _finite_time(
            service["monotonic_s"], "service_wait_time"
        )
        barrier.check_deadline(service_time)
        _require(
            service["base_returncode"] is None,
            "base_exited_during_service_wait",
        )
        _require(
            service["move_base_ready"] is True
            and service["teb_reconfigure_ready"] is True,
            "move_base_or_teb_service_not_ready",
        )
        service_readiness = {
            "move_base_ready": True,
            "teb_reconfigure_ready": True,
            "wait_callback_invoked_after_clock_release": True,
        }
        _require(
            tuple(call_order) == REQUIRED_CALL_ORDER,
            "bootstrap_call_order_drifted",
        )
    except R6I2RepairHarnessError:
        raise
    except (
        _FailClosed,
        R6I2BootstrapError,
        R6I2EnvironmentPolicyError,
        ValueError,
        TypeError,
    ) as exc:
        failure_reason = redact_credential_material(
            "{}:{}".format(type(exc).__name__, exc),
            source_environment
            if isinstance(source_environment, Mapping)
            else {},
        )
        failed = _receipt(
            status="failed_closed_execution_not_authorized",
            failure_reason=failure_reason,
            call_order=call_order,
            barrier=barrier,
            xacro_target=xacro_target,
            xacro_exact_match=xacro_exact_match,
            environment_audit=environment_audit,
            diagnostics=diagnostics,
            service_readiness=service_readiness,
        )
        raise R6I2RepairHarnessError(failure_reason, failed) from None

    return _receipt(
        status="bootstrap_integration_review_pass_execution_not_authorized",
        failure_reason=None,
        call_order=call_order,
        barrier=barrier,
        xacro_target=xacro_target,
        xacro_exact_match=xacro_exact_match,
        environment_audit=environment_audit,
        diagnostics=diagnostics,
        service_readiness=service_readiness,
    )
