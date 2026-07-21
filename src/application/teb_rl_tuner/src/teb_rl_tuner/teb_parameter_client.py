"""Validated, atomic dynamic_reconfigure client for the nine thesis parameters.

The core client is deliberately independent of rospy so its fail-closed behavior can
be unit tested.  :class:`RosDynamicReconfigureBackend` is the thin ROS adapter.
"""

import atexit
import math
import queue
import threading
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence

from .config import EXPECTED_THETA_ORDER
from .safety_gate import SimulationWriteContext, require_t02_simulation


class TebParameterError(RuntimeError):
    """Base class for a fail-closed TEB parameter-interface fault."""

    code = "interface_fault"


class ParameterDescriptionError(TebParameterError):
    code = "parameter_description_error"


class ParameterTypeError(TebParameterError):
    code = "parameter_type_error"


class ParameterRangeError(TebParameterError):
    code = "parameter_out_of_range"


class ParameterTimeoutError(TebParameterError):
    code = "parameter_timeout"


class AckMismatchError(TebParameterError):
    code = "ack_mismatch"


class ReadbackMismatchError(TebParameterError):
    code = "readback_mismatch"


class RestoreError(TebParameterError):
    code = "snapshot_restore_failed"


@dataclass(frozen=True)
class ParameterSpec:
    name: str
    type: str
    minimum: float
    maximum: float
    current: float
    description: str = ""


def _strict_float(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ParameterTypeError("{} must be a dynamic_reconfigure double".format(name))
    number = float(value)
    if not math.isfinite(number):
        raise ParameterTypeError("{} must be finite".format(name))
    return number


def _theta_values(values: Mapping[str, Any], context: str) -> Dict[str, float]:
    missing = [name for name in EXPECTED_THETA_ORDER if name not in values]
    if missing:
        raise ParameterDescriptionError(
            "{} missing parameters: {}".format(context, ", ".join(missing))
        )
    return {name: _strict_float(values[name], name) for name in EXPECTED_THETA_ORDER}


def _equal(expected: Mapping[str, float], actual: Mapping[str, float], tolerance: float) -> bool:
    return all(
        math.isclose(expected[name], actual[name], rel_tol=tolerance, abs_tol=tolerance)
        for name in EXPECTED_THETA_ORDER
    )


def validate_parameter_interface(
    descriptions: Sequence[Mapping[str, Any]], current: Mapping[str, Any]
) -> Dict[str, ParameterSpec]:
    """Validate names, double types, finite min/max, and current values."""

    by_name: Dict[str, Mapping[str, Any]] = {}
    for item in descriptions:
        name = item.get("name")
        if isinstance(name, str):
            if name in by_name:
                raise ParameterDescriptionError("duplicate description for {}".format(name))
            by_name[name] = item

    missing = [name for name in EXPECTED_THETA_ORDER if name not in by_name]
    if missing:
        raise ParameterDescriptionError("missing descriptions: {}".format(", ".join(missing)))

    current_theta = _theta_values(current, "current configuration")
    specs = {}
    for name in EXPECTED_THETA_ORDER:
        item = by_name[name]
        if item.get("type") != "double":
            raise ParameterTypeError(
                "{} description type must be double, got {}".format(name, item.get("type"))
            )
        minimum = _strict_float(item.get("min"), name + ".min")
        maximum = _strict_float(item.get("max"), name + ".max")
        if minimum > maximum:
            raise ParameterRangeError("{} has min greater than max".format(name))
        value = current_theta[name]
        if value < minimum or value > maximum:
            raise ParameterRangeError("{} current value is outside described bounds".format(name))
        specs[name] = ParameterSpec(
            name=name,
            type="double",
            minimum=minimum,
            maximum=maximum,
            current=value,
            description=str(item.get("description", "")),
        )
    return specs


class TebParameterClient:
    """Fail-closed transaction manager for exactly nine TEB parameters."""

    def __init__(
        self,
        backend: Any,
        simulation_context: SimulationWriteContext,
        timeout_s: float = 2.0,
        equality_tolerance: float = 1e-9,
        clock: Any = time.monotonic,
        wall_clock: Any = time.time,
    ) -> None:
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        require_t02_simulation(simulation_context)
        self._backend = backend
        self._timeout_s = float(timeout_s)
        self._tolerance = float(equality_tolerance)
        self._clock = clock
        self._wall_clock = wall_clock
        self._specs: Dict[str, ParameterSpec] = {}
        self._snapshot: Optional[Dict[str, float]] = None
        self._latest: Optional[Dict[str, float]] = None
        self._sequence = 0
        self._closed = False
        self._audit: List[Dict[str, Any]] = []
        self._atexit_registered = False

    @property
    def specs(self) -> Dict[str, ParameterSpec]:
        return dict(self._specs)

    @property
    def snapshot(self) -> Dict[str, float]:
        if self._snapshot is None:
            raise TebParameterError("client has not been initialized")
        return dict(self._snapshot)

    @property
    def audit_records(self) -> List[Dict[str, Any]]:
        return [dict(record) for record in self._audit]

    def initialize(self) -> Dict[str, ParameterSpec]:
        descriptions = self._backend.get_parameter_descriptions(self._timeout_s)
        if descriptions is None:
            raise ParameterTimeoutError("timed out waiting for parameter descriptions")
        current = self._backend.get_configuration(self._timeout_s)
        if current is None:
            raise ParameterTimeoutError("timed out waiting for current configuration")
        self._specs = validate_parameter_interface(descriptions, current)
        self._snapshot = {name: self._specs[name].current for name in EXPECTED_THETA_ORDER}
        self._latest = dict(self._snapshot)
        if not self._atexit_registered:
            atexit.register(self._restore_at_exit)
            self._atexit_registered = True
        return self.specs

    def validate_request(self, values: Mapping[str, Any]) -> Dict[str, float]:
        if not self._specs:
            raise TebParameterError("client has not been initialized")
        if set(values) != set(EXPECTED_THETA_ORDER):
            missing = sorted(set(EXPECTED_THETA_ORDER) - set(values))
            extra = sorted(set(values) - set(EXPECTED_THETA_ORDER))
            raise ParameterDescriptionError(
                "atomic request must contain exactly nine theta parameters; missing={}, extra={}".format(
                    missing, extra
                )
            )
        validated = {}
        for name in EXPECTED_THETA_ORDER:
            value = _strict_float(values[name], name)
            spec = self._specs[name]
            if value < spec.minimum or value > spec.maximum:
                raise ParameterRangeError(
                    "{}={} outside [{}, {}]".format(name, value, spec.minimum, spec.maximum)
                )
            validated[name] = value
        return validated

    def apply(self, values: Mapping[str, Any]) -> Dict[str, Any]:
        request = self.validate_request(values)
        self._sequence += 1
        record: Dict[str, Any] = {
            "config_seq": self._sequence,
            "operation": "apply",
            "request": dict(request),
            "ack": None,
            "readback": None,
            "latency_s": None,
            "t_request_wall_s": self._wall_clock(),
            "t_ack_wall_s": None,
            "t_readback_wall_s": None,
            "passed": False,
            "failure_code": None,
            "failure_message": None,
            "restore_attempted": False,
            "restore_succeeded": None,
        }
        started = self._clock()
        try:
            ack_raw = self._backend.update_configuration(request, self._timeout_s)
            if ack_raw is None:
                raise ParameterTimeoutError("set_parameters request timed out")
            ack = _theta_values(ack_raw, "service acknowledgement")
            record["t_ack_wall_s"] = self._wall_clock()
            record["ack"] = dict(ack)
            if not _equal(request, ack, self._tolerance):
                raise AckMismatchError("service acknowledgement differs from request")

            readback_raw = self._backend.wait_for_configuration(request, self._timeout_s)
            if readback_raw is None:
                raise ParameterTimeoutError("timed out waiting for parameter readback")
            readback = _theta_values(readback_raw, "parameter readback")
            record["t_readback_wall_s"] = self._wall_clock()
            record["readback"] = dict(readback)
            if not _equal(request, readback, self._tolerance):
                raise ReadbackMismatchError("final readback differs from request")
            self._latest = dict(readback)
            record["passed"] = True
            return record
        except Exception as exc:
            if not isinstance(exc, TebParameterError):
                exc = TebParameterError(str(exc))
            record["failure_code"] = exc.code
            record["failure_message"] = str(exc)
            record["restore_attempted"] = True
            try:
                self._restore_snapshot(operation="fault_restore")
                record["restore_succeeded"] = True
            except Exception as restore_exc:
                record["restore_succeeded"] = False
                record["restore_failure"] = str(restore_exc)
            raise exc
        finally:
            record["latency_s"] = max(0.0, self._clock() - started)
            self._audit.append(record)

    def _restore_snapshot(self, operation: str = "restore") -> Dict[str, Any]:
        if self._snapshot is None:
            raise RestoreError("no startup snapshot is available")
        started = self._clock()
        t_request_wall_s = self._wall_clock()
        ack_raw = self._backend.update_configuration(self._snapshot, self._timeout_s)
        if ack_raw is None:
            raise RestoreError("snapshot restore request timed out")
        ack = _theta_values(ack_raw, "restore acknowledgement")
        t_ack_wall_s = self._wall_clock()
        if not _equal(self._snapshot, ack, self._tolerance):
            raise RestoreError("snapshot restore acknowledgement mismatch")
        final_raw = self._backend.wait_for_configuration(self._snapshot, self._timeout_s)
        if final_raw is None:
            raise RestoreError("snapshot restore readback timed out")
        final = _theta_values(final_raw, "restore readback")
        t_readback_wall_s = self._wall_clock()
        if not _equal(self._snapshot, final, self._tolerance):
            raise RestoreError("snapshot restore readback mismatch")
        self._latest = dict(final)
        result = {
            "operation": operation,
            "request": dict(self._snapshot),
            "ack": ack,
            "readback": final,
            "latency_s": max(0.0, self._clock() - started),
            "t_request_wall_s": t_request_wall_s,
            "t_ack_wall_s": t_ack_wall_s,
            "t_readback_wall_s": t_readback_wall_s,
            "passed": True,
        }
        self._audit.append(result)
        return result

    def restore(self) -> Dict[str, Any]:
        return self._restore_snapshot()

    def _restore_at_exit(self) -> None:
        if self._closed or self._snapshot is None:
            return
        try:
            self._restore_snapshot(operation="atexit_restore")
        except Exception:
            pass

    def close(self) -> None:
        if self._closed:
            return
        try:
            if self._snapshot is not None:
                self._restore_snapshot(operation="close_restore")
        finally:
            self._closed = True
            if hasattr(self._backend, "close"):
                self._backend.close()

    def __enter__(self) -> "TebParameterClient":
        self.initialize()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> bool:
        self.close()
        return False


def _daemon_call(function: Any, timeout_s: float) -> Any:
    """Run a rospy service call with a client-side wall-clock deadline."""

    output: "queue.Queue[Any]" = queue.Queue(maxsize=1)

    def invoke() -> None:
        try:
            output.put((True, function()))
        except BaseException as exc:  # propagate rospy exceptions through the queue
            output.put((False, exc))

    thread = threading.Thread(target=invoke, name="teb-reconfigure-call")
    thread.daemon = True
    thread.start()
    try:
        succeeded, value = output.get(timeout=timeout_s)
    except queue.Empty:
        return None
    if not succeeded:
        raise TebParameterError("dynamic_reconfigure service call failed: {}".format(value))
    return value


class RosDynamicReconfigureBackend:
    """ROS adapter using one update_configuration call per nine-value request."""

    def __init__(self, namespace: str, connect_timeout_s: float) -> None:
        from dynamic_reconfigure.client import Client

        self._condition = threading.Condition()
        self._generation = 0
        self._last_configuration: Optional[Dict[str, Any]] = None
        self._request_generation = 0
        try:
            self._client = Client(namespace, timeout=connect_timeout_s)
        except Exception as exc:
            raise ParameterTimeoutError("cannot connect to dynamic_reconfigure server: {}".format(exc))
        self._client.set_config_callback(self._configuration_callback)

    def _configuration_callback(self, configuration: Optional[Mapping[str, Any]]) -> None:
        if configuration is None:
            return
        with self._condition:
            self._last_configuration = dict(configuration)
            self._generation += 1
            self._condition.notify_all()

    def get_parameter_descriptions(self, timeout_s: float) -> Any:
        return self._client.get_parameter_descriptions(timeout=timeout_s)

    def get_configuration(self, timeout_s: float) -> Any:
        configuration = self._client.get_configuration(timeout=timeout_s)
        return None if configuration is None else dict(configuration)

    def update_configuration(self, values: Mapping[str, float], timeout_s: float) -> Any:
        with self._condition:
            self._request_generation = self._generation
        return _daemon_call(lambda: self._client.update_configuration(dict(values)), timeout_s)

    def wait_for_configuration(self, expected: Mapping[str, float], timeout_s: float) -> Any:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._generation <= self._request_generation:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            return None if self._last_configuration is None else dict(self._last_configuration)

    def close(self) -> None:
        self._client.close()


def specs_as_dict(specs: Mapping[str, ParameterSpec]) -> Dict[str, Dict[str, Any]]:
    return {name: asdict(specs[name]) for name in EXPECTED_THETA_ORDER}
