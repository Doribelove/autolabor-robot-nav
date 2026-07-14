"""Fail-closed, simulation-only typed TEB dynamic-reconfigure transactions.

The core transaction manager is ROS-independent and accepts a narrow adapter.
The ROS adapter is imported only when explicitly constructed.  Every request
contains the complete ordered V2 profile, verifies description/type/range,
acknowledgement and callback readback, and restores the previous executed
profile after a failed write.  Shutdown restores the startup snapshot.
"""

import atexit
from dataclasses import dataclass
import math
import queue
import threading
import time
from typing import Any, Dict, List, Mapping, Optional, Sequence

from .action_pipeline import (
    ActionPipelineError,
    AnchorBank,
    ExecutionReceipt,
    TypedProfile,
)


EXPECTED_TEB_NAMESPACE = "/move_base/TebLocalPlannerROS"


class TypedTebTransactionError(ActionPipelineError):
    """Base error caught atomically by the rule transaction loop."""

    code = "typed_teb_transaction_fault"


class SimulationWriteGateError(TypedTebTransactionError):
    code = "simulation_write_denied"


class ParameterDescriptionError(TypedTebTransactionError):
    code = "parameter_description_error"


class ParameterTypeError(TypedTebTransactionError):
    code = "parameter_type_error"


class ParameterRangeError(TypedTebTransactionError):
    code = "parameter_range_error"


class ParameterTimeoutError(TypedTebTransactionError):
    code = "parameter_timeout"


class AckMismatchError(TypedTebTransactionError):
    code = "ack_mismatch"


class ReadbackMismatchError(TypedTebTransactionError):
    code = "readback_mismatch"


class RestoreError(TypedTebTransactionError):
    code = "restore_failed"


@dataclass(frozen=True)
class SimulationWriteContext:
    """Independent evidence required before constructing a write backend."""

    explicit_simulation_write: bool
    use_sim_time: bool
    simulation_marker: bool
    gazebo_clock_active: bool
    teb_namespace: str


@dataclass(frozen=True)
class DynamicParameterSpec:
    name: str
    parameter_type: str
    minimum: Optional[float]
    maximum: Optional[float]
    current: Any
    description: str = ""


def require_simulation_write(context: SimulationWriteContext) -> None:
    failures = []
    if context.explicit_simulation_write is not True:
        failures.append("explicit simulation write opt-in is not true")
    if context.use_sim_time is not True:
        failures.append("/use_sim_time is not true")
    if context.simulation_marker is not True:
        failures.append("/m2_gazebo/simulation_only is not true")
    if context.gazebo_clock_active is not True:
        failures.append("Gazebo simulation clock is not active")
    if context.teb_namespace != EXPECTED_TEB_NAMESPACE:
        failures.append("TEB namespace is not the exact simulation namespace")
    if failures:
        raise SimulationWriteGateError("parameter write denied: " + "; ".join(failures))


def _typed_value(value: Any, parameter_type: str, context: str) -> Any:
    if parameter_type == "bool":
        if type(value) is not bool:
            raise ParameterTypeError("{} must be bool".format(context))
        return value
    if parameter_type == "int":
        if type(value) is not int:
            raise ParameterTypeError("{} must be int".format(context))
        return value
    if parameter_type != "double":
        raise ParameterTypeError("{} has unsupported type {}".format(context, parameter_type))
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ParameterTypeError("{} must be double".format(context))
    result = float(value)
    if not math.isfinite(result):
        raise ParameterTypeError("{} must be finite".format(context))
    return result


def _extract_values(
    raw: Mapping[str, Any], bank: AnchorBank, context: str
) -> Dict[str, Any]:
    if not isinstance(raw, Mapping):
        raise ParameterDescriptionError("{} must be a mapping".format(context))
    missing = [name for name in bank.parameter_names if name not in raw]
    if missing:
        raise ParameterDescriptionError("{} missing {}".format(context, missing))
    return {
        name: _typed_value(raw[name], bank.definitions[name].parameter_type,
                           "{}.{}".format(context, name))
        for name in bank.parameter_names
    }


def _equal(
    expected: Mapping[str, Any], actual: Mapping[str, Any], bank: AnchorBank,
    tolerance: float,
) -> bool:
    for name in bank.parameter_names:
        parameter_type = bank.definitions[name].parameter_type
        if parameter_type == "double":
            if not math.isclose(float(expected[name]), float(actual[name]),
                                rel_tol=tolerance, abs_tol=tolerance):
                return False
        elif type(expected[name]) is not type(actual[name]) or expected[name] != actual[name]:
            return False
    return True


def validate_parameter_interface(
    descriptions: Sequence[Mapping[str, Any]], current: Mapping[str, Any], bank: AnchorBank
) -> Dict[str, DynamicParameterSpec]:
    """Validate the live server against all 20 ordered V2 parameter definitions."""

    if not isinstance(descriptions, Sequence):
        raise ParameterDescriptionError("parameter descriptions must be a sequence")
    by_name = {}
    for item in descriptions:
        if not isinstance(item, Mapping):
            continue
        name = item.get("name")
        if isinstance(name, str):
            if name in by_name:
                raise ParameterDescriptionError("duplicate description for {}".format(name))
            by_name[name] = item
    missing = [name for name in bank.parameter_names if name not in by_name]
    if missing:
        raise ParameterDescriptionError("missing descriptions: {}".format(missing))
    current_values = _extract_values(current, bank, "current configuration")
    specs = {}
    for name in bank.parameter_names:
        definition = bank.definitions[name]
        item = by_name[name]
        if item.get("type") != definition.parameter_type:
            raise ParameterTypeError(
                "{} description type is {}, expected {}".format(
                    name, item.get("type"), definition.parameter_type
                )
            )
        minimum = maximum = None
        if definition.parameter_type != "bool":
            minimum = _typed_value(item.get("min"), definition.parameter_type, name + ".min")
            maximum = _typed_value(item.get("max"), definition.parameter_type, name + ".max")
            if minimum > maximum:
                raise ParameterRangeError("{} described bounds are reversed".format(name))
            if definition.lower < minimum or definition.upper > maximum:
                raise ParameterRangeError(
                    "{} candidate domain [{}, {}] exceeds live domain [{}, {}]".format(
                        name, definition.lower, definition.upper, minimum, maximum
                    )
                )
            if current_values[name] < minimum or current_values[name] > maximum:
                raise ParameterRangeError("{} current value is outside live bounds".format(name))
        specs[name] = DynamicParameterSpec(
            name=name,
            parameter_type=definition.parameter_type,
            minimum=None if minimum is None else float(minimum),
            maximum=None if maximum is None else float(maximum),
            current=current_values[name],
            description=str(item.get("description", "")),
        )
    # The calibration launch must start inside the candidate bank so that the
    # first smooth transition has a valid previous-executed origin.
    bank.validate_values(current_values, "live startup configuration")
    return specs


def _daemon_call(function: Any, timeout_s: float) -> Any:
    output = queue.Queue(maxsize=1)

    def invoke() -> None:
        try:
            output.put((True, function()))
        except BaseException as exc:
            output.put((False, exc))

    thread = threading.Thread(target=invoke, name="v2-typed-teb-reconfigure")
    thread.daemon = True
    thread.start()
    try:
        succeeded, value = output.get(timeout=timeout_s)
    except queue.Empty:
        return None
    if not succeeded:
        raise TypedTebTransactionError(
            "dynamic_reconfigure call failed: {}".format(value)
        )
    return value


class RosTypedDynamicReconfigureAdapter:
    """Thin adapter; one update_configuration call is one atomic 20-value write."""

    def __init__(self, namespace: str, connect_timeout_s: float):
        from dynamic_reconfigure.client import Client

        self._condition = threading.Condition()
        self._generation = 0
        self._request_generation = 0
        self._last_configuration = None
        try:
            self._client = Client(namespace, timeout=connect_timeout_s)
        except Exception as exc:
            raise ParameterTimeoutError(
                "cannot connect to dynamic_reconfigure server: {}".format(exc)
            )
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
        result = self._client.get_configuration(timeout=timeout_s)
        return None if result is None else dict(result)

    def update_configuration(self, values: Mapping[str, Any], timeout_s: float) -> Any:
        with self._condition:
            self._request_generation = self._generation
        result = _daemon_call(
            lambda: self._client.update_configuration(dict(values)), timeout_s
        )
        return None if result is None else dict(result)

    def wait_for_configuration(self, expected: Mapping[str, Any], timeout_s: float) -> Any:
        del expected
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._generation <= self._request_generation:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
            return None if self._last_configuration is None else dict(self._last_configuration)

    def close(self) -> None:
        self._client.close()


class TypedTebTransactionBackend:
    """Execution backend compatible with :class:`RuleAnchorTransactionLoop`."""

    backend_id = "simulation_teb_dynamic_reconfigure"

    def __init__(
        self,
        bank: AnchorBank,
        adapter: Any,
        simulation_context: SimulationWriteContext,
        timeout_s: float = 2.0,
        equality_tolerance: float = 1.0e-9,
        time_source: Any = time.time,
    ):
        require_simulation_write(simulation_context)
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        if equality_tolerance < 0.0:
            raise ValueError("equality_tolerance must be non-negative")
        self.bank = bank
        self._adapter = adapter
        self._timeout_s = float(timeout_s)
        self._tolerance = float(equality_tolerance)
        self._time_source = time_source
        self._specs = {}
        self._startup = None
        self.current = None
        self._closed = False
        self._atexit_registered = False
        self.audit_records: List[Dict[str, Any]] = []

    @property
    def startup(self) -> TypedProfile:
        if self._startup is None:
            raise TypedTebTransactionError("typed backend is not initialized")
        return self._startup

    @property
    def specs(self) -> Dict[str, DynamicParameterSpec]:
        return dict(self._specs)

    def initialize(self) -> Dict[str, DynamicParameterSpec]:
        descriptions = self._adapter.get_parameter_descriptions(self._timeout_s)
        if descriptions is None:
            raise ParameterTimeoutError("parameter descriptions timed out")
        current = self._adapter.get_configuration(self._timeout_s)
        if current is None:
            raise ParameterTimeoutError("current configuration timed out")
        self._specs = validate_parameter_interface(descriptions, current, self.bank)
        values = {name: self._specs[name].current for name in self.bank.parameter_names}
        self._startup = TypedProfile("runtime_snapshot", "startup", dict(values))
        self.current = self._startup
        if not self._atexit_registered:
            atexit.register(self._restore_at_exit)
            self._atexit_registered = True
        return self.specs

    def _validate_request(self, profile: TypedProfile) -> Dict[str, Any]:
        if not self._specs or self.current is None:
            raise TypedTebTransactionError("typed backend is not initialized")
        values = self.bank.validate_values(profile.values, "typed request")
        for name, value in values.items():
            spec = self._specs[name]
            if spec.parameter_type != "bool" and not spec.minimum <= value <= spec.maximum:
                raise ParameterRangeError("{} is outside live bounds".format(name))
        return values

    def _transaction(
        self, profile: TypedProfile, values: Mapping[str, Any], t_request_s: float,
        operation: str,
    ) -> ExecutionReceipt:
        record = {
            "operation": operation,
            "profile_id": profile.profile_id,
            "request": dict(values),
            "ack": None,
            "readback": None,
            "passed": False,
        }
        try:
            ack_raw = self._adapter.update_configuration(values, self._timeout_s)
            if ack_raw is None:
                raise ParameterTimeoutError("configuration acknowledgement timed out")
            t_ack = max(float(t_request_s), float(self._time_source()))
            ack_values = _extract_values(ack_raw, self.bank, "acknowledgement")
            record["ack"] = dict(ack_values)
            if not _equal(values, ack_values, self.bank, self._tolerance):
                raise AckMismatchError("typed acknowledgement differs from request")
            readback_raw = self._adapter.wait_for_configuration(values, self._timeout_s)
            if readback_raw is None:
                raise ParameterTimeoutError("configuration readback timed out")
            t_readback = max(t_ack, float(self._time_source()))
            readback_values = _extract_values(readback_raw, self.bank, "readback")
            record["readback"] = dict(readback_values)
            if not _equal(values, readback_values, self.bank, self._tolerance):
                raise ReadbackMismatchError("typed readback differs from request")
            acknowledgement = TypedProfile(profile.anchor_id, profile.profile_id, ack_values)
            readback = TypedProfile(profile.anchor_id, profile.profile_id, readback_values)
            # TEB's reconfigure callback applies the config before the service
            # reply; callback readback is therefore a conservative activation barrier.
            receipt = ExecutionReceipt(
                profile, acknowledgement, readback, readback,
                float(t_request_s), t_ack, t_readback, t_readback,
            )
            record["passed"] = True
            return receipt
        finally:
            self.audit_records.append(record)

    def apply(self, requested: TypedProfile, now_s: float) -> ExecutionReceipt:
        values = self._validate_request(requested)
        previous = self.current
        write_attempted = False
        try:
            write_attempted = True
            receipt = self._transaction(requested, values, float(now_s), "apply")
            self.current = receipt.executed
            return receipt
        except Exception as exc:
            failure = exc if isinstance(exc, TypedTebTransactionError) else TypedTebTransactionError(str(exc))
            if write_attempted:
                try:
                    restore_time = max(float(now_s), float(self._time_source()))
                    restored = self._transaction(
                        previous, previous.values, restore_time, "fault_restore_previous"
                    )
                    self.current = restored.executed
                except Exception as restore_exc:
                    self.current = previous
                    raise RestoreError(
                        "{}; previous-executed restore failed: {}".format(failure, restore_exc)
                    )
            raise failure

    def restore_startup(self, operation: str = "restore_startup") -> ExecutionReceipt:
        startup = self.startup
        receipt = self._transaction(
            startup, startup.values, float(self._time_source()), operation
        )
        self.current = receipt.executed
        return receipt

    def _restore_at_exit(self) -> None:
        if self._closed or self._startup is None:
            return
        try:
            self.restore_startup("atexit_restore_startup")
        except Exception:
            pass

    def close(self) -> None:
        if self._closed:
            return
        try:
            if self._startup is not None:
                self.restore_startup("close_restore_startup")
        finally:
            self._closed = True
            if hasattr(self._adapter, "close"):
                self._adapter.close()

    def __enter__(self):
        self.initialize()
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> bool:
        self.close()
        return False
