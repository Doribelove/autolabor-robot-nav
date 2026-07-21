import copy

import pytest

from teb_rl_tuner import (
    AckMismatchError,
    ParameterDescriptionError,
    ParameterRangeError,
    ParameterTimeoutError,
    ParameterTypeError,
    ReadbackMismatchError,
    SimulationGateError,
    SimulationWriteContext,
    TebParameterClient,
)
from teb_rl_tuner.config import EXPECTED_THETA_ORDER


BOUNDS = {
    "max_vel_x": (0.01, 100.0),
    "max_vel_theta": (0.01, 100.0),
    "acc_lim_x": (0.01, 100.0),
    "acc_lim_theta": (0.01, 100.0),
    "min_obstacle_dist": (0.0, 10.0),
    "inflation_dist": (0.0, 15.0),
    "weight_obstacle": (0.0, 1000.0),
    "weight_viapoint": (0.0, 1000.0),
    "weight_optimaltime": (0.0, 1000.0),
}

SNAPSHOT = {
    "max_vel_x": 1.2,
    "max_vel_theta": 1.0,
    "acc_lim_x": 0.8,
    "acc_lim_theta": 0.8,
    "min_obstacle_dist": 0.25,
    "inflation_dist": 0.55,
    "weight_obstacle": 50.0,
    "weight_viapoint": 1.0,
    "weight_optimaltime": 1.0,
}

PROBE = {
    "max_vel_x": 1.1,
    "max_vel_theta": 0.9,
    "acc_lim_x": 0.7,
    "acc_lim_theta": 0.7,
    "min_obstacle_dist": 0.27,
    "inflation_dist": 0.57,
    "weight_obstacle": 55.0,
    "weight_viapoint": 1.1,
    "weight_optimaltime": 1.1,
}


def descriptions():
    return [
        {
            "name": name,
            "type": "double",
            "min": BOUNDS[name][0],
            "max": BOUNDS[name][1],
            "description": name,
        }
        for name in EXPECTED_THETA_ORDER
    ]


class FakeBackend:
    def __init__(self):
        self.descriptions = descriptions()
        self.current = dict(SNAPSHOT)
        self.calls = []
        self.description_timeout = False
        self.config_timeout = False
        self.update_timeout = False
        self.readback_timeout = False
        self.ack_override = None
        self.readback_override = None
        self.closed = False

    def get_parameter_descriptions(self, timeout_s):
        return None if self.description_timeout else copy.deepcopy(self.descriptions)

    def get_configuration(self, timeout_s):
        return None if self.config_timeout else dict(self.current)

    def update_configuration(self, values, timeout_s):
        self.calls.append(dict(values))
        if self.update_timeout:
            return None
        self.current.update(values)
        response = self.current if self.ack_override is None else self.ack_override
        self.ack_override = None
        return dict(response)

    def wait_for_configuration(self, expected, timeout_s):
        if self.readback_timeout:
            return None
        response = self.current if self.readback_override is None else self.readback_override
        self.readback_override = None
        return dict(response)

    def close(self):
        self.closed = True


def simulation_context(**overrides):
    values = {
        "explicit_simulation": True,
        "use_sim_time": True,
        "simulation_marker": True,
        "teb_namespace": "/move_base/TebLocalPlannerROS",
    }
    values.update(overrides)
    return SimulationWriteContext(**values)


def initialized(backend=None):
    backend = backend or FakeBackend()
    client = TebParameterClient(backend, simulation_context(), timeout_s=0.1)
    client.initialize()
    return client, backend


def test_write_gate_fails_closed():
    for override in [
        {"explicit_simulation": False},
        {"use_sim_time": False},
        {"simulation_marker": False},
        {"teb_namespace": "/other/TebLocalPlannerROS"},
    ]:
        with pytest.raises(SimulationGateError):
            TebParameterClient(FakeBackend(), simulation_context(**override))


def test_discovers_nine_double_specs_and_saves_snapshot():
    client, _ = initialized()
    assert list(client.specs) == list(EXPECTED_THETA_ORDER)
    assert client.snapshot == SNAPSHOT
    assert client.specs["min_obstacle_dist"].minimum == 0.0


def test_missing_description_is_rejected():
    backend = FakeBackend()
    backend.descriptions.pop()
    client = TebParameterClient(backend, simulation_context())
    with pytest.raises(ParameterDescriptionError):
        client.initialize()


def test_wrong_description_type_is_rejected():
    backend = FakeBackend()
    backend.descriptions[0]["type"] = "int"
    client = TebParameterClient(backend, simulation_context())
    with pytest.raises(ParameterTypeError):
        client.initialize()


def test_description_and_current_timeouts_are_supported():
    backend = FakeBackend()
    backend.description_timeout = True
    with pytest.raises(ParameterTimeoutError):
        TebParameterClient(backend, simulation_context()).initialize()
    backend = FakeBackend()
    backend.config_timeout = True
    with pytest.raises(ParameterTimeoutError):
        TebParameterClient(backend, simulation_context()).initialize()


def test_request_type_errors_are_rejected_without_write():
    client, backend = initialized()
    for bad_value in ["1.0", True, float("nan"), float("inf")]:
        request = dict(PROBE)
        request["max_vel_x"] = bad_value
        with pytest.raises(ParameterTypeError):
            client.apply(request)
    assert backend.calls == []


def test_missing_or_extra_request_parameter_is_rejected_without_write():
    client, backend = initialized()
    request = dict(PROBE)
    request.pop("weight_optimaltime")
    with pytest.raises(ParameterDescriptionError):
        client.apply(request)
    request = dict(PROBE, unknown_parameter=1.0)
    with pytest.raises(ParameterDescriptionError):
        client.apply(request)
    assert backend.calls == []


def test_out_of_range_is_rejected_without_write():
    client, backend = initialized()
    request = dict(PROBE, min_obstacle_dist=10.01)
    with pytest.raises(ParameterRangeError):
        client.apply(request)
    assert backend.calls == []


def test_atomic_apply_ack_readback_and_restore():
    client, backend = initialized()
    record = client.apply(PROBE)
    assert record["passed"] is True
    assert record["request"] == PROBE
    assert record["ack"] == PROBE
    assert record["readback"] == PROBE
    assert len(backend.calls) == 1
    assert set(backend.calls[0]) == set(EXPECTED_THETA_ORDER)
    restored = client.restore()
    assert restored["passed"] is True
    assert backend.current == SNAPSHOT
    assert len(backend.calls) == 2


def test_ack_mismatch_faults_and_restores_snapshot():
    client, backend = initialized()
    backend.ack_override = dict(PROBE, max_vel_x=1.05)
    with pytest.raises(AckMismatchError):
        client.apply(PROBE)
    assert backend.calls[0] == PROBE
    assert backend.calls[1] == SNAPSHOT
    assert client.audit_records[-1]["restore_succeeded"] is True


def test_readback_mismatch_faults_and_restores_snapshot():
    client, backend = initialized()
    backend.readback_override = dict(PROBE, max_vel_x=1.05)
    with pytest.raises(ReadbackMismatchError):
        client.apply(PROBE)
    assert backend.calls[-1] == SNAPSHOT
    assert client.audit_records[-1]["restore_succeeded"] is True


def test_service_timeout_faults_and_attempts_restore():
    client, backend = initialized()
    backend.update_timeout = True
    with pytest.raises(ParameterTimeoutError):
        client.apply(PROBE)
    record = client.audit_records[-1]
    assert record["failure_code"] == "parameter_timeout"
    assert record["restore_attempted"] is True
    assert record["restore_succeeded"] is False


def test_readback_timeout_faults_and_attempts_restore():
    client, backend = initialized()
    backend.readback_timeout = True
    with pytest.raises(ParameterTimeoutError):
        client.apply(PROBE)
    assert client.audit_records[-1]["restore_attempted"] is True


def test_context_manager_restores_and_closes_backend():
    backend = FakeBackend()
    with TebParameterClient(backend, simulation_context()) as client:
        client.apply(PROBE)
        assert backend.current == PROBE
    assert backend.current == SNAPSHOT
    assert backend.closed is True
